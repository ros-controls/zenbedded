// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <esp_wifi.h>
#include <math.h>
#include <zenbedded_transport/generated/interface_data.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <algorithm>
#include <zenbedded_rcl/codecs.hpp>
#include <zenbedded_rcl/zenbedded_client.hpp>

LOG_MODULE_REGISTER(inverted_pendulum_tier2, LOG_LEVEL_INF);

constexpr uint32_t kWifiConnectTimeout = 15000;  // ms
constexpr uint32_t kIpv4AcquireTimeout = 30000;  // ms
constexpr uint32_t kControlPeriodMs = 10;
constexpr double kControlPeriodSec = kControlPeriodMs / 1000.0;
constexpr uint32_t kCommandTimeoutMs = 200;

// A stale command must stop the stepper rather than latch the last velocity forever.
constexpr uint32_t kCommandTimeoutCycles = kCommandTimeoutMs / kControlPeriodMs;

// --- Fast encoder sampling params ---------------------------------------------
// encoder sampling + velocity filtering runs in its own thread at kEncoderSampleHz.
constexpr double kEncoderSampleHz = 400.0;
constexpr uint32_t kEncoderSamplePeriodUs = static_cast<uint32_t>(1.0e6 / kEncoderSampleHz);
constexpr double kVelocityFilterAlpha = 0.35;
constexpr double kVelocityFilterTauSec =
  (1 - kVelocityFilterAlpha) / (kVelocityFilterAlpha * kEncoderSampleHz);

// How far ahead of the current control cycle to project the commanded velocity
// before clamping and issuing it as a stepper_ctrl_move_to() target. 1.0 means
// "just enough to cover this control period"; a slightly larger value (e.g. 2-3)
// keeps the driver's move queue topped up so consecutive retargets don't cause
// it to decelerate to a stop between control cycles.
constexpr double kMoveToProjectionPeriods = 2.0;

BUILD_ASSERT(
  DT_NODE_HAS_PROP(DT_ALIAS(stepper_driver), micro_step_res),
  "stepper_driver needs micro-step-res: without it the published angles are off by that factor");

constexpr uint32_t motor_steps_per_rev = 200;
constexpr uint32_t micro_step_res = DT_PROP_OR(DT_ALIAS(stepper_driver), micro_step_res, 1);
constexpr int32_t micro_steps_per_rev = motor_steps_per_rev * micro_step_res;

const device * stepper_driver = DEVICE_DT_GET(DT_ALIAS(stepper_driver));
const device * stepper_ctrl = DEVICE_DT_GET(DT_ALIAS(stepper_ctrl));
const device * encoder_driver = DEVICE_DT_GET(DT_ALIAS(encoder_driver));

// LED Setup
#define STRIP_NUM_PIXELS DT_PROP_OR(DT_ALIAS(led_strip), chain_length, 1)
#define RGB(_r, _g, _b) \
  led_rgb { .r = (_r), .g = (_g), .b = (_b) }
const device * led = DEVICE_DT_GET(DT_ALIAS(led_strip));
static led_rgb pixels[STRIP_NUM_PIXELS];
bool led_ready = false;

// Hard travel limit on the stepper position, measured from the boot-time zero.
constexpr double kStepperAngleLimitRad = 135.0 * M_PI / 180.0;

// Absolute microstep bounds corresponding to +-kStepperAngleLimitRad around the
// boot-time zero. Every move_to() target is clamped into this range before being
// sent to the driver, so the driver's own step-generation logic - not a polling
// loop - is what actually prevents stepping past the limit.
int32_t stepper_min_position = 0;
int32_t stepper_max_position = 0;

constexpr double kMaxAngularVelocity = 7;            // rad/s
constexpr uint64_t kMinMicrostepIntervalNs = 60000;  // stepper limit
constexpr uint64_t kMaxMicrostepIntervalNs = 1000000000;
constexpr uint64_t kStepIntervalForMaxVelNs =
  static_cast<uint64_t>(1e9 * 2.0 * M_PI / (kMaxAngularVelocity * micro_steps_per_rev));

double microsteps_to_angle(int32_t microsteps)
{
  return 2.0 * M_PI * microsteps / micro_steps_per_rev;
}

// maps angle to [-pi, pi]
double wrap_angle(double angle)
{
  if (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  else if (angle <= -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void stop_stepper()
{
  int ret = stepper_ctrl_stop(stepper_ctrl);
  if (ret != 0)
  {
    LOG_ERR("Failed to stop stepper: %d", ret);
  }
}

void set_stepper_angular_vel(double rads_per_sec)
{
  if (!isfinite(rads_per_sec) || rads_per_sec == 0.0)
  {
    stop_stepper();
    return;
  }

  double microsteps_per_sec = fabs(rads_per_sec) * micro_steps_per_rev / (2 * M_PI);
  double interval_ns = 1e9 / microsteps_per_sec;

  if (
    !isfinite(interval_ns) || interval_ns < kMinMicrostepIntervalNs ||
    interval_ns > kMaxMicrostepIntervalNs)
  {
    LOG_WRN("Commanded velocity %f rad/s out of range, stopping", rads_per_sec);
    stop_stepper();
    return;
  }

  interval_ns = std::clamp(
    interval_ns, static_cast<double>(kStepIntervalForMaxVelNs),
    static_cast<double>(kMaxMicrostepIntervalNs));

  int ret = stepper_ctrl_set_microstep_interval(stepper_ctrl, static_cast<uint64_t>(interval_ns));
  if (ret != 0)
  {
    LOG_ERR("Failed to set microstep interval: %d", ret);
    return;
  }

  double signed_microsteps_per_sec =
    (rads_per_sec > 0.0) ? microsteps_per_sec : -microsteps_per_sec;
  double projected_microsteps =
    signed_microsteps_per_sec * kControlPeriodSec * kMoveToProjectionPeriods;

  int32_t actual_position = 0;
  stepper_ctrl_get_actual_position(stepper_ctrl, &actual_position);

  double target_position = static_cast<double>(actual_position) + projected_microsteps;
  target_position = std::clamp(
    target_position, static_cast<double>(stepper_min_position),
    static_cast<double>(stepper_max_position));

  ret = stepper_ctrl_move_to(stepper_ctrl, static_cast<int32_t>(lround(target_position)));
  if (ret != 0)
  {
    LOG_ERR("Failed to command stepper move: %d", ret);
  }
}

bool get_encoder_angle_deg(double * angle_rad)
{
  int ret = sensor_sample_fetch(encoder_driver);
  if (ret != 0)
  {
    return false;
  }

  sensor_value val{};
  ret = sensor_channel_get(encoder_driver, SENSOR_CHAN_ROTATION, &val);
  if (ret != 0)
  {
    return false;
  }

  *angle_rad = sensor_value_to_double(&val) * M_PI / 180;
  return true;
}

// Enforce the hard stepper travel limit in-place. If the commanded velocity would
// drive the stepper further past +-kStepperAngleLimitRad, it is zeroed; velocity that moves
// the stepper back toward the allowed range is left untouched. Recomputed every control
// cycle from the true current position, so it self-corrects rather than latching.
void enforce_stepper_hard_stop(double stepper_angle, double * stepper_angular_velocity)
{
  if (
    (stepper_angle >= kStepperAngleLimitRad && *stepper_angular_velocity > 0.0) ||
    (stepper_angle <= -kStepperAngleLimitRad && *stepper_angular_velocity < 0.0))
  {
    *stepper_angular_velocity = 0.0;
  }
}

/// --- Fast encoder sampling thread -------------------------------------------
struct EncoderState
{
  double angle = 0.0;
  double velocity = 0.0;
};

K_SEM_DEFINE(g_pendulum_ready, 0, 1);
K_MSGQ_DEFINE(encoder_state_msgq, sizeof(EncoderState), 1, 1);

void encoder_sample_thread_entry(void *, void *, void *)
{
  double encoder_angle_offset = 0.0;
  while (!get_encoder_angle_deg(&encoder_angle_offset))
  {
    LOG_WRN("Waiting for a valid encoder reading to calibrate the pendulum zero...");
    k_sleep(K_MSEC(1000));
  }

  double prev_encoder_angle = 0.0;
  bool have_prev = false;
  bool signaled_ready = false;
  uint32_t prev_time_us = k_cyc_to_us_floor32(k_cycle_get_32());
  EncoderState encoder_state;
  encoder_state.angle = M_PI;  // upside down

  while (true)
  {
    uint32_t loop_start_us = k_cyc_to_us_floor32(k_cycle_get_32());

    double raw_encoder_angle = 0.0;
    if (get_encoder_angle_deg(&raw_encoder_angle))
    {
      double angle =
        wrap_angle(fmod(raw_encoder_angle - encoder_angle_offset + 3 * M_PI, 2 * M_PI));

      uint32_t now_us = k_cyc_to_us_floor32(k_cycle_get_32());
      double dt = static_cast<double>(now_us - prev_time_us) / 1.0e6;
      prev_time_us = now_us;

      if (have_prev && dt > 0.0)
      {
        double raw_velocity = wrap_angle(angle - prev_encoder_angle) / dt;
        double alpha = dt / (kVelocityFilterTauSec + dt);

        encoder_state.angle = angle;
        encoder_state.velocity = alpha * raw_velocity + (1.0 - alpha) * encoder_state.velocity;

        if (!signaled_ready)
        {
          signaled_ready = true;
          k_sem_give(&g_pendulum_ready);
        }
      }
      else
      {
        encoder_state.angle = angle;
      }

      while (k_msgq_put(&encoder_state_msgq, &encoder_state, K_NO_WAIT) != 0)
      {
        EncoderState dummy;
        k_msgq_get(&encoder_state_msgq, &dummy, K_NO_WAIT);
      }

      prev_encoder_angle = angle;
      have_prev = true;
    }

    uint32_t elapsed_us = k_cyc_to_us_floor32(k_cycle_get_32()) - loop_start_us;
    k_sleep(
      K_USEC((elapsed_us < kEncoderSamplePeriodUs) ? kEncoderSamplePeriodUs - elapsed_us : 200));
  }
}

K_THREAD_STACK_DEFINE(encoder_sample_thread_stack, 1024);
k_thread encoder_sample_thread;
k_tid_t encoder_sample_thread_tid = k_thread_create(
  &encoder_sample_thread, encoder_sample_thread_stack,
  K_THREAD_STACK_SIZEOF(encoder_sample_thread_stack), encoder_sample_thread_entry, nullptr, nullptr,
  nullptr, CONFIG_ENCODER_SAMPLING_THREAD_PRIORITY, 0, K_FOREVER);

int main()
{
  // Set LEDs to Blue for the Setup Phase
  if (device_is_ready(led))
  {
    pixels[0] = RGB(0x00, 0x00, 0xFF);
    led_strip_update_rgb(led, pixels, STRIP_NUM_PIXELS);
    led_ready = true;
  }
  else
  {
    LOG_WRN("LED device not ready");
  }

  if (!device_is_ready(stepper_driver) || !device_is_ready(stepper_ctrl))
  {
    LOG_ERR("Stepper devices not ready\n");
    return 0;
  }

  if (!device_is_ready(encoder_driver))
  {
    LOG_ERR("Encoder device not ready\n");
    return 0;
  }
  else
  {
    k_thread_start(encoder_sample_thread_tid);
  }

  k_sem_take(&g_pendulum_ready, K_FOREVER);

  LOG_INF("Starting Inverted Pendulum Tier2");
  net_if * iface = net_if_get_default();

  LOG_INF("connecting to WiFi using stored credentials...");

  uint32_t timer = k_uptime_get_32();
  while (net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, nullptr, 0) != 0)
  {
    if (k_uptime_get_32() - timer > kWifiConnectTimeout)
    {
      LOG_ERR("Wifi Connection Timedout ...");
      return -1;
    }
    LOG_ERR("WiFi connect-stored request failed... retrying...");
    k_sleep(K_MSEC(200));
  }

  LOG_INF("Connected... Waiting for IPV4 address");
  timer = k_uptime_get_32();
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    if (k_uptime_get_32() - timer > kIpv4AcquireTimeout)
    {
      LOG_ERR(
        "No IPV4 address within %u ms; check the stored WiFi credentials", kIpv4AcquireTimeout);
      return -1;
    }
    k_sleep(K_MSEC(200));
  }
  LOG_INF("Got IPV4 address");

  // Disable Wi-Fi power saving
  esp_wifi_set_ps(WIFI_PS_NONE);
  k_sleep(K_MSEC(200));

  static ZenbeddedClient<RawCodec<zenbedded_state_t>, RawCodec<zenbedded_command_t> > client;
  int ret = client.init(100);
  if (ret != 0)
  {
    LOG_ERR("Failed to initialize ZenbeddedClient: %d", ret);
    return ret;
  }

  ret = stepper_enable(stepper_driver);
  if (ret != 0)
  {
    LOG_ERR("Failed to enable stepper: %d", ret);
    return ret;
  }

  int32_t stepper_position = 0;
  double stepper_angle = 0;
  double stepper_angle_offset = 0;
  double stepper_angular_velocity = 0;

  // Zero reference for the stepper: whatever position it is in at boot becomes 0,
  // so all reported/limited angles are relative to where the robot was powered on.
  ret = stepper_ctrl_get_actual_position(stepper_ctrl, &stepper_position);
  if (ret != 0)
  {
    LOG_ERR("Failed to read initial stepper position: %d", ret);
    return ret;
  }
  stepper_angle_offset = microsteps_to_angle(stepper_position);

  int32_t stepper_limit_microsteps =
    static_cast<int32_t>(lround(kStepperAngleLimitRad / (2.0 * M_PI) * micro_steps_per_rev));
  stepper_min_position = stepper_position - stepper_limit_microsteps;
  stepper_max_position = stepper_position + stepper_limit_microsteps;

  uint32_t last_command_count = client.accepted_command_count();
  uint32_t cycles_since_command = 0;
  uint32_t print_count = 0;
  double commanded_acceleration = 0.0;
  double prev_stepper_angle = stepper_angle;
  double loop_freq = 0.0;
  uint32_t last_state_time = k_uptime_get_32();
  EncoderState encoder_state;

  uint32_t led_toggle_counter = 0;
  bool is_flash_on = true;

  while (true)
  {
    uint32_t loop_start_ms = k_uptime_get_32();

    if (stepper_ctrl_get_actual_position(stepper_ctrl, &stepper_position) == 0)
    {
      stepper_angle = microsteps_to_angle(stepper_position) - stepper_angle_offset;
    }

    k_msgq_get(&encoder_state_msgq, &encoder_state, K_FOREVER);

    double dt = static_cast<double>(k_uptime_get_32() - last_state_time) / 1000.0;
    last_state_time = k_uptime_get_32();
    double motor_joint_velocity = (dt > 0.0) ? (stepper_angle - prev_stepper_angle) / dt : 0.0;
    prev_stepper_angle = stepper_angle;

    zenbedded_state_t state_val{
      .motor_joint_position = stepper_angle,
      .motor_joint_velocity = motor_joint_velocity,
      .pendulum_joint_position = encoder_state.angle,
      .pendulum_joint_velocity = encoder_state.velocity};

    if (++print_count == 100)
    {
      print_count = 0;
      printk(
        "stepper_angle=%.4f rad, stepper_vel=%.4f rad/s, "
        "encoder_angle=%.4f rad, encoder_vel=%.4f rad/s\n, loop_freq=%.2f Hz",
        stepper_angle, motor_joint_velocity, encoder_state.angle, encoder_state.velocity,
        loop_freq);
    }

    zenbedded_command_t cmd_val;

    client.write_state(state_val);

    const uint32_t command_count = client.accepted_command_count();
    if (command_count != last_command_count)
    {
      last_command_count = command_count;
      cycles_since_command = 0;
      if (client.read_command(cmd_val))
      {
        commanded_acceleration = cmd_val.motor_joint_acceleration;
      }
    }
    else if (cycles_since_command < kCommandTimeoutCycles)
    {
      ++cycles_since_command;
    }
    else if (commanded_acceleration != 0.0 || stepper_angular_velocity != 0.0)
    {
      LOG_WRN("No command for %u ms, stopping stepper", kCommandTimeoutMs);
      commanded_acceleration = 0.0;
      stepper_angular_velocity = 0.0;
    }

    // Integrate the (possibly stale-but-not-yet-timed-out) commanded acceleration into
    // velocity every control cycle, not just when a new command arrives.
    stepper_angular_velocity += commanded_acceleration * kControlPeriodSec;
    stepper_angular_velocity =
      std::clamp(stepper_angular_velocity, -kMaxAngularVelocity, kMaxAngularVelocity);

    enforce_stepper_hard_stop(stepper_angle, &stepper_angular_velocity);
    set_stepper_angular_vel(stepper_angular_velocity);

    // Flashes Green every 250ms
    if (led_ready && ++led_toggle_counter >= 25)
    {
      led_toggle_counter = 0;
      is_flash_on = !is_flash_on;
      pixels[0] = is_flash_on ? RGB(0x00, 0xFF, 0x00) : RGB(0x00, 0x00, 0x00);
      led_strip_update_rgb(led, pixels, STRIP_NUM_PIXELS);
    }

    // ensure kControlPeriodMs loop timing
    uint32_t loop_duration_ms = k_uptime_get_32() - loop_start_ms;
    uint32_t sleep_ms =
      (loop_duration_ms < kControlPeriodMs) ? kControlPeriodMs - loop_duration_ms : 1;
    k_sleep(K_MSEC(sleep_ms));
    loop_freq = 1000.0 / (k_uptime_get_32() - loop_start_ms);
  }

  client.destroy();
  return 0;
}
