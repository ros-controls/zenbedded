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
#if defined(CONFIG_INVERTED_PENDULUM_COMM_WIFI)
#include <esp_wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#elif defined(CONFIG_INVERTED_PENDULUM_COMM_USB_CDC)
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/usb/usbd.h>
extern "C"
{
#include <sample_usbd.h>
}
#endif
#if defined(CONFIG_NET_CONFIG_SETTINGS)
#include <zephyr/net/net_config.h>
#endif
#if defined(CONFIG_NET_DHCPV4)
#include <zephyr/net/dhcpv4.h>
#endif
#if defined(CONFIG_NET_DHCPV4_SERVER)
#include <zephyr/net/dhcpv4_server.h>
#endif
#endif
#include <math.h>
#include <time.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/sntp.h>
#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_rcl/zenbedded_client.hpp"

LOG_MODULE_REGISTER(inverted_pendulum_tier1, LOG_LEVEL_INF);

constexpr uint32_t kWifiConnectTimeout = 15000;  // ms
constexpr uint32_t kIpv4AcquireTimeout = 30000;  // ms
constexpr uint32_t kControlPeriodMs = 10;
constexpr uint32_t kCommandTimeoutMs = 200;

// A stale command must stop the stepper rather than latch the last velocity forever.
constexpr uint32_t kCommandTimeoutCycles = kCommandTimeoutMs / kControlPeriodMs;

// Bracket the commanded rate to what an A4988 can actually step; outside it, stop instead.
constexpr uint64_t kMinMicrostepIntervalNs = 20000;
constexpr uint64_t kMaxMicrostepIntervalNs = 1000000000;

constexpr uint32_t motor_steps_per_rev = 200;

BUILD_ASSERT(
  DT_NODE_HAS_PROP(DT_ALIAS(stepper_driver), micro_step_res),
  "stepper_driver needs micro-step-res: without it the published angles are off by that factor");

constexpr uint32_t micro_step_res = DT_PROP_OR(DT_ALIAS(stepper_driver), micro_step_res, 1);
constexpr int32_t micro_steps_per_rev = static_cast<int32_t>(motor_steps_per_rev * micro_step_res);

const device * stepper_driver = DEVICE_DT_GET(DT_ALIAS(stepper_driver));
const device * stepper_ctrl = DEVICE_DT_GET(DT_ALIAS(stepper_ctrl));
const device * encoder_driver = DEVICE_DT_GET(DT_ALIAS(encoder_driver));

double microsteps_to_angle(int32_t microsteps)
{
  return 2.0 * M_PI * microsteps / micro_steps_per_rev;
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

  int ret = stepper_ctrl_set_microstep_interval(stepper_ctrl, static_cast<uint64_t>(interval_ns));
  if (ret != 0)
  {
    LOG_ERR("Failed to set microstep interval: %d", ret);
    return;
  }

  ret = stepper_ctrl_run(
    stepper_ctrl,
    (rads_per_sec > 0.0) ? STEPPER_CTRL_DIRECTION_POSITIVE : STEPPER_CTRL_DIRECTION_NEGATIVE);
  if (ret != 0)
  {
    LOG_ERR("Failed to run stepper: %d", ret);
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

bool sync_time_from_ntp()
{
  sntp_time sntp_time;
  int rc = sntp_simple(CONFIG_NTP_SERVER, 5000, &sntp_time);

  if (rc < 0)
  {
    LOG_ERR("NTP sync failed: %d", rc);
    return false;
  }

  timespec ts{};
  ts.tv_sec = sntp_time.seconds;
  ts.tv_nsec = ((uint64_t)sntp_time.fraction * 1000000000ULL) >> 32;

  /* Update the central POSIX CLOCK_REALTIME */
  if (clock_settime(CLOCK_REALTIME, &ts) == 0)
  {
    tm timeinfo;
    gmtime_r(&ts.tv_sec, &timeinfo);

    char date_buf[32];
    strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);

    LOG_INF("System time updated: %s (Epoch: %llds)", date_buf, ts.tv_sec);
    return true;
  }
  else
  {
    LOG_ERR("Failed to set CLOCK_REALTIME");
  }
  return false;
}

#if defined(CONFIG_INVERTED_PENDULUM_COMM_WIFI)
static int init_network(net_if * iface)
{
  LOG_INF("connecting to WiFi using stored credentials...");

  uint32_t timer = k_uptime_get_32();
  while (net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, nullptr, 0) != 0)
  {
    if (k_uptime_get_32() - timer > kWifiConnectTimeout)
    {
      LOG_ERR("Wifi Connection Timedout ...");
      return -ETIMEDOUT;
    }
    LOG_DBG("WiFi connect-stored request failed... retrying...");
    k_sleep(K_MSEC(500));
  }

  LOG_INF("Connected... Waiting for IPV4 address");
  timer = k_uptime_get_32();
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    if (k_uptime_get_32() - timer > kIpv4AcquireTimeout)
    {
      LOG_ERR(
        "No IPV4 address within %u ms; check the stored WiFi credentials", kIpv4AcquireTimeout);
      return -ETIMEDOUT;
    }
    k_sleep(K_MSEC(200));
  }
  LOG_INF("Got IPV4 address");

  // Disable Wi-Fi power saving
  esp_wifi_set_ps(WIFI_PS_NONE);
  k_sleep(K_MSEC(200));
  return 0;
}
#elif defined(CONFIG_INVERTED_PENDULUM_COMM_USB_CDC)
static int init_network(net_if * iface)
{
  LOG_INF("Initializing USB CDC network interface...");
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
  usbd_context * sample_usbd = sample_usbd_init_device(nullptr);
  if (sample_usbd == nullptr)
  {
    LOG_ERR("Failed to initialize USB device context");
    return -ENODEV;
  }

  int usb_err = usbd_enable(sample_usbd);
  if (usb_err != 0)
  {
    LOG_ERR("Failed to enable USB device: %d", usb_err);
    return usb_err;
  }
  LOG_INF("USB device initialized and enabled");

#if defined(CONFIG_NET_CONFIG_SETTINGS)
  (void)net_config_init_app(nullptr, "Initializing USB network");
#elif defined(CONFIG_NET_DHCPV4)
  net_dhcpv4_start(iface);
#endif
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

  LOG_INF("Waiting for IPv4 address on USB CDC interface...");
  uint32_t timer = k_uptime_get_32();
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    if (k_uptime_get_32() - timer > kIpv4AcquireTimeout)
    {
      LOG_ERR("No IPv4 address on USB CDC interface within %u ms", kIpv4AcquireTimeout);
      return -ETIMEDOUT;
    }
    k_sleep(K_MSEC(200));
  }
  LOG_INF("Got IPv4 address on USB CDC interface");

#if defined(CONFIG_NET_DHCPV4_SERVER)
  in_addr pool_addr;
  if (net_addr_pton(AF_INET, CONFIG_INVERTED_PENDULUM_USB_CDC_PEER_IPV4_ADDR, &pool_addr) == 0)
  {
    int srv_err = net_dhcpv4_server_start(iface, &pool_addr);
    if (srv_err != 0)
    {
      LOG_ERR("Failed to start DHCPv4 server: %d", srv_err);
    }
    else
    {
      LOG_INF(
        "DHCPv4 server started leasing %s to host",
        CONFIG_INVERTED_PENDULUM_USB_CDC_PEER_IPV4_ADDR);
    }
  }

  LOG_INF("Waiting for host to acquire DHCP lease (or timeout)...");
  uint32_t dhcp_timer = k_uptime_get_32();
  while (k_uptime_get_32() - dhcp_timer < 5000)
  {
    bool lease_active = false;
    net_dhcpv4_server_foreach_lease(
      iface,
      [](struct net_if *, struct dhcpv4_addr_slot * slot, void * user_data)
      {
        if (slot && slot->state == DHCPV4_SERVER_ADDR_ALLOCATED)
        {
          *static_cast<bool *>(user_data) = true;
        }
      },
      &lease_active);
    if (lease_active)
    {
      LOG_INF("Host acquired DHCP lease on USB CDC interface!");
      break;
    }
    k_sleep(K_MSEC(250));
  }
#endif

  return 0;
}
#else
static int init_network(net_if *) { return 0; }
#endif

int main()
{
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

  LOG_INF("Starting Inverted Pendulum Tier1");
  net_if * iface = net_if_get_default();
  if (int net_err = init_network(iface); net_err != 0)
  {
    LOG_ERR("Failed to initialize network interface: %d", net_err);
    return net_err;
  }

  // Only the JointState header stamp needs wall-clock time, so a failure here must not take the rig
  // down.
  if (!sync_time_from_ntp())
  {
    LOG_WRN("Continuing without NTP sync; state timestamps will be wrong");
  }

  static ZenbeddedClient<JointStateCodec, JointCommandCodec> client;
  const char * joint_names[] = {"stepper_motor", "magnetic_encoder"};
  int ret = client.init(
    50, {.frame_id = "base_link", .joint_names = joint_names, .num_joints = 2},
    {.frame_id = "base_link",
     .joint_names = joint_names,
     .num_joints = 2,
     .interface_name = "velocity"});
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
  double stepper_angle = 0, encoder_angle = 0;
  double stepper_angle_offset = 0, encoder_angle_offset = 0;
  double stepper_angular_velocity = 0;

  ret = stepper_ctrl_get_actual_position(stepper_ctrl, &stepper_position);
  if (ret != 0)
  {
    LOG_ERR("Failed to read initial stepper position: %d", ret);
    return ret;
  }
  stepper_angle_offset = microsteps_to_angle(stepper_position);

  if (!get_encoder_angle_deg(&encoder_angle_offset))
  {
    LOG_ERR("Failed to read initial encoder angle; cannot calibrate the pendulum zero");
    return -EIO;
  }

  uint32_t last_command_count = client.accepted_command_count();
  uint32_t cycles_since_command = 0;

  while (true)
  {
    if (stepper_ctrl_get_actual_position(stepper_ctrl, &stepper_position) == 0)
    {
      stepper_angle = microsteps_to_angle(stepper_position) - stepper_angle_offset;
    }

    double raw_encoder_angle = 0;
    if (get_encoder_angle_deg(&raw_encoder_angle))
    {
      encoder_angle = raw_encoder_angle - encoder_angle_offset;
    }

    double states[] = {stepper_angle, encoder_angle};
    timespec tv{};
    clock_gettime(CLOCK_REALTIME, &tv);
    JointStateCodec::Value state_val = {
      .stamp_sec = (int32_t)tv.tv_sec, .stamp_nanosec = (uint32_t)tv.tv_nsec, .positions = states};

    double cmd[2];
    JointCommandCodec::Value cmd_val{.values = cmd};

    client.write_state(state_val);

    const uint32_t command_count = client.accepted_command_count();
    if (command_count != last_command_count)
    {
      last_command_count = command_count;
      cycles_since_command = 0;
      if (client.read_command(cmd_val))
      {
        stepper_angular_velocity = cmd[0];
      }
    }
    else if (cycles_since_command < kCommandTimeoutCycles)
    {
      ++cycles_since_command;
    }
    else if (stepper_angular_velocity != 0.0)
    {
      LOG_WRN("No command for %u ms, stopping stepper", kCommandTimeoutMs);
      stepper_angular_velocity = 0.0;
    }

    set_stepper_angular_vel(stepper_angular_velocity);

    k_sleep(K_MSEC(kControlPeriodMs));
  }

  client.destroy();
  return 0;
}
