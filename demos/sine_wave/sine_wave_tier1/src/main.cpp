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
#include <inttypes.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/sntp.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <zenbedded_common/network.hpp>
#include <zenbedded_rcl/codecs.hpp>
#include <zenbedded_rcl/zenbedded_client.hpp>

LOG_MODULE_REGISTER(sine_wave_tier1, LOG_LEVEL_INF);

using SineWaveClient = ZenbeddedClient<JointStateCodec, JointCommandCodec>;

double wave_amp = 5.0;
double wave_frequency = 2.0;  // hz
uint32_t loop_freq = 100;     // hz

static int extract_host_from_locator(const char * locator, char * out_host, size_t out_len)
{
  if (locator == nullptr || out_host == nullptr || out_len == 0)
  {
    return -EINVAL;
  }
  const char * start = locator;
  const char * slash = strchr(start, '/');
  if (slash != nullptr)
  {
    start = slash + 1;
  }
  const char * colon = strchr(start, ':');
  size_t host_len = (colon != nullptr) ? static_cast<size_t>(colon - start) : strlen(start);
  if (host_len >= out_len)
  {
    return -ENOSPC;
  }
  memcpy(out_host, start, host_len);
  out_host[host_len] = '\0';
  return 0;
}

static bool sync_time_from_ntp(const char * ntp_host)
{
  if (ntp_host == nullptr || ntp_host[0] == '\0')
  {
    LOG_WRN("No NTP host specified, skipping time synchronization");
    return false;
  }

  LOG_INF("Synchronizing time with NTP server '%s'...", ntp_host);
  sntp_time sntp_time;
  int rc = sntp_simple(ntp_host, 5000, &sntp_time);
  if (rc < 0)
  {
    LOG_ERR("NTP sync failed with server '%s': %d", ntp_host, rc);
    return false;
  }

  timespec ts{};
  ts.tv_sec = sntp_time.seconds;
  ts.tv_nsec = ((uint64_t)sntp_time.fraction * 1000000000ULL) >> 32;

  if (clock_settime(CLOCK_REALTIME, &ts) == 0)
  {
    tm timeinfo;
    gmtime_r(&ts.tv_sec, &timeinfo);

    char date_buf[32];
    strftime(date_buf, sizeof(date_buf), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);

    LOG_INF(
      "System time updated: %s (Epoch: %" PRIi64 "s)", date_buf, static_cast<int64_t>(ts.tv_sec));
    return true;
  }

  LOG_ERR("Failed to set CLOCK_REALTIME");
  return false;
}

int main()
{
  LOG_INF("Starting Sine Wave Tier 1 App");
  if (int net_err = zenbedded::common::init_network(); net_err != 0)
  {
    LOG_ERR("Failed to initialize network interface: %d", net_err);
    return net_err;
  }

  static SineWaveClient client;

  const char * joint_names[] = {"sine_wave"};
  JointStateCodec::InitParams state_params{
    .frame_id = "sine_wave",
    .joint_names = joint_names,
    .num_joints = 1,
  };

  JointCommandCodec::InitParams cmd_params{
    .frame_id = "sine_wave",
    .joint_names = joint_names,
    .num_joints = 1,
    .interface_name = "amplitude",
  };

  int ret = -1;
  const char * selected_locator = ZENBEDDED_DEFAULT_LOCATOR;

#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
  char usb_loc[32];
  if (zenbedded::common::is_usbd_ready())
  {
    if (
      zenbedded::common::get_usb_peer_locator(
        ZENBEDDED_DEFAULT_LOCATOR, usb_loc, sizeof(usb_loc)) == 0)
    {
      LOG_INF("Attempting Zenoh connection over USB CDC ECM (%s)...", usb_loc);
      ret = client.init(loop_freq, state_params, cmd_params, usb_loc);
      if (ret == 0)
      {
        selected_locator = usb_loc;
      }
      else
      {
        LOG_WRN("Failed to connect via USB CDC ECM (%s): %d", usb_loc, ret);
      }
    }
  }
#endif

#if defined(CONFIG_ZENBEDDED_COMMON_WIFI)
  if (ret != 0 && zenbedded::common::is_wifi_ready())
  {
    LOG_INF("Attempting Zenoh connection over Wi-Fi (%s)...", ZENBEDDED_DEFAULT_LOCATOR);
    ret = client.init(loop_freq, state_params, cmd_params, ZENBEDDED_DEFAULT_LOCATOR);
    if (ret == 0)
    {
      selected_locator = ZENBEDDED_DEFAULT_LOCATOR;
    }
  }
#endif

  if (ret != 0)
  {
    ret = client.init(loop_freq, state_params, cmd_params, ZENBEDDED_DEFAULT_LOCATOR);
    if (ret == 0)
    {
      selected_locator = ZENBEDDED_DEFAULT_LOCATOR;
    }
  }

  if (ret != 0)
  {
    LOG_ERR("ZenbeddedClient.init failed with %d, aborting", ret);
    return ret;
  }

  // Synchronize system clock with host Chrony NTP server extracted from selected locator
  char ntp_host[64];
  if (extract_host_from_locator(selected_locator, ntp_host, sizeof(ntp_host)) == 0)
  {
    sync_time_from_ntp(ntp_host);
  }

  int64_t start_time = k_uptime_get();
  int32_t cnt = 0;
  double cmd_value = 0.0;
  JointCommandCodec::Value cmd{
    .stamp_sec = 0,
    .stamp_nanosec = 0,
    .values = &cmd_value,
  };

  double pos = 0.0;
  JointStateCodec::Value state{
    .stamp_sec = 0,
    .stamp_nanosec = 0,
    .positions = &pos,
    .velocities = nullptr,
    .efforts = nullptr,
  };

  while (true)
  {
    if (client.read_command(cmd))
    {
      if (std::fabs(cmd_value) >= 1e-6)
      {
        wave_amp = std::fabs(cmd_value);
      }
    }

    int64_t elapsed_ms = k_uptime_get() - start_time;
    auto current_time_ms = static_cast<double>(elapsed_ms);
    double x = 2.0 * M_PI * current_time_ms * wave_frequency / 1000.0;
    pos = wave_amp * std::sin(x);

    timespec tv{};
    clock_gettime(CLOCK_REALTIME, &tv);
    state.stamp_sec = static_cast<int32_t>(tv.tv_sec);
    state.stamp_nanosec = static_cast<uint32_t>(tv.tv_nsec);

    if (++cnt == 500)
    {
      LOG_INF("Running... pos: %.3f", pos);
      cnt = 0;
    }

    client.write_state(state);

    k_sleep(K_MSEC(10));
  }

  client.destroy();
  return 0;
}
