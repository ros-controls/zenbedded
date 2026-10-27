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
#include <zenbedded_transport/generated/interface_data.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <cmath>
#include <zenbedded_common/network.hpp>
#include <zenbedded_rcl/zenbedded_client.hpp>

LOG_MODULE_REGISTER(sine_wave_tier2, LOG_LEVEL_INF);

using SineWaveClient = ZenbeddedClient<RawCodec<zenbedded_state_t>, RawCodec<zenbedded_command_t>>;

double wave_amp = 5;
double wave_frequency = 2.0;  // hz
uint32_t loop_freq = 100;     // hz

int main()
{
  LOG_INF("Starting Sine Wave Tier 2 App");
  if (int net_err = zenbedded::common::init_network(); net_err != 0)
  {
    LOG_ERR("Failed to initialize network interface: %d", net_err);
    return net_err;
  }

  static SineWaveClient client;

  int ret = -1;

#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
  if (zenbedded::common::is_usbd_ready())
  {
    char usb_loc[32];
    if (
      zenbedded::common::get_usb_peer_locator(
        ZENBEDDED_DEFAULT_LOCATOR, usb_loc, sizeof(usb_loc)) == 0)
    {
      LOG_INF("Attempting Zenoh connection over USB CDC ECM (%s)...", usb_loc);
      ret = client.init(loop_freq, usb_loc);
      if (ret != 0)
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
    ret = client.init(loop_freq, ZENBEDDED_DEFAULT_LOCATOR);
  }
#endif

  if (ret != 0)
  {
    ret = client.init(loop_freq, ZENBEDDED_DEFAULT_LOCATOR);
  }

  if (ret != 0)
  {
    LOG_ERR("ZenbeddedClient.init failed with %d, aborting", ret);
    return ret;
  }

  int64_t start_time = k_uptime_get();  // Get kernel boot time in ms
  int32_t cnt = 0;
  bool cmd_recv = false;
  zenbedded_state_t state{};
  zenbedded_command_t cmd{};

  while (true)
  {
    client.read_command(cmd);

    if (fabs(cmd.sine_wave_amplitude) >= 1e-6)
    {
      cmd_recv = true;
    }
    if (cmd_recv)
    {
      wave_amp = fabs(cmd.sine_wave_amplitude);
    }

    auto current_time_ms = static_cast<double>(k_uptime_get() - start_time);
    double x = 2.0 * M_PI * current_time_ms * wave_frequency / 1000.0;
    state.sine_wave_position = wave_amp * sin(x);

    if (++cnt == 500)
    {
      LOG_INF("Running... pos: %.3f", state.sine_wave_position);
      cnt = 0;
    }

    client.write_state(state);

    k_sleep(K_MSEC(20));
  }

  client.destroy();
  return 0;
}
