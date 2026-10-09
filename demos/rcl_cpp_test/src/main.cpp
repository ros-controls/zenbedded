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

#include <esp_wifi.h>
#include <zenbedded_transport/generated/interface_data.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <cmath>
#include <zenbedded/rcl/zenbedded_client.hpp>

LOG_MODULE_REGISTER(zenbedded_test_node, LOG_LEVEL_INF);

using TestClient = ZenbeddedClient<RawCodec<zenbedded_state_t>, RawCodec<zenbedded_command_t>>;

constexpr uint32_t kControlFreqHz = 50;
constexpr int kIterations = 200;            // ~4s at 50Hz
constexpr int kWifiConnectTimeout = 15000;  // ms

static void report(bool pass, const char * msg)
{
  if (pass)
  {
    LOG_INF("PASS: %s", msg);
  }
  else
  {
    LOG_ERR("FAIL: %s", msg);
  }
}

int main(void)
{
  LOG_INF("starting test node");

  LOG_INF("starting Sine Wave Zephyr App");
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
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    k_sleep(K_MSEC(200));
  }
  LOG_INF("Got IPV4 address");

  // Disable Wi-Fi power saving
  esp_wifi_set_ps(WIFI_PS_NONE);
  k_sleep(K_MSEC(200));

  static TestClient client;

  int ret = client.init(kControlFreqHz);
  report(ret == 0, "client.init() returned 0");
  if (ret != 0)
  {
    LOG_ERR("init failed with %d, aborting", ret);
    return ret;
  }
  report(client.is_initialized(), "client.is_initialized() is true after init()");

  bool cmd_val_changed = false;
  int write_calls = 0;

  zenbedded_state_t s{};
  zenbedded_command_t cmd{};

  for (int i = 0; i < kIterations; i++)
  {
    double di = static_cast<double>(i);
    s.motor_arm_position = sin(di * 0.1) * 90.0;
    s.pendulum_axis_position = di * 0.01;

    client.write_state(s);
    write_calls++;

    client.read_command(cmd);

    if (i % 20 == 0)
    {
      LOG_INF(
        "iter=%d state.motor_arm=%.2f "
        "state.pendulum=%.2f cmd.motor_arm=%.2f",
        i, s.motor_arm_position, s.pendulum_axis_position, cmd.motor_arm_position);
    }

    // check if the value changes
    if (fabs(cmd.motor_arm_position) >= 1e-9)
    {
      cmd_val_changed = true;
    }

    k_sleep(K_MSEC(1000 / kControlFreqHz));
  }

  report(write_calls == kIterations, "write_state() called for every iteration without crashing");

  report(
    cmd_val_changed,
    "received at least one non-zero command "
    "(requires tools/zenoh_echo_node.py running)");

  LOG_INF("is_control_thread_running=%d", client.is_control_thread_running());

  // --- destroy() ---
  client.destroy();
  report(!client.is_initialized(), "client.destroy() left client uninitialized");

  LOG_INF("done");
  return 0;
}
