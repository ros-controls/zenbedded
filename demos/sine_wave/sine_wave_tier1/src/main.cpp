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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <cmath>
#include <cstdlib>
#include <zenbedded_common/network.hpp>
#include <zenbedded_rcl/codecs.hpp>
#include <zenbedded_rcl/zenbedded_client.hpp>

LOG_MODULE_REGISTER(sine_wave_tier1, LOG_LEVEL_INF);

using SineWaveClient = ZenbeddedClient<JointStateCodec, JointCommandCodec>;

double wave_amp = 5.0;
double wave_frequency = 1.4;  // hz
uint32_t loop_freq = 100;     // hz

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

  int ret = client.init(loop_freq, state_params, cmd_params);
  if (ret != 0)
  {
    LOG_ERR("ZenbeddedClient.init failed with %d, aborting", ret);
    return ret;
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

    state.stamp_sec = static_cast<int32_t>(elapsed_ms / 1000);
    state.stamp_nanosec = static_cast<uint32_t>((elapsed_ms % 1000) * 1000000ULL);

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
