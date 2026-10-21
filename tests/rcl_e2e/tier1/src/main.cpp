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

#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_rcl/zenbedded_client.hpp"

int main()
{
  printk("\n=======================================\n");
  printk("  Zenbedded RCL tier1 test  \n");
  printk("  HIGH-SPEED 1-PUB/1-SUB EXECUTOR      \n");
  printk("=======================================\n");

#ifdef CONFIG_ZENBEDDED_TIER_1

  ZenbeddedClient<JointStateCodec, JointCommandCodec> client;

  const char * chassis_joints[] = {"stepper", "pendulum"};

  JointStateCodec::InitParams state_params{
    .frame_id = "base_link", .joint_names = chassis_joints, .num_joints = 2};

  JointCommandCodec::InitParams cmd_params{
    .frame_id = "base_link",
    .joint_names = chassis_joints,
    .num_joints = 2,
    .interface_name = "position"};

  // Initialize Client (spawns 100Hz background publish thread automatically)
  if (client.init(100, state_params, cmd_params) != 0)
  {
    printk("[FATAL] RCL init failed!\n");
    return -1;
  }

  // Setup State and Command Value structs
  double pos[2] = {0.0, 0.0};
  double vel[2] = {0.0, 0.0};
  double eff[2] = {0.0, 0.0};

  JointStateCodec::Value state_val{
    .stamp_sec = 0, .stamp_nanosec = 0, .positions = pos, .velocities = vel, .efforts = eff};

  double cmd_vals[2] = {0.0, 0.0};
  JointCommandCodec::Value cmd_val{.stamp_sec = 0, .stamp_nanosec = 0, .values = cmd_vals};

  uint32_t poll_count = 0;
  uint32_t last_hz_time = k_uptime_get_32();

  printk("\n[SYS] Entering High-Frequency Event Loop\n\n");

  while (1)
  {
    uint32_t current_time = k_uptime_get_32();

    // Update state and write to the double-buffer (thread-safe)
    pos[0] += 0.01;
    client.write_state(state_val);

    // Poll the latest command from the double-buffer
    if (client.read_command(cmd_val))
    {
      poll_count++;
    }

    if ((current_time - last_hz_time) >= 1000)
    {
      printk(
        "[SUB 1] Polls/sec: %u | stepper: %.2f | pendulum: %.2f\n", poll_count, cmd_vals[0],
        cmd_vals[1]);
      poll_count = 0;
      last_hz_time = current_time;
    }

    // Pace the main thread's state updates at 100Hz
    k_sleep(K_MSEC(10));
  }
#else
  printk("\nCONFIG_ZENBEDDED_TIER_1 is disabled. Skipping execution.\n");
#endif

  return 0;
}
