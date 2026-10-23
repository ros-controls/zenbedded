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
#include <zephyr/sys/printk.h>

#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_rcl/node.hpp"
#include "zenbedded_rcl/publisher.hpp"
#include "zenbedded_rcl/subscriber.hpp"

#if defined(CONFIG_ARCH_POSIX)
extern "C" void posix_exit(int exit_code);
#endif

static volatile uint32_t g_arm_cmd_calls = 0;
static volatile uint32_t g_gripper_cmd_calls = 0;

static void on_arm_cmd_received(const JointCommandCodec::Value & cmd, void * user_data)
{
  ARG_UNUSED(user_data);
  g_arm_cmd_calls++;
  printk(
    "[CB 1] Tier 1 Arm Cmd: stamp=%d.%09u val0=%.2f\n", cmd.stamp_sec, cmd.stamp_nanosec,
    cmd.values[0]);
}

static void on_gripper_cmd_received(const JointCommandCodec::Value & cmd, void * user_data)
{
  ARG_UNUSED(user_data);
  g_gripper_cmd_calls++;
  printk(
    "[CB 2] Tier 1 Gripper Cmd: stamp=%d.%09u val0=%.2f\n", cmd.stamp_sec, cmd.stamp_nanosec,
    cmd.values[0]);
}

int main()
{
  printk("\n=========================================\n");
  printk("  Zenbedded Multi Pub/Sub E2E Validation \n");
  printk("       (Tier 1 CDR: 2 Pub, 2 Sub)        \n");
  printk("=========================================\n");

  ZenbeddedNode node("tier1_multi_node");
  if (node.init() != 0)
  {
    printk("[FATAL] Failed to initialize ZenbeddedNode\n");
    return -1;
  }
  printk("[SYS] ZenbeddedNode initialized successfully\n");

  const char * arm_joints[] = {"shoulder", "elbow"};
  JointStateCodec::InitParams arm_state_params{
    .frame_id = "arm_base", .joint_names = arm_joints, .num_joints = 2};
  JointCommandCodec::InitParams arm_cmd_params{
    .frame_id = "arm_base",
    .joint_names = arm_joints,
    .num_joints = 2,
    .interface_name = "position"};

  const char * gripper_joints[] = {"finger"};
  JointStateCodec::InitParams gripper_state_params{
    .frame_id = "gripper_base", .joint_names = gripper_joints, .num_joints = 1};
  JointCommandCodec::InitParams gripper_cmd_params{
    .frame_id = "gripper_base",
    .joint_names = gripper_joints,
    .num_joints = 1,
    .interface_name = "effort"};

  // Declare 2 Tier 1 Publishers
  ZenbeddedPublisher<JointStateCodec> arm_pub;
  if (node.create_publisher(arm_pub, "robot/arm/joint_states", arm_state_params) != 0)
  {
    printk("[FATAL] Failed to create arm_pub\n");
    return -1;
  }
  printk("[SYS] Created Tier 1 Publisher 1 on 'robot/arm/joint_states'\n");

  ZenbeddedPublisher<JointStateCodec> gripper_pub;
  if (node.create_publisher(gripper_pub, "robot/gripper/joint_states", gripper_state_params) != 0)
  {
    printk("[FATAL] Failed to create gripper_pub\n");
    return -1;
  }
  printk("[SYS] Created Tier 1 Publisher 2 on 'robot/gripper/joint_states'\n");

  // Declare 2 Tier 1 Subscribers
  ZenbeddedSubscriber<JointCommandCodec> arm_sub;
  if (
    node.create_subscriber(
      arm_sub, "robot/arm/joint_cmd", on_arm_cmd_received, nullptr, arm_cmd_params) != 0)
  {
    printk("[FATAL] Failed to create arm_sub\n");
    return -1;
  }
  printk("[SYS] Created Tier 1 Subscriber 1 on 'robot/arm/joint_cmd'\n");

  ZenbeddedSubscriber<JointCommandCodec> gripper_sub;
  if (
    node.create_subscriber(
      gripper_sub, "robot/gripper/joint_cmd", on_gripper_cmd_received, nullptr,
      gripper_cmd_params) != 0)
  {
    printk("[FATAL] Failed to create gripper_sub\n");
    return -1;
  }
  printk("[SYS] Created Tier 1 Subscriber 2 on 'robot/gripper/joint_cmd'\n");

  // Data value containers
  double arm_pos[2] = {0.0, 0.0};
  double arm_vel[2] = {0.1, 0.1};
  JointStateCodec::Value arm_val{
    .stamp_sec = 1,
    .stamp_nanosec = 0,
    .positions = arm_pos,
    .velocities = arm_vel,
    .efforts = nullptr};

  double grip_pos[1] = {0.0};
  JointStateCodec::Value grip_val{
    .stamp_sec = 1,
    .stamp_nanosec = 0,
    .positions = grip_pos,
    .velocities = nullptr,
    .efforts = nullptr};

  printk("\n[SYS] Entering Multi-Channel Event Loop\n\n");

  uint32_t loop_count = 0;
  while (1)
  {
    arm_pos[0] += 0.05;
    arm_pos[1] += 0.10;
    grip_pos[0] += 0.01;

    arm_pub.publish(arm_val);
    gripper_pub.publish(grip_val);

    loop_count++;
    if (loop_count % 20 == 0)
    {
      printk(
        "[PUB] Arm=[%.2f, %.2f] | Gripper=[%.2f] | Calls: arm=%u grip=%u\n", arm_pos[0], arm_pos[1],
        grip_pos[0], g_arm_cmd_calls, g_gripper_cmd_calls);
    }

    k_sleep(K_MSEC(50));
  }

  return 0;
}
