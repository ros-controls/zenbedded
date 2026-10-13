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
#include "zenbedded_rcl/subscription.hpp"

#if defined(CONFIG_ZENBEDDED_TIER_1)

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

  ZenbeddedNode node("tier1_multi_node", 0);
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
  ZenbeddedSubscription<JointCommandCodec> arm_sub;
  if (
    node.create_subscription(
      arm_sub, "robot/arm/joint_cmd", on_arm_cmd_received, nullptr, arm_cmd_params) != 0)
  {
    printk("[FATAL] Failed to create arm_sub\n");
    return -1;
  }
  printk("[SYS] Created Tier 1 Subscription 1 on 'robot/arm/joint_cmd'\n");

  ZenbeddedSubscription<JointCommandCodec> gripper_sub;
  if (
    node.create_subscription(
      gripper_sub, "robot/gripper/joint_cmd", on_gripper_cmd_received, nullptr,
      gripper_cmd_params) != 0)
  {
    printk("[FATAL] Failed to create gripper_sub\n");
    return -1;
  }
  printk("[SYS] Created Tier 1 Subscription 2 on 'robot/gripper/joint_cmd'\n");

  // Stream samples on both publishers
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

  for (int i = 0; i < 5; i++)
  {
    arm_pos[0] += 0.05;
    arm_pos[1] += 0.10;
    grip_pos[0] += 0.01;

    arm_pub.publish(arm_val);
    gripper_pub.publish(grip_val);
    printk(
      "[PUB ITER %d] Arm=[%.2f, %.2f] | Gripper=[%.2f]\n", i + 1, arm_pos[0], arm_pos[1],
      grip_pos[0]);

    k_sleep(K_MSEC(50));
  }

  // Undeclare one publisher and one subscriber
  printk("[SYS] Undeclaring gripper_pub (Pub 2) and gripper_sub (Sub 2)...\n");
  gripper_pub.destroy();
  gripper_sub.destroy();

  arm_pos[0] += 0.5;
  arm_pub.publish(arm_val);
  printk("[SYS] Published on arm_pub after gripper handles destroyed: pos0=%.2f\n", arm_pos[0]);

  // Teardown remaining handles
  arm_pub.destroy();
  arm_sub.destroy();
  node.destroy();

  printk("[PASS] Tier 1 Multi Pub/Sub test completed successfully!\n");
  return 0;
}

#elif defined(CONFIG_ZENBEDDED_TIER_2)

struct __attribute__((packed)) TelemetryState
{
  float voltage;
  float temperature;
};

struct __attribute__((packed)) MotorState
{
  double position;
  double velocity;
};

struct __attribute__((packed)) MotorCommand
{
  double effort;
};

struct __attribute__((packed)) ModeCommand
{
  int32_t mode_id;
  int32_t flags;
};

static volatile uint32_t g_motor_cmd_calls = 0;
static volatile uint32_t g_mode_cmd_calls = 0;

static void on_motor_cmd_received(const MotorCommand & cmd, void * user_data)
{
  ARG_UNUSED(user_data);
  g_motor_cmd_calls++;
  printk("[CB 1] Motor Command received: effort=%.2f\n", cmd.effort);
}

static void on_mode_cmd_received(const ModeCommand & cmd, void * user_data)
{
  ARG_UNUSED(user_data);
  g_mode_cmd_calls++;
  printk("[CB 2] Mode Command received: mode=%d flags=0x%x\n", cmd.mode_id, cmd.flags);
}

int main()
{
  printk("\n=========================================\n");
  printk("  Zenbedded Multi Pub/Sub E2E Validation \n");
  printk("       (Tier 2 Raw: 2 Pub, 2 Sub)        \n");
  printk("=========================================\n");

  ZenbeddedNode node("tier2_multi_node", 0);
  if (node.init() != 0)
  {
    printk("[FATAL] Failed to initialize ZenbeddedNode\n");
    return -1;
  }
  printk("[SYS] ZenbeddedNode initialized successfully\n");

  // Create two distinct publishers on different topics
  ZenbeddedPublisher<RawCodec<MotorState> > motor_pub;
  if (node.create_publisher(motor_pub, "robot/motor/state") != 0)
  {
    printk("[FATAL] Failed to create motor_pub\n");
    return -1;
  }
  printk("[SYS] Created Publisher 1 on 'robot/motor/state'\n");

  ZenbeddedPublisher<RawCodec<TelemetryState> > telem_pub;
  int telem_ret = node.create_publisher(telem_pub, "robot/telemetry");
  if (telem_ret != 0)
  {
    printk("[FATAL] Failed to create telem_pub: ret=%d\n", telem_ret);
    return -1;
  }
  printk("[SYS] Created Publisher 2 on 'robot/telemetry'\n");

  // Create two distinct subscriptions with callbacks on different topics
  ZenbeddedSubscription<RawCodec<MotorCommand> > motor_sub;
  if (node.create_subscription(motor_sub, "robot/motor/cmd", on_motor_cmd_received, nullptr) != 0)
  {
    printk("[FATAL] Failed to create motor_sub\n");
    return -1;
  }
  printk("[SYS] Created Subscription 1 on 'robot/motor/cmd'\n");

  ZenbeddedSubscription<RawCodec<ModeCommand> > mode_sub;
  if (node.create_subscription(mode_sub, "robot/mode/cmd", on_mode_cmd_received, nullptr) != 0)
  {
    printk("[FATAL] Failed to create mode_sub\n");
    return -1;
  }
  printk("[SYS] Created Subscription 2 on 'robot/mode/cmd'\n");

  // Publish periodic samples on both publishers
  MotorState st{0.0, 0.0};
  TelemetryState telem{12.4f, 25.5f};

  for (int i = 0; i < 5; i++)
  {
    st.position += 0.1;
    st.velocity = 0.5;
    telem.temperature += 0.2f;

    motor_pub.publish(st);
    telem_pub.publish(telem);
    printk(
      "[PUB ITER %d] Motor pos=%.2f | Telem temp=%.1fC\n", i + 1, st.position,
      static_cast<double>(telem.temperature));

    k_sleep(K_MSEC(50));
  }

  // Undeclare one publisher and one subscriber, verifying the remaining stay operational
  printk("[SYS] Undeclaring telem_pub (Pub 2) and mode_sub (Sub 2)...\n");
  telem_pub.destroy();
  mode_sub.destroy();

  st.position += 0.5;
  motor_pub.publish(st);
  printk("[SYS] Published on motor_pub after telem_pub destroyed: pos=%.2f\n", st.position);

  // Tear down all remaining handles and node
  motor_pub.destroy();
  motor_sub.destroy();
  node.destroy();

  printk("[PASS] Tier 2 Multi Pub/Sub test completed successfully!\n");
  return 0;
}

#endif
