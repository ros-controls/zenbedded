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
static volatile double g_latest_effort = 0.0;
static volatile int32_t g_latest_mode = 0;
static volatile int32_t g_latest_flags = 0;

static void on_motor_cmd_received(const MotorCommand & cmd, void * user_data)
{
  ARG_UNUSED(user_data);
  g_motor_cmd_calls++;
  g_latest_effort = cmd.effort;
  printk("[CB 1] Motor Command received: effort=%.2f\n", cmd.effort);
}

static void on_mode_cmd_received(const ModeCommand & cmd, void * user_data)
{
  ARG_UNUSED(user_data);
  g_mode_cmd_calls++;
  g_latest_mode = cmd.mode_id;
  g_latest_flags = cmd.flags;
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

  // Create two distinct subscribers with callbacks on different topics
  ZenbeddedSubscriber<RawCodec<MotorCommand> > motor_sub;
  if (node.create_subscriber(motor_sub, "robot/motor/cmd", on_motor_cmd_received, nullptr) != 0)
  {
    printk("[FATAL] Failed to create motor_sub\n");
    return -1;
  }
  printk("[SYS] Created Subscriber 1 on 'robot/motor/cmd'\n");

  ZenbeddedSubscriber<RawCodec<ModeCommand> > mode_sub;
  if (node.create_subscriber(mode_sub, "robot/mode/cmd", on_mode_cmd_received, nullptr) != 0)
  {
    printk("[FATAL] Failed to create mode_sub\n");
    return -1;
  }
  printk("[SYS] Created Subscriber 2 on 'robot/mode/cmd'\n");

  // Periodic samples
  MotorState st{0.0, 0.0};
  TelemetryState telem{12.4f, 25.5f};

  printk("\n[SYS] Entering Multi-Channel Event Loop\n\n");

  uint32_t loop_count = 0;
  while (1)
  {
    st.position += 0.1;
    st.velocity = g_latest_effort * 0.5;
    telem.temperature += 0.2f;

    motor_pub.publish(st);
    telem_pub.publish(telem);

    loop_count++;
    if (loop_count % 20 == 0)
    {
      printk(
        "[PUB] Motor pos=%.2f vel=%.2f | Telem temp=%.1fC | CB Motor=%u Mode=%u\n", st.position,
        st.velocity, static_cast<double>(telem.temperature), g_motor_cmd_calls, g_mode_cmd_calls);
    }

    k_sleep(K_MSEC(50));
  }

  return 0;
}
