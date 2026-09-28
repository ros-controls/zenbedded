// Copyright 2026 Open Source Robotics Foundation, Inc.
// Licensed under the Apache License, Version 2.0

#include <stdint.h>
#include <stdio.h>
#include <zenoh-pico/config.h>
#include <zephyr/kernel.h>

#include "zenbedded_transport/serialization.h"
#include "zenbedded_transport/zenoh_transport.h"

#ifdef CONFIG_TIER1_TEST_INGRESS
// 1. Define global context so the callback knows how to parse the incoming bytes
zcdr_joint_command_ctx_t ctx_sub_1;

// 2. The independent subscriber callback
void cmd_1_rx_callback(const uint8_t * payload, size_t size, void * user_data)
{
  zcdr_joint_command_ctx_t * ctx = reinterpret_cast<zcdr_joint_command_ctx_t *>(user_data);
  double vals[2] = {0};

  static uint32_t rx_count = 0;
  static uint32_t last_hz_time = 0;

  rx_count++;
  uint32_t current_time = k_uptime_get_32();

  if (last_hz_time == 0)
  {
    last_hz_time = current_time;
  }

  bool success = zcdr_deserialize_joint_command(ctx, payload, size, NULL, NULL, vals);

  if ((current_time - last_hz_time) >= 1000)
  {
    if (success)
    {
      printf("[SUB 1] Rate: %u Hz | stepper: %.2f | pendulum: %.2f\n", rx_count, vals[0], vals[1]);
    }
    else
    {
      printf("[SUB 1] Rate: %u Hz | ERROR: Deserialization failed (size: %zu)\n", rx_count, size);
    }
    rx_count = 0;
    last_hz_time = current_time;
  }
}
#endif

int main(void)
{
  printf("\n=======================================\n");
  printf("  Zenbedded Transport: RCL Reference   \n");
  printf("  HIGH-SPEED 1-PUB/1-SUB EXECUTOR      \n");
  printf("=======================================\n");

#ifdef CONFIG_ZENBEDDED_TIER_1

  // Pass Kconfig macros directly into the transport API
  printf("[SYS] zenoh transport lease = %d ms\n", static_cast<int>(Z_TRANSPORT_LEASE));

  if (
    zenbedded_transport_init(
      CONFIG_ZENBEDDED_DOMAIN_ID, CONFIG_ZENBEDDED_NODE_NAME, CONFIG_ZENBEDDED_ZENOH_MODE,
      CONFIG_ZENBEDDED_ZENOH_IP_PORT) != 0)
  {
    printf("[FATAL] Transport init failed!\n");
    return -1;
  }

  // --- MEMORY & SERIALIZATION SETUP ---
  const char * chassis_joints[] = {"stepper", "pendulum"};

#ifdef CONFIG_TIER1_TEST_EGRESS
  // --- PUBLISHER SETUP ---
  zcdr_joint_state_ctx_t ctx_joints;
  uint8_t buf_joints[512] = {0};
  zcdr_init_joint_state(&ctx_joints, "base_link", chassis_joints, 2, buf_joints);

  zenbedded_pub_t pub_joints =
    zenbedded_transport_declare_publisher("joint_states", "sensor_msgs::msg::dds_::JointState_");
  k_msleep(10);  // Pace graph declarations

  double pos_2[] = {0.0, 0.0};
  double vel_2[] = {0.0, 0.0};
  double eff_2[] = {0.0, 0.0};
#endif

#ifdef CONFIG_TIER1_TEST_INGRESS
  // --- SUBSCRIBER SETUP ---
  uint8_t dummy_1[256] = {0};
  zcdr_init_joint_command(&ctx_sub_1, "base_link", chassis_joints, 2, "position", dummy_1);

  zenbedded_transport_declare_subscriber(
    "joint_commands", "control_msgs::msg::dds_::JointCommand_", cmd_1_rx_callback, &ctx_sub_1);
  k_msleep(10);  // Pace graph declarations
#endif

  // --- TIMING SETUP ---
  uint32_t last_100hz_time = k_uptime_get_32();

  printf("\n[SYS] Entering High-Frequency Event Loop...\n\n");

  while (1)
  {
    uint32_t current_time = k_uptime_get_32();
#ifdef CONFIG_TIER1_TEST_EGRESS
    // 2. PUB 1: 100Hz LOOP (Every 10 ms)
    if ((current_time - last_100hz_time) >= 10)
    {
      zcdr_serialize_joint_state(&ctx_joints, 0, 0, pos_2, vel_2, eff_2, buf_joints);
      zenbedded_transport_publish(pub_joints, buf_joints, ctx_joints.payload_size);
      pos_2[0] += 0.01;
      last_100hz_time = current_time;
    }
#endif
    // Yield CPU for 1ms to prevent starvation on native_sim
    k_sleep(K_MSEC(1));
  }
#else
  printf("\nCONFIG_ZENBEDDED_TIER_1 is disabled. Skipping execution.\n");
#endif

  return 0;
}
