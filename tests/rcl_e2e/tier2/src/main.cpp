// Copyright 2026 Open Source Robotics Foundation, Inc.
// Licensed under the Apache License, Version 2.0

#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_rcl/zenbedded_client.hpp"
#include "zenbedded_transport/generated/interface_data.h"

int main()
{
  printk("--- Zenbedded RCL Tier 2 Single-Threaded Node ---\n");

  // Instantiate the client using RawCodec for Tier 2 POD passthrough
  ZenbeddedClient<RawCodec<zenbedded_state_t>, RawCodec<zenbedded_command_t>> client;

  // Initialize client at 100Hz (spawns the background publish thread natively handled by the client
  // base)
  if (client.init(100) != 0)
  {
    printk("Failed to initialize ZenbeddedClient\n");
    return -1;
  }

  zenbedded_state_t current_state = {0.0, 0.0};
  zenbedded_command_t latest_cmd = {0.0};

  uint32_t loop_counter = 0;
  printk("[SYS] Entering Main Event Loop\n");

  while (1)
  {
    // Poll incoming commands non-blockingly from the client buffer
    zenbedded_command_t cmd;
    if (client.read_command(cmd))
    {
      latest_cmd = cmd;
    }

    // Perform state updates sequentially in the main thread
    current_state.test_motor_position += 0.01;
    current_state.test_motor_velocity = latest_cmd.test_motor_effort * 0.5;

    // Commit state write to the client double-buffer for transmission
    client.write_state(current_state);

    loop_counter++;
    if (loop_counter % 1000 == 0)
    {
      printk(
        "Processed 1000 Loops | Pos: %.2f | Last Eff: %.2f\n", current_state.test_motor_position,
        latest_cmd.test_motor_effort);
    }

    // Rate-limit main loop execution to 100Hz (10ms)
    k_sleep(K_MSEC(10));
  }

  return 0;
}
