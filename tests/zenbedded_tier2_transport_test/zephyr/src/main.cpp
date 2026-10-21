// Copyright 2026 Zenbedded
// Licensed under the Apache License, Version 2.0

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zenbedded_transport/generated/interface_data.h"
#include "zenbedded_transport/zenoh_transport.h"

LOG_MODULE_REGISTER(tier2_test, LOG_LEVEL_INF);

// --- GLOBAL HANDLES ---
static zenbedded_pub_t g_state_pub = NULL;
static zenbedded_sub_t g_cmd_sub = NULL;

// --- LOCK-FREE BUFFERING ---
static zenbedded_command_t g_latest_cmd = {0};
static zenbedded_state_t g_current_state = {0};

// --- RTOS SCHEDULING ---
K_SEM_DEFINE(g_control_sem, 0, 1);

// --- CALLBACK: NETWORK -> APP ---
static void on_command_received(const uint8_t * payload, size_t size, void * user_data)
{
  (void)user_data;
  if (size != sizeof(zenbedded_command_t))
  {
    LOG_WRN("Payload size mismatch. Expected %zu, got %zu", sizeof(zenbedded_command_t), size);
    return;
  }

  memcpy(&g_latest_cmd, payload, sizeof(zenbedded_command_t));

  k_sem_give(&g_control_sem);
}
// --- THREAD 2: MOTOR CONTROL LOOP (Event Driven) ---
static void motor_control_thread(void *, void *, void *)
{
  LOG_INF("Motor control thread initialized. Waiting for Host commands...");

  uint32_t loop_counter = 0;

  while (1)
  {
    // Sleep until the network callback gives the semaphore
    k_sem_take(&g_control_sem, K_FOREVER);

    g_current_state.test_motor_position += 0.01;
    g_current_state.test_motor_velocity = g_latest_cmd.test_motor_effort * 0.5;

    if (g_state_pub)
    {
      zenbedded_transport_publish(
        g_state_pub, reinterpret_cast<const uint8_t *>(&g_current_state),
        sizeof(zenbedded_state_t));
    }

    loop_counter++;
    if (loop_counter % 1000 == 0)
    {
      LOG_INF(
        "Processed 1000 Commands | Pos: %.2f | Last Eff: %.2f", g_current_state.test_motor_position,
        g_latest_cmd.test_motor_effort);
    }
  }
}
K_THREAD_DEFINE(ctrl_tid, 4096, motor_control_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, 0);

// --- MAIN SETUP ---
int main(void)
{
  LOG_INF("--- Zenbedded Tier 2 Ping-Pong Node ---");

  if (
    zenbedded_transport_init(
      0, "mcu", CONFIG_ZENBEDDED_ZENOH_MODE, CONFIG_ZENBEDDED_ZENOH_IP_PORT) != 0)
  {
    LOG_ERR("Failed to initialize transport");
    return -1;
  }

  k_sleep(K_MSEC(500));

  g_state_pub = zenbedded_transport_declare_publisher("test_motor/state", NULL);
  if (!g_state_pub)
  {
    LOG_ERR("Failed to declare publisher");
    return -1;
  }

  g_cmd_sub =
    zenbedded_transport_declare_subscriber("test_motor/cmd", NULL, on_command_received, NULL);
  if (!g_cmd_sub)
  {
    LOG_ERR("Failed to declare subscriber");
    return -1;
  }

  LOG_INF("Pipes created. RTOS threads take over from here.");

  while (1)
  {
    k_sleep(K_FOREVER);
  }
  return 0;
}
