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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <zenoh-pico.h>
#include <zephyr/kernel.h>

#define ENDPOINT "tcp/127.0.0.1:7447"
#define INGRESS_TOPIC "zenbedded/e2e/ingress"
#define EGRESS_TOPIC "zenbedded/e2e/egress"

#if defined(CONFIG_ZENOH_E2E_INGRESS)
#define INGRESS_STATE "on"
#else
#define INGRESS_STATE "off"
#endif

#if defined(CONFIG_ZENOH_E2E_EGRESS)
#define EGRESS_STATE "on"
#else
#define EGRESS_STATE "off"
#endif

#if defined(CONFIG_ZENOH_E2E_INGRESS)
static void on_test_message(z_loaned_sample_t * sample, void * arg)
{
  ARG_UNUSED(arg);

  z_owned_string_t payload;
  if (z_bytes_to_string(z_sample_payload(sample), &payload) < 0)
  {
    printf("Received test message: <invalid payload>\n");
    return;
  }

  printf(
    "Received test message: %.*s\n", (int)z_string_len(z_loan(payload)),
    z_string_data(z_loan(payload)));
  z_drop(z_move(payload));
}
#endif

int main(void)
{
  z_owned_config_t config;
  z_owned_session_t session;

  z_config_default(&config);

  zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, "client");
  zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, ENDPOINT);

  printf("Connecting client to %s\n", ENDPOINT);

  if (z_open(&session, z_move(config), NULL) < 0)
  {
    printf("Zenoh connection failed\n");
    return 1;
  }

  if (zp_start_read_task(z_loan_mut(session), NULL) < 0)
  {
    printf("Zenoh read task failed to start\n");
    z_close(z_loan_mut(session), NULL);
    return 1;
  }

  if (zp_start_lease_task(z_loan_mut(session), NULL) < 0)
  {
    printf("Zenoh lease task failed to start\n");
    zp_stop_read_task(z_loan_mut(session));
    z_close(z_loan_mut(session), NULL);
    return 1;
  }

#if defined(CONFIG_ZENOH_E2E_INGRESS)
  z_view_keyexpr_t ingress_keyexpr;
  z_view_keyexpr_from_str_unchecked(&ingress_keyexpr, INGRESS_TOPIC);

  z_owned_closure_sample_t ingress_callback;
  z_closure(&ingress_callback, on_test_message, NULL, NULL);

  z_owned_subscriber_t ingress_subscriber;
  if (
    z_declare_subscriber(
      z_loan(session), &ingress_subscriber, z_loan(ingress_keyexpr), z_move(ingress_callback), NULL) <
    0)
  {
    printf("Zenoh subscriber declaration failed\n");
    zp_stop_lease_task(z_loan_mut(session));
    zp_stop_read_task(z_loan_mut(session));
    z_close(z_loan_mut(session), NULL);
    return 1;
  }
#endif

#if defined(CONFIG_ZENOH_E2E_EGRESS)
  z_view_keyexpr_t egress_keyexpr;
  z_view_keyexpr_from_str_unchecked(&egress_keyexpr, EGRESS_TOPIC);

  z_owned_publisher_t egress_publisher;
  if (z_declare_publisher(z_loan(session), &egress_publisher, z_loan(egress_keyexpr), NULL) < 0)
  {
    printf("Zenoh publisher declaration failed\n");
    zp_stop_lease_task(z_loan_mut(session));
    zp_stop_read_task(z_loan_mut(session));
    z_close(z_loan_mut(session), NULL);
    return 1;
  }
#endif

  printf(
    "Zenoh client connected (ingress=%s, egress=%s, period_ms=%d)\n", INGRESS_STATE, EGRESS_STATE,
    CONFIG_ZENOH_E2E_PERIOD_MS);

#if defined(CONFIG_ZENOH_E2E_EGRESS)
  uint32_t message_sequence = 1;
  bool egress_started = false;
#endif
  for (;;)
  {
#if defined(CONFIG_ZENOH_E2E_EGRESS)
    if (!egress_started)
    {
      z_matching_status_t matching_status;
      if (z_publisher_get_matching_status(z_loan(egress_publisher), &matching_status) < 0)
      {
        printf("Zenoh publisher matching status failed\n");
        return 1;
      }

      if (!matching_status.matching)
      {
        k_sleep(K_MSEC(50));
        continue;
      }

      egress_started = true;
    }

    char message[64];
    int message_length = snprintf(
      message, sizeof(message), "hello from the firmware: %u", (unsigned int)message_sequence++);

    if (message_length > 0 && (size_t)message_length < sizeof(message))
    {
      z_owned_bytes_t payload;
      z_bytes_from_static_buf(&payload, (const uint8_t *)message, (size_t)message_length);

      if (z_publisher_put(z_loan(egress_publisher), z_move(payload), NULL) < 0)
      {
        printf("Zenoh test message publish failed\n");
      }
    }

    k_sleep(K_MSEC(CONFIG_ZENOH_E2E_PERIOD_MS));
#else
    k_sleep(K_FOREVER);
#endif
  }

  return 0;
}
