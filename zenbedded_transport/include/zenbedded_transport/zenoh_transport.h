// Copyright 2026 Open Source Robotics Foundation, Inc.
// Licensed under the Apache License, Version 2.0

#ifndef ZENBEDDED_TRANSPORT__ZENOH_TRANSPORT_H_
#define ZENBEDDED_TRANSPORT__ZENOH_TRANSPORT_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Initializes the Zenbedded transport engine.
   *
   * @param domain_id ROS 2 Domain ID (Tier 1). Ignored in Tier 2.
   * @param node_name ROS 2 Node Name (Tier 1). Ignored in Tier 2.
   * @param mode "client" or "peer".
   * @param locator Network endpoint (IP:PORT, e.g., "127.0.0.1:7447"). The transport layer
   * automatically prepends the protocol.
   * @return 0 on success, -1 on failure.
   */
  int zenbedded_transport_init(
    uint32_t domain_id, const char * node_name, const char * mode, const char * locator);

  /**
   * @brief Safely tears down the Zenoh session and un-declares Liveliness.
   */
  void zenbedded_transport_destroy(void);

  /**
   @deprecated Network I/O is now handled autonomously by Zenoh-Pico background tasks
   */
  void zenbedded_transport_spin(void);

  typedef struct zenbedded_pub_s * zenbedded_pub_t;
  typedef struct zenbedded_sub_s * zenbedded_sub_t;

  /**
   * @brief Declares a Publisher on the graph.
   *
   * @param topic_name The exact topic name (e.g., "joint_states" or "arm/state").
   * @param type_name DDS type namespace (Tier 1). Pass NULL for Tier 2 custom payloads.
   * @return Opaque handle, or NULL on failure.
   */
  zenbedded_pub_t zenbedded_transport_declare_publisher(
    const char * topic_name, const char * type_name);

  /**
   * @brief Pushes a payload over the Zenoh transport.
   *
   * @param pub The publisher handle.
   * @param payload Pointer to the CDR buffer (Tier 1) or raw struct memory (Tier 2).
   * @param payload_size Length of the payload in bytes.
   * @return 0 on success, negative error code on failure.
   */
  int zenbedded_transport_publish(
    zenbedded_pub_t pub, const uint8_t * payload, size_t payload_size);

  /**
   * @brief Callback signature for inbound subscriber messages.
   */
  typedef void (*zenbedded_recv_cb_t)(const uint8_t * payload, size_t size, void * user_data);

  /**
   * @brief Declares a Subscriber on the graph.
   *
   * @param topic_name The exact topic name (e.g., "joint_commands" or "arm/cmd").
   * @param type_name DDS type namespace (Tier 1). Pass NULL for Tier 2 custom payloads.
   * @param callback Function invoked when a network payload is received.
   * @param user_data Arbitrary data passed back to the callback context.
   * @return Opaque handle, or NULL on failure.
   */
  zenbedded_sub_t zenbedded_transport_declare_subscriber(
    const char * topic_name, const char * type_name, zenbedded_recv_cb_t callback,
    void * user_data);

  /**
   * @brief Undeclares a Publisher on the graph and releases its pool slot.
   *
   * @param pub The publisher handle to undeclare.
   * @return 0 on success, negative error code on failure.
   */
  int zenbedded_transport_undeclare_publisher(zenbedded_pub_t pub);

  /**
   * @brief Undeclares a Subscriber on the graph and releases its pool slot.
   *
   * @param sub The subscriber handle to undeclare.
   * @return 0 on success, negative error code on failure.
   */
  int zenbedded_transport_undeclare_subscriber(zenbedded_sub_t sub);

#ifdef __cplusplus
}
#endif

#endif  // ZENBEDDED_TRANSPORT__ZENOH_TRANSPORT_H_
