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

#ifndef ZENBEDDED_COMMON__NETWORK_HPP_
#define ZENBEDDED_COMMON__NETWORK_HPP_

#include <zephyr/net/net_if.h>
#include <cstddef>

namespace zenbedded::common
{

/**
 * @brief Initialize and connect Wi-Fi station using stored credentials.
 *
 * @param iface Target interface (or nullptr to auto-discover via net_if_get_wifi_sta()).
 * @return 0 on success, negative errno on failure or timeout.
 */
int init_wifi(net_if * iface = nullptr);

/**
 * @brief Initialize and bring up USB CDC ECM network interface.
 *
 * @param iface Target interface (or nullptr to auto-discover cdc_ecm_eth0).
 * @return 0 on success, negative errno on failure or timeout.
 */
int init_usbd(net_if * iface = nullptr);

/**
 * @brief Initialize all enabled network interfaces (USB CDC ECM and/or Wi-Fi).
 *
 * Supports bringing up both interfaces simultaneously when both are enabled.
 * In dual-transport mode, succeeds as long as at least one interface is operational.
 *
 * @return 0 on success, negative errno on failure or timeout.
 */
int init_network();

/**
 * @brief Checks whether the USB CDC ECM network interface has carrier and an assigned IPv4 address.
 *
 * @return true if USB CDC network interface is up and has an active IPv4 address.
 */
bool is_usbd_ready();

/**
 * @brief Checks whether the Wi-Fi interface is associated with an AP and has an assigned IPv4
 * address.
 *
 * @return true if Wi-Fi interface is up and has an active IPv4 address.
 */
bool is_wifi_ready();

/**
 * @brief Extracts the port from an arbitrary target locator and constructs a USB peer locator.
 *
 * For example, "10.69.75.128:10001" -> "192.0.2.1:10001".
 * If no port is specified (e.g. "10.69.75.128"), defaults to port 7447.
 *
 * @param target_locator Input locator (e.g. CONFIG_ZENBEDDED_ZENOH_LOCATOR).
 * @param out_usb_locator Buffer to receive the formatted USB peer locator.
 * @param out_len Maximum capacity of out_usb_locator buffer.
 * @return 0 on success, -EINVAL on invalid pointers, -ENOSPC if buffer is too small.
 */
int get_usb_peer_locator(const char * target_locator, char * out_usb_locator, size_t out_len);

}  // namespace zenbedded::common

#endif  // ZENBEDDED_COMMON__NETWORK_HPP_
