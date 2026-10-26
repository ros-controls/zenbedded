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
 *
 * @return 0 on success, negative errno on failure or timeout.
 */
int init_network();

}  // namespace zenbedded::common

#endif  // ZENBEDDED_COMMON__NETWORK_HPP_
