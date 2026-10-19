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

#pragma once

#include <zephyr/net/net_if.h>

namespace zenbedded::demo
{

/**
 * @brief Initialize network interface (USB CDC ECM or Wi-Fi station) based on Kconfig.
 *
 * Handles USB CDC device enablement, static IPv4 address assignment, and DHCPv4 server startup;
 * or Wi-Fi station connection using stored credentials with timeout and power management.
 *
 * @param iface Target interface (or nullptr to use net_if_get_default()).
 * @return 0 on success, negative errno on failure or timeout.
 */
int init_network(net_if * iface = nullptr);

}  // namespace zenbedded::demo
