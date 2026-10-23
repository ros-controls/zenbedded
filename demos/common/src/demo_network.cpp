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

#include <errno.h>

#include "zenbedded_demo_net.hpp"

#if defined(CONFIG_ZENBEDDED_DEMO_COMM_WIFI) || defined(CONFIG_INVERTED_PENDULUM_COMM_WIFI) || \
  defined(CONFIG_SINE_WAVE_COMM_WIFI)
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_wifi.h>
#endif
#include <zephyr/net/wifi_mgmt.h>
#elif defined(CONFIG_ZENBEDDED_DEMO_COMM_USB_CDC) || \
  defined(CONFIG_INVERTED_PENDULUM_COMM_USB_CDC) || defined(CONFIG_SINE_WAVE_COMM_USB_CDC)
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/usb/usbd.h>
extern "C"
{
#include <sample_usbd.h>
}
#endif
#if defined(CONFIG_NET_CONFIG_SETTINGS)
#include <zephyr/net/net_config.h>
#endif
#if defined(CONFIG_NET_DHCPV4)
#include <zephyr/net/dhcpv4.h>
#endif
#if defined(CONFIG_NET_DHCPV4_SERVER)
#include <zephyr/net/dhcpv4_server.h>
#endif
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zenbedded_demo_net, LOG_LEVEL_INF);

namespace zenbedded::demo
{

constexpr uint32_t kWifiConnectTimeout = 15000;  // ms
constexpr uint32_t kIpv4AcquireTimeout = 30000;  // ms

#if defined(CONFIG_ZENBEDDED_DEMO_USB_CDC_PEER_IPV4_ADDR)
#define PEER_IPV4_ADDR CONFIG_ZENBEDDED_DEMO_USB_CDC_PEER_IPV4_ADDR
#elif defined(CONFIG_INVERTED_PENDULUM_USB_CDC_PEER_IPV4_ADDR)
#define PEER_IPV4_ADDR CONFIG_INVERTED_PENDULUM_USB_CDC_PEER_IPV4_ADDR
#elif defined(CONFIG_SINE_WAVE_USB_CDC_PEER_IPV4_ADDR)
#define PEER_IPV4_ADDR CONFIG_SINE_WAVE_USB_CDC_PEER_IPV4_ADDR
#else
#define PEER_IPV4_ADDR "192.0.2.1"
#endif

int init_network(net_if * iface)
{
  if (iface == nullptr)
  {
    iface = net_if_get_default();
  }

#if defined(CONFIG_ZENBEDDED_DEMO_COMM_WIFI) || defined(CONFIG_INVERTED_PENDULUM_COMM_WIFI) || \
  defined(CONFIG_SINE_WAVE_COMM_WIFI)
  LOG_INF("Connecting to WiFi using stored credentials...");
  uint32_t timer = k_uptime_get_32();
  while (net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, nullptr, 0) != 0)
  {
    if (k_uptime_get_32() - timer > kWifiConnectTimeout)
    {
      LOG_ERR("WiFi connection timed out");
      return -ETIMEDOUT;
    }
    LOG_DBG("WiFi connect-stored request failed... retrying...");
    k_sleep(K_MSEC(500));
  }

  LOG_INF("Connected... Waiting for IPv4 address");
  timer = k_uptime_get_32();
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    if (k_uptime_get_32() - timer > kIpv4AcquireTimeout)
    {
      LOG_ERR(
        "No IPv4 address within %u ms; check the stored WiFi credentials", kIpv4AcquireTimeout);
      return -ETIMEDOUT;
    }
    k_sleep(K_MSEC(200));
  }
  LOG_INF("Got IPv4 address via WiFi");

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
  esp_wifi_set_ps(WIFI_PS_NONE);
  k_sleep(K_MSEC(200));
#endif
  return 0;

#elif defined(CONFIG_ZENBEDDED_DEMO_COMM_USB_CDC) || \
  defined(CONFIG_INVERTED_PENDULUM_COMM_USB_CDC) || defined(CONFIG_SINE_WAVE_COMM_USB_CDC)
  LOG_INF("Initializing USB CDC network interface...");
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
  usbd_context * sample_usbd = sample_usbd_init_device(nullptr);
  if (sample_usbd == nullptr)
  {
    LOG_ERR("Failed to initialize USB device context");
    return -ENODEV;
  }

  int usb_err = usbd_enable(sample_usbd);
  if (usb_err != 0)
  {
    LOG_ERR("Failed to enable USB device: %d", usb_err);
    return usb_err;
  }
  LOG_INF("USB device initialized and enabled");

#if defined(CONFIG_NET_CONFIG_SETTINGS)
  (void)net_config_init_app(nullptr, "Initializing USB network");
#elif defined(CONFIG_NET_DHCPV4)
  net_dhcpv4_start(iface);
#endif
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

  LOG_INF("Waiting for IPv4 address on USB CDC interface...");
  uint32_t timer = k_uptime_get_32();
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    if (k_uptime_get_32() - timer > kIpv4AcquireTimeout)
    {
      LOG_ERR("No IPv4 address on USB CDC interface within %u ms", kIpv4AcquireTimeout);
      return -ETIMEDOUT;
    }
    k_sleep(K_MSEC(200));
  }
  LOG_INF("Got IPv4 address on USB CDC interface");

#if defined(CONFIG_NET_DHCPV4_SERVER)
  in_addr pool_addr;
  if (net_addr_pton(AF_INET, PEER_IPV4_ADDR, &pool_addr) == 0)
  {
    int srv_err = net_dhcpv4_server_start(iface, &pool_addr);
    if (srv_err != 0)
    {
      LOG_ERR("Failed to start DHCPv4 server: %d", srv_err);
    }
    else
    {
      LOG_INF("DHCPv4 server started leasing %s to host", PEER_IPV4_ADDR);
    }
  }

  LOG_INF("Waiting for host to acquire DHCP lease (or timeout)...");
  uint32_t dhcp_timer = k_uptime_get_32();
  while (k_uptime_get_32() - dhcp_timer < 5000)
  {
    bool lease_active = false;
    net_dhcpv4_server_foreach_lease(
      iface,
      [](struct net_if *, struct dhcpv4_addr_slot * slot, void * user_data)
      {
        if (slot && slot->state == DHCPV4_SERVER_ADDR_ALLOCATED)
        {
          *static_cast<bool *>(user_data) = true;
        }
      },
      &lease_active);
    if (lease_active)
    {
      LOG_INF("Host acquired DHCP lease on USB CDC interface!");
      break;
    }
    k_sleep(K_MSEC(250));
  }
#endif

  return 0;
#else
  LOG_WRN("No network transport enabled in Kconfig");
  return 0;
#endif
}

}  // namespace zenbedded::demo
