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

#include <zenbedded_common/network.hpp>

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#if defined(CONFIG_ZENBEDDED_COMMON_WIFI)
#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
#include <esp_wifi.h>
#endif
#include <zephyr/net/wifi_mgmt.h>
#endif

#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zenbedded_common/usbd.h>
#include <zephyr/usb/usbd.h>
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

#include <cctype>
#include <cstdio>
#include <cstring>

LOG_MODULE_REGISTER(zenbedded_net, LOG_LEVEL_INF);

namespace zenbedded::common
{

constexpr uint32_t kIpv4AcquireTimeout = 30000;  // ms

#if defined(CONFIG_ZENBEDDED_COMMON_WIFI)
int init_wifi(net_if * iface)
{
  if (iface == nullptr)
  {
    iface = net_if_get_wifi_sta();
    if (iface == nullptr)
    {
      iface = net_if_get_default();
    }
  }

  // Allow Wi-Fi radio and L2 stack to fully stabilize after boot
  k_sleep(K_MSEC(500));

  LOG_INF("Connecting to WiFi using stored credentials...");
  int req_ret = net_mgmt(NET_REQUEST_WIFI_CONNECT_STORED, iface, nullptr, 0);
  if (req_ret != 0 && req_ret != -EALREADY)
  {
    LOG_ERR("WiFi connect-stored request failed: %d", req_ret);
    return req_ret;
  }

  LOG_INF("Connection requested, waiting for IPv4 address on WiFi interface...");

#if defined(CONFIG_NET_DHCPV4) && !defined(CONFIG_ESP32_WIFI_STA_AUTO_DHCPV4)
  net_dhcpv4_start(iface);
#endif

  uint32_t timer = k_uptime_get_32();
  while (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) == nullptr)
  {
    if (k_uptime_get_32() - timer > kIpv4AcquireTimeout)
    {
      LOG_ERR(
        "No IPv4 address on WiFi within %u ms; check stored credentials", kIpv4AcquireTimeout);
      return -ETIMEDOUT;
    }
    k_sleep(K_MSEC(200));
  }

#if defined(CONFIG_SOC_FAMILY_ESPRESSIF_ESP32)
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  LOG_INF("Got IPv4 address via WiFi");

  return 0;
}
#else
int init_wifi(net_if * iface)
{
  (void)iface;
  LOG_WRN("Wi-Fi transport not enabled in Kconfig");
  return -ENOTSUP;
}
#endif  // CONFIG_ZENBEDDED_COMMON_WIFI

#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
int init_usbd(net_if * iface)
{
  LOG_INF("Initializing USB CDC network interface...");
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
  usbd_context * usbd_ctx = zenbedded_usbd_init();
  if (usbd_ctx == nullptr)
  {
    LOG_ERR("Failed to initialize USB device context");
    return -ENODEV;
  }

  int usb_err = usbd_enable(usbd_ctx);
  if (usb_err != 0)
  {
    LOG_ERR("Failed to enable USB device: %d", usb_err);
    return usb_err;
  }
  LOG_INF("USB device initialized and enabled");

  if (iface == nullptr)
  {
#if defined(CONFIG_USBD_CDC_ECM_CLASS)
    const struct device * dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(cdc_ecm_eth0));
    if (dev != nullptr && device_is_ready(dev))
    {
      iface = net_if_lookup_by_dev(dev);
    }
#endif
    if (iface == nullptr)
    {
      iface = net_if_get_default();
    }
  }

  // Fast USB cable carrier check: if unplugged, fall back to Wi-Fi quickly
  uint32_t carrier_timer = k_uptime_get_32();
  while (!net_if_is_carrier_ok(iface))
  {
    if (k_uptime_get_32() - carrier_timer > 2000)
    {
      LOG_INF("USB CDC carrier down (cable unplugged), skipping USB transport");
      return -ENOTCONN;
    }
    k_sleep(K_MSEC(100));
  }

#if defined(CONFIG_NET_CONFIG_SETTINGS)
  const struct device * cdc_dev = nullptr;
#if defined(CONFIG_USBD_CDC_ECM_CLASS)
  cdc_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(cdc_ecm_eth0));
#endif
  int cfg_ret = net_config_init_app(cdc_dev, "Initializing USB network");
  if (cfg_ret != 0)
  {
    LOG_WRN("net_config_init_app returned %d for USB CDC", cfg_ret);
  }
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
  if (net_addr_pton(AF_INET, CONFIG_ZENBEDDED_USB_CDC_PEER_IPV4_ADDR, &pool_addr) == 0)
  {
    int srv_err = net_dhcpv4_server_start(iface, &pool_addr);
    if (srv_err != 0)
    {
      LOG_ERR("Failed to start DHCPv4 server: %d", srv_err);
    }
    else
    {
      LOG_INF("DHCPv4 server started leasing %s to host", CONFIG_ZENBEDDED_USB_CDC_PEER_IPV4_ADDR);
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
}
#else
int init_usbd(net_if * iface)
{
  (void)iface;
  LOG_WRN("USB CDC transport not enabled in Kconfig");
  return -ENOTSUP;
}
#endif  // CONFIG_ZENBEDDED_COMMON_USB_CDC

int init_network()
{
#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC) && defined(CONFIG_ZENBEDDED_COMMON_WIFI)
  int usb_ret = init_usbd();
  if (usb_ret != 0)
  {
    LOG_WRN("USB CDC interface not ready or timed out: %d", usb_ret);
  }

  int wifi_ret = init_wifi();
  if (wifi_ret != 0)
  {
    LOG_WRN("Wi-Fi interface not ready or timed out: %d", wifi_ret);
  }

  if (usb_ret == 0 || wifi_ret == 0)
  {
    LOG_INF(
      "Network initialized (USB: %s, Wi-Fi: %s)", usb_ret == 0 ? "READY" : "DOWN",
      wifi_ret == 0 ? "READY" : "DOWN");
    return 0;
  }
  return -ENETDOWN;
#elif defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
  return init_usbd();
#elif defined(CONFIG_ZENBEDDED_COMMON_WIFI)
  return init_wifi();
#else
  LOG_WRN("No network transport enabled in Kconfig");
  return 0;
#endif
}

bool is_usbd_ready()
{
#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
  struct net_if * iface = nullptr;
#if defined(CONFIG_USBD_CDC_ECM_CLASS)
  const struct device * dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(cdc_ecm_eth0));
  if (dev != nullptr && device_is_ready(dev))
  {
    iface = net_if_lookup_by_dev(dev);
  }
#endif
#if !defined(CONFIG_ZENBEDDED_COMMON_WIFI)
  if (iface == nullptr)
  {
    iface = net_if_get_default();
  }
#endif
  return iface != nullptr && net_if_is_up(iface) &&
         (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != nullptr);
#else
  return false;
#endif
}

bool is_wifi_ready()
{
#if defined(CONFIG_ZENBEDDED_COMMON_WIFI)
  struct net_if * iface = net_if_get_wifi_sta();
#if !defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
  if (iface == nullptr)
  {
    iface = net_if_get_default();
  }
#endif
  return iface != nullptr && net_if_is_up(iface) &&
         (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != nullptr);
#else
  return false;
#endif
}

int get_usb_peer_locator(const char * target_locator, char * out_usb_locator, size_t out_len)
{
  if (out_usb_locator == nullptr || out_len == 0)
  {
    return -EINVAL;
  }

  const char * port = "7447";
  if (target_locator != nullptr)
  {
    const char * colon = strrchr(target_locator, ':');
    if (
      colon != nullptr && *(colon + 1) != '\0' && isdigit(static_cast<unsigned char>(*(colon + 1))))
    {
      port = colon + 1;
    }
  }

#if defined(CONFIG_ZENBEDDED_USB_CDC_PEER_IPV4_ADDR)
  const char * peer_ip = CONFIG_ZENBEDDED_USB_CDC_PEER_IPV4_ADDR;
#else
  const char * peer_ip = "192.0.2.1";
#endif

  int written = snprintf(out_usb_locator, out_len, "%s:%s", peer_ip, port);
  if (written < 0 || static_cast<size_t>(written) >= out_len)
  {
    return -ENOSPC;
  }

  return 0;
}

}  // namespace zenbedded::common
