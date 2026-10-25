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

#include <zenbedded_demo/demo_usbd.h>

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/bos.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(demo_usbd, CONFIG_USBD_LOG_LEVEL);

/* By default, do not register the USB DFU class DFU mode instance. */
static const char * const blocklist[] = {
  "dfu_dfu",
  NULL,
};

USBD_DEVICE_DEFINE(
  demo_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_ZENBEDDED_DEMO_USBD_VID,
  CONFIG_ZENBEDDED_DEMO_USBD_PID);

USBD_DESC_LANG_DEFINE(demo_lang);
USBD_DESC_MANUFACTURER_DEFINE(demo_mfr, CONFIG_ZENBEDDED_DEMO_USBD_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(demo_product, CONFIG_ZENBEDDED_DEMO_USBD_PRODUCT);
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(demo_sn)));

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");

static const uint8_t attributes = USB_SCD_SELF_POWERED;

/* Full speed configuration */
USBD_CONFIGURATION_DEFINE(
  demo_fs_config, attributes, CONFIG_ZENBEDDED_DEMO_USBD_MAX_POWER, &fs_cfg_desc);

/* High speed configuration */
USBD_CONFIGURATION_DEFINE(
  demo_hs_config, attributes, CONFIG_ZENBEDDED_DEMO_USBD_MAX_POWER, &hs_cfg_desc);

static void demo_fix_code_triple(struct usbd_context * uds_ctx, const enum usbd_speed speed)
{
  if (
    IS_ENABLED(CONFIG_USBD_CDC_ECM_CLASS) || IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS) ||
    IS_ENABLED(CONFIG_USBD_CDC_NCM_CLASS))
  {
    usbd_device_set_code_triple(uds_ctx, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
  }
  else
  {
    usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
  }
}

struct usbd_context * zenbedded_demo_usbd_init(void)
{
  int err;

  err = usbd_add_descriptor(&demo_usbd, &demo_lang);
  if (err)
  {
    LOG_ERR("Failed to add language descriptor (%d)", err);
    return NULL;
  }

  err = usbd_add_descriptor(&demo_usbd, &demo_mfr);
  if (err)
  {
    LOG_ERR("Failed to add manufacturer descriptor (%d)", err);
    return NULL;
  }

  err = usbd_add_descriptor(&demo_usbd, &demo_product);
  if (err)
  {
    LOG_ERR("Failed to add product descriptor (%d)", err);
    return NULL;
  }

  IF_ENABLED(CONFIG_HWINFO, (err = usbd_add_descriptor(&demo_usbd, &demo_sn); if (err) {
               LOG_ERR("Failed to add SN descriptor (%d)", err);
               return NULL;
             }));

  if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(&demo_usbd) == USBD_SPEED_HS)
  {
    err = usbd_add_configuration(&demo_usbd, USBD_SPEED_HS, &demo_hs_config);
    if (err)
    {
      LOG_ERR("Failed to add High-Speed configuration (%d)", err);
      return NULL;
    }

    err = usbd_register_all_classes(&demo_usbd, USBD_SPEED_HS, 1, blocklist);
    if (err)
    {
      LOG_ERR("Failed to register HS classes (%d)", err);
      return NULL;
    }

    demo_fix_code_triple(&demo_usbd, USBD_SPEED_HS);
  }

  err = usbd_add_configuration(&demo_usbd, USBD_SPEED_FS, &demo_fs_config);
  if (err)
  {
    LOG_ERR("Failed to add Full-Speed configuration (%d)", err);
    return NULL;
  }

  err = usbd_register_all_classes(&demo_usbd, USBD_SPEED_FS, 1, blocklist);
  if (err)
  {
    LOG_ERR("Failed to register FS classes (%d)", err);
    return NULL;
  }

  demo_fix_code_triple(&demo_usbd, USBD_SPEED_FS);
  usbd_self_powered(&demo_usbd, attributes & USB_SCD_SELF_POWERED);

  err = usbd_init(&demo_usbd);
  if (err)
  {
    LOG_ERR("Failed to initialize USB device (%d)", err);
    return NULL;
  }

  return &demo_usbd;
}
