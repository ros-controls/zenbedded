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

#ifndef DEMO_USBD_H_
#define DEMO_USBD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/usb/usbd.h>

  /**
   * @brief Initialize and configure the default Zenbedded USB device context.
   *
   * Sets up device descriptors, manufacturer/product strings, FS/HS configurations,
   * and registers all enabled USB classes (e.g. CDC ECM).
   *
   * @return Initialized usbd_context pointer on success, NULL on failure.
   */
  struct usbd_context * zenbedded_demo_usbd_init(void);

#ifdef __cplusplus
}
#endif

#endif  // DEMO_USBD_H_
