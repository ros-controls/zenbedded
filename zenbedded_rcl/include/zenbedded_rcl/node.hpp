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

#ifndef ZENBEDDED_RCL__NODE_HPP_
#define ZENBEDDED_RCL__NODE_HPP_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zenbedded_rcl/publisher.hpp"
#include "zenbedded_rcl/subscriber.hpp"
#include "zenbedded_transport/zenoh_transport.h"

class ZenbeddedNode
{
public:
  ZenbeddedNode(
    const char * node_name = CONFIG_ZENBEDDED_NODE_NAME,
    uint32_t domain_id = CONFIG_ZENBEDDED_DOMAIN_ID)
  : domain_id_(domain_id), node_name_(node_name)
  {
  }

  ~ZenbeddedNode() { destroy(); }

  int init(
    const char * locator = CONFIG_ZENBEDDED_ZENOH_IP_PORT,
    const char * mode = CONFIG_ZENBEDDED_ZENOH_MODE)
  {
    if (initialized_)
    {
      return 0;
    }
    int ret = zenbedded_transport_init(domain_id_, node_name_, mode, locator);
    if (ret == 0)
    {
      initialized_ = true;
      k_msleep(20);  // Allow background read and lease tasks to settle
    }
    return ret;
  }

  void destroy()
  {
    if (!initialized_)
    {
      return;
    }
    zenbedded_transport_destroy();
    initialized_ = false;
  }

  [[nodiscard]] bool is_initialized() const { return initialized_; }

  template <typename Codec>
  int create_publisher(
    ZenbeddedPublisher<Codec> & pub, const char * topic_name,
    const typename Codec::InitParams & params = {})
  {
    if (!initialized_)
    {
      return -ENODEV;
    }
    int ret = -1;
    for (int attempt = 0; attempt < 5; attempt++)
    {
      ret = pub.init(topic_name, params);
      if (ret == 0)
      {
        k_msleep(10);  // Pace graph declarations
        return 0;
      }
      k_msleep(50);
    }
    return ret;
  }

  template <typename Codec>
  int create_subscriber(
    ZenbeddedSubscriber<Codec> & sub, const char * topic_name,
    typename ZenbeddedSubscriber<Codec>::CallbackFn cb = nullptr, void * user_data = nullptr,
    const typename Codec::InitParams & params = {})
  {
    if (!initialized_)
    {
      return -ENODEV;
    }
    int ret = -1;
    for (int attempt = 0; attempt < 5; attempt++)
    {
      ret = sub.init(topic_name, cb, user_data, params);
      if (ret == 0)
      {
        k_msleep(10);  // Pace graph declarations
        return 0;
      }
      k_msleep(50);
    }
    return ret;
  }

private:
  uint32_t domain_id_ = 0;
  const char * node_name_ = nullptr;
  bool initialized_ = false;
};

#endif  // ZENBEDDED_RCL__NODE_HPP_
