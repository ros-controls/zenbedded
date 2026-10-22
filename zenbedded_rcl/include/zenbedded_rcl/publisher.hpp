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

#ifndef ZENBEDDED_RCL__PUBLISHER_HPP_
#define ZENBEDDED_RCL__PUBLISHER_HPP_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_rcl/double_buffer.hpp"
#include "zenbedded_transport/zenoh_transport.h"

class ZenbeddedPublisherBase
{
public:
  ZenbeddedPublisherBase()
  {
    reset_buffer();
    periodic_ctx_.owner = this;
    k_work_init_delayable(&periodic_ctx_.work, periodic_work_handler);
  }

  ~ZenbeddedPublisherBase() { destroy(); }

  void destroy()
  {
    if (!initialized_)
    {
      return;
    }
    stop_periodic_publish();
    if (pub_handle_)
    {
      zenbedded_transport_undeclare_publisher(pub_handle_);
      pub_handle_ = nullptr;
    }
    initialized_ = false;
  }

  [[nodiscard]] bool is_initialized() const { return initialized_; }

  int publish_raw(const uint8_t * payload, size_t size)
  {
    if (!initialized_ || !pub_handle_)
    {
      return -ENODEV;
    }
    return zenbedded_transport_publish(pub_handle_, payload, size);
  }

  int start_periodic_publish(k_timeout_t period)
  {
    if (!initialized_)
    {
      return -ENODEV;
    }
    if (K_TIMEOUT_EQ(period, K_FOREVER) || K_TIMEOUT_EQ(period, K_NO_WAIT))
    {
      return -EINVAL;
    }
    if (atomic_get(&periodic_running_))
    {
      stop_periodic_publish();
    }

    publish_period_ = period;
    atomic_set(&periodic_running_, 1);
    k_work_schedule(&periodic_ctx_.work, publish_period_);
    return 0;
  }

  int start_periodic_publish(uint32_t period_ms)
  {
    if (period_ms == 0)
    {
      return -EINVAL;
    }
    return start_periodic_publish(K_MSEC(period_ms));
  }

  void stop_periodic_publish()
  {
    if (!atomic_get(&periodic_running_))
    {
      return;
    }
    atomic_set(&periodic_running_, 0);
    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&periodic_ctx_.work, &sync);
    publish_period_ = K_FOREVER;
  }

protected:
  int init_pub_base(const char * topic_name, const char * msg_type, size_t payload_size)
  {
    if (initialized_)
    {
      return 0;
    }
    payload_size_ = payload_size;
    if (payload_size_ > CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE)
    {
      return -ENOMEM;
    }

    pub_handle_ = zenbedded_transport_declare_publisher(topic_name, msg_type);
    if (!pub_handle_)
    {
      return -EIO;
    }

    initialized_ = true;
    return 0;
  }

  uint8_t * get_buffer_slot(size_t slot_idx) { return buffer_.slot(slot_idx); }

  uint8_t * prepare_write_slot(int & out_write_idx) { return buffer_.prepare_write(out_write_idx); }

  void commit_write_slot(int write_idx) { buffer_.commit_write(write_idx); }

  bool read_active_buffer(uint8_t * out, size_t size) { return buffer_.read_latest(out, size); }

  void reset_buffer() { buffer_.reset(); }

  struct PeriodicWorkContext
  {
    struct k_work_delayable work{};
    ZenbeddedPublisherBase * owner = nullptr;
  };

  zenbedded_pub_t pub_handle_ = nullptr;
  bool initialized_ = false;
  size_t payload_size_ = 0;

  PeriodicWorkContext periodic_ctx_{};
  k_timeout_t publish_period_ = K_FOREVER;
  atomic_t periodic_running_ = 0;

  static void periodic_work_handler(struct k_work * work)
  {
    auto * dwork = k_work_delayable_from_work(work);
    auto * ctx = CONTAINER_OF(dwork, PeriodicWorkContext, work);
    auto * self = ctx != nullptr ? ctx->owner : nullptr;
    if (!self || !atomic_get(&self->periodic_running_))
    {
      return;
    }

    uint8_t tmp[CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE];
    if (self->read_active_buffer(tmp, self->payload_size_))
    {
      self->publish_raw(tmp, self->payload_size_);
    }

    if (atomic_get(&self->periodic_running_))
    {
      k_work_schedule(&self->periodic_ctx_.work, self->publish_period_);
    }
  }

private:
  zenbedded::DoubleByteBuffer<CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE> buffer_;
};

template <typename Codec>
class ZenbeddedPublisher : public ZenbeddedPublisherBase
{
public:
  using InitParams = typename Codec::InitParams;
  using Value = typename Codec::Value;

  ZenbeddedPublisher()
  {
    static_assert(CheckCodec<Codec>::is_valid_state_codec, "Invalid Codec Type for Publisher!");
  }

  int init(const char * topic_name, const InitParams & params = {})
  {
    if (initialized_)
    {
      return 0;
    }

    reset_buffer();

    Codec::init(ctx_, params, get_buffer_slot(0));
    Codec::init(ctx_, params, get_buffer_slot(1));
    size_t size = Codec::payload_size(ctx_);

    return init_pub_base(topic_name, Codec::ros2_msg_type, size);
  }

  int publish(const Value & val)
  {
    if (!initialized_)
    {
      return -ENODEV;
    }
    uint8_t tmp[CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE];
    memcpy(tmp, get_buffer_slot(0), payload_size_);
    Codec::write(ctx_, val, tmp);
    return publish_raw(tmp, payload_size_);
  }

  void write_state(const Value & val)
  {
    if (!initialized_)
    {
      return;
    }
    int write_idx = 0;
    uint8_t * buf = prepare_write_slot(write_idx);
    Codec::write(ctx_, val, buf);
    commit_write_slot(write_idx);
  }

private:
  typename Codec::Ctx ctx_{};
};

#endif  // ZENBEDDED_RCL__PUBLISHER_HPP_
