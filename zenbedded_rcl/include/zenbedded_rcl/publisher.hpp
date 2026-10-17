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
  ZenbeddedPublisherBase() { reset_buffer(); }

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

  int start_periodic_publish(uint32_t freq_hz)
  {
    if (!initialized_)
    {
      return -ENODEV;
    }
    if (freq_hz == 0)
    {
      return -EINVAL;
    }
    if (thread_started_)
    {
      stop_periodic_publish();
      k_sleep(K_MSEC(10));
    }

    publish_freq_ = freq_hz;
    atomic_set(&thread_running_, 1);

    k_thread_create(
      &thread_, thread_stack_, K_KERNEL_STACK_SIZEOF(thread_stack_), thread_fn, this, nullptr,
      nullptr, CONFIG_ZENBEDDED_RCL_THREAD_PRIORITY, 0, K_NO_WAIT);

    thread_started_ = true;
    return 0;
  }

  void stop_periodic_publish()
  {
    if (!thread_started_)
    {
      return;
    }
    atomic_set(&thread_running_, 0);
    if (k_thread_join(&thread_, K_MSEC(200)) != 0)
    {
      k_thread_abort(&thread_);
    }
    thread_started_ = false;
    publish_freq_ = 0;
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

  zenbedded_pub_t pub_handle_ = nullptr;
  bool initialized_ = false;
  size_t payload_size_ = 0;

private:
  zenbedded::DoubleByteBuffer<CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE> buffer_;

  uint32_t publish_freq_ = 0;
  atomic_t thread_running_ = 0;
  bool thread_started_ = false;
  k_thread thread_{};
  K_KERNEL_STACK_MEMBER(thread_stack_, CONFIG_ZENBEDDED_RCL_THREAD_STACK_SIZE) {};

  static void thread_fn(void * arg1, void * arg2, void * arg3)
  {
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    auto * self = static_cast<ZenbeddedPublisherBase *>(arg1);
    if (!self)
    {
      return;
    }

    const uint32_t period_ms = MAX(1000 / self->publish_freq_, 1);
    uint32_t prev_time = k_uptime_get_32();
    uint8_t tmp[CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE];

    while (atomic_get(&self->thread_running_))
    {
      while (!self->read_active_buffer(tmp, self->payload_size_))
      {
      }
      self->publish_raw(tmp, self->payload_size_);

      const uint32_t diff = k_uptime_get_32() - prev_time;
      const uint32_t sleep_ms = (diff >= period_ms) ? 1 : (period_ms - diff);
      k_sleep(K_MSEC(sleep_ms));
      prev_time = k_uptime_get_32();
    }
  }
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
