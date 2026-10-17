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

#ifndef ZENBEDDED_RCL__SUBSCRIBER_HPP_
#define ZENBEDDED_RCL__SUBSCRIBER_HPP_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_rcl/double_buffer.hpp"
#include "zenbedded_transport/zenoh_transport.h"

class ZenbeddedSubscriberBase
{
public:
  using DispatchFn = void (*)(void * sub_obj, const uint8_t * payload, size_t size);

  explicit ZenbeddedSubscriberBase(DispatchFn dispatch_fn = nullptr)
  : dispatch_user_cb_fn_(dispatch_fn)
  {
    reset_buffer();
  }

  ~ZenbeddedSubscriberBase() { destroy(); }

  void destroy()
  {
    if (!initialized_)
    {
      return;
    }
    if (sub_handle_)
    {
      zenbedded_transport_undeclare_subscriber(sub_handle_);
      sub_handle_ = nullptr;
    }
    initialized_ = false;
  }

  [[nodiscard]] bool is_initialized() const { return initialized_; }

  [[nodiscard]] uint32_t accepted_count() const
  {
    return static_cast<uint32_t>(atomic_get(&accepted_count_));
  }

protected:
  int init_sub_base(const char * topic_name, const char * msg_type, size_t payload_size)
  {
    if (initialized_)
    {
      return 0;
    }
    payload_size_ = payload_size;
    if (payload_size_ > CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE)
    {
      return -ENOMEM;
    }

    sub_handle_ =
      zenbedded_transport_declare_subscriber(topic_name, msg_type, on_transport_cb, this);
    if (!sub_handle_)
    {
      return -EIO;
    }

    initialized_ = true;
    return 0;
  }

  bool read_latest_raw(uint8_t * data, size_t size) { return buffer_.read_latest(data, size); }

  void reset_buffer()
  {
    buffer_.reset();
    atomic_set(&accepted_count_, 0);
  }

  zenbedded_sub_t sub_handle_ = nullptr;
  bool initialized_ = false;
  size_t payload_size_ = 0;
  DispatchFn dispatch_user_cb_fn_ = nullptr;

private:
  zenbedded::DoubleByteBuffer<CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE> buffer_;
  atomic_t accepted_count_{};

  static void on_transport_cb(const uint8_t * payload, size_t size, void * user_data)
  {
    auto * self = static_cast<ZenbeddedSubscriberBase *>(user_data);
    if (!self || size != self->payload_size_)
    {
      return;
    }

    self->buffer_.write(payload, size);
    atomic_inc(&self->accepted_count_);

    if (self->dispatch_user_cb_fn_)
    {
      self->dispatch_user_cb_fn_(self, payload, size);
    }
  }
};

template <typename Codec>
class ZenbeddedSubscriber : public ZenbeddedSubscriberBase
{
public:
  using InitParams = typename Codec::InitParams;
  using Value = typename Codec::Value;
  using CallbackFn = void (*)(const Value &, void * user_data);

  ZenbeddedSubscriber() : ZenbeddedSubscriberBase(&ZenbeddedSubscriber::dispatch_thunk)
  {
    static_assert(CheckCodec<Codec>::is_valid_command_codec, "Invalid Codec Type for Subscriber!");
  }

  int init(
    const char * topic_name, CallbackFn cb = nullptr, void * user_data = nullptr,
    const InitParams & params = {})
  {
    if (initialized_)
    {
      return 0;
    }

    reset_buffer();
    cb_ = cb;
    cb_user_data_ = user_data;

    uint8_t scratch[CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE];
    Codec::init(ctx_, params, scratch);
    size_t size = Codec::payload_size(ctx_);

    return init_sub_base(topic_name, Codec::ros2_msg_type, size);
  }

  bool take(Value & out)
  {
    if (!initialized_)
    {
      return false;
    }
    uint8_t tmp[CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE];
    if (!read_latest_raw(tmp, payload_size_))
    {
      return false;
    }
    return Codec::read(ctx_, tmp, payload_size_, out);
  }

private:
  static void dispatch_thunk(void * sub_obj, const uint8_t * payload, size_t size)
  {
    auto * self = static_cast<ZenbeddedSubscriber<Codec> *>(sub_obj);
    if (!self || !self->cb_)
    {
      return;
    }
    Value val{};
    if (Codec::read(self->ctx_, payload, size, val))
    {
      self->cb_(val, self->cb_user_data_);
    }
  }

  typename Codec::Ctx ctx_{};
  CallbackFn cb_ = nullptr;
  void * cb_user_data_ = nullptr;
};

#endif  // ZENBEDDED_RCL__SUBSCRIBER_HPP_
