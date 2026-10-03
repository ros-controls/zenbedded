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

#ifndef ZENBEDDED_RCL__ZENBEDDED_CLIENT_HPP_
#define ZENBEDDED_RCL__ZENBEDDED_CLIENT_HPP_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#include <zenbedded/rcl/codecs.hpp>
#include "zenbedded_transport/zenoh_transport.h"

class ZenbeddedClientBase
{
public:
  ZenbeddedClientBase();
  ~ZenbeddedClientBase();

  /// @brief Deinitialize the client and underlying transport.
  void destroy();

  /// @brief Publish current state buffer over Zenoh transport.
  int zenoh_publish_state();

  /// @brief Start the control publish thread.
  int start_thread(uint32_t control_freq);

  /// @brief Stop the control publish thread.
  void stop_thread();

  /// @brief Check if client is initialized.
  [[nodiscard]] bool is_initialized() const { return initialized_; }

  /// @brief Check if publish thread is running.
  bool is_control_thread_running();

  /// @brief Monotonic count of command payloads accepted from the transport.
  ///
  /// read_command() cannot report whether a command is new: it blocks until the double buffer is
  /// consistent and the codecs only reject undersized payloads. Callers that must react to a
  /// stalled peer compare this counter across control cycles instead.
  [[nodiscard]] uint32_t accepted_command_count() const;

protected:
  /// @brief Initializes base buffers, transport layer, and background thread.
  int init_base(uint32_t control_freq, size_t state_payload_size, size_t cmd_payload_size);

  /// @brief Direct slot access for state codec template initialization.
  uint8_t * get_state_buffer_slot(size_t slot_idx);

  /// @brief Acquire inactive state buffer index for thread-safe writing.
  uint8_t * prepare_state_write_slot(int & out_write_idx);

  /// @brief Commit atomic state buffer write and flip active index.
  void commit_state_write_slot(int write_idx);

  /// @brief Read raw latest command bytes from double buffer.
  bool read_latest_command_raw(uint8_t * data, size_t size);

  /// @brief Reset atomic buffer states and versions.
  void reset_buffers();

  bool initialized_ = false;
  size_t state_payload_size_ = 0;
  size_t cmd_payload_size_ = 0;

private:
  zenbedded_pub_t pub_{};
  zenbedded_sub_t sub_{};

  uint8_t state_buffer_[2][CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE]{};
  atomic_t state_buffer_active_idx_{};
  atomic_t state_buffer_version_[2]{};

  uint8_t cmd_buffer_[2][CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE]{};
  atomic_t cmd_buffer_active_idx_{};
  atomic_t cmd_buffer_version_[2]{};
  atomic_t cmd_accepted_count_{};

  uint32_t control_freq_ = 0;
  atomic_t control_thread_running_ = 0;
  bool control_thread_started_ = false;
  k_thread control_thread_{};
  K_KERNEL_STACK_MEMBER(control_thread_stack_, CONFIG_ZENBEDDED_RCL_THREAD_STACK_SIZE) {};

  static void control_thread_fn(void * arg1, void * arg2, void * arg3);
  static void on_transport_cmd_cb(const uint8_t * payload, size_t size, void * user_data);

  void write_state_to_buffer(const uint8_t * data, size_t size);
  bool read_state_from_buffer(uint8_t * data, size_t size);
  void write_command_to_buffer(const uint8_t * data, size_t size);
  bool read_command_from_buffer(uint8_t * data, size_t size);
};

template <typename StateCodec, typename CommandCodec>
class ZenbeddedClient : public ZenbeddedClientBase
{
public:
  using StateInitParams = typename StateCodec::InitParams;
  using CommandInitParams = typename CommandCodec::InitParams;
  using StateValue = typename StateCodec::Value;
  using CommandValue = typename CommandCodec::Value;

  ZenbeddedClient()
  {
    static_assert(CheckCodec<StateCodec>::is_valid_state_codec, "Invalid StateCodec Type!");
    static_assert(CheckCodec<CommandCodec>::is_valid_command_codec, "Invalid CommandCodec Type!");
  }

  int init(
    uint32_t control_freq, const StateInitParams & state_params = {},
    const CommandInitParams & cmd_params = {})
  {
    if (initialized_)
    {
      return 0;
    }

    reset_buffers();

    // Initialize state headers/codecs directly in double-buffer slots
    StateCodec::init(state_ctx_, state_params, get_state_buffer_slot(0));
    StateCodec::init(state_ctx_, state_params, get_state_buffer_slot(1));
    size_t st_size = StateCodec::payload_size(state_ctx_);

    // Determine command payload size using scratchpad space
    uint8_t cmd_layout_scratch[CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE];
    CommandCodec::init(cmd_ctx_, cmd_params, cmd_layout_scratch);
    size_t cmd_size = CommandCodec::payload_size(cmd_ctx_);

    return init_base(control_freq, st_size, cmd_size);
  }

  void write_state(const StateValue & value)
  {
    if (!initialized_)
    {
      return;
    }
    int write_idx = 0;
    uint8_t * buf = prepare_state_write_slot(write_idx);
    StateCodec::write(state_ctx_, value, buf);
    commit_state_write_slot(write_idx);
  }

  bool read_command(CommandValue & out)
  {
    if (!initialized_)
    {
      return false;
    }
    uint8_t tmp[CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE];
    if (!read_latest_command_raw(tmp, cmd_payload_size_))
    {
      return false;
    }
    return CommandCodec::read(cmd_ctx_, tmp, cmd_payload_size_, out);
  }

private:
  typename StateCodec::Ctx state_ctx_{};
  typename CommandCodec::Ctx cmd_ctx_{};
};

#endif  // ZENBEDDED_RCL__ZENBEDDED_CLIENT_HPP_
