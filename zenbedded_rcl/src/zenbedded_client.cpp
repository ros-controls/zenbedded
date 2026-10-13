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

#include <zenbedded/rcl/zenbedded_client.hpp>
#include <zenbedded_transport/zenoh_transport.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zenbedded_client, LOG_LEVEL_INF);

BUILD_ASSERT(
  2 * CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE < CONFIG_ZENBEDDED_RCL_THREAD_STACK_SIZE,
  "CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE leaves too little control thread stack; "
  "raise CONFIG_ZENBEDDED_RCL_THREAD_STACK_SIZE");

ZenbeddedClientBase::ZenbeddedClientBase() { reset_buffers(); }

ZenbeddedClientBase::~ZenbeddedClientBase() { destroy(); }

int ZenbeddedClientBase::init_base(
  uint32_t control_freq, size_t state_payload_size, size_t cmd_payload_size)
{
  if (initialized_)
  {
    LOG_WRN("Zenbedded Client already initialized");
    return 0;
  }

  state_payload_size_ = state_payload_size;
  if (state_payload_size_ > CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE)
  {
    LOG_ERR(
      "State payload (%zu) exceeds CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE (%d)",
      state_payload_size_, CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE);
    return -ENOMEM;
  }

  cmd_payload_size_ = cmd_payload_size;
  if (cmd_payload_size_ > CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE)
  {
    LOG_ERR(
      "Command payload (%zu) exceeds CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE (%d)", cmd_payload_size_,
      CONFIG_ZENBEDDED_MAX_CMD_BUFFER_SIZE);
    return -ENOMEM;
  }

  if (
    zenbedded_transport_init(
      CONFIG_ZENBEDDED_DOMAIN_ID, CONFIG_ZENBEDDED_NODE_NAME, CONFIG_ZENBEDDED_ZENOH_MODE,
      CONFIG_ZENBEDDED_ZENOH_IP_PORT) != 0)
  {
    LOG_ERR("Failed to initialize zenbedded transport");
    return -EIO;
  }

  const char *state_type = nullptr, *cmd_type = nullptr;
#ifdef CONFIG_ZENBEDDED_TIER_1
  state_type = CONFIG_ZENBEDDED_RCL_STATE_MSG_TYPE_STRING;
  cmd_type = CONFIG_ZENBEDDED_RCL_CMD_MSG_TYPE_STRING;
#endif

  pub_ = zenbedded_transport_declare_publisher(CONFIG_ZENBEDDED_RCL_PUB_TOPIC, state_type);
  if (pub_ == nullptr)
  {
    LOG_ERR("Failed to declare publisher");
    return -EIO;
  }
  sub_ = zenbedded_transport_declare_subscriber(
    CONFIG_ZENBEDDED_RCL_SUB_TOPIC, cmd_type, on_transport_cmd_cb, this);
  if (sub_ == nullptr)
  {
    LOG_ERR("Failed to declare subscriber");
    return -EIO;
  }

  initialized_ = true;
  control_freq_ = control_freq;

  LOG_INF(
    "ZenbeddedClient initialized (state_size=%zuB, cmd_size=%zuB)", state_payload_size_,
    cmd_payload_size_);

  return start_thread(control_freq);
}

void ZenbeddedClientBase::destroy()
{
  if (!initialized_)
  {
    return;
  }
  stop_thread();
  zenbedded_transport_destroy();

  pub_ = nullptr;
  sub_ = nullptr;
  initialized_ = false;
  LOG_INF("ZenbeddedClient deinitialized");
}

uint8_t * ZenbeddedClientBase::get_state_buffer_slot(size_t slot_idx)
{
  return state_buffer_[slot_idx];
}

uint8_t * ZenbeddedClientBase::prepare_state_write_slot(int & out_write_idx)
{
  out_write_idx = atomic_get(&state_buffer_active_idx_) ^ 1;
  return state_buffer_[out_write_idx];
}

void ZenbeddedClientBase::commit_state_write_slot(int write_idx)
{
  // Ensure write completes before version increment
  __atomic_thread_fence(__ATOMIC_RELEASE);

  atomic_inc(&state_buffer_version_[write_idx]);
  atomic_set(&state_buffer_active_idx_, write_idx);  // Flip active index
}

bool ZenbeddedClientBase::read_latest_command_raw(uint8_t * data, size_t size)
{
  while (!read_command_from_buffer(data, size))
  {
  }
  return true;
}

int ZenbeddedClientBase::zenoh_publish_state()
{
  if (!initialized_)
  {
    LOG_ERR("Zenbedded Client not initialized");
    return -ENODEV;
  }

  uint8_t tmp[CONFIG_ZENBEDDED_MAX_STATE_BUFFER_SIZE];
  while (!read_state_from_buffer(tmp, state_payload_size_))
  {
  }

  int ret = zenbedded_transport_publish(pub_, tmp, state_payload_size_);
  if (ret < 0)
  {
    LOG_ERR("Failed to publish state: %d", ret);
  }
  return ret;
}

int ZenbeddedClientBase::start_thread(uint32_t control_freq)
{
  if (!initialized_)
  {
    LOG_ERR("Zenbedded Client not initialized");
    return -ENODEV;
  }

  if (control_freq == 0)
  {
    LOG_WRN("Control Thread frequency is zero, thread wouldn't be started");
    return -EINVAL;
  }

  if (control_thread_started_)
  {
    stop_thread();
    k_sleep(K_MSEC(10));
  }

  control_freq_ = control_freq;
  atomic_set(&control_thread_running_, 1);

  k_thread_create(
    &control_thread_, control_thread_stack_, K_KERNEL_STACK_SIZEOF(control_thread_stack_),
    control_thread_fn,
    this,     // arg1
    nullptr,  // arg2
    nullptr,  // arg3
    CONFIG_ZENBEDDED_RCL_THREAD_PRIORITY,
    0,         // options
    K_NO_WAIT  // start immediately
  );

  control_thread_started_ = true;
  LOG_INF("Started thread at %u Hz", control_freq);
  return 0;
}

void ZenbeddedClientBase::stop_thread()
{
  if (!control_thread_started_)
  {
    return;
  }

  LOG_INF("Stopping control thread");
  atomic_set(&control_thread_running_, 0);  // cleanly exit thread loop

  if (k_thread_join(&control_thread_, K_MSEC(200)) != 0)
  {
    LOG_WRN("Control thread did not stop gracefully");
    k_thread_abort(&control_thread_);
  }

  control_thread_started_ = false;
  control_freq_ = 0;
  LOG_INF("Publish thread stopped");
}

void ZenbeddedClientBase::control_thread_fn(void * arg1, void * arg2, void * arg3)
{
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  auto * self = static_cast<ZenbeddedClientBase *>(arg1);

  if (!self)
  {
    LOG_ERR("Invalid client instance in publish thread");
    return;
  }

  LOG_INF("Control thread started");

  const uint32_t period_ms = MAX(1000 / self->control_freq_, 1);
  uint32_t prev_time = k_uptime_get_32();

  while (atomic_get(&self->control_thread_running_))
  {
    int ret = self->zenoh_publish_state();
    if (ret < 0)
    {
      LOG_DBG("Zenoh Publish failed: %d", ret);
    }

    const uint32_t diff = k_uptime_get_32() - prev_time;
    const uint32_t sleep_ms = (diff >= period_ms) ? 1 : (period_ms - diff);
    k_sleep(K_MSEC(sleep_ms));
    prev_time = k_uptime_get_32();
  }

  atomic_set(&self->control_thread_running_, 0);
  LOG_INF("Publish thread stopped");
}

bool ZenbeddedClientBase::is_control_thread_running()
{
  return atomic_get(&control_thread_running_);
}

uint32_t ZenbeddedClientBase::accepted_command_count() const
{
  return static_cast<uint32_t>(atomic_get(&cmd_accepted_count_));
}

void ZenbeddedClientBase::reset_buffers()
{
  atomic_set(&state_buffer_active_idx_, 0);
  atomic_set(&cmd_buffer_active_idx_, 0);

  atomic_set(&state_buffer_version_[0], 0);
  atomic_set(&state_buffer_version_[1], 0);
  atomic_set(&cmd_buffer_version_[0], 0);
  atomic_set(&cmd_buffer_version_[1], 0);
  atomic_set(&cmd_accepted_count_, 0);

  memset(state_buffer_, 0, sizeof(state_buffer_));
  memset(cmd_buffer_, 0, sizeof(cmd_buffer_));
}

void ZenbeddedClientBase::on_transport_cmd_cb(
  const uint8_t * payload, size_t size, void * user_data)
{
  auto * self = static_cast<ZenbeddedClientBase *>(user_data);

  if (size != self->cmd_payload_size_)
  {
    LOG_WRN("Unexpected command payload size: %zu, expected %zu", size, self->cmd_payload_size_);
    return;
  }

  self->write_command_to_buffer(payload, size);
}

bool ZenbeddedClientBase::read_state_from_buffer(uint8_t * data, size_t size)
{
  const int read_idx = atomic_get(&state_buffer_active_idx_);
  const atomic_val_t ver_before = atomic_get(&state_buffer_version_[read_idx]);
  memcpy(data, state_buffer_[read_idx], size);

  __atomic_thread_fence(__ATOMIC_ACQUIRE);

  if (
    atomic_get(&state_buffer_active_idx_) != read_idx ||
    atomic_get(&state_buffer_version_[read_idx]) != ver_before)
  {
    return false;
  }
  return true;
}

void ZenbeddedClientBase::write_state_to_buffer(const uint8_t * data, size_t size)
{
  const int write_idx = atomic_get(&state_buffer_active_idx_) ^ 1;
  memcpy(state_buffer_[write_idx], data, size);

  __atomic_thread_fence(__ATOMIC_RELEASE);

  atomic_inc(&state_buffer_version_[write_idx]);
  atomic_set(&state_buffer_active_idx_, write_idx);
}

void ZenbeddedClientBase::write_command_to_buffer(const uint8_t * data, size_t size)
{
  const int write_idx = atomic_get(&cmd_buffer_active_idx_) ^ 1;
  memcpy(cmd_buffer_[write_idx], data, size);

  __atomic_thread_fence(__ATOMIC_RELEASE);

  atomic_inc(&cmd_buffer_version_[write_idx]);
  atomic_set(&cmd_buffer_active_idx_, write_idx);
  atomic_inc(&cmd_accepted_count_);
}

bool ZenbeddedClientBase::read_command_from_buffer(uint8_t * data, size_t size)
{
  const int read_idx = atomic_get(&cmd_buffer_active_idx_);
  const atomic_val_t ver_before = atomic_get(&cmd_buffer_version_[read_idx]);
  memcpy(data, cmd_buffer_[read_idx], size);

  __atomic_thread_fence(__ATOMIC_ACQUIRE);

  if (
    atomic_get(&cmd_buffer_active_idx_) != read_idx ||
    atomic_get(&cmd_buffer_version_[read_idx]) != ver_before)
  {
    return false;
  }
  return true;
}
