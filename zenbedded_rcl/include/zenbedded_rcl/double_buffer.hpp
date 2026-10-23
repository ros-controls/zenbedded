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

#ifndef ZENBEDDED_RCL__DOUBLE_BUFFER_HPP_
#define ZENBEDDED_RCL__DOUBLE_BUFFER_HPP_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/spinlock.h>

namespace zenbedded
{

template <size_t Capacity>
class DoubleByteBuffer
{
public:
  DoubleByteBuffer() { reset(); }

  void reset()
  {
    K_SPINLOCK(&lock_)
    {
      active_idx_ = 0;
      memset(buffer_, 0, sizeof(buffer_));
    }
  }

  uint8_t * slot(size_t idx) { return buffer_[idx]; }
  const uint8_t * slot(size_t idx) const { return buffer_[idx]; }

  uint8_t * prepare_write(int & out_write_idx)
  {
    K_SPINLOCK(&lock_) { out_write_idx = active_idx_ ^ 1; }
    return buffer_[out_write_idx];
  }

  void commit_write(int write_idx)
  {
    K_SPINLOCK(&lock_) { active_idx_ = write_idx; }
  }

  bool write(const uint8_t * data, size_t size)
  {
    if (!data || size > Capacity)
    {
      return false;
    }
    int write_idx = 0;
    uint8_t * dst = prepare_write(write_idx);
    memcpy(dst, data, size);
    commit_write(write_idx);
    return true;
  }

  bool read_latest(uint8_t * out, size_t size) const
  {
    if (!out || size > Capacity)
    {
      return false;
    }
    K_SPINLOCK(&lock_) { memcpy(out, buffer_[active_idx_], size); }
    return true;
  }

  static constexpr size_t capacity() { return Capacity; }

  [[nodiscard]] int active_index() const
  {
    int idx = 0;
    K_SPINLOCK(&lock_) { idx = active_idx_; }
    return idx;
  }

private:
  uint8_t buffer_[2][Capacity]{};
  mutable struct k_spinlock lock_{};
  int active_idx_{0};
};

template <typename T>
class DoubleBuffer
{
public:
  DoubleBuffer() { reset(); }

  void reset()
  {
    K_SPINLOCK(&lock_)
    {
      active_idx_ = 0;
      slots_[0] = T{};
      slots_[1] = T{};
    }
  }

  T & slot(size_t idx) { return slots_[idx]; }
  const T & slot(size_t idx) const { return slots_[idx]; }

  T & prepare_write(int & out_write_idx)
  {
    K_SPINLOCK(&lock_) { out_write_idx = active_idx_ ^ 1; }
    return slots_[out_write_idx];
  }

  void commit_write(int write_idx)
  {
    K_SPINLOCK(&lock_) { active_idx_ = write_idx; }
  }

  void write(const T & value)
  {
    int write_idx = 0;
    prepare_write(write_idx) = value;
    commit_write(write_idx);
  }

  bool read_latest(T & out) const
  {
    K_SPINLOCK(&lock_) { out = slots_[active_idx_]; }
    return true;
  }

  [[nodiscard]] int active_index() const
  {
    int idx = 0;
    K_SPINLOCK(&lock_) { idx = active_idx_; }
    return idx;
  }

private:
  T slots_[2]{};
  mutable struct k_spinlock lock_{};
  int active_idx_{0};
};

}  // namespace zenbedded

#endif  // ZENBEDDED_RCL__DOUBLE_BUFFER_HPP_
