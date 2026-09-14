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

#pragma once

#include <deque>
#include <random>
#include <utility>
#include <vector>

namespace tiny_nn
{

struct Transition
{
  std::vector<float> state;
  float action = 0.0f;
  float reward = 0.0f;
  std::vector<float> next_state;
};

// Fixed-capacity FIFO replay buffer with uniform random sampling.
class ReplayBuffer
{
public:
  explicit ReplayBuffer(size_t capacity = 100000) : capacity_(capacity) {}

  void push(Transition t)
  {
    if (buffer_.size() >= capacity_)
    {
      buffer_.pop_front();
    }
    buffer_.push_back(std::move(t));
  }

  size_t size() const { return buffer_.size(); }

  void sample(
    size_t batch_size, std::mt19937 & rng, std::vector<std::vector<float>> & states,
    std::vector<float> & actions, std::vector<float> & rewards,
    std::vector<std::vector<float>> & next_states) const
  {
    std::uniform_int_distribution<size_t> dist(0, buffer_.size() - 1);
    states.resize(batch_size);
    actions.resize(batch_size);
    rewards.resize(batch_size);
    next_states.resize(batch_size);
    for (size_t i = 0; i < batch_size; ++i)
    {
      const Transition & t = buffer_[dist(rng)];
      states[i] = t.state;
      actions[i] = t.action;
      rewards[i] = t.reward;
      next_states[i] = t.next_state;
    }
  }

private:
  std::deque<Transition> buffer_;
  size_t capacity_;
};

}  // namespace tiny_nn
