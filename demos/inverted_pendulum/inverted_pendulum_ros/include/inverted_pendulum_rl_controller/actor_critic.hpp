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

// Direct C++ port of the ActorNet / CriticNet from the Python NNFurutaController:
//   Actor:  30 -> 64 (ReLU) -> 32 (ReLU) -> 1 (tanh)
//   Critic: 31 -> 64 (ReLU) -> 32 (ReLU) -> 1 (linear)   [31 = 30-dim state + 1-dim action]
// 30 = 5 stacked timesteps * 6 features [sin(theta), cos(theta), theta_dot, alpha, alpha_dot,
// prev_action]. Hidden sizes (64/32) are left unchanged from the original 60-dim version:
// at this parameter count (a few thousand weights either way) a 5-frame stack doesn't call
// for a smaller net, and over-provisioning slightly is harmless here, whereas under-provisioning
// would not be.
//
// kStateDim is written into the checkpoint file itself (see kFormatVersion below) and checked
// on load, so a checkpoint saved by a build with a different kHistoryLen/kStateDim fails
// load() loudly (falls back to random init, logged by the caller) instead of silently
// loading mismatched weights into the wrong-shaped first layer - the same silent-mismatch
// failure mode `run_config.py`'s `check_config()` exists to catch on the training side.

#pragma once

#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <utility>

#include "inverted_pendulum_rl_controller/tiny_nn.hpp"

namespace tiny_nn
{

constexpr int kStateDim = 30;
constexpr uint32_t kActorMagic = 0x4E465541;   // 'NFUA'
constexpr uint32_t kCriticMagic = 0x4E465543;  // 'NFUC'
constexpr uint32_t kFormatVersion = 2;         // bumped: v1 files were kStateDim=60

class ActorNet
{
public:
  // No no-arg default constructor on purpose: a zero-dimension network would
  // silently accept load() calls and then segfault on the first forward().
  // Always construct with an rng (even if you're about to load() weights
  // over the random init - load() only overwrites already-correctly-sized
  // buffers, it doesn't allocate them).
  explicit ActorNet(std::mt19937 & rng)
  : fc1_(kStateDim, 64, Activation::kReLU, rng),
    fc2_(64, 32, Activation::kReLU, rng),
    fc3_(32, 1, Activation::kTanh, rng)
  {
  }

  // Single-sample inference, used for action selection every control cycle
  // (and exclusively on real hardware once training is disabled).
  float forward(const Vec & state) { return fc3_.forward(fc2_.forward(fc1_.forward(state)))[0]; }

  Mat forward_batch(const Mat & states)
  {
    return fc3_.forward_batch(fc2_.forward_batch(fc1_.forward_batch(states)));
  }

  // dAction: batch x 1, dLoss/dAction. Backprops through the whole actor and
  // takes an Adam step on all three layers.
  void backward_and_step(const Mat & dAction, float lr)
  {
    Mat dH2 = fc3_.backward_batch(dAction);
    Mat dH1 = fc2_.backward_batch(dH2);
    fc1_.backward_batch(dH1);
    fc1_.apply_adam(lr);
    fc2_.apply_adam(lr);
    fc3_.apply_adam(lr);
  }

  void soft_update_from(const ActorNet & src, float tau)
  {
    fc1_.soft_update_from(src.fc1_, tau);
    fc2_.soft_update_from(src.fc2_, tau);
    fc3_.soft_update_from(src.fc3_, tau);
  }

  void copy_from(const ActorNet & src)
  {
    fc1_.copy_from(src.fc1_);
    fc2_.copy_from(src.fc2_);
    fc3_.copy_from(src.fc3_);
  }

  bool save(const std::string & path) const
  {
    std::ofstream f(path, std::ios::binary);
    if (!f)
    {
      return false;
    }
    f.write(reinterpret_cast<const char *>(&kActorMagic), sizeof(kActorMagic));
    f.write(reinterpret_cast<const char *>(&kFormatVersion), sizeof(kFormatVersion));
    const int32_t state_dim = kStateDim;
    f.write(reinterpret_cast<const char *>(&state_dim), sizeof(state_dim));
    fc1_.write(f);
    fc2_.write(f);
    fc3_.write(f);
    return static_cast<bool>(f);
  }

  bool load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f)
    {
      return false;
    }
    uint32_t magic = 0;
    f.read(reinterpret_cast<char *>(&magic), sizeof(magic));
    if (magic != kActorMagic)
    {
      return false;
    }
    uint32_t version = 0;
    f.read(reinterpret_cast<char *>(&version), sizeof(version));
    int32_t state_dim = 0;
    f.read(reinterpret_cast<char *>(&state_dim), sizeof(state_dim));
    if (!f || version != kFormatVersion || state_dim != kStateDim)
    {
      // Deliberately loud failure: an older/differently-shaped checkpoint
      // (e.g. saved with kHistoryLen=10, kStateDim=60) must never be
      // silently loaded into a differently-shaped network.
      return false;
    }
    return fc1_.read(f) && fc2_.read(f) && fc3_.read(f);
  }

private:
  DenseLayer fc1_, fc2_, fc3_;
};

class CriticNet
{
public:
  // See ActorNet's note above: no no-arg default constructor, same reason.
  explicit CriticNet(std::mt19937 & rng)
  : fc1_(kStateDim + 1, 64, Activation::kReLU, rng),
    fc2_(64, 32, Activation::kReLU, rng),
    fc3_(32, 1, Activation::kLinear, rng)
  {
  }

  float forward(const Vec & state, float action)
  {
    Vec x = state;
    x.push_back(action);
    return fc3_.forward(fc2_.forward(fc1_.forward(x)))[0];
  }

  Mat forward_batch(const Mat & states, const Vec & actions)
  {
    Mat x(states.size());
    for (size_t s = 0; s < states.size(); ++s)
    {
      x[s] = states[s];
      x[s].push_back(actions[s]);
    }
    return fc3_.forward_batch(fc2_.forward_batch(fc1_.forward_batch(x)));
  }

  // Standard critic (Q-function) update: dQ = dLoss/dQ, applies Adam to all
  // three layers. Must be called right after forward_batch() so caches match.
  void backward_and_step(const Mat & dQ, float lr)
  {
    Mat dH2 = fc3_.backward_batch(dQ);
    Mat dH1 = fc2_.backward_batch(dH2);
    fc1_.backward_batch(dH1);
    fc1_.apply_adam(lr);
    fc2_.apply_adam(lr);
    fc3_.apply_adam(lr);
  }

  // Used only for the DDPG actor update (actor_loss = -mean(critic(s, actor(s)))).
  // Backprops through the critic to get dLoss/dAction WITHOUT touching the
  // critic's own weights, matching the Python code where critic_opt.step()
  // is not called in the actor-update block. Must be called right after a
  // forward_batch(states, actor_actions) so the caches match.
  Vec backward_for_actor_only(const Mat & dQ)
  {
    Mat dH2 = fc3_.backward_batch(dQ);
    Mat dH1 = fc2_.backward_batch(dH2);
    Mat dX = fc1_.backward_batch(dH1);  // batch x (kStateDim + 1)
    Vec dAction(dX.size());
    for (size_t s = 0; s < dX.size(); ++s)
    {
      dAction[s] = dX[s][kStateDim];
    }
    return dAction;
  }

  void soft_update_from(const CriticNet & src, float tau)
  {
    fc1_.soft_update_from(src.fc1_, tau);
    fc2_.soft_update_from(src.fc2_, tau);
    fc3_.soft_update_from(src.fc3_, tau);
  }

  void copy_from(const CriticNet & src)
  {
    fc1_.copy_from(src.fc1_);
    fc2_.copy_from(src.fc2_);
    fc3_.copy_from(src.fc3_);
  }

  bool save(const std::string & path) const
  {
    std::ofstream f(path, std::ios::binary);
    if (!f)
    {
      return false;
    }
    f.write(reinterpret_cast<const char *>(&kCriticMagic), sizeof(kCriticMagic));
    f.write(reinterpret_cast<const char *>(&kFormatVersion), sizeof(kFormatVersion));
    const int32_t state_dim = kStateDim;
    f.write(reinterpret_cast<const char *>(&state_dim), sizeof(state_dim));
    fc1_.write(f);
    fc2_.write(f);
    fc3_.write(f);
    return static_cast<bool>(f);
  }

  bool load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f)
    {
      return false;
    }
    uint32_t magic = 0;
    f.read(reinterpret_cast<char *>(&magic), sizeof(magic));
    if (magic != kCriticMagic)
    {
      return false;
    }
    uint32_t version = 0;
    f.read(reinterpret_cast<char *>(&version), sizeof(version));
    int32_t state_dim = 0;
    f.read(reinterpret_cast<char *>(&state_dim), sizeof(state_dim));
    if (!f || version != kFormatVersion || state_dim != kStateDim)
    {
      return false;
    }
    return fc1_.read(f) && fc2_.read(f) && fc3_.read(f);
  }

private:
  DenseLayer fc1_, fc2_, fc3_;
};

}  // namespace tiny_nn
