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

// A minimal, dependency-free feed-forward NN + Adam optimizer, sized for the
// small actor/critic MLPs used by the Inverted pendulum DDPG agent. .
// The backward pass for each network is hand-derived in inverted_pendulum_rl_controller.hpp,
// this file only supplies the per-layer matmul/activation/Adam building blocks.

#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace tiny_nn
{

using Vec = std::vector<float>;
using Mat = std::vector<Vec>;  // batch-major: Mat[sample][feature]

enum class Activation
{
  kReLU,
  kTanh,
  kLinear
};

inline float apply_activation(float z, Activation act)
{
  switch (act)
  {
    case Activation::kReLU:
      return z > 0.0f ? z : 0.0f;
    case Activation::kTanh:
      return std::tanh(z);
    case Activation::kLinear:
    default:
      return z;
  }
}

// Derivative expressed in terms of the layer's own (z, a=activation(z)) pair.
inline float activation_grad(float a, float z, Activation act)
{
  switch (act)
  {
    case Activation::kReLU:
      return z > 0.0f ? 1.0f : 0.0f;
    case Activation::kTanh:
      return 1.0f - a * a;
    case Activation::kLinear:
    default:
      return 1.0f;
  }
}

inline Vec col0(const Mat & m)
{
  Vec v(m.size());
  for (size_t i = 0; i < m.size(); ++i)
  {
    v[i] = m[i][0];
  }
  return v;
}

// A single fully-connected layer with its own Adam moment buffers.
// Supports both a cheap single-sample forward() (for action
// selection / real-hardware inference) and a batched forward_batch()/
// backward_batch() pair (used only during train_step()).
class DenseLayer
{
public:
  DenseLayer() = default;

  DenseLayer(int in_dim, int out_dim, Activation act, std::mt19937 & rng)
  : in_dim_(in_dim),
    out_dim_(out_dim),
    act_(act),
    W_(out_dim, Vec(in_dim, 0.0f)),
    b_(out_dim, 0.0f),
    mW_(out_dim, Vec(in_dim, 0.0f)),
    vW_(out_dim, Vec(in_dim, 0.0f)),
    mb_(out_dim, 0.0f),
    vb_(out_dim, 0.0f)
  {
    // Glorot-uniform init.
    const float limit = std::sqrt(6.0f / static_cast<float>(in_dim + out_dim));
    std::uniform_real_distribution<float> dist(-limit, limit);
    for (int o = 0; o < out_dim; ++o)
    {
      for (int i = 0; i < in_dim; ++i)
      {
        W_[o][i] = dist(rng);
      }
    }
  }

  Vec forward(const Vec & x)
  {
    last_x_single_ = x;
    last_z_single_.assign(out_dim_, 0.0f);
    Vec y(out_dim_, 0.0f);
    for (int o = 0; o < out_dim_; ++o)
    {
      float z = b_[o];
      const Vec & w_row = W_[o];
      for (int i = 0; i < in_dim_; ++i)
      {
        z += w_row[i] * x[i];
      }
      last_z_single_[o] = z;
      y[o] = apply_activation(z, act_);
    }
    return y;
  }

  Mat forward_batch(const Mat & X)
  {
    const size_t n = X.size();
    last_x_batch_ = X;
    last_z_batch_.assign(n, Vec(out_dim_, 0.0f));
    Mat Y(n, Vec(out_dim_, 0.0f));
    for (size_t s = 0; s < n; ++s)
    {
      const Vec & x = X[s];
      for (int o = 0; o < out_dim_; ++o)
      {
        float z = b_[o];
        const Vec & w_row = W_[o];
        for (int i = 0; i < in_dim_; ++i)
        {
          z += w_row[i] * x[i];
        }
        last_z_batch_[s][o] = z;
        Y[s][o] = apply_activation(z, act_);
      }
    }
    last_y_batch_ = Y;
    return Y;
  }

  // dY: batch x out_dim, gradient of the loss wrt this layer's (post-activation)
  // output. Accumulates averaged weight/bias gradients internally (call
  // apply_adam() to actually take a step) and returns dX = gradient wrt this
  // layer's input, for the caller to chain into the previous layer.
  Mat backward_batch(const Mat & dY)
  {
    const size_t n = dY.size();
    Mat dX(n, Vec(in_dim_, 0.0f));
    gW_.assign(out_dim_, Vec(in_dim_, 0.0f));
    gb_.assign(out_dim_, 0.0f);

    for (size_t s = 0; s < n; ++s)
    {
      const Vec & x = last_x_batch_[s];
      const Vec & z = last_z_batch_[s];
      const Vec & y = last_y_batch_[s];
      for (int o = 0; o < out_dim_; ++o)
      {
        const float dz = dY[s][o] * activation_grad(y[o], z[o], act_);
        gb_[o] += dz;
        Vec & gw_row = gW_[o];
        const Vec & w_row = W_[o];
        for (int i = 0; i < in_dim_; ++i)
        {
          gw_row[i] += dz * x[i];
          dX[s][i] += dz * w_row[i];
        }
      }
    }
    const float inv_n = 1.0f / static_cast<float>(n);
    for (int o = 0; o < out_dim_; ++o)
    {
      gb_[o] *= inv_n;
      for (int i = 0; i < in_dim_; ++i)
      {
        gW_[o][i] *= inv_n;
      }
    }
    return dX;
  }

  void apply_adam(float lr, float beta1 = 0.9f, float beta2 = 0.999f, float eps = 1e-8f)
  {
    ++t_;
    const float bc1 = 1.0f - std::pow(beta1, static_cast<float>(t_));
    const float bc2 = 1.0f - std::pow(beta2, static_cast<float>(t_));
    for (int o = 0; o < out_dim_; ++o)
    {
      mb_[o] = beta1 * mb_[o] + (1 - beta1) * gb_[o];
      vb_[o] = beta2 * vb_[o] + (1 - beta2) * gb_[o] * gb_[o];
      b_[o] -= lr * (mb_[o] / bc1) / (std::sqrt(vb_[o] / bc2) + eps);
      for (int i = 0; i < in_dim_; ++i)
      {
        mW_[o][i] = beta1 * mW_[o][i] + (1 - beta1) * gW_[o][i];
        vW_[o][i] = beta2 * vW_[o][i] + (1 - beta2) * gW_[o][i] * gW_[o][i];
        W_[o][i] -= lr * (mW_[o][i] / bc1) / (std::sqrt(vW_[o][i] / bc2) + eps);
      }
    }
  }

  void soft_update_from(const DenseLayer & src, float tau)
  {
    for (int o = 0; o < out_dim_; ++o)
    {
      b_[o] = tau * src.b_[o] + (1.0f - tau) * b_[o];
      for (int i = 0; i < in_dim_; ++i)
      {
        W_[o][i] = tau * src.W_[o][i] + (1.0f - tau) * W_[o][i];
      }
    }
  }

  void copy_from(const DenseLayer & src)
  {
    W_ = src.W_;
    b_ = src.b_;
  }

  void write(std::ofstream & f) const
  {
    for (const auto & row : W_)
    {
      f.write(reinterpret_cast<const char *>(row.data()), row.size() * sizeof(float));
    }
    f.write(reinterpret_cast<const char *>(b_.data()), b_.size() * sizeof(float));
  }

  bool read(std::ifstream & f)
  {
    for (auto & row : W_)
    {
      f.read(reinterpret_cast<char *>(row.data()), row.size() * sizeof(float));
    }
    f.read(reinterpret_cast<char *>(b_.data()), b_.size() * sizeof(float));
    return static_cast<bool>(f);
  }

  int in_dim() const { return in_dim_; }
  int out_dim() const { return out_dim_; }

private:
  int in_dim_ = 0;
  int out_dim_ = 0;
  Activation act_ = Activation::kLinear;

  Mat W_;
  Vec b_;

  // Adam state.
  Mat mW_, vW_;
  Vec mb_, vb_;
  uint64_t t_ = 0;

  // Gradients accumulated by backward_batch(), consumed by apply_adam().
  Mat gW_;
  Vec gb_;

  // Caches for backward.
  Vec last_x_single_, last_z_single_;
  Mat last_x_batch_, last_z_batch_, last_y_batch_;
};

}  // namespace tiny_nn
