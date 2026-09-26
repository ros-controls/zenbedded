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

// Single source of truth for the RL reward function - C++ port of the
// Python `reward.py` used by the sim trainer, kept deliberately identical
// in form (same term names, same weight names, same defaults) so the
// reward this controller optimizes for is provably the same function the
// sim policy was trained against, not a hand-re-derived approximation of it.
// If sim and controller ever need to diverge, do it by changing
// RewardWeights defaults (still readable as a diff against reward.py), not
// by hand-editing the cost expression in two places.
#pragma once

#include <cmath>

namespace inverted_pendulum_rl_controller
{

// Canonical Quanser quadratic cost weights + optional extras. Defaults
// match reward.py's 2026-05-16 canonical reward (5-term Quanser form) with
// the alive-offset and upright-alive-bonus extras from the 2026-07-21 audit
// (see compute_reward() below) turned on, since this controller has no
// separate "canonical vs. audited" mode to preserve - unlike reward.py it
// only needs to be the one true current version.
struct RewardWeights
{
  // Canonical Quanser quadratic-cost form: cost = sum of k * x^2. The
  // pendulum-angle term itself has implicit weight 1.0 - everything else is
  // weighted relative to it.
  float k_pen_vel = 0.02f;    // pendulum angular velocity (theta_dot) penalty
  float k_motor_pos = 0.8f;   // motor/arm position (alpha) penalty - centering
  float k_motor_vel = 0.01f;  // motor angular velocity (alpha_dot) penalty
  float k_action = 0.05f;     // action magnitude penalty - effort

  // Extras (0.0 = disabled).
  float k_action_rate = 0.03f;  // penalty on (a_t - a_{t-1})^2 ("command jerk" / minimum effort)
  float k_motor_jerk = 0.0f;    // penalty on (alpha_dot_t - alpha_dot_{t-1})^2

  // Multiplicative stillness bonus: ADDS k * exp(-theta^2/sigma_theta^2) *
  // exp(-alpha_dot^2/sigma_motor_vel^2). Only large when BOTH the pendulum
  // is upright AND the motor has stopped moving - off by default, matching
  // reward.py; enable if the upright-alive bonus alone isn't enough to stop
  // a "spin gently forever" exploit.
  float k_stillness_bonus = 0.0f;
  float sigma_theta = 0.3f;      // bonus active within ~17 deg
  float sigma_motor_vel = 1.0f;  // full bonus only below ~1 rad/s

  // Per-step ALIVE OFFSET (reward.py audit finding F1): with an all-negative
  // quadratic cost, ending an episode early (e.g. by driving into the
  // motor-position hard limit, see is_position_limit_violation() in the
  // controller) can be cheaper than living, which makes "crash into the
  // rail" a real local optimum under curriculum stages where balancing is
  // hard. A constant per-step offset >= the worst realistic per-step cost
  // makes the per-step reward non-negative, so terminating always forfeits
  // value instead of avoiding a cost.
  float k_alive_offset = 15.0f;

  // Velocity-gated UPRIGHT ALIVE bonus: ADDS k when |theta| <=
  // alive_theta_band AND |theta_dot| <= alive_pen_vel_limit. A pendulum
  // swinging *through* upright at speed earns nothing here, so spin-through
  // farming is unprofitable. Deliberately gated on the PENDULUM only (not
  // motor position) - centering pressure comes from k_motor_pos instead, so
  // this bonus stays a pure "is it actually balanced" signal.
  float k_upright_alive = 5.0f;
  float alive_theta_band = 0.2618f;  // 15 deg
  float alive_pen_vel_limit = 2.0f;  // rad/s
};

// Reward = alive_offset + upright_alive_bonus + stillness_bonus - quadratic_cost.
//
// Sign convention: theta = 0 is upright after wrapping to (-pi, pi], action
// in [-1, 1], motor_vel and pen_vel are SI angular velocity (rad/s).
inline float compute_reward(
  float theta, float pen_vel, float motor_pos, float motor_vel, float action, float prev_action,
  float prev_motor_vel, const RewardWeights & w)
{
  const float action_delta = action - prev_action;
  const float motor_vel_delta = motor_vel - prev_motor_vel;

  const float cost = theta * theta + w.k_pen_vel * pen_vel * pen_vel +
                     w.k_motor_pos * motor_pos * motor_pos + w.k_motor_vel * motor_vel * motor_vel +
                     w.k_action * action * action + w.k_action_rate * action_delta * action_delta +
                     w.k_motor_jerk * motor_vel_delta * motor_vel_delta;

  float bonus = 0.0f;
  if (w.k_stillness_bonus > 0.0f)
  {
    const float sigma_theta_sq = w.sigma_theta * w.sigma_theta;
    const float sigma_motor_vel_sq = w.sigma_motor_vel * w.sigma_motor_vel;
    const float upright_score = std::exp(-(theta * theta) / sigma_theta_sq);
    const float stillness_score = std::exp(-(motor_vel * motor_vel) / sigma_motor_vel_sq);
    bonus = w.k_stillness_bonus * upright_score * stillness_score;
  }

  float alive = w.k_alive_offset;
  if (
    w.k_upright_alive > 0.0f && std::abs(theta) <= w.alive_theta_band &&
    std::abs(pen_vel) <= w.alive_pen_vel_limit)
  {
    alive += w.k_upright_alive;
  }

  return alive + bonus - cost;
}

}  // namespace inverted_pendulum_rl_controller
