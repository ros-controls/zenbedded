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

#include <array>
#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "inverted_pendulum_rl_controller/actor_critic.hpp"
#include "inverted_pendulum_rl_controller/replay_buffer.hpp"

namespace inverted_pendulum_rl_controller
{

// DDPG agent for the Inverted pendulum, it uses state/command interfaces as:
// motor_joint {position, velocity, acceleration(cmd)}, pendulum_joint {position, velocity}.
//
// Because that abstraction is hardware-agnostic, this controller is meant to
// be used unmodified in two deployments:
//   1. Simulation / training: spawned against mock_pendulum_hardware, with
//      "enable_training" true. Episode stats are logged, and the actor's
//      weights are checkpointed to disk as the rolling-average reward improves.
//   2. Real robot: spawned against ZenbeddedHardware (zenoh transport to the
//      Zephyr firmware), with "enable_training" false and "actor_weights_path"
//      pointing at a checkpoint produced by (1). In that mode only the actor
//      network runs (a single cheap forward pass per control cycle) - no
//      replay buffer, no critic, no gradient updates. The firmware itself
//      never sees the network; it only ever receives an acceleration setpoint.
class InvertedPendulumRlController : public controller_interface::ControllerInterface
{
public:
  InvertedPendulumRlController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr int kHistoryLen = 10;
  static constexpr int kFeaturesPerStep =
    6;  // [sin, cos, theta_dot, alpha, alpha_dot, prev_action]

  // -- interface names (parameters) --
  std::string motor_joint_state_name_;
  std::string motor_joint_vel_name_;
  std::string pendulum_joint_state_name_;
  std::string pendulum_joint_vel_name_;
  std::string motor_joint_command_name_;

  // -- RL / control hyperparameters (parameters) --
  double max_acceleration_ = 50.0;
  double actor_lr_ = 1e-4;
  double critic_lr_ = 1e-3;
  double gamma_ = 0.99;
  double tau_ = 0.005;
  double exploration_noise_std_ = 0.1;
  double state_noise_std_ = 0.01;
  double reward_threshold_ = 18.0;
  int batch_size_ = 64;
  int min_buffer_size_to_train_ = 256;
  int replay_buffer_capacity_ = 100000;
  int episode_length_ = 500;
  bool mirror_augmentation_ = true;
  std::string actor_weights_path_load_;
  std::string weights_save_dir_;

  // "enable_training" is re-read from the live parameter every cycle (and can
  // be flipped at runtime via `ros2 param set`), so it doubles as the
  // "stop learning" flag requested for deployment: flip it false and the
  // controller falls back to pure actor inference without a restart.
  std::atomic_bool enable_training_{true};
  // Whether critic/replay-buffer/optimizer state was allocated at configure
  // time. Real-hardware deployments should configure with enable_training
  // false from the start so this stays false and none of that memory/CPU is used.
  bool training_infra_allocated_ = false;

  // -- networks & training state --
  std::mt19937 rng_;
  std::unique_ptr<tiny_nn::ActorNet> actor_;
  std::unique_ptr<tiny_nn::ActorNet> actor_target_;
  std::unique_ptr<tiny_nn::CriticNet> critic_;
  std::unique_ptr<tiny_nn::CriticNet> critic_target_;
  std::unique_ptr<tiny_nn::ReplayBuffer> replay_buffer_;

  std::deque<std::array<float, kFeaturesPerStep>> state_history_;
  std::vector<float> last_flat_state_;
  bool has_last_state_ = false;
  float prev_action_ = 0.0f;

  double current_episode_reward_ = 0.0;
  int episode_step_count_ = 0;
  std::vector<double> episode_rolling_rewards_;
  double best_rolling_avg_ = -std::numeric_limits<double>::infinity();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // -- helpers --
  static float wrap_angle(float angle);
  static float compute_reward(
    float theta, float theta_dot, float alpha, float alpha_dot, float action);
  std::vector<float> flatten_history() const;
  static tiny_nn::Transition mirror_transition(const tiny_nn::Transition & t);
  void train_step();
  bool save_actor_checkpoint(const std::string & suffix) const;
};

}  // namespace inverted_pendulum_rl_controller
