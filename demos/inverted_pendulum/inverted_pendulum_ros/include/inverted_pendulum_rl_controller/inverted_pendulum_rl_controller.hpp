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

#ifndef INVERTED_PENDULUM_RL_CONTROLLER__INVERTED_PENDULUM_RL_CONTROLLER_HPP_
#define INVERTED_PENDULUM_RL_CONTROLLER__INVERTED_PENDULUM_RL_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
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
//
// IMPORTANT (real-time budget): `update()` is called on the RT control thread
// at the controller_manager's update rate (typically 100 Hz / 10 ms budget).
// It must never itself run the DDPG gradient update - that involves batched
// forward/backward passes through two MLPs with heap-allocating Mat/Vec
// buffers, which is exactly the kind of variable-latency work that blows a
// 10 ms RT budget (allocator contention, page faults, etc). Training instead
// runs continuously on a dedicated background thread (see training_thread_
// / training_thread_main()); update() only ever does an O(1) inference
// forward pass and an O(state_dim) transition push, both against
// pre-sized buffers.
class InvertedPendulumRlController : public controller_interface::ControllerInterface
{
public:
  InvertedPendulumRlController();
  ~InvertedPendulumRlController() override;

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

  // Episode lifecycle: kRunning is the normal policy/training loop. kSettling
  // is entered once an episode ends: the controller commands zero
  // acceleration and waits for the system to actually stop moving (not just
  // for a step counter to roll over) before the next episode is allowed to
  // start, so every episode has a clean, comparable initial condition.
  enum class EpisodePhase
  {
    kRunning,
    kSettling
  };

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

  // -- "settle to rest between episodes" parameters --
  // An episode is only allowed to (re)start once both joint speeds have
  // stayed under settle_velocity_threshold_ for settle_hold_steps_
  // consecutive control cycles. settle_timeout_steps_ is a safety fallback
  // (e.g. sensor noise floor never quite reaches the threshold) so the
  // controller can't get stuck in kSettling forever.
  double settle_velocity_threshold_ = 0.05;  // rad/s
  int settle_hold_steps_ = 20;               // ~0.2 s @ 100 Hz
  int settle_timeout_steps_ = 1000;          // ~10 s @ 100 Hz safety fallback

  // -- mid-episode disturbance injection parameters --
  // Once the policy has held theta/alpha near the target for
  // min_stable_steps_before_perturbation_ consecutive steps, each further
  // step has perturbation_probability_ chance of injecting a short kick of
  // extra commanded acceleration (as a fraction of max_acceleration_) so the
  // agent has to learn to recover from a disturbance rather than only
  // learning to hold still from a fixed start. Training only - never active
  // during real-hardware inference.
  bool enable_perturbations_ = true;
  double perturbation_probability_ = 0.003;  // chance per step, once eligible
  double perturbation_magnitude_ = 0.6;      // fraction of max_acceleration_
  int perturbation_duration_steps_ = 4;
  int min_stable_steps_before_perturbation_ = 150;  // ~1.5 s @ 100 Hz

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
  // actor_ is the RT-thread's inference copy: update() only ever reads it
  // (under actor_infer_mutex_) via a single-sample forward(). It is kept in
  // sync by the background training thread copying actor_learn_ into it
  // periodically - update() never mutates it and never runs backprop on it.
  std::mt19937 rng_;        // RT thread only (noise / exploration / perturbations)
  std::mt19937 rng_train_;  // background training thread only (replay sampling)
  std::unique_ptr<tiny_nn::ActorNet> actor_;
  std::mutex actor_infer_mutex_;

  // Background-thread-only training state: never touched from update().
  std::unique_ptr<tiny_nn::ActorNet> actor_learn_;
  std::unique_ptr<tiny_nn::ActorNet> actor_target_;
  std::unique_ptr<tiny_nn::CriticNet> critic_;
  std::unique_ptr<tiny_nn::CriticNet> critic_target_;

  // Replay buffer: pushed to (cheaply) from the RT thread, sampled from the
  // background training thread. Guarded by replay_mutex_; critical sections
  // on both sides are kept to a single push/sample call, no network math.
  std::unique_ptr<tiny_nn::ReplayBuffer> replay_buffer_;
  std::mutex replay_mutex_;

  std::thread training_thread_;
  std::atomic_bool training_thread_should_run_{false};

  std::deque<std::array<float, kFeaturesPerStep>> state_history_;
  std::vector<float> last_flat_state_;
  bool has_last_state_ = false;
  float prev_action_ = 0.0f;
  float prev_prev_action_ = 0.0f;  // for the action-smoothness ("minimum effort") reward term

  double current_episode_reward_ = 0.0;
  int episode_step_count_ = 0;
  std::vector<double> episode_rolling_rewards_;
  double best_rolling_avg_ = -std::numeric_limits<double>::infinity();

  EpisodePhase episode_phase_ = EpisodePhase::kRunning;
  int settle_stable_count_ = 0;
  int settle_step_count_ = 0;

  // Checkpoint file writes are disk I/O and must not happen on the RT
  // thread either (same reasoning as training). update() just raises a
  // request flag; the background training thread performs the actual
  // save (of actor_learn_, which it owns and needs no locking to read).
  enum class CheckpointRequest
  {
    kNone,
    kBest,
    kFinal
  };
  std::atomic<CheckpointRequest> checkpoint_request_{CheckpointRequest::kNone};

  int stable_step_count_ = 0;  // consecutive steps balanced+centered, for perturbation gating
  int perturbation_steps_remaining_ = 0;
  float current_perturbation_accel_ = 0.0f;  // fraction of max_acceleration_

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // -- helpers --
  static float wrap_angle(float angle);
  static float compute_reward(
    float theta, float theta_dot, float alpha, float alpha_dot, float action, float prev_action);
  static bool is_balanced(float theta, float theta_dot, float alpha);
  std::vector<float> flatten_history() const;
  static tiny_nn::Transition mirror_transition(const tiny_nn::Transition & t);
  void reset_episode_state();
  void start_training_thread();
  void stop_training_thread();
  void training_thread_main();
  bool save_actor_checkpoint_net(const tiny_nn::ActorNet & net, const std::string & suffix) const;
};

}  // namespace inverted_pendulum_rl_controller

#endif  // INVERTED_PENDULUM_RL_CONTROLLER__INVERTED_PENDULUM_RL_CONTROLLER_HPP_
