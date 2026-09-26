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

#include "inverted_pendulum_rl_controller/inverted_pendulum_rl_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <random>

namespace inverted_pendulum_rl_controller
{

InvertedPendulumRlController::InvertedPendulumRlController()
: controller_interface::ControllerInterface(),
  rng_(std::random_device{}()),
  rng_train_(std::random_device{}())
{
}

InvertedPendulumRlController::~InvertedPendulumRlController()
{
  // Defensive: make sure the background thread is never left running past
  // the controller object's lifetime (e.g. if on_deactivate was skipped).
  stop_training_thread();
}

controller_interface::CallbackReturn InvertedPendulumRlController::on_init()
{
  try
  {
    auto_declare<std::string>("motor_joint_state_name", "motor_joint/position");
    auto_declare<std::string>("motor_joint_vel_name", "motor_joint/velocity");
    auto_declare<std::string>("pendulum_joint_state_name", "pendulum_joint/position");
    auto_declare<std::string>("pendulum_joint_vel_name", "pendulum_joint/velocity");
    auto_declare<std::string>("motor_joint_command_name", "motor_joint/acceleration");

    auto_declare<double>("max_acceleration", 120.0);
    auto_declare<double>("actor_lr", 1e-4);
    auto_declare<double>("critic_lr", 1e-3);
    auto_declare<double>("gamma", 0.99);
    auto_declare<double>("tau", 0.005);

    // Decay exploration parameters
    auto_declare<double>("exploration_noise_std", 1.0);
    auto_declare<double>("exploration_noise_std_min", 0.01);
    auto_declare<double>("exploration_noise_decay", 0.995);

    auto_declare<double>("state_noise_std", 0.01);
    auto_declare<double>("reward_threshold", 18.0);
    auto_declare<int>("batch_size", 64);
    auto_declare<int>("min_buffer_size_to_train", 256);
    auto_declare<int>("replay_buffer_capacity", 100000);
    auto_declare<int>("episode_length", 500);
    auto_declare<bool>("mirror_augmentation", true);
    auto_declare<bool>("ignore_config_mismatch", false);

    // Observation velocity scaling to prevent network saturation
    auto_declare<double>("obs_vel_scale", 0.1);
    auto_declare<double>("obs_vel_clip", 2.0);

    // Cap background thread updates per environment step
    auto_declare<int>("train_updates_per_env_step", 1);

    // -- motor position hard safety limit --
    auto_declare<double>("motor_pos_limit_rad", 2.356194);  // 135 deg

    // -- reward shaping weights (see reward.hpp) --
    auto_declare<double>("reward.k_pen_vel", 0.02);
    auto_declare<double>("reward.k_motor_pos", 0.8);
    auto_declare<double>("reward.k_motor_vel", 0.01);
    auto_declare<double>("reward.k_action", 0.05);
    auto_declare<double>("reward.k_action_rate", 0.03);
    auto_declare<double>("reward.k_motor_jerk", 0.0);
    auto_declare<double>("reward.k_stillness_bonus", 0.0);
    auto_declare<double>("reward.sigma_theta", 0.3);
    auto_declare<double>("reward.sigma_motor_vel", 1.0);
    auto_declare<double>("reward.k_alive_offset", 15.0);
    auto_declare<double>("reward.k_upright_alive", 5.0);
    auto_declare<double>("reward.alive_theta_band", 0.2618);
    auto_declare<double>("reward.alive_pen_vel_limit", 2.0);

    // -- settle-to-rest between episodes --
    auto_declare<double>("settle_velocity_threshold", 0.05);
    auto_declare<int>("settle_hold_steps", 20);
    auto_declare<int>("settle_timeout_steps", 1000);

    // -- mid-episode disturbance injection (training only) --
    auto_declare<bool>("enable_perturbations", true);
    auto_declare<double>("perturbation_probability", 0.003);
    auto_declare<double>("perturbation_magnitude", 0.6);
    auto_declare<int>("perturbation_duration_steps", 4);
    auto_declare<int>("min_stable_steps_before_perturbation", 150);

    // Master switch: true (default) allocates critic/replay-buffer/optimizer
    // state and trains continuously on a background thread, matching the
    // simulation workflow. Set false for real-hardware deployment so only
    // the actor's forward pass runs. Can also be flipped false at runtime
    // via `ros2 param set` to freeze learning without restarting the
    // controller (it cannot be flipped back to true at runtime unless it
    // was true at configure time).
    auto_declare<bool>("enable_training", true);

    // If non-empty, loaded into the actor at on_configure - this is how a
    // checkpoint produced during simulation training gets deployed to the
    // real robot (or used to resume training from a warm start).
    auto_declare<std::string>("actor_weights_path", "");

    // Directory checkpoints are written to during training
    // (actor_best.bin whenever the rolling-average reward improves,
    // actor_final.bin once reward_threshold is exceeded and training stops).
    auto_declare<std::string>("weights_save_dir", "./furuta_rl_weights");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
InvertedPendulumRlController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(motor_joint_command_name_);
  return config;
}

controller_interface::InterfaceConfiguration
InvertedPendulumRlController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(motor_joint_state_name_);
  config.names.push_back(motor_joint_vel_name_);
  config.names.push_back(pendulum_joint_state_name_);
  config.names.push_back(pendulum_joint_vel_name_);
  return config;
}

controller_interface::CallbackReturn InvertedPendulumRlController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Defensive: if we're being reconfigured without a clean deactivate in
  // between, make sure no background thread is still touching the networks
  // we're about to reallocate below.
  stop_training_thread();

  auto node = get_node();

  motor_joint_state_name_ = node->get_parameter("motor_joint_state_name").as_string();
  motor_joint_vel_name_ = node->get_parameter("motor_joint_vel_name").as_string();
  pendulum_joint_state_name_ = node->get_parameter("pendulum_joint_state_name").as_string();
  pendulum_joint_vel_name_ = node->get_parameter("pendulum_joint_vel_name").as_string();
  motor_joint_command_name_ = node->get_parameter("motor_joint_command_name").as_string();

  max_acceleration_ = node->get_parameter("max_acceleration").as_double();
  actor_lr_ = node->get_parameter("actor_lr").as_double();
  critic_lr_ = node->get_parameter("critic_lr").as_double();
  gamma_ = node->get_parameter("gamma").as_double();
  tau_ = node->get_parameter("tau").as_double();

  exploration_noise_std_ = node->get_parameter("exploration_noise_std").as_double();
  exploration_noise_std_min_ = node->get_parameter("exploration_noise_std_min").as_double();
  exploration_noise_decay_ = node->get_parameter("exploration_noise_decay").as_double();

  state_noise_std_ = node->get_parameter("state_noise_std").as_double();
  reward_threshold_ = node->get_parameter("reward_threshold").as_double();
  batch_size_ = static_cast<int>(node->get_parameter("batch_size").as_int());
  min_buffer_size_to_train_ =
    static_cast<int>(node->get_parameter("min_buffer_size_to_train").as_int());
  replay_buffer_capacity_ =
    static_cast<int>(node->get_parameter("replay_buffer_capacity").as_int());
  episode_length_ = static_cast<int>(node->get_parameter("episode_length").as_int());
  mirror_augmentation_ = node->get_parameter("mirror_augmentation").as_bool();

  obs_vel_scale_ = node->get_parameter("obs_vel_scale").as_double();
  obs_vel_clip_ = node->get_parameter("obs_vel_clip").as_double();
  train_updates_per_env_step_ =
    static_cast<int>(node->get_parameter("train_updates_per_env_step").as_int());

  actor_weights_path_load_ = node->get_parameter("actor_weights_path").as_string();
  weights_save_dir_ = node->get_parameter("weights_save_dir").as_string();
  ignore_config_mismatch_ = node->get_parameter("ignore_config_mismatch").as_bool();
  motor_pos_limit_rad_ = node->get_parameter("motor_pos_limit_rad").as_double();

  reward_weights_.k_pen_vel =
    static_cast<float>(node->get_parameter("reward.k_pen_vel").as_double());
  reward_weights_.k_motor_pos =
    static_cast<float>(node->get_parameter("reward.k_motor_pos").as_double());
  reward_weights_.k_motor_vel =
    static_cast<float>(node->get_parameter("reward.k_motor_vel").as_double());
  reward_weights_.k_action = static_cast<float>(node->get_parameter("reward.k_action").as_double());
  reward_weights_.k_action_rate =
    static_cast<float>(node->get_parameter("reward.k_action_rate").as_double());
  reward_weights_.k_motor_jerk =
    static_cast<float>(node->get_parameter("reward.k_motor_jerk").as_double());
  reward_weights_.k_stillness_bonus =
    static_cast<float>(node->get_parameter("reward.k_stillness_bonus").as_double());
  reward_weights_.sigma_theta =
    static_cast<float>(node->get_parameter("reward.sigma_theta").as_double());
  reward_weights_.sigma_motor_vel =
    static_cast<float>(node->get_parameter("reward.sigma_motor_vel").as_double());
  reward_weights_.k_alive_offset =
    static_cast<float>(node->get_parameter("reward.k_alive_offset").as_double());
  reward_weights_.k_upright_alive =
    static_cast<float>(node->get_parameter("reward.k_upright_alive").as_double());
  reward_weights_.alive_theta_band =
    static_cast<float>(node->get_parameter("reward.alive_theta_band").as_double());
  reward_weights_.alive_pen_vel_limit =
    static_cast<float>(node->get_parameter("reward.alive_pen_vel_limit").as_double());

  settle_velocity_threshold_ = node->get_parameter("settle_velocity_threshold").as_double();
  settle_hold_steps_ = static_cast<int>(node->get_parameter("settle_hold_steps").as_int());
  settle_timeout_steps_ = static_cast<int>(node->get_parameter("settle_timeout_steps").as_int());

  enable_perturbations_ = node->get_parameter("enable_perturbations").as_bool();
  perturbation_probability_ = node->get_parameter("perturbation_probability").as_double();
  // uniform_real_distribution requires a <= b; a negative magnitude here
  // would flip that (mag(-neg, +neg) = mag(positive, negative)) and hit the
  // same class of libstdc++ assertion-abort as the normal_distribution
  // stddev issue above. Clamp defensively rather than trusting the param.
  perturbation_magnitude_ = std::abs(node->get_parameter("perturbation_magnitude").as_double());
  perturbation_duration_steps_ =
    static_cast<int>(node->get_parameter("perturbation_duration_steps").as_int());
  min_stable_steps_before_perturbation_ =
    static_cast<int>(node->get_parameter("min_stable_steps_before_perturbation").as_int());

  const bool enable_training_initial = node->get_parameter("enable_training").as_bool();
  enable_training_.store(enable_training_initial);

  actor_ = std::make_unique<tiny_nn::ActorNet>(rng_);

  if (!actor_weights_path_load_.empty())
  {
    // Provenance check BEFORE load(): a state_dim/history_len mismatch is
    // already caught by load() itself (actor_critic.hpp's format-version
    // guard) and falls back to random init below, but this check also
    // catches non-shape mismatches load() can't see (action scaling,
    // reward shape) and gives a specific, actionable message instead of a
    // bare "load failed". Mirrors run_config.py's check_config().
    const auto saved_config = run_config::find_run_config(actor_weights_path_load_);
    if (saved_config)
    {
      const auto result = run_config::check(*saved_config, build_run_config());
      if (!result.ok())
      {
        const std::string msg =
          "config mismatch against '" + actor_weights_path_load_ + "'s recorded run_config:";
        if (ignore_config_mismatch_)
        {
          RCLCPP_WARN(node->get_logger(), "%s", msg.c_str());
          for (const auto & line : result.mismatches)
          {
            RCLCPP_WARN(node->get_logger(), "%s", line.c_str());
          }
          RCLCPP_WARN(node->get_logger(), "ignore_config_mismatch=true; continuing anyway.");
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
          for (const auto & line : result.mismatches)
          {
            RCLCPP_ERROR(node->get_logger(), "%s", line.c_str());
          }
          RCLCPP_ERROR(
            node->get_logger(),
            "Fix parameters to match the checkpoint's config, or set "
            "ignore_config_mismatch:=true to load anyway.");
          return controller_interface::CallbackReturn::ERROR;
        }
      }
    }

    if (actor_->load(actor_weights_path_load_))
    {
      RCLCPP_INFO(
        node->get_logger(), "Loaded actor weights from '%s'", actor_weights_path_load_.c_str());
    }
    else
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to load actor weights from '%s' (missing file, or a shape/format-version "
        "mismatch - see actor_critic.hpp); starting from random init.",
        actor_weights_path_load_.c_str());
    }
  }

  if (enable_training_initial)
  {
    // actor_learn_ is the background thread's working copy; it starts as an
    // exact copy of the (possibly checkpoint-loaded) inference actor so
    // training resumes from the same point that was just loaded.
    actor_learn_ = std::make_unique<tiny_nn::ActorNet>(rng_);
    actor_learn_->copy_from(*actor_);

    actor_target_ = std::make_unique<tiny_nn::ActorNet>(rng_);
    actor_target_->copy_from(*actor_learn_);

    critic_ = std::make_unique<tiny_nn::CriticNet>(rng_);
    critic_target_ = std::make_unique<tiny_nn::CriticNet>(rng_);
    critic_target_->copy_from(*critic_);

    replay_buffer_ =
      std::make_unique<tiny_nn::ReplayBuffer>(static_cast<size_t>(replay_buffer_capacity_));

    training_infra_allocated_ = true;
    RCLCPP_INFO(
      node->get_logger(), "Training enabled: critic + replay buffer (capacity %d) allocated.",
      replay_buffer_capacity_);
  }
  else
  {
    actor_learn_.reset();
    actor_target_.reset();
    critic_.reset();
    critic_target_.reset();
    replay_buffer_.reset();
    training_infra_allocated_ = false;
    RCLCPP_INFO(node->get_logger(), "Training disabled: running actor-only inference.");
  }

  // Allow flipping "enable_training" off (always) or back on (only if it was
  // on at configure time, since that's what decided whether the critic /
  // replay buffer exist) without restarting the controller.
  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params)
      {
        if (p.get_name() == "enable_training")
        {
          const bool requested = p.as_bool();
          if (requested && !training_infra_allocated_)
          {
            result.successful = false;
            result.reason =
              "Cannot enable training at runtime: this controller instance was configured "
              "with enable_training=false, so no critic/replay buffer was allocated. "
              "Reconfigure (deactivate/configure) with enable_training:=true instead.";
          }
          else
          {
            enable_training_.store(requested);
            RCLCPP_INFO(
              get_node()->get_logger(), "enable_training set to %s", requested ? "true" : "false");
          }
        }
      }
      return result;
    });

  RCLCPP_INFO(
    node->get_logger(), "Configured NN Furuta RL controller (max_accel=%.2f, training=%s)",
    max_acceleration_, training_infra_allocated_ ? "on" : "off");

  return controller_interface::CallbackReturn::SUCCESS;
}

run_config::ConfigMap InvertedPendulumRlController::build_run_config() const
{
  // Enforced (checked by run_config::check()): change the observation/
  // action layout or the objective's meaning, so a checkpoint trained with
  // different values would behave nonsensically if loaded silently.
  // Provenance-only (see run_config.hpp's provenance_only_keys()): recorded
  // for reference but never blocks a load.
  run_config::ConfigMap config;
  config["state_dim"] = std::to_string(tiny_nn::kStateDim);
  config["history_len"] = std::to_string(kHistoryLen);
  config["features_per_step"] = std::to_string(kFeaturesPerStep);
  config["max_acceleration"] = std::to_string(max_acceleration_);
  config["motor_pos_limit_rad"] = std::to_string(motor_pos_limit_rad_);
  config["mirror_augmentation"] = mirror_augmentation_ ? "true" : "false";
  config["gamma"] = std::to_string(gamma_);
  config["tau"] = std::to_string(tau_);
  config["actor_lr"] = std::to_string(actor_lr_);
  config["critic_lr"] = std::to_string(critic_lr_);
  config["obs_vel_scale"] = std::to_string(obs_vel_scale_);
  config["obs_vel_clip"] = std::to_string(obs_vel_clip_);
  return config;
}

void InvertedPendulumRlController::reset_episode_state()
{
  state_history_.clear();
  const std::array<float, kFeaturesPerStep> zero_step = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for (int i = 0; i < kHistoryLen; ++i)
  {
    state_history_.push_back(zero_step);
  }

  has_last_state_ = false;
  last_flat_state_.clear();
  prev_action_ = 0.0f;
  prev_prev_action_ = 0.0f;
  prev_motor_vel_ = 0.0f;

  current_episode_reward_ = 0.0;
  episode_step_count_ = 0;

  stable_step_count_ = 0;
  perturbation_steps_remaining_ = 0;
  current_perturbation_accel_ = 0.0f;

  settle_stable_count_ = 0;
  settle_step_count_ = 0;
}

void InvertedPendulumRlController::start_training_thread()
{
  if (!training_infra_allocated_ || training_thread_.joinable())
  {
    return;
  }
  training_thread_should_run_.store(true);
  training_thread_ = std::thread(&InvertedPendulumRlController::training_thread_main, this);
}

void InvertedPendulumRlController::stop_training_thread()
{
  training_thread_should_run_.store(false);
  if (training_thread_.joinable())
  {
    training_thread_.join();
  }
}

controller_interface::CallbackReturn InvertedPendulumRlController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_episode_state();
  episode_rolling_rewards_.clear();
  best_rolling_avg_ = -std::numeric_limits<double>::infinity();
  episode_phase_ = EpisodePhase::kRunning;

  total_episode_count_ = 0;
  train_steps_available_.store(0);

  start_training_thread();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InvertedPendulumRlController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_training_thread();
  return controller_interface::CallbackReturn::SUCCESS;
}

float InvertedPendulumRlController::wrap_angle(float angle)
{
  while (angle > static_cast<float>(M_PI))
  {
    angle -= 2.0f * static_cast<float>(M_PI);
  }
  while (angle <= -static_cast<float>(M_PI))
  {
    angle += 2.0f * static_cast<float>(M_PI);
  }
  return angle;
}

bool InvertedPendulumRlController::is_balanced(float theta, float theta_dot, float alpha)
{
  // "Balanced" for the purposes of the alive bonus AND disturbance gating:
  // upright (15 deg) *and* centered (roughly 20 deg of arm travel), not just
  // upright - this is what stops the policy from happily drifting the arm
  // off-center as long as the pendulum itself stays vertical.
  constexpr float kUprightRad = 0.2618f;   // 15 deg
  constexpr float kCenteredRad = 0.3491f;  // 20 deg
  constexpr float kUprightRateMax = 2.0f;  // rad/s
  return std::abs(theta) <= kUprightRad && std::abs(theta_dot) <= kUprightRateMax &&
         std::abs(alpha) <= kCenteredRad;
}

std::vector<float> InvertedPendulumRlController::flatten_history() const
{
  std::vector<float> flat;
  flat.reserve(static_cast<size_t>(kHistoryLen) * kFeaturesPerStep);
  for (const auto & step : state_history_)
  {
    for (float v : step)
    {
      flat.push_back(v);
    }
  }
  return flat;
}

tiny_nn::Transition InvertedPendulumRlController::mirror_transition(const tiny_nn::Transition & t)
{
  // state layout: [sin, cos, theta_dot, alpha, alpha_dot, prev_action] * kHistoryLen
  // Mirror rule: sin(-x) = -sin(x), cos(-x) = cos(x); everything else that is
  // "signed w.r.t. rotation direction" flips along with the action.
  static const std::array<float, kFeaturesPerStep> flip = {-1.0f, 1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

  tiny_nn::Transition m;
  m.state = t.state;
  m.next_state = t.next_state;
  for (size_t i = 0; i < t.state.size(); ++i)
  {
    const float f = flip[i % kFeaturesPerStep];
    m.state[i] = t.state[i] * f;
    m.next_state[i] = t.next_state[i] * f;
  }
  m.action = -t.action;
  m.reward = t.reward;
  return m;
}

// Runs entirely on a dedicated background thread - NEVER on the RT control
// thread. This is where all of the heap-allocating batched forward/backward
// passes live. It only ever touches actor_learn_/actor_target_/critic_/
// critic_target_ (background-thread-only state) and replay_buffer_ (under
// replay_mutex_, brief critical sections only). The one place it talks to
// the RT thread is publishing a refreshed copy into actor_ under
// actor_infer_mutex_ so update() picks up newly-trained weights.
void InvertedPendulumRlController::training_thread_main()
{
  while (training_thread_should_run_.load())
  {
    if (!training_infra_allocated_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    // Service any pending checkpoint save even if training itself is
    // currently paused (e.g. right after crossing reward_threshold_,
    // enable_training_ is already false but the "final" checkpoint still
    // needs to be written). Disk I/O happens here, never on the RT thread.
    const CheckpointRequest req = checkpoint_request_.exchange(CheckpointRequest::kNone);
    if (req != CheckpointRequest::kNone)
    {
      save_actor_checkpoint_net(*actor_learn_, req == CheckpointRequest::kFinal ? "final" : "best");
    }

    if (!enable_training_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    if (train_steps_available_.load() <= 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    size_t buffer_size = 0;
    {
      std::lock_guard<std::mutex> lock(replay_mutex_);
      buffer_size = replay_buffer_->size();
    }
    if (buffer_size < static_cast<size_t>(min_buffer_size_to_train_))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    const size_t batch_size = static_cast<size_t>(batch_size_);
    std::vector<std::vector<float>> states, next_states;
    std::vector<float> actions, rewards;
    {
      std::lock_guard<std::mutex> lock(replay_mutex_);
      replay_buffer_->sample(batch_size, rng_train_, states, actions, rewards, next_states);
    }

    // --- Critic update: minimize MSE(Q(s,a), r + gamma * Q_target(s', pi_target(s'))) ---
    const tiny_nn::Vec next_actions = tiny_nn::col0(actor_target_->forward_batch(next_states));
    const tiny_nn::Mat target_q = critic_target_->forward_batch(next_states, next_actions);

    tiny_nn::Vec target_y(batch_size);
    for (size_t i = 0; i < batch_size; ++i)
    {
      target_y[i] = rewards[i] + static_cast<float>(gamma_) * target_q[i][0];
    }

    const tiny_nn::Mat current_q = critic_->forward_batch(states, actions);
    tiny_nn::Mat dQ(batch_size, tiny_nn::Vec(1, 0.0f));
    for (size_t i = 0; i < batch_size; ++i)
    {
      dQ[i][0] = 2.0f * (current_q[i][0] - target_y[i]) / static_cast<float>(batch_size);
    }
    critic_->backward_and_step(dQ, static_cast<float>(critic_lr_));

    // --- Actor update: maximize Q(s, pi(s)), i.e. minimize -mean(Q(s, pi(s))) ---
    // Critic weights must NOT change here - only used to route a gradient
    // back into the actor (see CriticNet::backward_for_actor_only).
    const tiny_nn::Mat actor_actions_mat = actor_learn_->forward_batch(states);
    const tiny_nn::Vec actor_actions = tiny_nn::col0(actor_actions_mat);
    critic_->forward_batch(states, actor_actions);  // refresh critic caches, updated weights

    const tiny_nn::Mat dQ_actor(
      batch_size, tiny_nn::Vec(1, -1.0f / static_cast<float>(batch_size)));
    const tiny_nn::Vec dAction_flat = critic_->backward_for_actor_only(dQ_actor);

    tiny_nn::Mat dAction(batch_size, tiny_nn::Vec(1, 0.0f));
    for (size_t i = 0; i < batch_size; ++i)
    {
      dAction[i][0] = dAction_flat[i];
    }
    actor_learn_->backward_and_step(dAction, static_cast<float>(actor_lr_));

    // --- Soft-update target networks ---
    actor_target_->soft_update_from(*actor_learn_, static_cast<float>(tau_));
    critic_target_->soft_update_from(*critic_, static_cast<float>(tau_));

    // --- Publish the freshly-trained actor to the RT thread's inference copy ---
    // copy_from() just does an element-wise copy into already-correctly-sized
    // buffers (no allocation), so this critical section is bounded and short
    // even though it's shared with the RT thread's forward() call.
    {
      std::lock_guard<std::mutex> lock(actor_infer_mutex_);
      actor_->copy_from(*actor_learn_);
    }

    train_steps_available_.fetch_sub(1);
  }
}

bool InvertedPendulumRlController::save_actor_checkpoint_net(
  const tiny_nn::ActorNet & net, const std::string & suffix) const
{
  try
  {
    std::filesystem::create_directories(weights_save_dir_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Could not create weights directory '%s': %s",
      weights_save_dir_.c_str(), e.what());
    return false;
  }

  const std::string path = weights_save_dir_ + "/actor_" + suffix + ".bin";
  const bool ok = net.save(path);
  if (ok)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Saved actor checkpoint to '%s'", path.c_str());
    // Sibling run_config, checked by whatever later loads this checkpoint
    // (see the on_configure() provenance check above).
    const std::string config_path = weights_save_dir_ + "/actor_" + suffix + ".config.txt";
    if (!run_config::save(config_path, build_run_config()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Failed to save run_config alongside '%s' (non-fatal)",
        path.c_str());
    }
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to save actor checkpoint to '%s'", path.c_str());
  }
  return ok;
}

controller_interface::return_type InvertedPendulumRlController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const bool training = enable_training_.load();

  // State interface ordering matches state_interface_configuration():
  //   [0] motor_joint/position   [1] motor_joint/velocity
  //   [2] pendulum_joint/position [3] pendulum_joint/velocity
  const double motor_pos = state_interfaces_[0].get_optional().value_or(0.0);
  const double motor_vel = state_interfaces_[1].get_optional().value_or(0.0);
  const double pendulum_pos = state_interfaces_[2].get_optional().value_or(0.0);
  const double pendulum_vel = state_interfaces_[3].get_optional().value_or(0.0);

  float theta = wrap_angle(static_cast<float>(pendulum_pos));
  float theta_dot = static_cast<float>(pendulum_vel);
  float alpha = static_cast<float>(motor_pos);
  float alpha_dot = static_cast<float>(motor_vel);

  // Clean (un-noised) motor position, kept separate from `alpha` below once
  // training noise is mixed in - hardware safety must never be fooled by
  // synthetic domain-randomization noise.
  const float alpha_raw = alpha;

  // --- Settling phase: wait for the system to actually stop moving before
  // the next episode is allowed to start. No policy/exploration/training
  // happens here - just a zero command and a velocity check. ---
  if (episode_phase_ == EpisodePhase::kSettling)
  {
    ++settle_step_count_;

    const bool at_rest = std::abs(theta_dot) <= static_cast<float>(settle_velocity_threshold_) &&
                         std::abs(alpha_dot) <= static_cast<float>(settle_velocity_threshold_);
    settle_stable_count_ = at_rest ? settle_stable_count_ + 1 : 0;

    if (!command_interfaces_[0].set_value(0.0))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set acceleration command interface value while settling!");
      return controller_interface::return_type::ERROR;
    }

    const bool settled = settle_stable_count_ >= settle_hold_steps_;
    const bool timed_out = settle_step_count_ >= settle_timeout_steps_;
    if (settled || timed_out)
    {
      if (timed_out && !settled)
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Settle timeout (%d steps) reached before reaching rest (|vel| <= %.3f); "
          "starting next episode anyway.",
          settle_timeout_steps_, settle_velocity_threshold_);
      }
      reset_episode_state();
      episode_phase_ = EpisodePhase::kRunning;
    }
    return controller_interface::return_type::OK;
  }

  // Domain-randomization noise, training only (never perturbs real sensor
  // readings used for actual balancing/deployment). std::normal_distribution
  // requires stddev > 0 (libstdc++ asserts on exactly 0, rather than just
  // degenerating to "always returns the mean" - it aborts the process), so
  // state_noise_std=0 has to be handled as "no noise", not "zero-width noise".
  if (training && state_noise_std_ > 0.0)
  {
    std::normal_distribution<float> noise(0.0f, static_cast<float>(state_noise_std_));
    alpha += noise(rng_);
    theta += noise(rng_);
  }

  const float scaled_theta_dot = std::clamp(
    theta_dot * static_cast<float>(obs_vel_scale_), -static_cast<float>(obs_vel_clip_),
    static_cast<float>(obs_vel_clip_));
  const float scaled_alpha_dot = std::clamp(
    alpha_dot * static_cast<float>(obs_vel_scale_), -static_cast<float>(obs_vel_clip_),
    static_cast<float>(obs_vel_clip_));

  const std::array<float, kFeaturesPerStep> current_obs = {
    std::sin(theta), std::cos(theta), scaled_theta_dot, alpha, scaled_alpha_dot, prev_action_};

  state_history_.push_back(current_obs);
  while (state_history_.size() > static_cast<size_t>(kHistoryLen))
  {
    state_history_.pop_front();
  }

  const float reward = compute_reward(
    theta, theta_dot, alpha, alpha_dot, prev_action_, prev_prev_action_, prev_motor_vel_,
    reward_weights_);
  prev_motor_vel_ = alpha_dot;
  current_episode_reward_ += reward;
  ++episode_step_count_;

  const std::vector<float> flat_state = flatten_history();

  if (has_last_state_ && training && replay_buffer_)
  {
    tiny_nn::Transition transition{last_flat_state_, prev_action_, reward, flat_state};
    std::lock_guard<std::mutex> lock(replay_mutex_);
    replay_buffer_->push(transition);
    if (mirror_augmentation_)
    {
      replay_buffer_->push(mirror_transition(transition));
    }
    train_steps_available_.fetch_add(train_updates_per_env_step_);
  }
  last_flat_state_ = flat_state;
  has_last_state_ = true;

  // Action selection - this single forward() call (and the brief lock around
  // it) is the only thing that runs on real hardware once training is
  // disabled. No training ever happens on this thread.
  float action;
  {
    std::lock_guard<std::mutex> lock(actor_infer_mutex_);
    action = actor_->forward(flat_state);
  }

  // Same std::normal_distribution stddev>0 requirement as the state-noise
  // block above - exploration_noise_std=0 ("pure exploitation, no explore
  // noise") is a normal thing to want and must not crash the RT thread.
  if (training && exploration_noise_std_ > 0.0)
  {
    std::normal_distribution<float> explore_noise(0.0f, static_cast<float>(exploration_noise_std_));
    action = std::clamp(action + explore_noise(rng_), -1.0f, 1.0f);
  }

  prev_prev_action_ = prev_action_;
  prev_action_ = action;

  // --- Mid-episode disturbance injection (training only): once the policy
  // has held balance for a while, occasionally kick the commanded
  // acceleration so it has to learn to recover, not just hold still. ---
  float disturbance_accel_frac = 0.0f;
  if (training)
  {
    stable_step_count_ = is_balanced(theta, theta_dot, alpha) ? stable_step_count_ + 1 : 0;

    if (
      enable_perturbations_ && perturbation_steps_remaining_ == 0 &&
      stable_step_count_ >= min_stable_steps_before_perturbation_)
    {
      std::uniform_real_distribution<float> trigger(0.0f, 1.0f);
      if (trigger(rng_) < static_cast<float>(perturbation_probability_))
      {
        std::uniform_real_distribution<float> mag(
          -static_cast<float>(perturbation_magnitude_),
          static_cast<float>(perturbation_magnitude_));
        current_perturbation_accel_ = mag(rng_);
        perturbation_steps_remaining_ = perturbation_duration_steps_;
        stable_step_count_ = 0;
        RCLCPP_INFO(
          get_node()->get_logger(), "Injecting disturbance: %.2f x max_acceleration for %d steps",
          current_perturbation_accel_, perturbation_duration_steps_);
      }
    }

    if (perturbation_steps_remaining_ > 0)
    {
      disturbance_accel_frac = current_perturbation_accel_;
      --perturbation_steps_remaining_;
    }
  }

  // Disturbance is allowed to push past the policy's own +/-1 action range
  // (that's the point - it has to feel like an external kick) but is still
  // bounded to a safety margin above max_acceleration_.
  double commanded_accel = std::clamp(
    static_cast<double>(action) * max_acceleration_ +
      static_cast<double>(disturbance_accel_frac) * max_acceleration_,
    -1.5 * max_acceleration_, 1.5 * max_acceleration_);

  // Hard motor-position safety clamp - active in BOTH training and
  // inference, independent of the RL policy: once the arm is at or past the
  // +-motor_pos_limit_rad_ travel limit, refuse to command further motion
  // in that direction (a residual command that would brake/reverse is still
  // allowed). Uses alpha_raw (the un-noised reading), never the
  // training-noised `alpha` used for reward/observations.
  if (alpha_raw >= static_cast<float>(motor_pos_limit_rad_) && commanded_accel > 0.0)
  {
    commanded_accel = 0.0;
  }
  else if (alpha_raw <= -static_cast<float>(motor_pos_limit_rad_) && commanded_accel < 0.0)
  {
    commanded_accel = 0.0;
  }

  if (!command_interfaces_[0].set_value(commanded_accel))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to set acceleration command interface value!");
    return controller_interface::return_type::ERROR;
  }

  if (episode_step_count_ >= episode_length_)
  {
    total_episode_count_++;

    const double avg_reward = current_episode_reward_ / static_cast<double>(episode_length_);
    episode_rolling_rewards_.push_back(avg_reward);
    if (episode_rolling_rewards_.size() > 10)
    {
      episode_rolling_rewards_.erase(episode_rolling_rewards_.begin());
    }

    double rolling_avg = 0.0;
    for (double v : episode_rolling_rewards_)
    {
      rolling_avg += v;
    }
    rolling_avg /= static_cast<double>(episode_rolling_rewards_.size());

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Episode: %d | Avg reward: %.2f | Rolling(10): %.2f | Noise: %.3f%s", total_episode_count_,
      avg_reward, rolling_avg, exploration_noise_std_, training ? "" : " [inference only]");

    if (training)
    {
      exploration_noise_std_ =
        std::max(exploration_noise_std_min_, exploration_noise_std_ * exploration_noise_decay_);

      if (rolling_avg > reward_threshold_)
      {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Reward threshold (%.2f) exceeded - stopping learning and saving final weights.",
          reward_threshold_);
        enable_training_.store(false);
        checkpoint_request_.store(CheckpointRequest::kFinal);
      }
      else if (rolling_avg > best_rolling_avg_)
      {
        best_rolling_avg_ = rolling_avg;
        checkpoint_request_.store(CheckpointRequest::kBest);
      }
    }

    // Don't immediately reset counters and start acting again next cycle -
    // hand off to the settling phase so the next episode starts from an
    // actual rest state instead of whatever the system happened to be doing
    // when the step counter rolled over.
    episode_phase_ = EpisodePhase::kSettling;
    settle_stable_count_ = 0;
    settle_step_count_ = 0;
  }

  return controller_interface::return_type::OK;
}

}  // namespace inverted_pendulum_rl_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  inverted_pendulum_rl_controller::InvertedPendulumRlController,
  controller_interface::ControllerInterface)
