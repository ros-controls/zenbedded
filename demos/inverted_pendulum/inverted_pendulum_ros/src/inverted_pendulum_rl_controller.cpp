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
#include <cmath>
#include <filesystem>
#include <random>

namespace inverted_pendulum_rl_controller
{

InvertedPendulumRlController::InvertedPendulumRlController()
: controller_interface::ControllerInterface(), rng_(std::random_device{}())
{
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

    auto_declare<double>("max_acceleration", 50.0);
    auto_declare<double>("actor_lr", 1e-4);
    auto_declare<double>("critic_lr", 1e-3);
    auto_declare<double>("gamma", 0.99);
    auto_declare<double>("tau", 0.005);
    auto_declare<double>("exploration_noise_std", 0.1);
    auto_declare<double>("state_noise_std", 0.01);
    auto_declare<double>("reward_threshold", 18.0);
    auto_declare<int>("batch_size", 64);
    auto_declare<int>("min_buffer_size_to_train", 256);
    auto_declare<int>("replay_buffer_capacity", 100000);
    auto_declare<int>("episode_length", 500);
    auto_declare<bool>("mirror_augmentation", true);

    // Master switch: true (default) allocates critic/replay-buffer/optimizer
    // state and trains online, matching the simulation workflow. Set false
    // for real-hardware deployment so only the actor's forward pass runs.
    // Can also be flipped false at runtime via `ros2 param set` to freeze
    // learning without restarting the controller (it cannot be flipped back
    // to true at runtime unless it was true at configure time).
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
  state_noise_std_ = node->get_parameter("state_noise_std").as_double();
  reward_threshold_ = node->get_parameter("reward_threshold").as_double();
  batch_size_ = static_cast<int>(node->get_parameter("batch_size").as_int());
  min_buffer_size_to_train_ =
    static_cast<int>(node->get_parameter("min_buffer_size_to_train").as_int());
  replay_buffer_capacity_ =
    static_cast<int>(node->get_parameter("replay_buffer_capacity").as_int());
  episode_length_ = static_cast<int>(node->get_parameter("episode_length").as_int());
  mirror_augmentation_ = node->get_parameter("mirror_augmentation").as_bool();
  actor_weights_path_load_ = node->get_parameter("actor_weights_path").as_string();
  weights_save_dir_ = node->get_parameter("weights_save_dir").as_string();

  const bool enable_training_initial = node->get_parameter("enable_training").as_bool();
  enable_training_.store(enable_training_initial);

  actor_ = std::make_unique<tiny_nn::ActorNet>(rng_);

  if (!actor_weights_path_load_.empty())
  {
    if (actor_->load(actor_weights_path_load_))
    {
      RCLCPP_INFO(
        node->get_logger(), "Loaded actor weights from '%s'", actor_weights_path_load_.c_str());
    }
    else
    {
      RCLCPP_ERROR(
        node->get_logger(), "Failed to load actor weights from '%s'; starting from random init.",
        actor_weights_path_load_.c_str());
    }
  }

  if (enable_training_initial)
  {
    actor_target_ = std::make_unique<tiny_nn::ActorNet>(rng_);
    actor_target_->copy_from(*actor_);

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

controller_interface::CallbackReturn InvertedPendulumRlController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
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

  current_episode_reward_ = 0.0;
  episode_step_count_ = 0;
  episode_rolling_rewards_.clear();
  best_rolling_avg_ = -std::numeric_limits<double>::infinity();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InvertedPendulumRlController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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

float InvertedPendulumRlController::compute_reward(
  float theta, float theta_dot, float alpha, float alpha_dot, float action)
{
  float r =
    -(theta * theta + 0.001f * theta_dot * theta_dot + 0.5f * alpha * alpha +
      0.005f * alpha_dot * alpha_dot + 0.05f * action * action);
  r += 15.0f;  // Alive offset.

  // Upright alive bonus (15 deg = 0.2618 rad).
  if (std::abs(theta) <= 0.2618f && std::abs(theta_dot) <= 2.0f)
  {
    r += 5.0f;
  }
  return r;
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

void InvertedPendulumRlController::train_step()
{
  if (!training_infra_allocated_ || !replay_buffer_)
  {
    return;
  }
  if (replay_buffer_->size() < static_cast<size_t>(min_buffer_size_to_train_))
  {
    return;
  }

  const size_t batch_size = static_cast<size_t>(batch_size_);
  std::vector<std::vector<float>> states, next_states;
  std::vector<float> actions, rewards;
  replay_buffer_->sample(batch_size, rng_, states, actions, rewards, next_states);

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
  // Critic weights must NOT change here - only used to route a gradient back
  // into the actor (see CriticNet::backward_for_actor_only).
  const tiny_nn::Mat actor_actions_mat = actor_->forward_batch(states);
  const tiny_nn::Vec actor_actions = tiny_nn::col0(actor_actions_mat);
  critic_->forward_batch(states, actor_actions);  // refresh critic caches, updated weights

  const tiny_nn::Mat dQ_actor(batch_size, tiny_nn::Vec(1, -1.0f / static_cast<float>(batch_size)));
  const tiny_nn::Vec dAction_flat = critic_->backward_for_actor_only(dQ_actor);

  tiny_nn::Mat dAction(batch_size, tiny_nn::Vec(1, 0.0f));
  for (size_t i = 0; i < batch_size; ++i)
  {
    dAction[i][0] = dAction_flat[i];
  }
  actor_->backward_and_step(dAction, static_cast<float>(actor_lr_));

  // --- Soft-update target networks ---
  actor_target_->soft_update_from(*actor_, static_cast<float>(tau_));
  critic_target_->soft_update_from(*critic_, static_cast<float>(tau_));
}

bool InvertedPendulumRlController::save_actor_checkpoint(const std::string & suffix) const
{
  if (!actor_)
  {
    return false;
  }
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
  const bool ok = actor_->save(path);
  if (ok)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Saved actor checkpoint to '%s'", path.c_str());
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

  // Domain-randomization noise, training only (never perturbs real sensor
  // readings used for actual balancing/deployment).
  if (training)
  {
    std::normal_distribution<float> noise(0.0f, static_cast<float>(state_noise_std_));
    alpha += noise(rng_);
    theta += noise(rng_);
  }

  const std::array<float, kFeaturesPerStep> current_obs = {
    std::sin(theta), std::cos(theta), theta_dot, alpha, alpha_dot, prev_action_};

  state_history_.push_back(current_obs);
  while (state_history_.size() > static_cast<size_t>(kHistoryLen))
  {
    state_history_.pop_front();
  }

  const float reward = compute_reward(theta, theta_dot, alpha, alpha_dot, prev_action_);
  current_episode_reward_ += reward;
  ++episode_step_count_;

  const std::vector<float> flat_state = flatten_history();

  if (has_last_state_ && training && replay_buffer_)
  {
    tiny_nn::Transition transition{last_flat_state_, prev_action_, reward, flat_state};
    replay_buffer_->push(transition);
    if (mirror_augmentation_)
    {
      replay_buffer_->push(mirror_transition(transition));
    }
  }
  last_flat_state_ = flat_state;
  has_last_state_ = true;

  // Action selection - this single forward() call is the only thing that
  // runs on real hardware once training is disabled.
  float action = actor_->forward(flat_state);

  if (training)
  {
    std::normal_distribution<float> explore_noise(0.0f, static_cast<float>(exploration_noise_std_));
    action = std::clamp(action + explore_noise(rng_), -1.0f, 1.0f);
    train_step();
  }

  prev_action_ = action;

  if (!command_interfaces_[0].set_value(static_cast<double>(action) * max_acceleration_))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to set acceleration command interface value!");
    return controller_interface::return_type::ERROR;
  }

  if (episode_step_count_ >= episode_length_)
  {
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
      get_node()->get_logger(), "Episode avg reward: %.2f | Rolling(10): %.2f%s", avg_reward,
      rolling_avg, training ? "" : " [inference only]");

    if (training)
    {
      if (rolling_avg > reward_threshold_)
      {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Reward threshold (%.2f) exceeded - stopping learning and saving final weights.",
          reward_threshold_);
        enable_training_.store(false);
        save_actor_checkpoint("final");
      }
      else if (rolling_avg > best_rolling_avg_)
      {
        best_rolling_avg_ = rolling_avg;
        save_actor_checkpoint("best");
      }
    }

    current_episode_reward_ = 0.0;
    episode_step_count_ = 0;
  }

  return controller_interface::return_type::OK;
}

}  // namespace inverted_pendulum_rl_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  inverted_pendulum_rl_controller::InvertedPendulumRlController,
  controller_interface::ControllerInterface)
