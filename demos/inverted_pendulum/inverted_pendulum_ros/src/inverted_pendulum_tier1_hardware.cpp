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

#include "inverted_pendulum_tier1_hardware/inverted_pendulum_tier1_hardware.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace zenbedded_inverted_pendulum
{

CallbackReturn InvertedPendulumTier1Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto & hw_params = info_.hardware_parameters;
  if (const auto it = hw_params.find("state_topic"); it != hw_params.end())
  {
    state_topic_ = it->second;
  }
  if (const auto it = hw_params.find("command_topic"); it != hw_params.end())
  {
    command_topic_ = it->second;
  }

  hw_states_.assign(4, 0.0);
  hw_commands_.assign(1, 0.0);

  state_interfaces_.clear();
  state_interfaces_.reserve(4);

  // Index 0: motor_joint/position
  state_interfaces_.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      "motor_joint", "position", &hw_states_[0]));
  // Index 1: motor_joint/velocity
  state_interfaces_.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      "motor_joint", "velocity", &hw_states_[1]));
  // Index 2: pendulum_joint/position
  state_interfaces_.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      "pendulum_joint", "position", &hw_states_[2]));
  // Index 3: pendulum_joint/velocity
  state_interfaces_.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      "pendulum_joint", "velocity", &hw_states_[3]));

  command_interfaces_.clear();
  command_interfaces_.reserve(1);
  // Index 0: motor_joint/acceleration
  command_interfaces_.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      "motor_joint", "acceleration", &hw_commands_[0]));

  RCLCPP_INFO(
    rclcpp::get_logger("InvertedPendulumTier1Hardware"),
    "InvertedPendulumTier1Hardware initialized: state_topic='%s', command_topic='%s'",
    state_topic_.c_str(), command_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
InvertedPendulumTier1Hardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> out;
  out.reserve(state_interfaces_.size());
  for (const auto & si : state_interfaces_)
  {
    out.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(si));
  }
  return out;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
InvertedPendulumTier1Hardware::on_export_command_interfaces()
{
  return command_interfaces_;
}

void InvertedPendulumTier1Hardware::topic_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "motor_joint")
    {
      if (i < msg->position.size())
      {
        latest_q1_ = msg->position[i];
      }
      if (i < msg->velocity.size())
      {
        latest_q1_dot_ = msg->velocity[i];
      }
    }
    else if (msg->name[i] == "pendulum_joint")
    {
      if (i < msg->position.size())
      {
        latest_q2_ = msg->position[i];
      }
      if (i < msg->velocity.size())
      {
        latest_q2_dot_ = msg->velocity[i];
      }
    }
  }
  has_received_state_.store(true, std::memory_order_relaxed);
}

CallbackReturn InvertedPendulumTier1Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  latest_q1_ = 0.0;
  latest_q1_dot_ = 0.0;
  latest_q2_ = 0.0;
  latest_q2_dot_ = 0.0;
  has_received_state_.store(false, std::memory_order_relaxed);

  hw_states_.assign(4, 0.0);
  hw_commands_.assign(1, 0.0);

  node_ = std::make_shared<rclcpp::Node>("inverted_pendulum_tier1_hw");

  rclcpp::SensorDataQoS qos;
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos,
    std::bind(&InvertedPendulumTier1Hardware::topic_callback, this, std::placeholders::_1));

  cmd_pub_ = node_->create_publisher<control_msgs::msg::JointCommand>(
    command_topic_, rclcpp::SystemDefaultsQoS());

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(
    rclcpp::get_logger("InvertedPendulumTier1Hardware"),
    "InvertedPendulumTier1Hardware activated on topics '%s' and '%s'", state_topic_.c_str(),
    command_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn InvertedPendulumTier1Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (cmd_pub_)
  {
    control_msgs::msg::JointCommand msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base";
    msg.joint_names = {"motor_joint"};
    msg.interface_name = "acceleration";
    msg.values = {0.0};
    cmd_pub_->publish(msg);
  }

  if (executor_)
  {
    executor_->cancel();
  }
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }

  state_sub_.reset();
  cmd_pub_.reset();
  executor_.reset();
  node_.reset();

  RCLCPP_INFO(
    rclcpp::get_logger("InvertedPendulumTier1Hardware"),
    "InvertedPendulumTier1Hardware deactivated");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type InvertedPendulumTier1Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (has_received_state_.load(std::memory_order_relaxed))
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hw_states_[0] = latest_q1_;
    hw_states_[1] = latest_q1_dot_;
    hw_states_[2] = latest_q2_;
    hw_states_[3] = latest_q2_dot_;
  }

  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    std::ignore = state_interfaces_[i]->set_value(hw_states_[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InvertedPendulumTier1Hardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!cmd_pub_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!command_interfaces_.empty())
  {
    const auto opt = command_interfaces_[0]->get_optional<double>();
    if (opt.has_value() && !std::isnan(*opt))
    {
      hw_commands_[0] = *opt;
    }
  }

  control_msgs::msg::JointCommand msg;
  msg.header.stamp = time;
  msg.header.frame_id = "base";
  msg.joint_names = {"motor_joint"};
  msg.interface_name = "acceleration";
  msg.values = {hw_commands_[0]};

  cmd_pub_->publish(msg);

  return hardware_interface::return_type::OK;
}

}  // namespace zenbedded_inverted_pendulum

PLUGINLIB_EXPORT_CLASS(
  zenbedded_inverted_pendulum::InvertedPendulumTier1Hardware, hardware_interface::SystemInterface)
