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

#include "inverted_pendulum_controller/inverted_pendulum_controller.hpp"

#include <algorithm>
#include <cmath>

namespace inverted_pendulum_controller
{

InvertedPendulumController::InvertedPendulumController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn InvertedPendulumController::on_init()
{
  try
  {
    auto_declare<std::string>("motor_joint_state_name", "motor_joint/position");
    auto_declare<std::string>("motor_joint_vel_name", "motor_joint/velocity");
    auto_declare<std::string>("pendulum_joint_state_name", "pendulum_joint/position");
    auto_declare<std::string>("pendulum_joint_vel_name", "pendulum_joint/velocity");
    auto_declare<std::string>("motor_joint_command_name", "motor_joint/acceleration");
    auto_declare<double>("balance_angle", 0.0);
    auto_declare<double>("kp", 1.0);
    auto_declare<double>("kd", 0.1);
    auto_declare<double>("k_motor_pos", 0.0);
    auto_declare<double>("k_motor_vel", 0.0);
    auto_declare<double>("max_acceleration", 100.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
InvertedPendulumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(motor_joint_command_name_);
  return config;
}

controller_interface::InterfaceConfiguration
InvertedPendulumController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(motor_joint_state_name_);
  config.names.push_back(motor_joint_vel_name_);
  config.names.push_back(pendulum_joint_state_name_);
  config.names.push_back(pendulum_joint_vel_name_);
  return config;
}

controller_interface::CallbackReturn InvertedPendulumController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motor_joint_state_name_ = get_node()->get_parameter("motor_joint_state_name").as_string();
  motor_joint_vel_name_ = get_node()->get_parameter("motor_joint_vel_name").as_string();
  pendulum_joint_state_name_ = get_node()->get_parameter("pendulum_joint_state_name").as_string();
  pendulum_joint_vel_name_ = get_node()->get_parameter("pendulum_joint_vel_name").as_string();
  motor_joint_command_name_ = get_node()->get_parameter("motor_joint_command_name").as_string();
  balance_angle_ = get_node()->get_parameter("balance_angle").as_double();
  kp_ = get_node()->get_parameter("kp").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();
  k_motor_pos_ = get_node()->get_parameter("k_motor_pos").as_double();
  k_motor_vel_ = get_node()->get_parameter("k_motor_vel").as_double();
  max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured Furuta controller: balance_angle=%.3f kp=%.3f kd=%.3f max_accel=%.3f",
    balance_angle_, kp_, kd_, max_acceleration_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InvertedPendulumController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InvertedPendulumController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type InvertedPendulumController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // State interface ordering:
  //   [0] motor_joint/position
  //   [1] motor_joint/velocity
  //   [2] pendulum_joint/position
  //   [3] pendulum_joint/velocity
  const double motor_pos = state_interfaces_[0].get_optional().value_or(0.0);
  const double motor_vel = state_interfaces_[1].get_optional().value_or(0.0);
  const double pendulum_pos = state_interfaces_[2].get_optional().value_or(0.0);
  const double pendulum_vel = state_interfaces_[3].get_optional().value_or(0.0);

  double angle_error = balance_angle_ - pendulum_pos;

  // Convert 20 degrees to radians for the threshold
  const double activation_threshold_rad = 20.0 * M_PI / 180.0;

  double u = 0.0;

  // Only apply control effort if the pendulum is within ±20 degrees of the target
  if (std::abs(angle_error) <= activation_threshold_rad)
  {
    // Full-State Feedback Control
    u = (kp_ * angle_error) - (kd_ * pendulum_vel) - (k_motor_pos_ * motor_pos) -
        (k_motor_vel_ * motor_vel);

    u = std::clamp(u, -max_acceleration_, max_acceleration_);
  }

  if (!command_interfaces_[0].set_value(u))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to set acceleration command interface value!");
    return controller_interface::return_type::ERROR;
  }

  // Log the state and calculated output at 500ms intervals
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 500,
    "State: [m_pos: %.3f, m_vel: %.3f, p_pos: %.3f, p_vel: %.3f] | Err: %.3f | Cmd (accel): %.3f",
    motor_pos, motor_vel, pendulum_pos, pendulum_vel, angle_error, u);

  return controller_interface::return_type::OK;
}

}  // namespace inverted_pendulum_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  inverted_pendulum_controller::InvertedPendulumController,
  controller_interface::ControllerInterface)
