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

#include "mock_pendulum_hardware/mock_pendulum_hardware.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mock_pendulum_hardware
{

CallbackReturn MockPendulumHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    J1_ = std::stod(info_.hardware_parameters.at("J1"));
    J2_ = std::stod(info_.hardware_parameters.at("J2"));
    m2_ = std::stod(info_.hardware_parameters.at("m2"));
    l2_ = std::stod(info_.hardware_parameters.at("l2"));
    b1_ = std::stod(info_.hardware_parameters.at("b1"));
    b2_ = std::stod(info_.hardware_parameters.at("b2"));
    g_ = std::stod(info_.hardware_parameters.at("g"));
    initial_q2_ = std::stod(info_.hardware_parameters.at("initial_q2"));
  }
  catch (const std::out_of_range &)
  {
    RCLCPP_FATAL(rclcpp::get_logger("MockPendulumHardware"), "Missing required URDF parameter");
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(4, 0.0);
  hw_commands_.resize(1, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MockPendulumHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back("motor_joint", "position", &hw_states_[0]);
  interfaces.emplace_back("motor_joint", "velocity", &hw_states_[1]);
  interfaces.emplace_back("pendulum_joint", "position", &hw_states_[2]);
  interfaces.emplace_back("pendulum_joint", "velocity", &hw_states_[3]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MockPendulumHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back("motor_joint", "acceleration", &hw_commands_[0]);
  return interfaces;
}

CallbackReturn MockPendulumHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  q1_ = 0.0;
  q1_dot_ = 0.0;
  q2_ = initial_q2_;
  q2_dot_ = 0.0;

  hw_states_[0] = q1_;
  hw_states_[1] = q1_dot_;
  hw_states_[2] = q2_;
  hw_states_[3] = q2_dot_;
  hw_commands_[0] = 0.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn MockPendulumHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MockPendulumHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0)
  {
    return hardware_interface::return_type::OK;
  }

  // Commanded acceleration applied directly to motor joint
  const double q1_ddot_cmd = hw_commands_[0];

  // ---- Coupled 2-DOF Lagrangian dynamics ----
  //
  //   a11*q1'' + a12*q2'' = tau_motor - b1*q1' + J2*sin(q2)*q2'^2
  //   a21*q1'' + a22*q2'' = -b2*q2' - m2*g*l2*sin(q2)
  //
  // For acceleration command mode, tau_motor is whatever torque produces
  // the commanded q1_ddot. We substitute q1'' = q1_ddot_cmd and solve
  // equation (2) for q2''.
  //
  // From (2):  q2'' = (-b2*q2' - m2*g*l2*sin(q2) - J2*cos(q2)*q1_ddot_cmd) / J2

  const double sinq2 = std::sin(q2_);
  const double cosq2 = std::cos(q2_);

  // Solve for pendulum acceleration given commanded motor acceleration
  const double q2_ddot =
    (-b2_ * q2_dot_ - m2_ * g_ * l2_ * sinq2 - J2_ * cosq2 * q1_ddot_cmd) / J2_;

  // Semi-implicit Euler integration
  q1_dot_ += q1_ddot_cmd * dt;
  q1_ += q1_dot_ * dt;
  q2_dot_ += q2_ddot * dt;
  q2_ += q2_dot_ * dt;

  // Publish all 4 state interfaces
  hw_states_[0] = q1_;
  hw_states_[1] = q1_dot_;
  hw_states_[2] = q2_;
  hw_states_[3] = q2_dot_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockPendulumHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_commands_[0] holds the commanded motor acceleration set by the
  // controller via the command interface. Consumed in read().
  return hardware_interface::return_type::OK;
}

}  // namespace mock_pendulum_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mock_pendulum_hardware::MockPendulumHardware, hardware_interface::SystemInterface)
