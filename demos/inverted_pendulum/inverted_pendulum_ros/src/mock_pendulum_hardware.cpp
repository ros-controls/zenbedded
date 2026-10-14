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
#include <memory>
#include <utility>

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

  // Build interface handles — no raw pointers; values set via set_value() in read/write
  state_interfaces_.clear();
  const std::array<std::pair<const char *, const char *>, 4> state_specs = {
    std::make_pair("motor_joint", "position"),
    std::make_pair("motor_joint", "velocity"),
    std::make_pair("pendulum_joint", "position"),
    std::make_pair("pendulum_joint", "velocity"),
  };
  for (const auto & [component, iface] : state_specs)
  {
    auto si = std::make_shared<hardware_interface::StateInterface>(component, iface);
    std::ignore = si->set_value(0.0);
    state_interfaces_.push_back(si);
  }

  command_interfaces_.clear();
  auto ci = std::make_shared<hardware_interface::CommandInterface>("motor_joint", "acceleration");
  std::ignore = ci->set_value(0.0);
  command_interfaces_.push_back(ci);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
MockPendulumHardware::on_export_state_interfaces()
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
MockPendulumHardware::on_export_command_interfaces()
{
  return command_interfaces_;
}

CallbackReturn MockPendulumHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  q1_ = 0.0;
  q1_dot_ = 0.0;
  q2_ = initial_q2_;
  q2_dot_ = 0.0;
  hw_commands_[0] = 0.0;

  std::ignore = state_interfaces_[0]->set_value(q1_);
  std::ignore = state_interfaces_[1]->set_value(q1_dot_);
  std::ignore = state_interfaces_[2]->set_value(q2_);
  std::ignore = state_interfaces_[3]->set_value(q2_dot_);
  std::ignore = command_interfaces_[0]->set_value(0.0);

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
  const auto cmd_opt = command_interfaces_[0]->get_optional<double>();
  if (cmd_opt.has_value() && !std::isnan(*cmd_opt))
  {
    hw_commands_[0] = *cmd_opt;
  }
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

  // Publish all 4 state interfaces via handles
  std::ignore = state_interfaces_[0]->set_value(q1_);
  std::ignore = state_interfaces_[1]->set_value(q1_dot_);
  std::ignore = state_interfaces_[2]->set_value(q2_);
  std::ignore = state_interfaces_[3]->set_value(q2_dot_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockPendulumHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The commanded acceleration is read from the command interface handle in read().
  // Nothing to do here.
  return hardware_interface::return_type::OK;
}

}  // namespace mock_pendulum_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mock_pendulum_hardware::MockPendulumHardware, hardware_interface::SystemInterface)
