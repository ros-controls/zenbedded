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

// Helper for wrapping the pendulum angle between -PI and PI (0 is upright, +/- PI is face down)
static double wrap_angle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

CallbackReturn MockPendulumHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Load exact URDF physical constraints for the Furuta pendulum.
  // We provide these as defaults to act as a true simulation without strict XML parameter
  // dependency.
  m2_ = 0.014;  // Pendulum mass (14g)
  l2_ = 0.051;  // Pendulum COM drop (51mm)
  g_ = 9.81;
  b2_ = 0.0001;  // Small pivot friction

  // Calculate inertia about the pivot using the Parallel Axis Theorem (I_axis = I_xx + m*l^2)
  // I_xx = 0.00000806 kg*m^2 from the URDF. Resulting J2_ ≈ 4.447e-5 kg*m^2
  J2_ = 0.00000806 + (m2_ * l2_ * l2_);

  // Initial angle: 180 degrees (M_PI) represents hanging face down
  initial_q2_ = M_PI;

  // Attempt to override with provided ros2_control hardware parameters if available
  try
  {
    if (info_.hardware_parameters.count("m2"))
    {
      m2_ = std::stod(info_.hardware_parameters.at("m2"));
    }
    if (info_.hardware_parameters.count("l2"))
    {
      l2_ = std::stod(info_.hardware_parameters.at("l2"));
    }
    if (info_.hardware_parameters.count("b2"))
    {
      b2_ = std::stod(info_.hardware_parameters.at("b2"));
    }
    if (info_.hardware_parameters.count("g"))
    {
      g_ = std::stod(info_.hardware_parameters.at("g"));
    }
    if (info_.hardware_parameters.count("initial_q2"))
    {
      initial_q2_ = std::stod(info_.hardware_parameters.at("initial_q2"));
    }
  }
  catch (const std::out_of_range &)
  {
    RCLCPP_WARN(rclcpp::get_logger("MockPendulumHardware"), "Using default URDF exact parameters.");
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

  // URDF-specific kinematics & constraints
  const double r = 0.062;                 // Arm reach (arm_to_pendulum origin x-offset)
  const double motor_limit = 2.35619449;  // Mechanical hard stops (+-135 deg)
  const double max_velocity = 196.0;      // Firmware step-rate envelope

  // ---- True Furuta Pendulum Lagrangian dynamics ----
  // With angle 0 as upright, the pendulum experiences two primary torques:
  // 1. Gravity torque acting to pull it away from the upright equilibrium: + m2 * g * l2 * sin(q2)
  // 2. Fictitious force torque from the base arm's acceleration: - m2 * r * l2 * cos(q2) * q1_ddot

  const double sinq2 = std::sin(q2_);
  const double cosq2 = std::cos(q2_);

  const double q2_ddot =
    (m2_ * g_ * l2_ * sinq2 - m2_ * r * l2_ * cosq2 * q1_ddot_cmd - b2_ * q2_dot_) / J2_;

  // Semi-implicit Euler integration
  q1_dot_ += q1_ddot_cmd * dt;

  // Enforce firmware max velocity
  q1_dot_ = std::clamp(q1_dot_, -max_velocity, max_velocity);

  q1_ += q1_dot_ * dt;

  // Enforce firmware hard stops for motor position
  if (q1_ > motor_limit)
  {
    q1_ = motor_limit;
    q1_dot_ = 0.0;  // Crash into hardstop zeroes out velocity
  }
  else if (q1_ < -motor_limit)
  {
    q1_ = -motor_limit;
    q1_dot_ = 0.0;
  }

  q2_dot_ += q2_ddot * dt;
  q2_ += q2_dot_ * dt;

  // Wrap pendulum angle continuously between -pi and pi
  q2_ = wrap_angle(q2_);

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
