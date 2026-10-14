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

#ifndef MOCK_PENDULUM_HARDWARE__MOCK_PENDULUM_HARDWARE_HPP_
#define MOCK_PENDULUM_HARDWARE__MOCK_PENDULUM_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mock_pendulum_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// In-process mock hardware that simulates Furuta pendulum dynamics.
///
/// State interfaces:
///   motor_joint/position     (rad)
///   motor_joint/velocity     (rad/s)
///   pendulum_joint/position  (rad)
///   pendulum_joint/velocity  (rad/s)
///
/// Command interfaces:
///   motor_joint/acceleration (rad/s^2)
///
/// The commanded acceleration is applied directly to the motor joint.
/// Coupled 2-DOF Lagrangian dynamics determine the pendulum response.
class MockPendulumHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockPendulumHardware)

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // State: [motor/pos, motor/vel, pendulum/pos, pendulum/vel]
  std::vector<double> hw_states_;
  // Command: [motor/acceleration]
  std::vector<double> hw_commands_;

  // Dynamics state
  double q1_;      // motor joint angle (rad)
  double q1_dot_;  // motor joint velocity (rad/s)
  double q2_;      // pendulum angle from upright (rad)
  double q2_dot_;  // pendulum velocity (rad/s)

  // Physical parameters
  double J1_;  // motor joint inertia (kg*m^2)
  double J2_;  // pendulum inertia (kg*m^2)
  double m2_;  // pendulum mass (kg)
  double l2_;  // pendulum CoM distance (m)
  double b1_;  // motor joint damping (N*m*s/rad)
  double b2_;  // pendulum damping (N*m*s/rad)
  double g_;   // gravity (m/s^2)

  double initial_q2_;  // initial pendulum perturbation (rad)
};

}  // namespace mock_pendulum_hardware

#endif  // MOCK_PENDULUM_HARDWARE__MOCK_PENDULUM_HARDWARE_HPP_
