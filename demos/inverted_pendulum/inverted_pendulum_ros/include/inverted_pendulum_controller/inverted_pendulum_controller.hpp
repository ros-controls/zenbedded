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

#ifndef INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_
#define INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace inverted_pendulum_controller
{
/// Feedback controller for a Furuta (rotary) inverted pendulum.
///
/// Reads pendulum position and velocity from state interfaces, outputs a
/// motor joint acceleration command via PD control on the pendulum angle.
class InvertedPendulumController : public controller_interface::ControllerInterface
{
public:
  InvertedPendulumController();

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

protected:
  // Interface names (read in on_configure)
  std::string motor_joint_state_name_;
  std::string motor_joint_vel_name_;
  std::string pendulum_joint_state_name_;
  std::string pendulum_joint_vel_name_;
  std::string motor_joint_command_name_;

  // Control parameters
  double balance_angle_;
  double kp_;
  double kd_;
  double k_motor_pos_;
  double k_motor_vel_;
  double max_acceleration_;
};

}  // namespace inverted_pendulum_controller

#endif  // INVERTED_PENDULUM_CONTROLLER__INVERTED_PENDULUM_CONTROLLER_HPP_
