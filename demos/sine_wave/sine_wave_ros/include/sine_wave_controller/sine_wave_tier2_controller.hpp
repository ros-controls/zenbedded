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

#ifndef SINE_WAVE_CONTROLLER__SINE_WAVE_TIER2_CONTROLLER_HPP_
#define SINE_WAVE_CONTROLLER__SINE_WAVE_TIER2_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace sine_wave_controller
{
/**
 * @brief Open-loop Tier 2 controller that writes a sine wave command
 * to the ros2_control amplitude command interface.
 */
class SineWaveTier2Controller : public controller_interface::ControllerInterface
{
public:
  SineWaveTier2Controller();

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
  std::string component_name_;
  std::string cmd_interface_name_;
  double amplitude_;
  double frequency_;  // Hz
  double offset_;

  rclcpp::Time start_time_;
  bool start_time_set_ = false;
};

}  // namespace sine_wave_controller

#endif  // SINE_WAVE_CONTROLLER__SINE_WAVE_TIER2_CONTROLLER_HPP_
