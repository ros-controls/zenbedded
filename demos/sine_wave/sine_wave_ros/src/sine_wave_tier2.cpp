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

#include "sine_wave_controller/sine_wave_tier2_controller.hpp"

#include <cmath>

namespace sine_wave_controller
{

SineWaveTier2Controller::SineWaveTier2Controller() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn SineWaveTier2Controller::on_init()
{
  try
  {
    auto_declare<std::string>("component_name", "sine_wave");
    auto_declare<std::string>("cmd_interface_name", "amplitude");
    auto_declare<double>("amplitude", 5.0);
    auto_declare<double>("frequency", 2.0);
    auto_declare<double>("offset", 0.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SineWaveTier2Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(component_name_ + "/" + cmd_interface_name_);
  return config;
}

controller_interface::InterfaceConfiguration
SineWaveTier2Controller::state_interface_configuration() const
{
  // Open-loop sine command needs no state feedback.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn SineWaveTier2Controller::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  component_name_ = get_node()->get_parameter("component_name").as_string();
  cmd_interface_name_ = get_node()->get_parameter("cmd_interface_name").as_string();
  amplitude_ = get_node()->get_parameter("amplitude").as_double();
  frequency_ = get_node()->get_parameter("frequency").as_double();
  offset_ = get_node()->get_parameter("offset").as_double();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured sine wave cmd output on '%s/%s': amplitude=%.3f frequency=%.3f offset=%.3f",
    component_name_.c_str(), cmd_interface_name_.c_str(), amplitude_, frequency_, offset_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineWaveTier2Controller::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset the phase clock every time the controller is (re)activated.
  start_time_set_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineWaveTier2Controller::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SineWaveTier2Controller::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!start_time_set_)
  {
    start_time_ = time;
    start_time_set_ = true;
  }

  const double t = (time - start_time_).seconds();
  const double value = offset_ + amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);

  if (!command_interfaces_[0].set_value(value))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(),
      1000,  // Log at most once every 1000ms to avoid flooding logs
      "Failed to set command interface value!");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace sine_wave_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sine_wave_controller::SineWaveTier2Controller, controller_interface::ControllerInterface)
