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

#include "zenbedded_hardware_interface/zenbedded_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// #include "hardware_interface/introspection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zenbedded_hardware_interface/interface_schema.hpp"

namespace zenbedded
{

CallbackReturn ZenbeddedHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    zenoh_endpoint_ = info_.hardware_parameters.at("zenoh_endpoint");
    state_topic_ = info_.hardware_parameters.at("state_topic");
    command_topic_ = info_.hardware_parameters.at("command_topic");
    zenoh_mode_ = info_.hardware_parameters.count("zenoh_mode")
                    ? info_.hardware_parameters.at("zenoh_mode")
                    : "client";
  }
  catch (const std::out_of_range & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ZenbeddedHardware"), "Missing required URDF parameter");
    return CallbackReturn::ERROR;
  }

  // Load required schema
  std::string schema_path;
  try
  {
    schema_path = info_.hardware_parameters.at("schema_path");
  }
  catch (const std::out_of_range &)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("ZenbeddedHardware"), "Missing required URDF parameter: schema_path");
    return CallbackReturn::ERROR;
  }

  schema_ = InterfaceSchema::from_yaml_file(schema_path);
  if (!schema_.valid())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("ZenbeddedHardware"), "Schema error: %s", schema_.error().c_str());
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(schema_.total_state_interfaces(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(schema_.total_command_interfaces(), std::numeric_limits<double>::quiet_NaN());
  state_buffer_.set(std::vector<uint8_t>(schema_.state_buffer_size(), 0));
  command_buffer_.resize(schema_.command_buffer_size(), 0);

  state_interfaces_.clear();
  state_interfaces_.reserve(schema_.total_state_interfaces());
  for (size_t i = 0; i < schema_.total_state_interfaces(); i++)
  {
    const auto & f = schema_.state_fields()[i];
    auto si = std::make_shared<hardware_interface::StateInterface>(f.component, f.field);
    std::ignore = si->set_value(0.0);
    state_interfaces_.push_back(si);
    REGISTER_ROS2_CONTROL_INTROSPECTION(f.component + "/" + f.field, &hw_states_[i]);
  }

  command_interfaces_.clear();
  command_interfaces_.reserve(schema_.total_command_interfaces());
  for (size_t i = 0; i < schema_.total_command_interfaces(); i++)
  {
    const auto & f = schema_.command_fields()[i];
    auto ci = std::make_shared<hardware_interface::CommandInterface>(f.component, f.field);
    std::ignore = ci->set_value(0.0);
    command_interfaces_.push_back(ci);
    REGISTER_ROS2_CONTROL_INTROSPECTION(f.component + "/" + f.field, &hw_commands_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("ZenbeddedHardware"), "Hardware Interface Initialized!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ZenbeddedHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(state_interfaces_.size());
  for (const auto & si : state_interfaces_)
  {
    state_interfaces.push_back(
      std::const_pointer_cast<const hardware_interface::StateInterface>(si));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ZenbeddedHardware::on_export_command_interfaces()
{
  return command_interfaces_;
}

CallbackReturn ZenbeddedHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    auto config = zenoh::Config::create_default();
    config.insert_json5("connect/endpoints", "[\"" + zenoh_endpoint_ + "\"]");
    config.insert_json5("mode", "\"" + zenoh_mode_ + "\"");
    session_ = std::make_unique<zenoh::Session>(std::move(config));

    state_sub_ = std::make_unique<zenoh::Subscriber<void>>(session_->declare_subscriber(
      zenoh::KeyExpr(state_topic_),
      [this](zenoh::Sample & sample)
      {
        const auto & payload = sample.get_payload();
        auto bytes = payload.as_vector();
        if (bytes.size() != schema_.state_buffer_size())
        {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("ZenbeddedHardware"), steady_clock_, 1000,
            "Dropping state payload of %zu bytes, schema expects %zu", bytes.size(),
            schema_.state_buffer_size());
          return;
        }
        state_buffer_.set(std::move(bytes));
      },
      []() {}, zenoh::Session::SubscriberOptions::create_default(), nullptr));

    command_pub_ = std::make_unique<zenoh::Publisher>(session_->declare_publisher(
      zenoh::KeyExpr(command_topic_), zenoh::Session::PublisherOptions::create_default(), nullptr));

    for (size_t i = 0; i < hw_states_.size(); i++)
    {
      if (std::isnan(hw_states_[i]))
      {
        hw_states_[i] = 0;
      }
      if (i < state_interfaces_.size())
      {
        std::ignore = state_interfaces_[i]->set_value(hw_states_[i]);
      }
    }

    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
      if (std::isnan(hw_commands_[i]))
      {
        hw_commands_[i] = 0;
      }
      if (i < command_interfaces_.size())
      {
        std::ignore = command_interfaces_[i]->set_value(hw_commands_[i]);
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ZenbeddedHardware"), "Zenoh failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("ZenbeddedHardware"), "Hardware activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZenbeddedHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_sub_.reset();
  command_pub_.reset();
  session_.reset();
  RCLCPP_INFO(rclcpp::get_logger("ZenbeddedHardware"), "Hardware deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZenbeddedHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  state_buffer_.try_get(
    [this](const std::vector<uint8_t> & buf)
    {
      if (buf.size() == schema_.state_buffer_size())
      {
        for (size_t i = 0; i < schema_.total_state_interfaces(); i++)
        {
          hw_states_[i] = schema_.read_state_field(buf.data(), i);
          if (i < state_interfaces_.size())
          {
            std::ignore = state_interfaces_[i]->set_value(hw_states_[i]);
          }
        }
      }
    });
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZenbeddedHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!command_pub_)
  {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < schema_.total_command_interfaces(); i++)
  {
    if (i < command_interfaces_.size())
    {
      auto opt = command_interfaces_[i]->get_optional<double>();
      if (opt.has_value() && !std::isnan(*opt))
      {
        hw_commands_[i] = *opt;
      }
    }
    schema_.write_command_field(command_buffer_.data(), i, hw_commands_[i]);
  }
  command_pub_->put(command_buffer_);
  return hardware_interface::return_type::OK;
}

}  // namespace zenbedded

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zenbedded::ZenbeddedHardware, hardware_interface::SystemInterface)
