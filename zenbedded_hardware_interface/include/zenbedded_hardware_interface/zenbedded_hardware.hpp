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

#ifndef ZENBEDDED_HARDWARE_INTERFACE__ZENBEDDED_HARDWARE_HPP_
#define ZENBEDDED_HARDWARE_INTERFACE__ZENBEDDED_HARDWARE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <zenoh.hxx>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "zenbedded_hardware_interface/interface_schema.hpp"

namespace zenbedded
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ZenbeddedHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ZenbeddedHardware)

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<zenoh::Session> session_;

  InterfaceSchema schema_;

  // Real-time safe state buffer
  realtime_tools::RealtimeThreadSafeBox<std::vector<uint8_t>> state_buffer_;
  std::vector<uint8_t> command_buffer_;

  // Zenoh sub/pub
  std::unique_ptr<zenoh::Subscriber<void>> state_sub_;
  std::unique_ptr<zenoh::Publisher> command_pub_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
  std::string zenoh_endpoint_;
  std::string zenoh_mode_;
  std::string state_topic_;
  std::string command_topic_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace zenbedded

#endif  // ZENBEDDED_HARDWARE_INTERFACE__ZENBEDDED_HARDWARE_HPP_
