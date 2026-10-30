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

#ifndef INVERTED_PENDULUM_TIER1_HARDWARE__INVERTED_PENDULUM_TIER1_HARDWARE_HPP_
#define INVERTED_PENDULUM_TIER1_HARDWARE__INVERTED_PENDULUM_TIER1_HARDWARE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/joint_command.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace zenbedded_inverted_pendulum
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Topic-based hardware interface for Inverted Pendulum Tier 1.
///
/// Subscribes to sensor_msgs/msg/JointState from the Tier 1 Zephyr MCU firmware
/// and publishes control_msgs/msg/JointCommand acceleration back to the MCU.
class InvertedPendulumTier1Hardware : public hardware_interface::SystemInterface
{
public:
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
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::string state_topic_{"/zenbedded/inverted_pendulum/state"};
  std::string command_topic_{"/zenbedded/inverted_pendulum/cmd"};

  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  // hw_states_: [0] motor_pos, [1] motor_vel, [2] pendulum_pos, [3] pendulum_vel
  std::vector<double> hw_states_{0.0, 0.0, 0.0, 0.0};
  // hw_commands_: [0] motor_accel
  std::vector<double> hw_commands_{0.0};

  std::mutex state_mutex_;
  double latest_q1_{0.0};
  double latest_q1_dot_{0.0};
  double latest_q2_{0.0};
  double latest_q2_dot_{0.0};
  std::atomic<bool> has_received_state_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<control_msgs::msg::JointCommand>::SharedPtr cmd_pub_;
};

}  // namespace zenbedded_inverted_pendulum

#endif  // INVERTED_PENDULUM_TIER1_HARDWARE__INVERTED_PENDULUM_TIER1_HARDWARE_HPP_
