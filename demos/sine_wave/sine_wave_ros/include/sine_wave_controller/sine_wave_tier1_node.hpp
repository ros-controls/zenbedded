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

#ifndef SINE_WAVE_CONTROLLER__SINE_WAVE_TIER1_NODE_HPP_
#define SINE_WAVE_CONTROLLER__SINE_WAVE_TIER1_NODE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "control_msgs/msg/joint_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sine_wave_controller
{

/**
 * @brief Standalone ROS 2 node that computes and streams an oscillating
 * amplitude command to the Tier 1 Zephyr application via control_msgs/msg/JointCommand.
 */
class SineWaveTier1Node : public rclcpp::Node
{
public:
  explicit SineWaveTier1Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SineWaveTier1Node() = default;

private:
  void timer_callback();

  std::string topic_name_;
  std::string joint_name_;
  std::string interface_name_;
  double amplitude_;
  double frequency_;
  double offset_;
  double publish_rate_;

  rclcpp::Time start_time_;
  bool start_time_set_{false};

  rclcpp::Publisher<control_msgs::msg::JointCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sine_wave_controller

#endif  // SINE_WAVE_CONTROLLER__SINE_WAVE_TIER1_NODE_HPP_
