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

#include "sine_wave_controller/sine_wave_tier1_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

namespace sine_wave_controller
{

SineWaveTier1Node::SineWaveTier1Node(const rclcpp::NodeOptions & options)
: Node("sine_wave_tier1", options)
{
  topic_name_ = this->declare_parameter<std::string>("topic_name", "/zenbedded/sine_wave/cmd");
  joint_name_ = this->declare_parameter<std::string>("joint_name", "sine_wave");
  interface_name_ = this->declare_parameter<std::string>("interface_name", "amplitude");
  amplitude_ = this->declare_parameter<double>("amplitude", 5.0);
  frequency_ = this->declare_parameter<double>("frequency", 0.2);
  offset_ = this->declare_parameter<double>("offset", 5.0);
  publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);

  publisher_ = this->create_publisher<control_msgs::msg::JointCommand>(topic_name_, 10);

  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&SineWaveTier1Node::timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "SineWaveTier1Node initialized: topic='%s', joint='%s', iface='%s', amp=%.2f, freq=%.2f Hz, "
    "offset=%.2f, rate=%.1f Hz",
    topic_name_.c_str(), joint_name_.c_str(), interface_name_.c_str(), amplitude_, frequency_,
    offset_, publish_rate_);
}

void SineWaveTier1Node::timer_callback()
{
  auto now = this->now();
  if (!start_time_set_)
  {
    start_time_ = now;
    start_time_set_ = true;
  }

  const double t = (now - start_time_).seconds();
  const double value = offset_ + amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);

  control_msgs::msg::JointCommand msg;
  msg.header.stamp = now;
  msg.header.frame_id = joint_name_;
  msg.joint_names = {joint_name_};
  msg.interface_name = interface_name_;
  msg.values = {value};

  publisher_->publish(msg);
}

}  // namespace sine_wave_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sine_wave_controller::SineWaveTier1Node>());
  rclcpp::shutdown();
  return 0;
}
