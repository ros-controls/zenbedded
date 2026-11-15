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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "zenbedded_transport/serialization.h"

namespace
{

constexpr int32_t kStampSec = 12;
constexpr uint32_t kStampNanosec = 34;
constexpr uint32_t kNumJoints = 2;
const char * kFrameId = "base_link";
const char * kJointNames[] = {"motor_arm", "pendulum_axis"};
const double kPositions[] = {1.25, -2.5};
const double kVelocities[] = {0.5, 0.75};
const double kEfforts[] = {10.0, -20.0};

sensor_msgs::msg::JointState make_reference_message()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp.sec = kStampSec;
  msg.header.stamp.nanosec = kStampNanosec;
  msg.header.frame_id = kFrameId;
  msg.name = {kJointNames[0], kJointNames[1]};
  msg.position = {kPositions[0], kPositions[1]};
  msg.velocity = {kVelocities[0], kVelocities[1]};
  msg.effort = {kEfforts[0], kEfforts[1]};
  return msg;
}

}  // namespace

TEST(CdrConformance, zcdr_reads_a_ros2_serialized_joint_state)
{
  auto msg = make_reference_message();
  rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
  rclcpp::SerializedMessage wire;
  serializer.serialize_message(&msg, &wire);
  const auto & rcl_msg = wire.get_rcl_serialized_message();

  uint8_t layout_buffer[256] = {0};
  zcdr_joint_state_ctx_t ctx;
  ASSERT_EQ(
    0, zcdr_init_joint_state(
         &ctx, kFrameId, kJointNames, kNumJoints, layout_buffer, sizeof(layout_buffer)));

  int32_t sec = 0;
  uint32_t nanosec = 0;
  double positions[kNumJoints] = {0};
  double velocities[kNumJoints] = {0};
  double efforts[kNumJoints] = {0};
  ASSERT_TRUE(zcdr_deserialize_joint_state(
    &ctx, rcl_msg.buffer, rcl_msg.buffer_length, &sec, &nanosec, positions, velocities, efforts));

  EXPECT_EQ(sec, kStampSec);
  EXPECT_EQ(nanosec, kStampNanosec);
  for (uint32_t i = 0; i < kNumJoints; i++)
  {
    EXPECT_DOUBLE_EQ(positions[i], kPositions[i]) << "position " << i;
    EXPECT_DOUBLE_EQ(velocities[i], kVelocities[i]) << "velocity " << i;
    EXPECT_DOUBLE_EQ(efforts[i], kEfforts[i]) << "effort " << i;
  }
}

TEST(CdrConformance, ros2_reads_a_zcdr_serialized_joint_state)
{
  uint8_t buffer[256] = {0};
  zcdr_joint_state_ctx_t ctx;
  ASSERT_EQ(
    0, zcdr_init_joint_state(&ctx, kFrameId, kJointNames, kNumJoints, buffer, sizeof(buffer)));
  zcdr_serialize_joint_state(
    &ctx, kStampSec, kStampNanosec, kPositions, kVelocities, kEfforts, buffer);

  rclcpp::SerializedMessage wire(ctx.payload_size);
  auto & rcl_msg = wire.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, buffer, ctx.payload_size);
  rcl_msg.buffer_length = ctx.payload_size;

  sensor_msgs::msg::JointState msg;
  rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
  ASSERT_NO_THROW(serializer.deserialize_message(&wire, &msg));

  EXPECT_EQ(msg.header.stamp.sec, kStampSec);
  EXPECT_EQ(msg.header.stamp.nanosec, kStampNanosec);
  EXPECT_EQ(msg.header.frame_id, kFrameId);
  ASSERT_EQ(msg.name.size(), kNumJoints);
  ASSERT_EQ(msg.position.size(), kNumJoints);
  ASSERT_EQ(msg.velocity.size(), kNumJoints);
  ASSERT_EQ(msg.effort.size(), kNumJoints);
  for (uint32_t i = 0; i < kNumJoints; i++)
  {
    EXPECT_EQ(msg.name[i], kJointNames[i]) << "name " << i;
    EXPECT_DOUBLE_EQ(msg.position[i], kPositions[i]) << "position " << i;
    EXPECT_DOUBLE_EQ(msg.velocity[i], kVelocities[i]) << "velocity " << i;
    EXPECT_DOUBLE_EQ(msg.effort[i], kEfforts[i]) << "effort " << i;
  }
}
