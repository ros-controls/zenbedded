// Copyright (c) 2026 Process Mission
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

#include <cerrno>
#include <cstdint>
#include <string>
#include <vector>

#include "zenbedded_rcl/codecs.hpp"
#include "zenbedded_transport/serialization.h"

namespace
{

const char * kJointNames[] = {"joint_0", "joint_1", "joint_2", "joint_3",
                              "joint_4", "joint_5", "joint_6", "joint_7"};

class CdrBounds : public testing::TestWithParam<bool>
{
protected:
  int init(
    uint8_t * buffer, size_t capacity, const char * frame = "base_link",
    const char ** names = kJointNames, uint32_t joints = 8, const char * interface = "position")
  {
    if (GetParam())
    {
      return zcdr_init_joint_state(&state_, frame, names, joints, buffer, capacity);
    }
    return zcdr_init_joint_command(&command_, frame, names, joints, interface, buffer, capacity);
  }

  size_t payload_size() const { return GetParam() ? state_.payload_size : command_.payload_size; }

  size_t required_capacity() const { return GetParam() ? 340 : 212; }

  zcdr_joint_state_ctx_t state_{};
  zcdr_joint_command_ctx_t command_{};
};

TEST_P(CdrBounds, exact_capacity_accepts_initialization_and_serialization)
{
  std::vector<uint8_t> buffer(required_capacity(), 0);
  ASSERT_EQ(0, init(buffer.data(), buffer.size()));
  ASSERT_EQ(required_capacity(), payload_size());
  double values[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  double decoded[8] = {};
  if (GetParam())
  {
    zcdr_serialize_joint_state(&state_, 12, 34, values, values, values, buffer.data());
    ASSERT_TRUE(zcdr_deserialize_joint_state(
      &state_, buffer.data(), buffer.size(), nullptr, nullptr, decoded, nullptr, nullptr));
  }
  else
  {
    zcdr_serialize_joint_command(&command_, 12, 34, values, buffer.data());
    ASSERT_TRUE(zcdr_deserialize_joint_command(
      &command_, buffer.data(), buffer.size(), nullptr, nullptr, decoded));
  }
  for (size_t i = 0; i < 8; i++)
  {
    EXPECT_DOUBLE_EQ(values[i], decoded[i]);
  }
}

TEST_P(CdrBounds, every_undersized_capacity_fails_without_changing_buffer_or_context)
{
  for (size_t capacity = 0; capacity < required_capacity(); capacity++)
  {
    SCOPED_TRACE(capacity);
    std::vector<uint8_t> buffer(capacity ? capacity : 1, 0xa5);
    const auto original = buffer;
    state_.payload_size = command_.payload_size = 123;
    EXPECT_EQ(-ENOMEM, init(buffer.data(), capacity));
    EXPECT_EQ(original, buffer);
    EXPECT_EQ(123u, payload_size());
  }
}

TEST_P(CdrBounds, long_frame_is_rejected_before_writing)
{
  std::vector<uint8_t> buffer(256, 0xa5);
  const auto original = buffer;
  std::string frame(300, 'f');
  EXPECT_EQ(-ENOMEM, init(buffer.data(), buffer.size(), frame.c_str()));
  EXPECT_EQ(original, buffer);
}

TEST_P(CdrBounds, long_joint_name_is_rejected_before_writing)
{
  std::vector<uint8_t> buffer(256, 0xa5);
  const auto original = buffer;
  std::string name(300, 'j');
  const char * names[] = {name.c_str()};
  EXPECT_EQ(-ENOMEM, init(buffer.data(), buffer.size(), "base_link", names, 1));
  EXPECT_EQ(original, buffer);
}

TEST_P(CdrBounds, excessive_joint_count_is_rejected_without_integer_wraparound)
{
  uint8_t buffer[256] = {};
  EXPECT_EQ(-ENOMEM, init(buffer, sizeof(buffer), "base_link", kJointNames, UINT32_MAX));
}

TEST_P(CdrBounds, missing_joint_array_is_invalid)
{
  uint8_t buffer[256] = {};
  EXPECT_EQ(-EINVAL, init(buffer, sizeof(buffer), "base_link", nullptr, 1));
}

TEST_P(CdrBounds, missing_joint_name_is_invalid)
{
  uint8_t buffer[256] = {};
  const char * names[] = {nullptr};
  EXPECT_EQ(-EINVAL, init(buffer, sizeof(buffer), "base_link", names, 1));
}

TEST_P(CdrBounds, missing_buffer_is_invalid) { EXPECT_EQ(-EINVAL, init(nullptr, 256)); }

TEST_P(CdrBounds, zero_joints_and_optional_null_strings_are_supported)
{
  uint8_t buffer[64] = {};
  EXPECT_EQ(0, init(buffer, sizeof(buffer), nullptr, nullptr, 0, nullptr));
}

INSTANTIATE_TEST_SUITE_P(JointStateAndCommand, CdrBounds, testing::Bool());

TEST(CdrBounds, missing_context_is_invalid)
{
  uint8_t buffer[256] = {};
  EXPECT_EQ(-EINVAL, zcdr_init_joint_state(nullptr, nullptr, nullptr, 0, buffer, sizeof(buffer)));
  EXPECT_EQ(
    -EINVAL,
    zcdr_init_joint_command(nullptr, nullptr, nullptr, 0, nullptr, buffer, sizeof(buffer)));
}

TEST(CdrBounds, long_command_interface_is_rejected_before_writing)
{
  std::vector<uint8_t> buffer(256, 0xa5);
  const auto original = buffer;
  std::string interface(300, 'i');
  zcdr_joint_command_ctx_t ctx{};
  EXPECT_EQ(
    -ENOMEM, zcdr_init_joint_command(
               &ctx, "base_link", kJointNames, 1, interface.c_str(), buffer.data(), buffer.size()));
  EXPECT_EQ(original, buffer);
}

TEST(CodecBounds, built_in_codecs_propagate_capacity_errors)
{
  static_assert(CheckCodec<JointStateCodec>::is_valid_state_codec, "Invalid state codec");
  static_assert(CheckCodec<JointCommandCodec>::is_valid_command_codec, "Invalid command codec");
  static_assert(CheckCodec<RawCodec<uint64_t>>::is_valid_state_codec, "Invalid raw state codec");
  static_assert(
    CheckCodec<RawCodec<uint64_t>>::is_valid_command_codec, "Invalid raw command codec");
  uint8_t buffer[32] = {};
  JointStateCodec::Ctx state{};
  JointCommandCodec::Ctx command{};
  EXPECT_EQ(
    -ENOMEM, JointStateCodec::init(state, {"base_link", kJointNames, 8}, buffer, sizeof(buffer)));
  EXPECT_EQ(
    -ENOMEM, JointCommandCodec::init(
               command, {"base_link", kJointNames, 8, "position"}, buffer, sizeof(buffer)));
  RawCodec<uint64_t>::Ctx raw{};
  EXPECT_EQ(-ENOMEM, RawCodec<uint64_t>::init(raw, {}, buffer, sizeof(uint64_t) - 1));
  EXPECT_EQ(0, RawCodec<uint64_t>::init(raw, {}, buffer, sizeof(uint64_t)));
}

}  // namespace
