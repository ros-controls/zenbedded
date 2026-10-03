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

#ifndef ZENBEDDED_RCL__CODECS_HPP_
#define ZENBEDDED_RCL__CODECS_HPP_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef CONFIG_ZENBEDDED_TIER_1
#include <zenbedded_transport/serialization.h>
#endif  // CONFIG_ZENBEDDED_TIER_1

// -------------------------------------------------------------------------------------------
// The Codec concept
// -------------------------------------------------------------------------------------------
// ZenbeddedClient<StateCodec, CommandCodec> (zenbedded_client.hpp) never mentions a concrete
// ROS 2 message type or a Tier by name. It only relies on this duck-typed contract, checked at
// compile time when the template is instantiated:
//
//   Ctx           -- per-instance layout/offset state (e.g. precomputed CDR field offsets).
//                    Opaque to the client; must be default-constructible.
//   InitParams    -- whatever this message type needs at init time (topic-independent framing
//                    like frame_id, joint_names, ...). Can be an empty struct.
//   Value         -- the value passed to write() / populated by read().
//
//   static void init(Ctx &, const InitParams &, uint8_t * buf);
//       Called once (per double-buffer slot) during ZenbeddedClient::init(). May write a
//       persistent template into `buf` (e.g. Tier 1's CDR header/field names) that write()
//       will patch on every call rather than rewrite from scratch -- see JointStateCodec.
//   static size_t payload_size(const Ctx &);
//       Wire size in bytes. Fixed after init() for the lifetime of the client.
//
// A codec used as a STATE (published/outgoing) channel additionally needs:
//   static void write(const Ctx &, const Value &, uint8_t * buf);
//
// A codec used as a COMMAND (subscribed/incoming) channel additionally needs:
//   static bool read(const Ctx &, const uint8_t * buf, size_t size, Value & out);
//
// A codec only needs to implement the direction(s) it's actually used for.
// It Could have both read and write functions so its used for state/command codecs
// -------------------------------------------------------------------------------------------

// This class is a helper to check if a Codec is a valid Codec Type
template <typename Codec>
struct CheckCodec
{
private:
  struct true_type
  {
    static constexpr bool value = true;
  };
  struct false_type
  {
    static constexpr bool value = false;
  };

  template <typename U>
  static U declval_helper() noexcept;

  // clang-format off
  template <typename C>
  static auto check_state_codec(int) -> decltype(C::init(declval_helper<typename C::Ctx &>(), declval_helper<const typename C::InitParams &>(), static_cast<uint8_t *>(nullptr)), C::payload_size(declval_helper<const typename C::Ctx &>()), C::write(declval_helper<const typename C::Ctx &>(), declval_helper<const typename C::Value &>(), static_cast<uint8_t *>(nullptr)), true_type{}); // NOLINT

  template <typename C>
  static auto check_command_codec(int) -> decltype(C::init(declval_helper<typename C::Ctx &>(), declval_helper<const typename C::InitParams &>(), static_cast<uint8_t *>(nullptr)), C::payload_size(declval_helper<const typename C::Ctx &>()), C::read(declval_helper<const typename C::Ctx &>(), static_cast<const uint8_t *>(nullptr), size_t{0}, declval_helper<typename C::Value &>()), true_type{}); // NOLINT
  // clang-format on

  template <typename>
  static false_type check_state_codec(...);

  template <typename>
  static false_type check_command_codec(...);

public:
  static constexpr bool is_valid_state_codec = decltype(check_state_codec<Codec>(0))::value;
  static constexpr bool is_valid_command_codec = decltype(check_command_codec<Codec>(0))::value;
};

// Codec for any fixed-layout "just copy the bytes" type: a generic
// passthrough codec for any trivially-copyable POD type T. No CDR framing, no per-instance
// layout to compute -- InitParams is empty, payload_size is simply sizeof(T), and Ctx carries
// nothing.
template <typename T>
struct RawCodec
{
  struct Ctx
  {
  };

  struct InitParams
  {
  };

  using Value = T;

  static void init(Ctx &, const InitParams &, uint8_t *) {}

  static size_t payload_size(const Ctx &) { return sizeof(T); }

  static void write(const Ctx &, const Value & v, uint8_t * buf) { memcpy(buf, &v, sizeof(T)); }

  static bool read(const Ctx &, const uint8_t * buf, size_t size, Value & out)
  {
    if (size < sizeof(T))
    {
      return false;
    }
    memcpy(&out, buf, sizeof(T));
    return true;
  }
};

#ifdef CONFIG_ZENBEDDED_TIER_1

// Codec for sensor_msgs/msg/JointState
struct JointStateCodec
{
  using Ctx = zcdr_joint_state_ctx_t;

  struct InitParams
  {
    const char * frame_id;
    const char ** joint_names;
    uint32_t num_joints;
  };

  struct Value
  {
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    const double * positions;
    const double * velocities;
    const double * efforts;
  };

  static void init(Ctx & ctx, const InitParams & p, uint8_t * buf)
  {
    zcdr_init_joint_state(&ctx, p.frame_id, p.joint_names, p.num_joints, buf);
  }

  static size_t payload_size(const Ctx & ctx) { return ctx.payload_size; }

  static void write(const Ctx & ctx, const Value & v, uint8_t * buf)
  {
    zcdr_serialize_joint_state(
      &ctx, v.stamp_sec, v.stamp_nanosec, v.positions, v.velocities, v.efforts, buf);
  }
};

// Codec for control_msgs/msg/JointCommand
struct JointCommandCodec
{
  using Ctx = zcdr_joint_command_ctx_t;

  struct InitParams
  {
    const char * frame_id;
    const char ** joint_names;
    uint32_t num_joints;
    const char * interface_name;
  };

  struct Value
  {
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    double * values;
  };

  static void init(Ctx & ctx, const InitParams & p, uint8_t * buf)
  {
    zcdr_init_joint_command(&ctx, p.frame_id, p.joint_names, p.num_joints, p.interface_name, buf);
  }

  static size_t payload_size(const Ctx & ctx) { return ctx.payload_size; }

  static bool read(const Ctx & ctx, const uint8_t * buf, size_t size, Value & out)
  {
    return zcdr_deserialize_joint_command(
      &ctx, buf, size, &out.stamp_sec, &out.stamp_nanosec, out.values);
  }
};

#endif  // CONFIG_ZENBEDDED_TIER_1

#endif  // ZENBEDDED_RCL__CODECS_HPP_
