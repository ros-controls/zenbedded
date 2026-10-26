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

#ifndef ZENBEDDED_TRANSPORT__SERIALIZATION_H_
#define ZENBEDDED_TRANSPORT__SERIALIZATION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CONFIG_ZENBEDDED_TIER_1

  /**
   * @brief Context for JointState serialization and deserialization.
   */
  typedef struct
  {
    uint32_t num_joints;
    size_t offset_timestamp;
    size_t offset_position;
    size_t offset_velocity;
    size_t offset_effort;
    size_t payload_size;
  } zcdr_joint_state_ctx_t;

  /**
   * @brief Initializes the static CDR payload structure for JointState.
   *
   * @param ctx Pointer to the context struct to initialize.
   * @param frame_id The TF coordinate frame ID string.
   * @param joint_names Array of joint name strings.
   * @param num_joints The number of joints in the arrays.
   * @param out_buffer Pre-allocated byte buffer to hold the initialized CDR template.
   */
  void zcdr_init_joint_state(
    zcdr_joint_state_ctx_t * ctx, const char * frame_id, const char ** joint_names,
    uint32_t num_joints, uint8_t * out_buffer);

  /**
   * @brief Serializes ROS 2 JointState into the pre-calculated CDR network buffer.
   *
   * @param ctx Pointer to the initialized context struct.
   * @param stamp_sec Message timestamp seconds component.
   * @param stamp_nanosec Message timestamp nanoseconds component.
   * @param positions Array of joint position values.
   * @param velocities Array of joint velocity values.
   * @param efforts Array of joint effort values.
   * @param out_buffer Pre-allocated byte buffer that receives the packed CDR payload.
   */
  void zcdr_serialize_joint_state(
    const zcdr_joint_state_ctx_t * ctx, int32_t stamp_sec, uint32_t stamp_nanosec,
    const double * positions, const double * velocities, const double * efforts,
    uint8_t * out_buffer);

  /**
   * @brief Deserializes a CDR network buffer into primitive JointState arrays.
   *
   * @param ctx Pointer to the initialized context struct.
   * @param in_buffer Byte buffer containing the received CDR payload.
   * @param buffer_size The size of the received buffer payload in bytes.
   * @param out_stamp_sec Pointer receiving the deserialized seconds component.
   * @param out_stamp_nanosec Pointer receiving the deserialized nanoseconds component.
   * @param out_positions Array receiving the deserialized position values.
   * @param out_velocities Array receiving the deserialized velocity values.
   * @param out_efforts Array receiving the deserialized effort values.
   * @return true if deserialization succeeded, false otherwise.
   */
  bool zcdr_deserialize_joint_state(
    const zcdr_joint_state_ctx_t * ctx, const uint8_t * in_buffer, size_t buffer_size,
    int32_t * out_stamp_sec, uint32_t * out_stamp_nanosec, double * out_positions,
    double * out_velocities, double * out_efforts);

  /**
   * @brief Context for JointCommand serialization and deserialization.
   */
  typedef struct
  {
    uint32_t num_joints;
    size_t offset_timestamp;
    size_t offset_values;
    size_t payload_size;
  } zcdr_joint_command_ctx_t;

  /**
   * @brief Initializes the static CDR payload structure for JointCommand.
   *
   * @param ctx Pointer to the context struct to initialize.
   * @param frame_id The TF coordinate frame ID string.
   * @param joint_names Array of joint name strings.
   * @param num_joints The number of joints in the arrays.
   * @param interface_name The hardware interface name string.
   * @param out_buffer Pre-allocated byte buffer to hold the initialized CDR template.
   */
  void zcdr_init_joint_command(
    zcdr_joint_command_ctx_t * ctx, const char * frame_id, const char ** joint_names,
    uint32_t num_joints, const char * interface_name, uint8_t * out_buffer);

  /**
   * @brief Serializes ROS 2 JointCommand into the pre-calculated CDR network buffer.
   *
   * @param ctx Pointer to the initialized context struct.
   * @param stamp_sec Message timestamp seconds component.
   * @param stamp_nanosec Message timestamp nanoseconds component.
   * @param values Array of command values.
   * @param out_buffer Pre-allocated byte buffer that receives the packed CDR payload.
   */
  void zcdr_serialize_joint_command(
    const zcdr_joint_command_ctx_t * ctx, int32_t stamp_sec, uint32_t stamp_nanosec,
    const double * values, uint8_t * out_buffer);

  /**
   * @brief Deserializes a CDR network buffer into primitive JointCommand arrays.
   *
   * @param ctx Pointer to the initialized context struct.
   * @param in_buffer Byte buffer containing the received CDR payload.
   * @param buffer_size The size of the received buffer payload in bytes.
   * @param out_stamp_sec Pointer receiving the deserialized seconds component.
   * @param out_stamp_nanosec Pointer receiving the deserialized nanoseconds component.
   * @param out_values Array receiving the deserialized command values.
   * @return true if deserialization succeeded, false otherwise.
   */
  bool zcdr_deserialize_joint_command(
    const zcdr_joint_command_ctx_t * ctx, const uint8_t * in_buffer, size_t buffer_size,
    int32_t * out_stamp_sec, uint32_t * out_stamp_nanosec, double * out_values);
#endif  // CONFIG_ZENBEDDED_TIER_1

#ifdef __cplusplus
}
#endif

#endif  // ZENBEDDED_TRANSPORT__SERIALIZATION_H_
