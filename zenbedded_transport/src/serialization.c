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

#include "zenbedded_transport/serialization.h"

#include <string.h>

// Note: Layout writes assume a little-endian host, matching the CDR_LE encapsulation.

#ifdef CONFIG_ZENBEDDED_TIER_1

#define CDR_HEADER_SIZE 4
// Note: Subtracts CDR_HEADER_SIZE to compute alignment relative to the encapsulated payload.
#define ALIGN_UP(offset, alignment) \
  (CDR_HEADER_SIZE + ((((offset) - CDR_HEADER_SIZE) + (alignment) - 1) & ~((alignment) - 1)))

static const uint8_t kCdrHeader[CDR_HEADER_SIZE] = {0x00, 0x01, 0x00, 0x00};

void zcdr_init_joint_state(
  zcdr_joint_state_ctx_t * ctx, const char * frame_id, const char ** joint_names,
  uint32_t num_joints, uint8_t * out_buffer)
{
  ctx->num_joints = num_joints;
  size_t cursor = 0;

  memcpy(&out_buffer[cursor], kCdrHeader, CDR_HEADER_SIZE);
  cursor += CDR_HEADER_SIZE;

  ctx->offset_timestamp = cursor;
  cursor += 8;

  uint32_t frame_id_len = frame_id ? strlen(frame_id) + 1 : 1;
  memcpy(&out_buffer[cursor], &frame_id_len, 4);
  cursor += 4;

  if (frame_id)
  {
    memcpy(&out_buffer[cursor], frame_id, frame_id_len);
  }
  else
  {
    out_buffer[cursor] = '\0';
  }
  cursor += frame_id_len;

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;

  for (uint32_t i = 0; i < num_joints; i++)
  {
    uint32_t name_len = strlen(joint_names[i]) + 1;
    cursor = ALIGN_UP(cursor, 4);
    memcpy(&out_buffer[cursor], &name_len, 4);
    cursor += 4;
    memcpy(&out_buffer[cursor], joint_names[i], name_len);
    cursor += name_len;
  }

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;
  cursor = ALIGN_UP(cursor, 8);
  ctx->offset_position = cursor;
  cursor += (8 * num_joints);

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;
  cursor = ALIGN_UP(cursor, 8);
  ctx->offset_velocity = cursor;
  cursor += (8 * num_joints);

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;
  cursor = ALIGN_UP(cursor, 8);
  ctx->offset_effort = cursor;
  cursor += (8 * num_joints);

  ctx->payload_size = cursor;
}

void zcdr_serialize_joint_state(
  const zcdr_joint_state_ctx_t * ctx, int32_t stamp_sec, uint32_t stamp_nanosec,
  const double * positions, const double * velocities, const double * efforts, uint8_t * out_buffer)
{
  memcpy(&out_buffer[ctx->offset_timestamp], &stamp_sec, 4);
  memcpy(&out_buffer[ctx->offset_timestamp + 4], &stamp_nanosec, 4);

  size_t array_bytes = ctx->num_joints * 8;

  if (positions)
  {
    memcpy(&out_buffer[ctx->offset_position], positions, array_bytes);
  }
  if (velocities)
  {
    memcpy(&out_buffer[ctx->offset_velocity], velocities, array_bytes);
  }
  if (efforts)
  {
    memcpy(&out_buffer[ctx->offset_effort], efforts, array_bytes);
  }
}

bool zcdr_deserialize_joint_state(
  const zcdr_joint_state_ctx_t * ctx, const uint8_t * in_buffer, size_t buffer_size,
  int32_t * out_stamp_sec, uint32_t * out_stamp_nanosec, double * out_positions,
  double * out_velocities, double * out_efforts)
{
  if (buffer_size < ctx->payload_size)
  {
    return false;
  }

  if (out_stamp_sec)
  {
    memcpy(out_stamp_sec, &in_buffer[ctx->offset_timestamp], 4);
  }
  if (out_stamp_nanosec)
  {
    memcpy(out_stamp_nanosec, &in_buffer[ctx->offset_timestamp + 4], 4);
  }

  size_t array_bytes = ctx->num_joints * 8;

  if (out_positions)
  {
    memcpy(out_positions, &in_buffer[ctx->offset_position], array_bytes);
  }
  if (out_velocities)
  {
    memcpy(out_velocities, &in_buffer[ctx->offset_velocity], array_bytes);
  }
  if (out_efforts)
  {
    memcpy(out_efforts, &in_buffer[ctx->offset_effort], array_bytes);
  }

  return true;
}

void zcdr_init_joint_command(
  zcdr_joint_command_ctx_t * ctx, const char * frame_id, const char ** joint_names,
  uint32_t num_joints, const char * interface_name, uint8_t * out_buffer)
{
  ctx->num_joints = num_joints;
  size_t cursor = 0;

  memcpy(&out_buffer[cursor], kCdrHeader, CDR_HEADER_SIZE);
  cursor += CDR_HEADER_SIZE;

  ctx->offset_timestamp = cursor;
  cursor += 8;

  uint32_t frame_id_len = frame_id ? strlen(frame_id) + 1 : 1;
  memcpy(&out_buffer[cursor], &frame_id_len, 4);
  cursor += 4;

  if (frame_id)
  {
    memcpy(&out_buffer[cursor], frame_id, frame_id_len);
  }
  else
  {
    out_buffer[cursor] = '\0';
  }
  cursor += frame_id_len;

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;

  for (uint32_t i = 0; i < num_joints; i++)
  {
    uint32_t name_len = strlen(joint_names[i]) + 1;
    cursor = ALIGN_UP(cursor, 4);
    memcpy(&out_buffer[cursor], &name_len, 4);
    cursor += 4;
    memcpy(&out_buffer[cursor], joint_names[i], name_len);
    cursor += name_len;
  }

  uint32_t interface_len = interface_name ? strlen(interface_name) + 1 : 1;
  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &interface_len, 4);
  cursor += 4;

  if (interface_name)
  {
    memcpy(&out_buffer[cursor], interface_name, interface_len);
  }
  else
  {
    out_buffer[cursor] = '\0';
  }
  cursor += interface_len;

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;

  cursor = ALIGN_UP(cursor, 8);
  ctx->offset_values = cursor;
  cursor += (8 * num_joints);

  ctx->payload_size = cursor;
}

void zcdr_serialize_joint_command(
  const zcdr_joint_command_ctx_t * ctx, int32_t stamp_sec, uint32_t stamp_nanosec,
  const double * values, uint8_t * out_buffer)
{
  memcpy(&out_buffer[ctx->offset_timestamp], &stamp_sec, 4);
  memcpy(&out_buffer[ctx->offset_timestamp + 4], &stamp_nanosec, 4);

  if (values)
  {
    memcpy(&out_buffer[ctx->offset_values], values, ctx->num_joints * 8);
  }
}

bool zcdr_deserialize_joint_command(
  const zcdr_joint_command_ctx_t * ctx, const uint8_t * in_buffer, size_t buffer_size,
  int32_t * out_stamp_sec, uint32_t * out_stamp_nanosec, double * out_values)
{
  if (buffer_size < ctx->payload_size)
  {
    return false;
  }

  if (out_stamp_sec)
  {
    memcpy(out_stamp_sec, &in_buffer[ctx->offset_timestamp], 4);
  }
  if (out_stamp_nanosec)
  {
    memcpy(out_stamp_nanosec, &in_buffer[ctx->offset_timestamp + 4], 4);
  }

  if (out_values)
  {
    memcpy(out_values, &in_buffer[ctx->offset_values], ctx->num_joints * 8);
  }

  return true;
}

#endif  // CONFIG_ZENBEDDED_TIER_1
