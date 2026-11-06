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

#include <errno.h>
#include <string.h>

// Note: Layout writes assume a little-endian host, matching the CDR_LE encapsulation.

#ifdef CONFIG_ZENBEDDED_TIER_1

#define CDR_HEADER_SIZE 4
// Note: Subtracts CDR_HEADER_SIZE to compute alignment relative to the encapsulated payload.
#define ALIGN_UP(offset, alignment) \
  (CDR_HEADER_SIZE + ((((offset) - CDR_HEADER_SIZE) + (alignment) - 1) & ~((alignment) - 1)))

static const uint8_t kCdrHeader[CDR_HEADER_SIZE] = {0x00, 0x01, 0x00, 0x00};

// Validate the complete layout before writing any template bytes or context offsets.
static bool reserve_bytes(size_t * cursor, size_t capacity, size_t alignment, size_t size)
{
  size_t padding = (alignment - (*cursor - CDR_HEADER_SIZE) % alignment) % alignment;
  if (*cursor > capacity || padding > capacity - *cursor || size > capacity - *cursor - padding)
  {
    return false;
  }
  *cursor += padding + size;
  return true;
}

static bool reserve_string(size_t * cursor, size_t capacity, const char * value)
{
  if (!reserve_bytes(cursor, capacity, 4, sizeof(uint32_t)))
  {
    return false;
  }
  size_t remaining = capacity - *cursor;
  size_t length = 0;
  if (value)
  {
    while (length < remaining && length < UINT32_MAX && value[length] != '\0')
    {
      length++;
    }
  }
  if (length == remaining || length == UINT32_MAX)
  {
    return false;
  }
  *cursor += length + 1;
  return true;
}

static int check_layout(
  const char * frame_id, const char ** joint_names, uint32_t num_joints,
  const char * interface_name, size_t num_arrays, size_t capacity)
{
  if (num_joints && !joint_names)
  {
    return -EINVAL;
  }
  if (capacity < CDR_HEADER_SIZE || num_joints > capacity / sizeof(double) / num_arrays)
  {
    return -ENOMEM;
  }
  size_t cursor = CDR_HEADER_SIZE;
  if (
    !reserve_bytes(&cursor, capacity, 1, 8) || !reserve_string(&cursor, capacity, frame_id) ||
    !reserve_bytes(&cursor, capacity, 4, sizeof(uint32_t)))
  {
    return -ENOMEM;
  }
  for (uint32_t i = 0; i < num_joints; i++)
  {
    if (!joint_names[i])
    {
      return -EINVAL;
    }
    if (!reserve_string(&cursor, capacity, joint_names[i]))
    {
      return -ENOMEM;
    }
  }
  if (num_arrays == 1 && !reserve_string(&cursor, capacity, interface_name))
  {
    return -ENOMEM;
  }
  for (size_t i = 0; i < num_arrays; i++)
  {
    if (
      !reserve_bytes(&cursor, capacity, 4, sizeof(uint32_t)) ||
      !reserve_bytes(&cursor, capacity, 8, (size_t)num_joints * sizeof(double)))
    {
      return -ENOMEM;
    }
  }
  return 0;
}

int zcdr_init_joint_state(
  zcdr_joint_state_ctx_t * ctx, const char * frame_id, const char ** joint_names,
  uint32_t num_joints, uint8_t * out_buffer, size_t buffer_size)
{
  if (!ctx || !out_buffer)
  {
    return -EINVAL;
  }
  int ret = check_layout(frame_id, joint_names, num_joints, NULL, 3, buffer_size);
  if (ret != 0)
  {
    return ret;
  }
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
  cursor += ((size_t)num_joints * sizeof(double));

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;
  cursor = ALIGN_UP(cursor, 8);
  ctx->offset_velocity = cursor;
  cursor += ((size_t)num_joints * sizeof(double));

  cursor = ALIGN_UP(cursor, 4);
  memcpy(&out_buffer[cursor], &num_joints, 4);
  cursor += 4;
  cursor = ALIGN_UP(cursor, 8);
  ctx->offset_effort = cursor;
  cursor += ((size_t)num_joints * sizeof(double));

  ctx->payload_size = cursor;
  return 0;
}

void zcdr_serialize_joint_state(
  const zcdr_joint_state_ctx_t * ctx, int32_t stamp_sec, uint32_t stamp_nanosec,
  const double * positions, const double * velocities, const double * efforts, uint8_t * out_buffer)
{
  memcpy(&out_buffer[ctx->offset_timestamp], &stamp_sec, 4);
  memcpy(&out_buffer[ctx->offset_timestamp + 4], &stamp_nanosec, 4);

  size_t array_bytes = (size_t)ctx->num_joints * sizeof(double);

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

  size_t array_bytes = (size_t)ctx->num_joints * sizeof(double);

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

int zcdr_init_joint_command(
  zcdr_joint_command_ctx_t * ctx, const char * frame_id, const char ** joint_names,
  uint32_t num_joints, const char * interface_name, uint8_t * out_buffer, size_t buffer_size)
{
  if (!ctx || !out_buffer)
  {
    return -EINVAL;
  }
  int ret = check_layout(frame_id, joint_names, num_joints, interface_name, 1, buffer_size);
  if (ret != 0)
  {
    return ret;
  }
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
  cursor += ((size_t)num_joints * sizeof(double));

  ctx->payload_size = cursor;
  return 0;
}

void zcdr_serialize_joint_command(
  const zcdr_joint_command_ctx_t * ctx, int32_t stamp_sec, uint32_t stamp_nanosec,
  const double * values, uint8_t * out_buffer)
{
  memcpy(&out_buffer[ctx->offset_timestamp], &stamp_sec, 4);
  memcpy(&out_buffer[ctx->offset_timestamp + 4], &stamp_nanosec, 4);

  if (values)
  {
    memcpy(&out_buffer[ctx->offset_values], values, (size_t)ctx->num_joints * sizeof(double));
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
    memcpy(out_values, &in_buffer[ctx->offset_values], (size_t)ctx->num_joints * sizeof(double));
  }

  return true;
}

#endif  // CONFIG_ZENBEDDED_TIER_1
