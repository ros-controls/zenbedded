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

// zenbedded_transport/include/zenbedded_transport/rihs_ledger.h
#ifndef ZENBEDDED_TRANSPORT__RIHS_LEDGER_H_
#define ZENBEDDED_TRANSPORT__RIHS_LEDGER_H_

#include <stddef.h>
#include <string.h>

typedef struct
{
  const char * type_name;
  const char * rihs_hash;
} zenbedded_rihs_mapping_t;

static const zenbedded_rihs_mapping_t RIHS_LEDGER[] = {
  {"sensor_msgs::msg::dds_::JointState_",
   "RIHS01_a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e"},
  {"control_msgs::msg::dds_::JointCommand_",
   "RIHS01_6080a1df9d28b6badffa5efb27d4ba4ae657c4f6dd2b519b178a32db12405985"},
  {"sensor_msgs::msg::dds_::Imu_",
   "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b"}};

static inline const char * zenbedded_get_rihs_hash(const char * type_name)
{
  if (type_name == NULL)
  {
    return NULL;
  }
  size_t ledger_size = sizeof(RIHS_LEDGER) / sizeof(RIHS_LEDGER[0]);
  for (size_t i = 0; i < ledger_size; i++)
  {
    if (strcmp(RIHS_LEDGER[i].type_name, type_name) == 0)
    {
      return RIHS_LEDGER[i].rihs_hash;
    }
  }
  return NULL;
}

#endif  // ZENBEDDED_TRANSPORT__RIHS_LEDGER_H_
