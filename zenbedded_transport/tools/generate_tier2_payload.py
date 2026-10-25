#!/usr/bin/env python3

# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Parses a YAML schema (state_interfaces: <comp>: {<field>: <type>}) to generate C headers. Supported TYPE_MAP scalars: float32, float64, double, int32, uint64, int16, uint8."""

import sys
import yaml
import argparse
import os

TYPE_MAP = {
    "float32": ("float", 4),
    "float64": ("double", 8),
    "double": ("double", 8),
    "int32": ("int32_t", 4),
    "uint64": ("uint64_t", 8),
    "int16": ("int16_t", 2),
    "uint8": ("uint8_t", 1),
}


def generate_header(yaml_path, output_path):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    state_size = 0
    command_size = 0
    state_fields = []
    command_fields = []

    for comp, fields in data.get("state_interfaces", {}).items():
        for field, ftype in fields.items():
            if ftype not in TYPE_MAP:
                print(f"Error: Unknown type '{ftype}' in state {comp}.{field}")
                sys.exit(1)
            ctype, size = TYPE_MAP[ftype]
            state_fields.append(f"  {ctype} {comp}_{field};")
            state_size += size

    for comp, fields in data.get("command_interfaces", {}).items():
        for field, ftype in fields.items():
            if ftype not in TYPE_MAP:
                print(f"Error: Unknown type '{ftype}' in command {comp}.{field}")
                sys.exit(1)
            ctype, size = TYPE_MAP[ftype]
            command_fields.append(f"  {ctype} {comp}_{field};")
            command_size += size

    header = f"""#ifndef ZENBEDDED_SCHEMA__GENERATED__INTERFACE_DATA_H_
#define ZENBEDDED_SCHEMA__GENERATED__INTERFACE_DATA_H_

#include <stdint.h>

#define ZENBEDDED_STATE_BYTE_SIZE {state_size}
#define ZENBEDDED_COMMAND_BYTE_SIZE {command_size}

#pragma pack(push, 1)
typedef struct
{{
{chr(10).join(state_fields)}
}} zenbedded_state_t;

typedef struct
{{
{chr(10).join(command_fields)}
}} zenbedded_command_t;
#pragma pack(pop)

#endif  // ZENBEDDED_SCHEMA__GENERATED__INTERFACE_DATA_H_
"""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    with open(output_path, "w") as f:
        f.write(header)
    print(f"Successfully generated Tier 2 payload header at: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate Zenbedded Tier 2 C-Header from YAML")
    parser.add_argument("--yaml", required=True, help="Path to input interface_schema.yaml")
    parser.add_argument("--output", required=True, help="Path to output interface_data.h")
    args = parser.parse_args()

    generate_header(args.yaml, args.output)
