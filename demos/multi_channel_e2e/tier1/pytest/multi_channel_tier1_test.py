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

import time

from twister_harness import DeviceAdapter

TIMEOUT_SEC = 10.0


def test_tier1_mcu_publishes_multi_channel(zenoh_router, dut: DeviceAdapter, host_context):
    """TEST 1: MCU to Host Multi-Channel Publication."""
    # 1. Wait for MCU to initialize and enter event loop first
    dut.readlines_until(regex=r"\[SYS\] Entering Multi-Channel Event Loop", timeout=TIMEOUT_SEC)

    # 2. Boot ROS 2 Python Node after MCU is ready
    with host_context() as node:
        # 3. Verify Arm joint states
        arm_msg = node.arm_state_queue.get(timeout=TIMEOUT_SEC)
        assert list(arm_msg.name) == ["shoulder", "elbow"]
        assert arm_msg.header.frame_id == "arm_base"
        assert len(arm_msg.position) == 2
        assert len(arm_msg.velocity) == 2
        assert arm_msg.position[0] > 0.0

        # 4. Verify Gripper joint states
        gripper_msg = node.gripper_state_queue.get(timeout=TIMEOUT_SEC)
        assert list(gripper_msg.name) == ["finger"]
        assert gripper_msg.header.frame_id == "gripper_base"
        assert len(gripper_msg.position) == 1
        assert gripper_msg.position[0] > 0.0


def test_tier1_host_commands_multi_channel(zenoh_router, dut: DeviceAdapter, host_context):
    """TEST 2: Host to MCU Multi-Channel Commands."""
    # 1. Wait for MCU boot
    dut.readlines_until(regex=r"\[SYS\] Entering Multi-Channel Event Loop", timeout=TIMEOUT_SEC)

    # 2. Boot ROS 2 Python Node
    with host_context() as node:
        time.sleep(0.5)

        timer_arm = node.create_timer(
            0.1,
            lambda: node.publish_arm_cmd(["shoulder", "elbow"], "position", [1.57, -0.78]),
        )
        timer_gripper = node.create_timer(
            0.1,
            lambda: node.publish_gripper_cmd(["finger"], "effort", [12.50]),
        )

        try:
            dut.readlines_until(
                regex=r"\[CB 1\] Tier 1 Arm Cmd: .* val0=1\.57",
                timeout=TIMEOUT_SEC,
            )
            dut.readlines_until(
                regex=r"\[CB 2\] Tier 1 Gripper Cmd: .* val0=12\.50",
                timeout=TIMEOUT_SEC,
            )
        finally:
            node.destroy_timer(timer_arm)
            node.destroy_timer(timer_gripper)


def test_tier1_bidirectional_roundtrip(zenoh_router, dut: DeviceAdapter, host_context):
    """TEST 3: Simultaneous Back-and-Forth Multi-Channel Validation."""
    dut.readlines_until(regex=r"\[SYS\] Entering Multi-Channel Event Loop", timeout=TIMEOUT_SEC)

    with host_context() as node:
        # Validate egress
        arm_msg = node.arm_state_queue.get(timeout=TIMEOUT_SEC)
        gripper_msg = node.gripper_state_queue.get(timeout=TIMEOUT_SEC)
        assert len(arm_msg.position) == 2
        assert len(gripper_msg.position) == 1

        # Validate ingress
        timer_arm = node.create_timer(
            0.1,
            lambda: node.publish_arm_cmd(["shoulder", "elbow"], "position", [3.14, 0.0]),
        )
        timer_gripper = node.create_timer(
            0.1,
            lambda: node.publish_gripper_cmd(["finger"], "effort", [25.0]),
        )

        try:
            dut.readlines_until(
                regex=r"\[CB 1\] Tier 1 Arm Cmd: .* val0=3\.14",
                timeout=TIMEOUT_SEC,
            )
            dut.readlines_until(
                regex=r"\[CB 2\] Tier 1 Gripper Cmd: .* val0=25\.00",
                timeout=TIMEOUT_SEC,
            )
        finally:
            node.destroy_timer(timer_arm)
            node.destroy_timer(timer_gripper)
