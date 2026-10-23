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

import queue
import threading
import time

import pytest
import rclpy
from control_msgs.msg import JointCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from twister_harness import DeviceAdapter

TIMEOUT_SEC = 10.0


class MultiChannelHostNode(Node):
    def __init__(self):
        super().__init__("multi_channel_host_node")
        self.arm_state_queue = queue.Queue()
        self.gripper_state_queue = queue.Queue()

        self.sub_arm = self.create_subscription(
            JointState,
            "/robot/arm/joint_states",
            lambda msg: self.arm_state_queue.put(msg),
            10,
        )
        self.sub_gripper = self.create_subscription(
            JointState,
            "/robot/gripper/joint_states",
            lambda msg: self.gripper_state_queue.put(msg),
            10,
        )

        self.pub_arm_cmd = self.create_publisher(JointCommand, "/robot/arm/joint_cmd", 10)
        self.pub_gripper_cmd = self.create_publisher(JointCommand, "/robot/gripper/joint_cmd", 10)

    def publish_arm_cmd(self, joint_names, interface_name, values):
        cmd = JointCommand(
            joint_names=joint_names,
            interface_name=interface_name,
            values=values,
        )
        cmd.header.frame_id = "arm_base"
        self.pub_arm_cmd.publish(cmd)

    def publish_gripper_cmd(self, joint_names, interface_name, values):
        cmd = JointCommand(
            joint_names=joint_names,
            interface_name=interface_name,
            values=values,
        )
        cmd.header.frame_id = "gripper_base"
        self.pub_gripper_cmd.publish(cmd)


class MultiChannelHostContext:
    def __enter__(self):
        rclpy.init()
        self.node = MultiChannelHostNode()
        self.executor_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.executor_thread.start()
        return self.node

    def __exit__(self, exc_type, exc_val, exc_tb):
        rclpy.shutdown()
        self.executor_thread.join(timeout=1.0)


@pytest.fixture
def host_context(zenoh_router):
    return MultiChannelHostContext


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
