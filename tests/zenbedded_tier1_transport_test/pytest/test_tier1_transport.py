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
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointCommand
from twister_harness import DeviceAdapter

TIMEOUT_SEC = 5.0


# --- ROS 2 HOST NODE ---
class TransportTestNode(Node):
    def __init__(self):
        super().__init__("zenbedded_pytest_host")
        self.rx_queues = {"joint_states": queue.Queue()}

        self.sub_1 = self.create_subscription(
            JointState, "/joint_states", lambda msg: self.rx_queues["joint_states"].put(msg), 10
        )

        self.pub_cmd_1 = self.create_publisher(JointCommand, "/joint_commands", 10)


# --- THE TESTS ---


def test_mcu_publishes_to_ros2(zenoh_router, dut: DeviceAdapter):
    """TEST A: MCU to Host Single Publisher."""
    # 1. BOOT MCU FIRST: Let it claim the network ports
    dut.readlines_until(regex=r"\[SYS\] Entering High-Frequency Event Loop", timeout=TIMEOUT_SEC)

    # 2. NOW boot ROS 2 Python Node
    rclpy.init()
    node = TransportTestNode()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    try:
        msg_joints = node.rx_queues["joint_states"].get(timeout=TIMEOUT_SEC)

        # SMART ASSERTIONS:
        assert list(msg_joints.name) == ["stepper", "pendulum"]
        assert len(msg_joints.position) == 2
        assert msg_joints.position[1] == 0.0  # Pendulum position is static in main.cpp

    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


def test_ros2_commands_mcu(zenoh_router, dut: DeviceAdapter):
    """TEST B: Host to MCU Single Subscriber."""
    # 1. BOOT MCU FIRST
    dut.readlines_until(regex=r"\[SYS\] Entering High-Frequency Event Loop", timeout=TIMEOUT_SEC)

    # 2. NOW boot ROS 2 Python Node
    rclpy.init()
    node = TransportTestNode()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    try:
        cmd1 = JointCommand(
            joint_names=["stepper", "pendulum"], interface_name="position", values=[45.0, -90.5]
        )
        cmd1.header.frame_id = "base_link"

        time.sleep(0.5)

        timer = node.create_timer(0.1, lambda: node.pub_cmd_1.publish(cmd1))

        try:
            dut.readlines_until(
                regex=r"\[SUB 1\] Rate: \d+ Hz \| stepper: 45\.00 \| pendulum: -90\.50",
                timeout=TIMEOUT_SEC,
            )
        finally:
            node.destroy_timer(timer)

    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)
