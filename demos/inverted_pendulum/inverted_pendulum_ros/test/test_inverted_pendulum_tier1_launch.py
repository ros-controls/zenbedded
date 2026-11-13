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

import math
import os
import time
import unittest

from control_msgs.msg import JointCommand
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import launch_testing
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running, check_node_running


@pytest.mark.rostest
def generate_test_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("zenbedded_inverted_pendulum"),
                        "launch/inverted_pendulum.launch.py",
                    )
                ),
                launch_arguments={
                    "tier": "1",
                    "use_mock_hardware": "False",
                    "use_rviz": "False",
                }.items(),
            ),
            ReadyToTest(),
        ]
    )


class TestInvertedPendulumTier1Hardware(unittest.TestCase):
    """Integration test: launch Tier 1 topic-based hardware + controller and test relay loop."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_tier1_pendulum")

    def tearDown(self):
        self.node.destroy_node()

    def _call_list_controllers(self, timeout=5.0):
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        try:
            client.wait_for_service(timeout_sec=timeout)
            req = ListControllers.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def _call_list_hardware_interfaces(self, timeout=5.0):
        client = self.node.create_client(
            ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces"
        )
        try:
            client.wait_for_service(timeout_sec=timeout)
            req = ListHardwareInterfaces.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def test_node_start(self):
        check_node_running(self.node, "robot_state_publisher")

    def test_controller_running(self, proc_info):
        cnames = ["inverted_pendulum_controller", "joint_state_broadcaster"]
        check_controllers_running(self.node, cnames)

        proc_info.assertWaitForShutdown(
            process="spawner", cmd_args="joint_state_broadcaster", timeout=30
        )
        launch_testing.asserts.assertExitCodes(
            proc_info, process="spawner", cmd_args="joint_state_broadcaster"
        )
        proc_info.assertWaitForShutdown(
            process="spawner", cmd_args="inverted_pendulum_controller", timeout=30
        )
        launch_testing.asserts.assertExitCodes(
            proc_info, process="spawner", cmd_args="inverted_pendulum_controller"
        )

        check_controllers_running(self.node, cnames)

    def test_hardware_interfaces_configured(self):
        result = self._call_list_hardware_interfaces()
        self.assertIsNotNone(result, "Failed to call list_hardware_interfaces")

        state_names = [i.name for i in result.state_interfaces]
        command_names = [i.name for i in result.command_interfaces]

        self.assertIn("motor_joint/position", state_names)
        self.assertIn("motor_joint/velocity", state_names)
        self.assertIn("pendulum_joint/position", state_names)
        self.assertIn("pendulum_joint/velocity", state_names)
        self.assertIn("motor_joint/acceleration", command_names)

    def test_topic_relay_loop(self):
        """Publish synthetic state, verify controller receives it and publishes acceleration cmd."""
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        state_pub = self.node.create_publisher(
            JointState, "/zenbedded/inverted_pendulum/state", sensor_qos
        )

        received_cmds = []

        def cmd_cb(msg: JointCommand):
            received_cmds.append(msg)

        cmd_sub = self.node.create_subscription(
            JointCommand, "/zenbedded/inverted_pendulum/cmd", cmd_cb, 10
        )

        # Publish periodic state messages (pendulum at 0.05 rad - within catch window)
        start_time = time.time()
        timeout = 10.0
        while time.time() - start_time < timeout and not received_cmds:
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = ["motor_joint", "pendulum_joint"]
            msg.position = [0.0, 0.05]
            msg.velocity = [0.0, 0.0]
            state_pub.publish(msg)

            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.node.destroy_subscription(cmd_sub)
        self.node.destroy_publisher(state_pub)

        self.assertTrue(
            len(received_cmds) > 0,
            "Did not receive any JointCommand on /zenbedded/inverted_pendulum/cmd",
        )
        last_cmd = received_cmds[-1]
        self.assertEqual(last_cmd.interface_name, "acceleration")
        self.assertIn("motor_joint", last_cmd.joint_names)
        self.assertTrue(len(last_cmd.values) > 0)
        self.assertFalse(math.isnan(last_cmd.values[0]))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=(0, -2, 1),
        )
