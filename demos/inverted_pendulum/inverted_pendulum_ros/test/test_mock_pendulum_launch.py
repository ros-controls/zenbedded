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

import os
import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

import launch_testing.markers
import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces

from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)


# Executes the given launch file and checks if all nodes can be started
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
                launch_arguments={"use_mock_hardware": "True"}.items(),
            ),
            ReadyToTest(),
        ]
    )


class TestMockPendulumHardware(unittest.TestCase):
    """Integration test: launch mock hardware + controller, verify everything works."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_mock_pendulum")

    def tearDown(self):
        self.node.destroy_node()

    def _call_list_controllers(self, timeout=5.0):
        """Call /controller_manager/list_controllers and return the response."""
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
        """Call /controller_manager/list_hardware_interfaces."""
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

    def _wait_for_controller_active(self, name, timeout=15.0):
        """Poll list_controllers until the named controller is active."""
        start = time.time()
        while time.time() - start < timeout:
            result = self._call_list_controllers()
            if result:
                for ctrl in result.controller:
                    if ctrl.name == name and ctrl.state == "active":
                        return ctrl, []
            time.sleep(0.5)
        result = self._call_list_controllers()
        names = [c.name for c in result.controller] if result else []
        return None, names

    def test_node_start(self):
        check_node_running(self.node, "robot_state_publisher")

    def test_controller_running(self, proc_info):

        cnames = ["inverted_pendulum_controller", "joint_state_broadcaster"]

        check_controllers_running(self.node, cnames)

        # Wait for controller_spawner to finish and verify successful exit.
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

        # Re-check controllers after spawner has exited.
        check_controllers_running(self.node, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", ["motor_joint", "pendulum_joint"])

    def test_hardware_interfaces_configured(self):
        """Mock hardware should expose correct state and command interfaces."""
        result = self._call_list_hardware_interfaces()
        self.assertIsNotNone(result, "Failed to call list_hardware_interfaces")

        state_names = [i.name for i in result.state_interfaces]
        command_names = [i.name for i in result.command_interfaces]

        self.assertIn("motor_joint/position", state_names)
        self.assertIn("motor_joint/velocity", state_names)
        self.assertIn("pendulum_joint/position", state_names)
        self.assertIn("pendulum_joint/velocity", state_names)
        self.assertIn("motor_joint/acceleration", command_names)

    def test_controller_writes_commands(self):
        """Active controller should claim the acceleration command interface."""
        ctrl, _ = self._wait_for_controller_active("inverted_pendulum_controller")
        self.assertIsNotNone(ctrl, "Controller not active")
        self.assertEqual(ctrl.state, "active")
        self.assertIn("motor_joint/acceleration", ctrl.claimed_interfaces)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify all processes exit cleanly after shutdown."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=(0, -2, 1),
        )
