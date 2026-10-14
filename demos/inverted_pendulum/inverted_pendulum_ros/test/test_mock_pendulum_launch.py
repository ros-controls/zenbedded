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

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory("zenbedded_inverted_pendulum")

    urdf_path = os.path.join(pkg_share, "urdf", "inverted_pendulum_mock.urdf")
    with open(urdf_path) as f:
        urdf_content = f.read()

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf_content}],
        output="both",
    )

    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="both",
    )

    controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "inverted_pendulum_controller",
            "--param-file",
            controllers_yaml,
        ],
        output="both",
    )

    return launch.LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            controller_spawner,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMockPendulumHardware(unittest.TestCase):
    """Integration test: launch mock hardware + controller, verify everything works."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_mock_pendulum")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_node(self, node_name, timeout=10.0):
        """Wait for a node to appear in the graph."""
        start = time.time()
        while time.time() - start < timeout:
            if node_name in self.node.get_node_names():
                return True
            time.sleep(0.1)
        return False

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

    def test_controller_manager_starts(self):
        """controller_manager node should be running."""
        self.assertTrue(
            self._wait_for_node("controller_manager"),
            "controller_manager node not found",
        )

    def test_controller_active(self):
        """inverted_pendulum_controller should reach active state."""
        ctrl, available = self._wait_for_controller_active("inverted_pendulum_controller")
        if ctrl is None:
            self.fail(f"inverted_pendulum_controller not active. " f"Available: {available}")
        self.assertEqual(ctrl.state, "active")

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
            allowable_exit_codes=(0, -2),
        )
