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
import xacro
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("zenbedded_inverted_pendulum")

    robot_description_content = xacro.process_file(
        os.path.join(pkg_share, "urdf", "inverted_pendulum_mock.urdf"),
    ).toxml()

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen",
    )

    delay_ros2_control_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[ros2_control_node],
        )
    )

    inverted_pendulum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "inverted_pendulum_controller",
            "--param-file",
            controllers_yaml,
        ],
    )

    delay_controller_after_ros2_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[inverted_pendulum_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            delay_ros2_control_after_rsp,
            delay_controller_after_ros2_control,
        ]
    )
