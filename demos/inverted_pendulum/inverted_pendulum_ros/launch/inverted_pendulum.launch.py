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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    "xacro",
                                    " ",
                                    PathSubstitution(
                                        FindPackageShare("zenbedded_inverted_pendulum")
                                    )
                                    / "urdf"
                                    / "inverted_pendulum.urdf.xacro",
                                    " ",
                                    "use_mock_hardware:=",
                                    LaunchConfiguration("use_mock_hardware"),
                                ]
                            ),
                            value_type=str,
                        )
                    }
                ],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"update_rate": 100}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "inverted_pendulum_controller",
                    "joint_state_broadcaster",
                    "--param-file",
                    PathSubstitution(FindPackageShare("zenbedded_inverted_pendulum"))
                    / "config"
                    / "controllers.yaml",
                ],
            ),
        ]
    )
