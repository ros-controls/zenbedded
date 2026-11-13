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
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = PathSubstitution(FindPackageShare("zenbedded_inverted_pendulum"))
    controllers_yaml = pkg_share / "config" / "controllers.yaml"
    urdf_path = pkg_share / "urdf" / "inverted_pendulum.urdf.xacro"
    rviz_config = pkg_share / "config" / "pendulum.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tier",
                default_value="2",
                choices=["1", "2"],
                description="Hardware tier: '1' for ROS 2 topics (CDR), '2' for Zenbedded binary struct",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "enable_kicking",
                default_value="true",
                description="Enable periodic random kicks to the mock pendulum in simulation.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 automatically.",
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
                                    urdf_path,
                                    " ",
                                    "tier:=",
                                    LaunchConfiguration("tier"),
                                    " ",
                                    "use_mock_hardware:=",
                                    LaunchConfiguration("use_mock_hardware"),
                                    " ",
                                    "enable_kicking:=",
                                    LaunchConfiguration("enable_kicking"),
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
                    controllers_yaml,
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
