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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = PathSubstitution(FindPackageShare("zenbedded_sine_wave"))
    controllers_yaml = pkg_share / "config" / "controllers.yaml"
    urdf_path = pkg_share / "urdf" / "sine_wave.urdf"

    is_tier1 = PythonExpression(["'", LaunchConfiguration("tier"), "' == '1'"])
    is_plot = PythonExpression(["'", LaunchConfiguration("use_rqt_plot"), "' == 'true'"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", urdf_path]),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tier",
                default_value="1",
                choices=["1", "2"],
                description="Demo tier: '1' for ROS 2 CDR topics, '2' for ros2_control hardware interface",
            ),
            DeclareLaunchArgument(
                "use_rqt_plot",
                default_value="true",
                description="Launch rqt_plot for real-time wave visualization",
            ),
            DeclareLaunchArgument(
                "introspection_topic",
                default_value="/controller_manager/introspection_data/values",
                description="Base introspection topic used for Tier 2 rqt_plot",
            ),
            # Tier 1
            Node(
                package="zenbedded_sine_wave",
                executable="sine_wave_tier1",
                output="screen",
                parameters=[controllers_yaml],
                condition=IfCondition(is_tier1),
            ),
            Node(
                package="rqt_plot",
                executable="rqt_plot",
                name="rqt_plot_tier1",
                arguments=[
                    "/zenbedded/sine_wave/state/position[0]",
                    "/zenbedded/sine_wave/cmd/values[0]",
                ],
                output="screen",
                condition=IfCondition(PythonExpression([is_tier1, " and ", is_plot])),
            ),
            # Tier 2
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
                condition=UnlessCondition(is_tier1),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_yaml],
                output="screen",
                condition=UnlessCondition(is_tier1),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "sine_wave_tier2",
                    "joint_state_broadcaster",
                    "--param-file",
                    controllers_yaml,
                ],
                condition=UnlessCondition(is_tier1),
            ),
            Node(
                package="rqt_plot",
                executable="rqt_plot",
                name="rqt_plot_tier2",
                arguments=[
                    PythonExpression(
                        ["'", LaunchConfiguration("introspection_topic"), "/values[4]'"]
                    ),
                    PythonExpression(
                        ["'", LaunchConfiguration("introspection_topic"), "/values[5]'"]
                    ),
                ],
                output="screen",
                condition=IfCondition(PythonExpression(["not ", is_tier1, " and ", is_plot])),
            ),
        ]
    )
