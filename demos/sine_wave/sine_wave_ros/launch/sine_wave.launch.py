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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("zenbedded_sine_wave")

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")
    urdf_path = os.path.join(pkg_share, "urdf", "sine_wave.urdf")

    tier_arg = DeclareLaunchArgument(
        "tier",
        default_value="1",
        choices=["1", "2"],
        description="Demo tier: '1' for ROS 2 CDR topics, '2' for ros2_control hardware interface",
    )

    use_rqt_plot_arg = DeclareLaunchArgument(
        "use_rqt_plot",
        default_value="true",
        description="Launch rqt_plot for real-time wave visualization",
    )

    introspection_topic_arg = DeclareLaunchArgument(
        "introspection_topic",
        default_value="/controller_manager/introspection_data/values",
        description="Base introspection topic used for Tier 2 rqt_plot",
    )

    is_tier1 = PythonExpression(["'", LaunchConfiguration("tier"), "' == '1'"])
    is_tier2 = PythonExpression(["'", LaunchConfiguration("tier"), "' == '2'"])

    is_tier1_plot = PythonExpression(
        [
            "'",
            LaunchConfiguration("tier"),
            "' == '1' and '",
            LaunchConfiguration("use_rqt_plot"),
            "' == 'true'",
        ]
    )
    is_tier2_plot = PythonExpression(
        [
            "'",
            LaunchConfiguration("tier"),
            "' == '2' and '",
            LaunchConfiguration("use_rqt_plot"),
            "' == 'true'",
        ]
    )

    # =========================================================================
    # Tier 1 Components
    # =========================================================================
    sine_wave_tier1_node = Node(
        package="zenbedded_sine_wave",
        executable="sine_wave_tier1",
        output="screen",
        parameters=[controllers_yaml],
        condition=IfCondition(is_tier1),
    )

    rqt_plot_tier1 = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot_tier1",
        arguments=[
            "/zenbedded/sine_wave/state/position[0]",
            "/zenbedded/sine_wave/cmd/values[0]",
        ],
        output="screen",
        condition=IfCondition(is_tier1_plot),
    )

    # =========================================================================
    # Tier 2 Components (ros2_control)
    # =========================================================================
    robot_description_content = xacro.process_file(urdf_path).toxml()
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(is_tier2),
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen",
        condition=IfCondition(is_tier2),
    )

    delay_ros2_control_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[ros2_control_node],
        ),
        condition=IfCondition(is_tier2),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", controllers_yaml],
        condition=IfCondition(is_tier2),
    )

    delay_broadcaster_after_ros2_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(is_tier2),
    )

    sine_wave_tier2_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sine_wave_tier2", "--param-file", controllers_yaml],
        condition=IfCondition(is_tier2),
    )

    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[sine_wave_tier2_spawner],
        ),
        condition=IfCondition(is_tier2),
    )

    rqt_plot_tier2 = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot_tier2",
        arguments=[
            PythonExpression(["'", LaunchConfiguration("introspection_topic"), "/values[0]'"]),
            PythonExpression(["'", LaunchConfiguration("introspection_topic"), "/values[1]'"]),
        ],
        output="screen",
        condition=IfCondition(is_tier2_plot),
    )

    return LaunchDescription(
        [
            tier_arg,
            use_rqt_plot_arg,
            introspection_topic_arg,
            # Tier 1
            sine_wave_tier1_node,
            rqt_plot_tier1,
            # Tier 2
            robot_state_publisher_node,
            delay_ros2_control_after_rsp,
            delay_broadcaster_after_ros2_control,
            delay_controller_after_broadcaster,
            rqt_plot_tier2,
        ]
    )
