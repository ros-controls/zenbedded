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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("zenbedded_sine_wave")
    main_launch = os.path.join(pkg_share, "launch", "sine_wave.launch.py")

    use_rqt_plot_arg = DeclareLaunchArgument(
        "use_rqt_plot",
        default_value="true",
        description="Launch rqt_plot for real-time wave visualization",
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch),
        launch_arguments={
            "tier": "1",
            "use_rqt_plot": LaunchConfiguration("use_rqt_plot"),
        }.items(),
    )

    return LaunchDescription([use_rqt_plot_arg, included_launch])
