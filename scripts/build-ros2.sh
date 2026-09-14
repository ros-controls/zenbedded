#!/usr/bin/env bash

set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_workspace="${ROS2_WORKSPACE:-/ros2_ws}"
ros_distro="${ROS_DISTRO:-lyrical}"

usage() {
    cat <<EOF
Install dependencies and build the mounted ROS 2 workspace.

Usage:
  $0 [colcon build options]

Environment:
  ROS2_WORKSPACE ROS 2 workspace (default: /ros2_ws)
  ROS_DISTRO    ROS distribution (default: lyrical)
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    exit 0
fi

[[ -f "/opt/ros/$ros_distro/setup.bash" ]] || { echo "Error: ROS 2 setup not found: /opt/ros/$ros_distro/setup.bash" >&2; exit 1; }
mkdir -p "$ros_workspace/src"

# CI checks out the repository separately; compose mounts these packages directly.
for package in zenbedded_transport zenbedded_hardware_interface; do
    source_package="$repository_root/$package"
    target_package="$ros_workspace/src/$package"
    if [[ -d "$source_package" && ! -e "$target_package" ]]; then
        ln -s "$source_package" "$target_package"
    fi
done

# shellcheck disable=SC1091
source "/opt/ros/$ros_distro/setup.bash"
cd "$ros_workspace"
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build "$@"