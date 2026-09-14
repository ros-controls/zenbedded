#!/usr/bin/env bash

set -euo pipefail

zephyr_workspace="${ZEPHYR_WORKSPACE:-/zephyr_ws}"
zephyr_base="${ZEPHYR_BASE:-${zephyr_workspace}/zephyr}"
ros_workspace="${ROS2_WORKSPACE:-/ros2_ws}"
ros_distro="${ROS_DISTRO:-lyrical}"
testsuite_root="${TWISTER_TESTSUITE_ROOT:-${zephyr_workspace}/demos}"
outdir="${TWISTER_OUTDIR:-twister-out}"
test_selector=""

usage() {
    cat <<EOF
Run the repository's Zephyr Twister test matrix.

Usage:
  $0 [test name] [twister options]

Examples:
  $0 zenoh_e2e
  $0 rcl_e2e/tier1 --build-only
  $0

Environment:
  ZEPHYR_WORKSPACE      Zephyr workspace (default: /zephyr_ws)
  ROS2_WORKSPACE        ROS 2 workspace (default: /ros2_ws)
  ROS_DISTRO            ROS 2 distribution (default: lyrical)
  TWISTER_TESTSUITE_ROOT Test root (default: /zephyr_ws/demos)
  TWISTER_OUTDIR        Twister output directory (default: twister-out)
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    exit 0
fi

if [[ -n "${1:-}" && "${1:0:1}" != "-" ]]; then
  test_selector="$1"
  shift
  testsuite_root="$testsuite_root/$test_selector"
fi

[[ -x "$zephyr_base/scripts/twister" ]] || { echo "Error: Twister not found at $zephyr_base/scripts/twister" >&2; exit 1; }
[[ -d "$testsuite_root" ]] || {
  if [[ -n "$test_selector" ]]; then
    echo "Error: test selector not found: $test_selector" >&2
  else
    echo "Error: test suite root not found: $testsuite_root" >&2
  fi
  exit 1
}

# Twister's host-side pytest suites invoke ros2 directly.
[[ -f "/opt/ros/$ros_distro/setup.bash" ]] || { echo "Error: ROS 2 setup not found: /opt/ros/$ros_distro/setup.bash" >&2; exit 1; }
# shellcheck disable=SC1091
set +u
source "/opt/ros/$ros_distro/setup.bash"
if [[ -f "$ros_workspace/install/setup.bash" ]]; then
  # shellcheck disable=SC1091,SC1090
    source "$ros_workspace/install/setup.bash"
fi
set -u

export ZEPHYR_BASE="$zephyr_base"
rm -rf -- "$outdir"
"$ZEPHYR_BASE/scripts/twister" \
    --testsuite-root "$testsuite_root" \
    --platform native_sim/native/64 \
    --platform esp32s3_devkitc/esp32s3/procpu \
    --inline-logs \
    -v \
    --outdir "$outdir" \
    --jobs 1 \
    "$@"