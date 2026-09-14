#!/usr/bin/env bash

set -euo pipefail

zephyr_workspace="${ZEPHYR_WORKSPACE:-/zephyr_ws}"
zephyr_env="${ZEPHYR_ENV_SCRIPT:-${zephyr_workspace}/zephyr/zephyr-env.sh}"
firmware_app="${FIRMWARE_APP:-demos/rcl_cpp_test}"
firmware_board="${FIRMWARE_BOARD:-esp32s3_devkitc/esp32s3/procpu}"
if [[ -n "${FIRMWARE_BUILD_DIR:-}" ]]; then
  build_dir="$FIRMWARE_BUILD_DIR"
elif [[ -w "$zephyr_workspace" ]]; then
  build_dir="${zephyr_workspace}/build/firmware"
else
  build_dir="${WORKSPACE_ROOT:-/workspace}/build/firmware"
fi

usage() {
    cat <<EOF
Build a Zephyr firmware application.

Usage:
  $0 [west build options]

Environment:
  ZEPHYR_WORKSPACE   Zephyr workspace (default: /zephyr_ws)
  FIRMWARE_APP       Application path relative to the workspace
                     (default: demos/rcl_cpp_test)
  FIRMWARE_BOARD     Zephyr board (default: esp32s3_devkitc/esp32s3/procpu)
  FIRMWARE_BUILD_DIR Build output directory (default: /zephyr_ws/build/firmware,
                     or /workspace/build/firmware when /zephyr_ws is read-only)
  WORKSPACE_ROOT     Writable fallback root (default: /workspace)

Examples:
  FIRMWARE_APP=demos/sine_wave/sine_wave_zephyr $0
  FIRMWARE_BOARD=native_sim/native/64 $0
  $0 -- -DWIFI_SSID=YourSSID
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    exit 0
fi

[[ -d "$zephyr_workspace" ]] || { echo "Error: Zephyr workspace not found: $zephyr_workspace" >&2; exit 1; }
[[ -f "$zephyr_env" ]] || { echo "Error: Zephyr environment script not found: $zephyr_env" >&2; exit 1; }
[[ -d "$zephyr_workspace/$firmware_app" ]] || { echo "Error: firmware application not found: $zephyr_workspace/$firmware_app" >&2; exit 1; }

# The image initializes these repositories as root, while the dev container may
# run this script as another user. Trust only the repositories in this workspace
# so West can resolve the imported Zephyr manifest.
git config --global --add safe.directory "$zephyr_workspace/manifest-repo"
git config --global --add safe.directory "$zephyr_workspace/zephyr"

# west build is a Zephyr extension loaded from the active Zephyr manifest.
# shellcheck disable=SC1090
source "$zephyr_env"
cd "$zephyr_workspace"
west build \
    --pristine=always \
    --board "$firmware_board" \
    "$firmware_app" \
    --build-dir "$build_dir" \
    "$@"