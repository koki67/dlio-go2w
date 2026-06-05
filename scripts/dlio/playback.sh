#!/bin/bash
# Play a recorded SLAM bag and open RViz2 in one command.
#
# Usage (from anywhere inside the workspace):
#   bash scripts/dlio/playback.sh <bag_directory> [ros2 bag play args...]
#
# Examples:
#   bash scripts/dlio/playback.sh humble_ws/bags/dlio_20250301_143022
#   bash scripts/dlio/playback.sh humble_ws/bags/dlio_20250301_143022 --rate 2.0

set -eo pipefail

BAG="${1:?Error: bag path required. Usage: $0 <bag_directory> [ros2 bag play args...]}"
shift
EXTRA_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --loop)
            echo "Error: --loop is not supported in this playback helper." >&2
            exit 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            ;;
    esac
    shift
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO_ROOT/config/dlio/dlio.rviz"
ROS_SETUP="/opt/ros/humble/setup.bash"
DESKTOP_SETUP="$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash"

source_setup_safely() {
    local setup_script="$1"
    local restore_nounset=0
    local rc=0

    case $- in
        *u*)
            restore_nounset=1
            set +u
            ;;
    esac

    export COLCON_TRACE="${COLCON_TRACE-}"
    # shellcheck source=/dev/null
    source "$setup_script" || rc=$?

    if [ "$restore_nounset" -eq 1 ]; then
        set -u
    fi

    return "$rc"
}

# Resolve repo-relative bag paths for convenience.
if [ ! -d "$BAG" ] && [ -d "$REPO_ROOT/$BAG" ]; then
    BAG="$REPO_ROOT/$BAG"
fi

if [ ! -d "$BAG" ]; then
    echo "Error: bag directory not found: $BAG" >&2
    exit 1
fi

if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "Error: metadata.yaml not found in bag directory: $BAG" >&2
    exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

# Some shells export ROS_DISTRO without actually adding ros2/rviz2 to PATH.
# Source ROS when the commands we need are missing, not only when ROS_DISTRO is empty.
if ! command -v ros2 >/dev/null 2>&1 || ! command -v rviz2 >/dev/null 2>&1; then
    source_setup_safely "$ROS_SETUP"
fi

if [ -f "$DESKTOP_SETUP" ]; then
    source_setup_safely "$DESKTOP_SETUP"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 not found in PATH after sourcing $ROS_SETUP" >&2
    echo "Open this repository in the desktop devcontainer, or rerun: bash .devcontainer/postCreate.sh" >&2
    exit 1
fi

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "Error: rviz2 not found in PATH after sourcing $ROS_SETUP" >&2
    echo "This script requires ROS 2 Humble Desktop (ros-humble-desktop). Ensure the correct ROS setup is sourced." >&2
    exit 1
fi

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo ""

# Start bag player in the background.
PLAY_ARGS=(--clock)
PLAY_ARGS+=("${EXTRA_ARGS[@]}")
ros2 bag play "$BAG" "${PLAY_ARGS[@]}" &
BAG_PID=$!

# When this script exits (rviz2 closed, Ctrl+C, or any error),
# kill the bag player so it does not linger in the background.
trap "echo 'Stopping bag player...'; kill $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null" EXIT

# Give the bag player a moment to start publishing before RViz2 opens.
sleep 2

# Run RViz2 in the foreground; the script ends when the window is closed.
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
