#!/bin/bash
# Launch D-LIO TF visualization for validating GO2-W sensor extrinsics.
#
# Usage:
#   bash scripts/diagnosis/check_tf.sh [--tf-profile legacy|urdf-imu] [--rviz false|true]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ=true
TF_PROFILE=legacy
DLIO_CONFIG=""
DLIO_PARAMS_CONFIG=""
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/humble_ws/install/setup.bash"
    "$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash"
)

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

select_tf_profile() {
    local cfg_dir="$REPO_ROOT/humble_ws/src/direct_lidar_inertial_odometry/cfg"

    case "$TF_PROFILE" in
        legacy)
            DLIO_CONFIG="$cfg_dir/dlio.yaml"
            DLIO_PARAMS_CONFIG="$cfg_dir/params.yaml"
            ;;
        urdf-imu)
            DLIO_CONFIG="$cfg_dir/dlio_urdf_imu.yaml"
            DLIO_PARAMS_CONFIG="$cfg_dir/params_urdf_imu.yaml"
            ;;
        *)
            echo "Error: --tf-profile must be legacy or urdf-imu." >&2
            exit 1
            ;;
    esac

    for config in "$DLIO_CONFIG" "$DLIO_PARAMS_CONFIG"; do
        if [ ! -f "$config" ]; then
            echo "Error: TF profile config not found: $config" >&2
            echo "Update submodules and rebuild the workspace, then rerun this script." >&2
            exit 1
        fi
    done
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --tf-profile)
            TF_PROFILE="${2:?Error: --tf-profile requires a value}"
            shift 2
            ;;
        --rviz)
            RVIZ="${2:?Error: --rviz requires a value}"
            shift 2
            ;;
        -h|--help)
            sed -n '2,6p' "$0"
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            echo "Usage: bash scripts/diagnosis/check_tf.sh [--tf-profile legacy|urdf-imu] [--rviz false|true]" >&2
            exit 1
            ;;
    esac
done

case "$RVIZ" in
    true|false)
        ;;
    *)
        echo "Error: --rviz must be true or false." >&2
        exit 1
        ;;
esac

select_tf_profile

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    source_setup_safely "$ROS_SETUP"
fi

for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ -f "$candidate" ]; then
        WS_SETUP="$candidate"
        break
    fi
done

if [ -z "$WS_SETUP" ]; then
    echo "Error: no D-LIO workspace setup was found." >&2
    echo "Looked for:" >&2
    for candidate in "${WS_SETUP_CANDIDATES[@]}"; do
        echo "  - $candidate" >&2
    done
    echo "Build the workspace first, then rerun this script." >&2
    exit 1
fi

source_setup_safely "$WS_SETUP"

if ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1; then
    echo "Error: direct_lidar_inertial_odometry is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    exit 1
fi

if [ "$RVIZ" = true ] && [ -z "${DISPLAY:-}" ]; then
    echo "Error: DISPLAY is not set. Start this from a desktop session or fix X11 forwarding." >&2
    exit 1
fi

echo "D-LIO setup: $WS_SETUP"
echo "TF profile: $TF_PROFILE"
echo "D-LIO config: $DLIO_CONFIG"
echo "Params config: $DLIO_PARAMS_CONFIG"
echo "RViz: $RVIZ"

ros2 launch direct_lidar_inertial_odometry check_tf.launch.py \
    use_rviz:="$RVIZ" \
    dlio_config:="$DLIO_CONFIG" \
    params_config:="$DLIO_PARAMS_CONFIG"
