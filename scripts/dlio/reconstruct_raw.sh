#!/bin/bash
# Reconstruct D-LIO outputs from a raw sensor bag and open RViz2.
#
# Usage (from anywhere inside the repository):
#   bash scripts/dlio/reconstruct_raw.sh [--tf-profile legacy|urdf-imu|urdf-imu-lidar-legacy] <bag_directory> [ros2 bag play args...]
#
# Default TF profile: urdf-imu-lidar-legacy
#
# Examples:
#   bash scripts/dlio/reconstruct_raw.sh humble_ws/bags/raw_20260312_024403
#   bash scripts/dlio/reconstruct_raw.sh --tf-profile legacy humble_ws/bags/raw_20260312_024403
#   bash scripts/dlio/reconstruct_raw.sh humble_ws/bags/raw_20260312_024403 --rate 2.0

set -eo pipefail

TF_PROFILE=urdf-imu-lidar-legacy
BAG=""
EXTRA_ARGS=()

usage() {
    sed -n "2,12p" "$0"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --tf-profile)
            TF_PROFILE="${2:?Error: --tf-profile requires a value}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            if [ -z "$BAG" ]; then
                echo "Error: bag path required before --." >&2
                usage >&2
                exit 2
            fi
            EXTRA_ARGS+=("$@")
            break
            ;;
        *)
            if [ -z "$BAG" ]; then
                if [[ "$1" == -* ]]; then
                    echo "Error: unknown option before bag path: $1" >&2
                    usage >&2
                    exit 2
                fi
                BAG="$1"
            else
                EXTRA_ARGS+=("$1")
            fi
            shift
            ;;
    esac
done

if [ -z "$BAG" ]; then
    echo "Error: bag path required." >&2
    usage >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
RVIZ_CFG="$REPO_ROOT/config/dlio/dlio.rviz"
DLIO_CONFIG=""
DLIO_PARAMS_CONFIG=""
WS_SETUP=""
WS_SETUP_CANDIDATES=(
    "$REPO_ROOT/humble_ws/install/setup.bash"
    "$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash"
)

select_tf_profile() {
    local cfg_dir="$REPO_ROOT/humble_ws/src/direct_lidar_inertial_odometry/cfg"

    case "$TF_PROFILE" in
        legacy)
            DLIO_CONFIG="$cfg_dir/dlio_legacy.yaml"
            DLIO_PARAMS_CONFIG="$cfg_dir/params_legacy.yaml"
            ;;
        urdf-imu)
            DLIO_CONFIG="$cfg_dir/dlio_urdf_imu.yaml"
            DLIO_PARAMS_CONFIG="$cfg_dir/params_urdf_imu.yaml"
            ;;
        urdf-imu-lidar-legacy)
            DLIO_CONFIG="$cfg_dir/dlio.yaml"
            DLIO_PARAMS_CONFIG="$cfg_dir/params.yaml"
            ;;
        *)
            echo "Error: --tf-profile must be legacy, urdf-imu, or urdf-imu-lidar-legacy." >&2
            exit 2
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

select_tf_profile

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

# Parse topic list once; used for validation and --clock detection below.
bag_topics=$(python3 -c "
import yaml, sys
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
info = d.get('rosbag2_bagfile_information', {})
for t in info.get('topics_with_message_count', []):
    print(t['topic_metadata']['name'])
" "$BAG/metadata.yaml" 2>/dev/null) \
    || { echo "Error: failed to parse $BAG/metadata.yaml" >&2; exit 1; }

bag_input_counts=$(python3 -c "
import yaml, sys
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
info = d.get('rosbag2_bagfile_information', {})
for t in info.get('topics_with_message_count', []):
    m = t['topic_metadata']
    if m['name'] in ('/points_raw', '/go2w/imu'):
        print(f\"{m['name']}\t{t['message_count']}\")
" "$BAG/metadata.yaml" 2>/dev/null) || { echo "Error: failed to read input message counts from $BAG/metadata.yaml" >&2; exit 1; }

BAG_POINT_COUNT=""
BAG_IMU_COUNT=""
while IFS=$'\t' read -r topic count; do
    case "$topic" in
        /points_raw) BAG_POINT_COUNT="$count" ;;
        /go2w/imu) BAG_IMU_COUNT="$count" ;;
    esac
done <<< "$bag_input_counts"

for required in /points_raw /go2w/imu; do
    if ! grep -qxF "$required" <<< "$bag_topics"; then
        echo "Error: bag is missing required topic: $required" >&2
        echo "Topics found in bag:" >&2
        while IFS= read -r t; do echo "  $t" >&2; done <<< "$bag_topics"
        exit 1
    fi
done

# Add --clock only when the bag does not already contain /clock messages.
CLOCK_ARG=(--clock)
if grep -qxF "/clock" <<< "$bag_topics"; then
    CLOCK_ARG=()
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 setup not found: $ROS_SETUP" >&2
    exit 1
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
    echo "Create the desktop devcontainer or build the workspace first, then rerun this script." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WS_SETUP"

if ! ros2 pkg prefix direct_lidar_inertial_odometry >/dev/null 2>&1; then
    echo "Error: direct_lidar_inertial_odometry is not available after sourcing:" >&2
    echo "  $WS_SETUP" >&2
    if [ -f "$REPO_ROOT/.devcontainer/postCreate.sh" ]; then
        echo "In the desktop devcontainer, rerun: bash .devcontainer/postCreate.sh" >&2
    fi
    exit 1
fi

terminate_process_group() {
    local signal="$1"
    local pgid="${2:-}"
    if [ -n "$pgid" ]; then
        kill -"$signal" -- "-$pgid" 2>/dev/null || true
    fi
}

is_process_group_alive() {
    local pgid="${1:-}"
    [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null
}

cleanup() {
    echo "Stopping raw-bag D-LIO reconstruction..."
    # Each background ROS command is launched in its own process group. Signal only
    # those groups so unrelated D-LIO sessions in the same container are not touched.
    for pgid in "${BAG_PGID:-}" "${DLIO_PGID:-}"; do
        terminate_process_group INT "$pgid"
    done

    for _ in 1 2 3 4 5; do
        local any_alive=0
        for pgid in "${BAG_PGID:-}" "${DLIO_PGID:-}"; do
            if is_process_group_alive "$pgid"; then
                any_alive=1
                break
            fi
        done
        [ "$any_alive" -eq 0 ] && break
        sleep 1
    done

    for pgid in "${BAG_PGID:-}" "${DLIO_PGID:-}"; do
        terminate_process_group KILL "$pgid"
    done

    for pid in "${BAG_PID:-}" "${DLIO_PID:-}"; do
        if [ -n "${pid:-}" ]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT INT TERM

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "D-LIO setup: $WS_SETUP"
echo "TF profile: $TF_PROFILE"
echo "D-LIO config: $DLIO_CONFIG"
echo "Params config: $DLIO_PARAMS_CONFIG"
echo "Mode: replay raw topics, run D-LIO offline, visualize generated outputs"
echo "Expected D-LIO inputs: /points_raw=$BAG_POINT_COUNT, /go2w/imu=$BAG_IMU_COUNT"
echo ""

setsid ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=false \
    launch_drivers:=false \
    dlio_config:="$DLIO_CONFIG" \
    params_config:="$DLIO_PARAMS_CONFIG" \
    use_sim_time:=true \
    offline_replay:=true \
    pointcloud_topic:=points_raw \
    imu_topic:=go2w/imu &
DLIO_PID=$!
DLIO_PGID=$DLIO_PID

topic_has_subscriber() {
    local topic="$1"
    local info

    if ! info="$(ros2 topic info "$topic" 2>/dev/null)"; then
        return 1
    fi

    if grep -Eq "^Subscription count: [1-9][0-9]*$" <<< "$info"; then
        return 0
    fi
    return 1
}

topic_has_publisher_and_subscriber() {
    local topic="$1"
    local info

    if ! info="$(ros2 topic info "$topic" 2>/dev/null)"; then
        return 1
    fi

    if ! grep -Eq "^Publisher count: [1-9][0-9]*$" <<< "$info"; then
        return 1
    fi
    if grep -Eq "^Subscription count: [1-9][0-9]*$" <<< "$info"; then
        return 0
    fi
    return 1
}

service_has_type() {
    local service="$1"
    local expected_type="$2"
    local actual_type

    if ! actual_type="$(ros2 service type "$service" 2>/dev/null)"; then
        return 1
    fi
    [ "$actual_type" = "$expected_type" ]
}

wait_for_dlio_input_subscriptions() {
    echo -n "Waiting for D-LIO input subscriptions..."
    for _ in {1..50}; do
        if ! kill -0 "$DLIO_PID" 2>/dev/null; then
            echo " D-LIO exited." >&2
            return 2
        fi
        if topic_has_subscriber /points_raw; then
            if topic_has_subscriber /go2w/imu; then
                echo " ready"
                return 0
            fi
        fi
        sleep 0.1
    done

    echo " timed out." >&2
    return 1
}

wait_for_paused_bag_player() {
    echo -n "Waiting for paused bag player connections..."
    for _ in {1..50}; do
        if ! kill -0 "$BAG_PID" 2>/dev/null; then
            echo " player exited." >&2
            return 2
        fi
        if service_has_type /rosbag2_player/resume rosbag2_interfaces/srv/Resume; then
            if topic_has_publisher_and_subscriber /points_raw; then
                if topic_has_publisher_and_subscriber /go2w/imu; then
                    echo " ready"
                    return 0
                fi
            fi
        fi
        sleep 0.1
    done

    echo " timed out." >&2
    return 1
}

if ! wait_for_dlio_input_subscriptions; then
    echo "Error: D-LIO input subscriptions did not become ready." >&2
    if ! wait "$DLIO_PID" 2>/dev/null; then
        :
    fi
    exit 1
fi

# This is a fresh D-LIO process, so its state is already empty. Do not wait on
# reset services here: they may not be available during node startup and each
# timed-out call delays bag playback by five seconds.

setsid ros2 bag play "$BAG" --start-paused --wait-for-all-acked 1000 "${CLOCK_ARG[@]}" "${EXTRA_ARGS[@]}" &
BAG_PID=$!
BAG_PGID=$BAG_PID

if ! wait_for_paused_bag_player; then
    echo "Error: paused bag player did not establish all input connections." >&2
    exit 1
fi

# Resume only after the player publishers and D-LIO input subscriptions are
# present. This keeps the first best-effort IMU messages from being emitted
# while the endpoints are still being discovered.
echo "Starting bag playback..."
if ! ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"; then
    echo "Error: failed to resume the bag player." >&2
    exit 1
fi

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
