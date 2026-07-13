#!/bin/bash
# Reconstruct D-LIO outputs from a raw sensor bag and open RViz2.
#
# Usage (from anywhere inside the repository):
#   bash scripts/dlio/reconstruct_raw.sh [--tf-profile legacy|urdf-imu|urdf-imu-lidar-legacy] [--tuning-profile profile] <bag_directory> [ros2 bag play args...]
#
# Default TF profile: urdf-imu-lidar-legacy
#
# Examples:
#   bash scripts/dlio/reconstruct_raw.sh humble_ws/bags/raw_20260312_024403
#   bash scripts/dlio/reconstruct_raw.sh --tf-profile legacy humble_ws/bags/raw_20260312_024403
#   bash scripts/dlio/reconstruct_raw.sh humble_ws/bags/raw_20260312_024403 --rate 2.0
#   bash scripts/dlio/reconstruct_raw.sh --tuning-profile baseline humble_ws/bags/raw_20260312_024403

set -eo pipefail

TF_PROFILE=urdf-imu-lidar-legacy
TUNING_PROFILE=none
TUNING_PROFILE_PATH=""
TUNING_RUN_DIR=""
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
        --tuning-profile)
            TUNING_PROFILE="${2:?Error: --tuning-profile requires a value}"
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

select_tuning_profile() {
    if [ "$TUNING_PROFILE" = "none" ]; then
        return
    fi

    case "$TUNING_PROFILE" in
        *[!A-Za-z0-9_-]*|'')
            echo "Error: --tuning-profile must contain only letters, numbers, underscores, and hyphens." >&2
            exit 2
            ;;
    esac

    TUNING_PROFILE_PATH="$REPO_ROOT/config/dlio/tuning/profiles/$TUNING_PROFILE.yaml"
    if [ ! -f "$TUNING_PROFILE_PATH" ]; then
        echo "Error: tuning profile not found: $TUNING_PROFILE_PATH" >&2
        echo "Available profiles are under: $REPO_ROOT/config/dlio/tuning/profiles" >&2
        exit 1
    fi
}

select_tuning_profile

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

prepare_tuning_run() {
    if [ "$TUNING_PROFILE" = "none" ]; then
        return
    fi

    local bag_name
    bag_name="$(basename "$BAG")"
    TUNING_RUN_DIR="$REPO_ROOT/.dlio-tuning-runs/$bag_name/$TUNING_PROFILE-$(date -u +%Y%m%dT%H%M%SZ)"
    mkdir -p "$TUNING_RUN_DIR"

    if ! python3 - "$DLIO_CONFIG" "$DLIO_PARAMS_CONFIG" "$TUNING_PROFILE_PATH" "$TUNING_RUN_DIR" "$BAG" "${EXTRA_ARGS[@]}" <<'PY'
import hashlib
import pathlib
import sys
import yaml

base_dlio, base_params, profile_path, run_dir, bag_path = sys.argv[1:6]
replay_args = sys.argv[6:]
run_path = pathlib.Path(run_dir)

with open(profile_path) as f:
    profile = yaml.safe_load(f) or {}
if not isinstance(profile, dict):
    raise SystemExit("tuning profile must be a YAML mapping")

allowed = {"name", "description", "dlio", "params"}
unknown = set(profile) - allowed
if unknown:
    raise SystemExit(f"unsupported tuning profile keys: {sorted(unknown)}")

def overrides_for(section):
    overrides = profile.get(section, {})
    if not isinstance(overrides, dict):
        raise SystemExit(f"{section} overrides must be a mapping")
    return overrides

def write_effective(base_path, overrides, output_name):
    with open(base_path) as f:
        config = yaml.safe_load(f) or {}
    parameters = config.setdefault("/**", {}).setdefault("ros__parameters", {})
    if not isinstance(parameters, dict):
        raise SystemExit(f"invalid ros__parameters in {base_path}")
    parameters.update(overrides)
    output = run_path / output_name
    with open(output, "w") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return output

def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

effective_dlio = write_effective(base_dlio, overrides_for("dlio"), "effective-dlio.yaml")
effective_params = write_effective(base_params, overrides_for("params"), "effective-params.yaml")

with open(pathlib.Path(bag_path) / "metadata.yaml") as f:
    bag_metadata = yaml.safe_load(f) or {}
topics = bag_metadata.get("rosbag2_bagfile_information", {}).get("topics_with_message_count", [])
counts = {
    entry["topic_metadata"]["name"]: entry["message_count"]
    for entry in topics
    if entry["topic_metadata"]["name"] in {"/points_raw", "/go2w/imu"}
}

manifest = {
    "profile": profile.get("name", pathlib.Path(profile_path).stem),
    "description": profile.get("description", ""),
    "profile_file": profile_path,
    "bag": bag_path,
    "expected_input_messages": counts,
    "replay_arguments": replay_args,
    "source_configs": {"dlio": base_dlio, "params": base_params},
    "effective_configs": {
        "dlio": {"path": str(effective_dlio), "sha256": sha256(effective_dlio)},
        "params": {"path": str(effective_params), "sha256": sha256(effective_params)},
    },
}
with open(run_path / "manifest.yaml", "w") as f:
    yaml.safe_dump(manifest, f, sort_keys=False)
PY
    then
        echo "Error: failed to prepare tuning profile: $TUNING_PROFILE" >&2
        exit 1
    fi

    DLIO_CONFIG="$TUNING_RUN_DIR/effective-dlio.yaml"
    DLIO_PARAMS_CONFIG="$TUNING_RUN_DIR/effective-params.yaml"
}

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

prepare_tuning_run

echo "Bag:  $BAG"
echo "RViz: $RVIZ_CFG"
echo "D-LIO setup: $WS_SETUP"
echo "TF profile: $TF_PROFILE"
echo "D-LIO config: $DLIO_CONFIG"
echo "Params config: $DLIO_PARAMS_CONFIG"
echo "Mode: replay raw topics, run D-LIO offline, visualize generated outputs"
echo "Tuning profile: $TUNING_PROFILE"
if [ -n "$TUNING_RUN_DIR" ]; then
    echo "Tuning manifest: $TUNING_RUN_DIR/manifest.yaml"
fi
echo ""

setsid ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=false \
    launch_drivers:=false \
    dlio_config:="$DLIO_CONFIG" \
    params_config:="$DLIO_PARAMS_CONFIG" \
    use_sim_time:=true \
    pointcloud_topic:=points_raw \
    imu_topic:=go2w/imu &
DLIO_PID=$!
DLIO_PGID=$DLIO_PID

sleep 3
if ! kill -0 "$DLIO_PID" 2>/dev/null; then
    echo "Error: D-LIO exited during startup." >&2
    wait "$DLIO_PID" || true
    exit 1
fi

# Reset D-LIO state before playback to clear any leftover data from previous runs.
# timeout prevents hanging indefinitely if a node didn't come up cleanly.
echo "Resetting D-LIO node state..."
timeout 5 ros2 service call /dlio_odom_node/reset_map direct_lidar_inertial_odometry/srv/ResetMap || true
timeout 5 ros2 service call /dlio_map_node/reset_map direct_lidar_inertial_odometry/srv/ResetMap || true

setsid ros2 bag play "$BAG" "${CLOCK_ARG[@]}" "${EXTRA_ARGS[@]}" &
BAG_PID=$!
BAG_PGID=$BAG_PID

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    wait "$BAG_PID"
fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
