#!/usr/bin/env bash
# Run reproducible headless D-LIO against a recorded Pandar XT-16 ROS 2 bag.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/run_dlio_offline.sh BAG [options]

Options:
  --start-offset SEC  Start this many seconds into the bag (default: 0)
  --duration SEC      Approximate bag seconds via wall timer (smoke tests only)
  --rate RATE         Rosbag playback multiplier (default: 1.0)
  --domain-id ID      Isolated ROS domain ID (default: 77)
  --output DIR        Result directory (default: ${DLIO_RESULTS_ROOT}/dlio/<bag>/...)
  --dlio-config YAML  Override calibration/intrinsics parameters
  --params-config YAML
                      Override runtime/tuning parameters while preserving the
                      fixed headless publisher contract
  --no-analyze        Keep the result bag but skip automatic artifact generation
  --map-voxel-size M  Final map voxel edge length (default: 0.20)
  --preview-max-points N
                      Maximum points in the RViz preview map (default: 500000)
  --plane-random-seed N
                      Deterministic local-plane sample seed (default: 7)
  -h, --help          Show this help

The workspace must already be built. The runner plays only /points_raw and
/go2w/imu, starts playback paused, waits for every endpoint, then resumes
through the rosbag player service.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

BAG=""
START_OFFSET="0"
DURATION=""
RATE="1.0"
DOMAIN_ID="77"
OUTPUT_DIR=""
DLIO_CONFIG_OVERRIDE=""
PARAMS_CONFIG_OVERRIDE=""
ANALYZE="true"
MAP_VOXEL_SIZE="0.20"
PREVIEW_MAX_POINTS="500000"
PLANE_RANDOM_SEED="7"
DLIO_RESULTS_ROOT="${DLIO_RESULTS_ROOT:-$REPO_ROOT/results}"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --start-offset)
            START_OFFSET="${2:?Error: --start-offset requires a value}"
            shift 2
            ;;
        --duration)
            DURATION="${2:?Error: --duration requires a value}"
            shift 2
            ;;
        --rate)
            RATE="${2:?Error: --rate requires a value}"
            shift 2
            ;;
        --domain-id)
            DOMAIN_ID="${2:?Error: --domain-id requires a value}"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="${2:?Error: --output requires a value}"
            shift 2
            ;;
        --dlio-config)
            DLIO_CONFIG_OVERRIDE="${2:?Error: --dlio-config requires a value}"
            shift 2
            ;;
        --params-config)
            PARAMS_CONFIG_OVERRIDE="${2:?Error: --params-config requires a value}"
            shift 2
            ;;
        --no-analyze)
            ANALYZE="false"
            shift
            ;;
        --map-voxel-size)
            MAP_VOXEL_SIZE="${2:?Error: --map-voxel-size requires a value}"
            shift 2
            ;;
        --preview-max-points)
            PREVIEW_MAX_POINTS="${2:?Error: --preview-max-points requires a value}"
            shift 2
            ;;
        --plane-random-seed)
            PLANE_RANDOM_SEED="${2:?Error: --plane-random-seed requires a value}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            die "unknown option: $1"
            ;;
        *)
            [ -z "$BAG" ] || die "only one bag path may be supplied"
            BAG="$1"
            shift
            ;;
    esac
done

[ -n "$BAG" ] || die "a bag directory is required"

python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value >= 0)' "$START_OFFSET" \
    || die "--start-offset must be a non-negative number"
python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$RATE" \
    || die "--rate must be greater than zero"
if [ "$ANALYZE" = "true" ]; then
    python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$MAP_VOXEL_SIZE" \
        || die "--map-voxel-size must be greater than zero"
    [[ "$PREVIEW_MAX_POINTS" =~ ^[1-9][0-9]*$ ]] \
        || die "--preview-max-points must be a positive integer"
    [[ "$PLANE_RANDOM_SEED" =~ ^[0-9]+$ ]] \
        || die "--plane-random-seed must be a non-negative integer"
fi
if [ -n "$DURATION" ]; then
    python3 -c 'import sys; value=float(sys.argv[1]); sys.exit(not value > 0)' "$DURATION" \
        || die "--duration must be greater than zero"
fi
[[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || die "--domain-id must be a non-negative integer"

BAG="$(realpath "$BAG")"
[ -d "$BAG" ] || die "bag directory not found: $BAG"
[ -f "$BAG/metadata.yaml" ] || die "bag metadata not found: $BAG/metadata.yaml"
for topic in /points_raw /go2w/imu; do
    grep -Eq "name: ${topic}$" "$BAG/metadata.yaml" \
        || die "required topic $topic is absent from the bag"
done

PACKAGE_SOURCE="$REPO_ROOT/humble_ws/src/direct_lidar_inertial_odometry"
CONFIG_DIR="$PACKAGE_SOURCE/cfg"

resolve_config() {
    local override="$1"
    local default_name="$2"
    local label="$3"
    if [ -z "$override" ]; then
        realpath "$CONFIG_DIR/$default_name"
    elif [ -f "$override" ]; then
        realpath "$override"
    elif [ -f "$REPO_ROOT/$override" ]; then
        realpath "$REPO_ROOT/$override"
    elif [ -f "$CONFIG_DIR/$override" ]; then
        realpath "$CONFIG_DIR/$override"
    else
        die "$label config not found: $override"
    fi
}

DLIO_CONFIG="$(resolve_config "$DLIO_CONFIG_OVERRIDE" dlio.yaml D-LIO)"
PARAMS_CONFIG="$(resolve_config "$PARAMS_CONFIG_OVERRIDE" params.yaml D-LIO-runtime)"
OFFLINE_CONFIG="$(realpath "$CONFIG_DIR/offline.yaml")"

if [ -z "$OUTPUT_DIR" ]; then
    BAG_NAME="$(basename "$BAG")"
    RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
    OUTPUT_DIR="$(realpath -m "${DLIO_RESULTS_ROOT}/dlio/$BAG_NAME/$RUN_STAMP")"
else
    OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
fi
if [ -d "$OUTPUT_DIR" ] && [ -n "$(find "$OUTPUT_DIR" -mindepth 1 -print -quit)" ]; then
    die "output directory is not empty: $OUTPUT_DIR"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
elif [ "${ROS_DISTRO:-}" != "humble" ]; then
    die "ROS 2 Humble is required (run this inside the project container)"
fi

for command in ros2 setsid python3 sha256sum cp git realpath; do
    command -v "$command" >/dev/null || die "required command not found: $command"
done
python3 -c 'import yaml' >/dev/null 2>&1 \
    || die "PyYAML is required for result validation"

LAUNCH_SOURCE="$PACKAGE_SOURCE/launch/offline_dlio.launch.py"
[ -f "$LAUNCH_SOURCE" ] || die "experiment launch source not found: $LAUNCH_SOURCE"
LAUNCH_SHA256="$(sha256sum "$LAUNCH_SOURCE" | awk '{print $1}')"

ANALYZER_SOURCE=""
if [ "$ANALYZE" = "true" ]; then
    ANALYZER_SOURCE="$SCRIPT_DIR/analyze_dlio_run.py"
    [ -f "$ANALYZER_SOURCE" ] || die "analyzer not found: $ANALYZER_SOURCE"
fi

candidate_is_usable() {
    local candidate="$1"
    local install_root package_prefix launch_runtime offline_runtime dlio_runtime
    install_root="$(dirname "$candidate")"
    package_prefix="$install_root/direct_lidar_inertial_odometry"
    launch_runtime="$package_prefix/share/direct_lidar_inertial_odometry/launch/offline_dlio.launch.py"
    offline_runtime="$package_prefix/share/direct_lidar_inertial_odometry/cfg/offline.yaml"
    dlio_runtime="$package_prefix/lib/direct_lidar_inertial_odometry/dlio_odom_node"

    [ -f "$candidate" ] || return 1
    [ -f "$launch_runtime" ] || return 1
    [ -f "$offline_runtime" ] || return 1
    [ -x "$dlio_runtime" ] || return 1
    [ "$(sha256sum "$launch_runtime" | awk '{print $1}')" = "$LAUNCH_SHA256" ] \
        || return 1
    [ "$(sha256sum "$offline_runtime" | awk '{print $1}')" = \
      "$(sha256sum "$OFFLINE_CONFIG" | awk '{print $1}')" ] || return 1
    [ "$dlio_runtime" -nt "$PACKAGE_SOURCE/src/dlio/odom.cc" ] || return 1
    [ "$dlio_runtime" -nt "$PACKAGE_SOURCE/include/dlio/odom.h" ] || return 1

    (
        set +u
        # shellcheck source=/dev/null
        source "$candidate"
        set -u
        [ "$(ros2 pkg prefix direct_lidar_inertial_odometry 2>/dev/null)" = \
          "$package_prefix" ] \
            || exit 1
    )
}

WORKSPACE_SETUP=""
for candidate in \
    "$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash" \
    "$REPO_ROOT/humble_ws/install/setup.bash"; do
    if candidate_is_usable "$candidate"; then
        WORKSPACE_SETUP="$candidate"
        break
    fi
done
[ -n "$WORKSPACE_SETUP" ] \
    || die "no usable current Humble workspace overlay was found; rebuild the workspace"
set +u
# shellcheck source=/dev/null
source "$WORKSPACE_SETUP"
set -u

INSTALL_ROOT="$(dirname "$WORKSPACE_SETUP")"
PACKAGE_PREFIX="$INSTALL_ROOT/direct_lidar_inertial_odometry"
LAUNCH_RUNTIME="$PACKAGE_PREFIX/share/direct_lidar_inertial_odometry/launch/offline_dlio.launch.py"
DLIO_RUNTIME="$PACKAGE_PREFIX/lib/direct_lidar_inertial_odometry/dlio_odom_node"
DLIO_RUNTIME_SHA256="$(sha256sum "$DLIO_RUNTIME" | awk '{print $1}')"

python3 -c 'import rosbag2_py' >/dev/null 2>&1 \
    || die "rosbag2 Python support is required for result validation"
if [ "$ANALYZE" = "true" ]; then
    python3 -c 'import numpy, rclpy; from rosidl_runtime_py.utilities import get_message' \
        >/dev/null 2>&1 || die "automatic analysis dependencies are unavailable"
    python3 "$ANALYZER_SOURCE" analyze --help >/dev/null \
        || die "analyzer CLI preflight failed"
fi

# Create the run directory only after all read-only source and overlay preflight passes.
if [ -d "$OUTPUT_DIR" ] && [ -n "$(find "$OUTPUT_DIR" -mindepth 1 -print -quit)" ]; then
    die "output directory is not empty: $OUTPUT_DIR"
fi
mkdir -p "$OUTPUT_DIR"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RCUTILS_COLORIZED_OUTPUT=0
export ROS_LOG_DIR="$OUTPUT_DIR/ros-logs"
mkdir -p "$ROS_LOG_DIR"

RESULT_BAG="$OUTPUT_DIR/rosbag"
DLIO_CONFIG_SNAPSHOT="$OUTPUT_DIR/dlio_calibration.yaml"
PARAMS_CONFIG_SNAPSHOT="$OUTPUT_DIR/dlio_params.yaml"
OFFLINE_CONFIG_SNAPSHOT="$OUTPUT_DIR/dlio_offline.yaml"
DLIO_PARAMETERS_SNAPSHOT="$OUTPUT_DIR/dlio_odom_node.yaml"
DLIO_PARAMETERS_SHA256=""
ANALYZER_SNAPSHOT=""
ANALYZER_SHA256=""
ANALYSIS_LOG=""
if [ "$ANALYZE" = "true" ]; then
    ANALYZER_SNAPSHOT="$OUTPUT_DIR/analyze_dlio_run.py"
    ANALYSIS_LOG="$OUTPUT_DIR/analysis.log"
    cp "$ANALYZER_SOURCE" "$ANALYZER_SNAPSHOT"
    ANALYZER_SHA256="$(sha256sum "$ANALYZER_SNAPSHOT" | awk '{print $1}')"
fi
PARAMETER_DUMP_LOG="$OUTPUT_DIR/parameter-dump.log"
PARAMETER_VALIDATION_LOG="$OUTPUT_DIR/dlio-parameter-validation.log"
RESULT_BAG_VALIDATION_LOG="$OUTPUT_DIR/result-bag-validation.log"
RESOURCE_VALIDATION_LOG="$OUTPUT_DIR/resource-validation.log"

cp "$DLIO_CONFIG" "$DLIO_CONFIG_SNAPSHOT"
cp "$PARAMS_CONFIG" "$PARAMS_CONFIG_SNAPSHOT"
cp "$OFFLINE_CONFIG" "$OFFLINE_CONFIG_SNAPSHOT"
git -C "$REPO_ROOT" status --short > "$OUTPUT_DIR/git-status.txt"

MANIFEST="$OUTPUT_DIR/manifest.json"
METRICS_CSV="$OUTPUT_DIR/resource_metrics.csv"
METRICS_SUMMARY="$OUTPUT_DIR/resource_summary.json"
STOP_FILE="$OUTPUT_DIR/.stop-metrics"
DURATION_MARKER="$OUTPUT_DIR/.duration-reached"
RUN_STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
DLIO_CONFIG_SHA256="$(sha256sum "$DLIO_CONFIG_SNAPSHOT" | awk '{print $1}')"
PARAMS_CONFIG_SHA256="$(sha256sum "$PARAMS_CONFIG_SNAPSHOT" | awk '{print $1}')"
OFFLINE_CONFIG_SHA256="$(sha256sum "$OFFLINE_CONFIG_SNAPSHOT" | awk '{print $1}')"
METADATA_SHA256="$(sha256sum "$BAG/metadata.yaml" | awk '{print $1}')"
RESULT_METADATA_SHA256=""
RESOURCE_METRICS_SHA256=""
RESOURCE_SUMMARY_SHA256=""
DRAIN_STABLE_SECONDS="5"
DRAIN_TIMEOUT_SECONDS="120"
DRAIN_POLL_SECONDS="1"
DRAIN_OUTCOME="not_started"
DRAIN_ELAPSED_SECONDS=""
DRAIN_FINAL_SIZE_BYTES=""
DRAIN_FINAL_MTIME_NS=""

GIT_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"

LAUNCH_PID=""
RECORDER_PID=""
PLAYER_PID=""
SAMPLER_PID=""
WATCHDOG_PID=""
PLAYER_STATUS=""
CLEANUP_COMPLETE="false"

write_manifest() {
    local state="$1"
    local exit_code="${2:-}"
    MANIFEST_STATE="$state" MANIFEST_EXIT_CODE="$exit_code" \
    EXP_BAG="$BAG" EXP_START_OFFSET="$START_OFFSET" \
    EXP_DURATION="$DURATION" EXP_RATE="$RATE" EXP_DOMAIN_ID="$DOMAIN_ID" \
    EXP_STARTED_AT="$RUN_STARTED_AT" \
    EXP_DLIO_CONFIG="$DLIO_CONFIG" EXP_DLIO_CONFIG_SHA="$DLIO_CONFIG_SHA256" \
    EXP_DLIO_CONFIG_SNAPSHOT="$DLIO_CONFIG_SNAPSHOT" \
    EXP_PARAMS_CONFIG="$PARAMS_CONFIG" EXP_PARAMS_CONFIG_SHA="$PARAMS_CONFIG_SHA256" \
    EXP_PARAMS_CONFIG_SNAPSHOT="$PARAMS_CONFIG_SNAPSHOT" \
    EXP_OFFLINE_CONFIG="$OFFLINE_CONFIG" EXP_OFFLINE_CONFIG_SHA="$OFFLINE_CONFIG_SHA256" \
    EXP_OFFLINE_CONFIG_SNAPSHOT="$OFFLINE_CONFIG_SNAPSHOT" \
    EXP_DLIO_PARAMETERS="$DLIO_PARAMETERS_SNAPSHOT" \
    EXP_DLIO_PARAMETERS_SHA="$DLIO_PARAMETERS_SHA256" \
    EXP_METADATA_SHA="$METADATA_SHA256" EXP_LAUNCH_SHA="$LAUNCH_SHA256" \
    EXP_GIT_COMMIT="$GIT_COMMIT" EXP_OUTPUT="$OUTPUT_DIR" \
    EXP_WORKSPACE_SETUP="$WORKSPACE_SETUP" \
    EXP_LAUNCH_SOURCE="$LAUNCH_SOURCE" EXP_LAUNCH_RUNTIME="$LAUNCH_RUNTIME" \
    EXP_DLIO_RUNTIME="$DLIO_RUNTIME" \
    EXP_DLIO_RUNTIME_SHA="$DLIO_RUNTIME_SHA256" \
    EXP_LAUNCH_PID="$LAUNCH_PID" EXP_RECORDER_PID="$RECORDER_PID" \
    EXP_PLAYER_PID="$PLAYER_PID" EXP_SAMPLER_PID="$SAMPLER_PID" \
    EXP_ANALYZE="$ANALYZE" EXP_MAP_VOXEL_SIZE="$MAP_VOXEL_SIZE" \
    EXP_PREVIEW_MAX_POINTS="$PREVIEW_MAX_POINTS" \
    EXP_PLANE_RANDOM_SEED="$PLANE_RANDOM_SEED" \
    EXP_ANALYZER_SOURCE="$ANALYZER_SOURCE" \
    EXP_ANALYZER_SNAPSHOT="$ANALYZER_SNAPSHOT" \
    EXP_ANALYZER_SHA="$ANALYZER_SHA256" \
    EXP_ANALYSIS_LOG="$ANALYSIS_LOG" \
    EXP_RESULT_METADATA_SHA="$RESULT_METADATA_SHA256" \
    EXP_RESOURCE_METRICS_SHA="$RESOURCE_METRICS_SHA256" \
    EXP_RESOURCE_SUMMARY_SHA="$RESOURCE_SUMMARY_SHA256" \
    EXP_PARAMETER_DUMP_LOG="$PARAMETER_DUMP_LOG" \
    EXP_PARAMETER_VALIDATION_LOG="$PARAMETER_VALIDATION_LOG" \
    EXP_RESULT_VALIDATION_LOG="$RESULT_BAG_VALIDATION_LOG" \
    EXP_RESOURCE_VALIDATION_LOG="$RESOURCE_VALIDATION_LOG" \
    EXP_DRAIN_STABLE_SECONDS="$DRAIN_STABLE_SECONDS" \
    EXP_DRAIN_TIMEOUT_SECONDS="$DRAIN_TIMEOUT_SECONDS" \
    EXP_DRAIN_POLL_SECONDS="$DRAIN_POLL_SECONDS" \
    EXP_DRAIN_OUTCOME="$DRAIN_OUTCOME" \
    EXP_DRAIN_ELAPSED_SECONDS="$DRAIN_ELAPSED_SECONDS" \
    EXP_DRAIN_FINAL_SIZE_BYTES="$DRAIN_FINAL_SIZE_BYTES" \
    EXP_DRAIN_FINAL_MTIME_NS="$DRAIN_FINAL_MTIME_NS" \
    python3 - "$MANIFEST" <<'PY'
import json
import os
import pathlib
import sys
from datetime import datetime, timezone

env = os.environ

def optional_float(value):
    return float(value) if value else None

def optional_int(value):
    return int(value) if value else None

def optional_name(value):
    return pathlib.Path(value).name if value else None

analysis_enabled = env["EXP_ANALYZE"] == "true"
analysis_details = {
    "enabled": analysis_enabled,
    "voxel_size_m": None,
    "preview_max_points": None,
    "plane_random_seed": None,
    "analyzer_source": None,
    "analyzer_snapshot": None,
    "analyzer_sha256": None,
    "log": None,
    "artifacts": None,
}
if analysis_enabled:
    analysis_details.update(
        {
            "voxel_size_m": float(env["EXP_MAP_VOXEL_SIZE"]),
            "preview_max_points": int(env["EXP_PREVIEW_MAX_POINTS"]),
            "plane_random_seed": int(env["EXP_PLANE_RANDOM_SEED"]),
            "analyzer_source": env["EXP_ANALYZER_SOURCE"],
            "analyzer_snapshot": optional_name(env["EXP_ANALYZER_SNAPSHOT"]),
            "analyzer_sha256": env["EXP_ANALYZER_SHA"],
            "log": optional_name(env["EXP_ANALYSIS_LOG"]),
            "artifacts": {
                "summary": "summary.json",
                "map": "map_voxelized.pcd",
                "map_preview": "map_preview.pcd",
                "primary_trajectory": "trajectory.csv",
            },
        }
    )

run_logs = [
    "launch.log",
    "recorder.log",
    "player.log",
    "resume.log",
    "metrics.log",
    optional_name(env["EXP_PARAMETER_DUMP_LOG"]),
    optional_name(env["EXP_PARAMETER_VALIDATION_LOG"]),
    optional_name(env["EXP_RESULT_VALIDATION_LOG"]),
    optional_name(env["EXP_RESOURCE_VALIDATION_LOG"]),
]
if analysis_enabled:
    run_logs.append(optional_name(env["EXP_ANALYSIS_LOG"]))


document = {
    "state": env["MANIFEST_STATE"],
    "exit_code": optional_int(env.get("MANIFEST_EXIT_CODE", "")),
    "started_at_utc": env["EXP_STARTED_AT"],
    "updated_at_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
    "bag": {
        "path": env["EXP_BAG"],
        "metadata_sha256": env["EXP_METADATA_SHA"],
    },
    "profile": "baseline",
    "playback": {
        "topics": ["/points_raw", "/go2w/imu"],
        "start_offset_s": float(env["EXP_START_OFFSET"]),
        "duration_s": optional_float(env["EXP_DURATION"]),
        "rate": float(env["EXP_RATE"]),
        "ros_domain_id": int(env["EXP_DOMAIN_ID"]),
    },
    "drain": {
        "stable_seconds": float(env["EXP_DRAIN_STABLE_SECONDS"]),
        "timeout_seconds": float(env["EXP_DRAIN_TIMEOUT_SECONDS"]),
        "poll_seconds": float(env["EXP_DRAIN_POLL_SECONDS"]),
        "outcome": env["EXP_DRAIN_OUTCOME"],
        "elapsed_seconds": optional_float(env["EXP_DRAIN_ELAPSED_SECONDS"]),
        "final_storage_size_bytes": optional_int(env["EXP_DRAIN_FINAL_SIZE_BYTES"]),
        "final_storage_mtime_ns": optional_int(env["EXP_DRAIN_FINAL_MTIME_NS"]),
    },
    "dlio": {
        "calibration_config_source": env["EXP_DLIO_CONFIG"],
        "calibration_config_snapshot": optional_name(env["EXP_DLIO_CONFIG_SNAPSHOT"]),
        "calibration_config_sha256": env["EXP_DLIO_CONFIG_SHA"],
        "runtime_config_source": env["EXP_PARAMS_CONFIG"],
        "runtime_config_snapshot": optional_name(env["EXP_PARAMS_CONFIG_SNAPSHOT"]),
        "runtime_config_sha256": env["EXP_PARAMS_CONFIG_SHA"],
        "offline_config_source": env["EXP_OFFLINE_CONFIG"],
        "offline_config_snapshot": optional_name(env["EXP_OFFLINE_CONFIG_SNAPSHOT"]),
        "offline_config_sha256": env["EXP_OFFLINE_CONFIG_SHA"],
        "parameters_snapshot": pathlib.Path(env["EXP_DLIO_PARAMETERS"]).name,
        "parameters_sha256": env.get("EXP_DLIO_PARAMETERS_SHA") or None,
        "event_driven_odometry": True,
        "map_wait_until_move": False,
        "publish_odom": True,
        "publish_pose": False,
        "publish_path": False,
        "publish_keyframes": False,
        "publish_deskewed": True,
        "publish_tf": False,
    },
    "analysis": analysis_details,
    "git": {
        "commit": env["EXP_GIT_COMMIT"],
        "status_file": "git-status.txt",
    },
    "workspace_setup": env["EXP_WORKSPACE_SETUP"],
    "launch_sha256": env["EXP_LAUNCH_SHA"],
    "launch_source": env["EXP_LAUNCH_SOURCE"],
    "launch_runtime": env["EXP_LAUNCH_RUNTIME"],
    "runtime_executables": {
        "dlio": {
            "path": env["EXP_DLIO_RUNTIME"],
            "sha256": env["EXP_DLIO_RUNTIME_SHA"],
        },
    },
    "process_ids": {
        "launch": optional_int(env["EXP_LAUNCH_PID"]),
        "recorder": optional_int(env["EXP_RECORDER_PID"]),
        "player": optional_int(env["EXP_PLAYER_PID"]),
        "sampler": optional_int(env["EXP_SAMPLER_PID"]),
    },
    "artifacts": {
        "recorded_bag": "rosbag",
        "recorded_bag_metadata_sha256": env.get("EXP_RESULT_METADATA_SHA") or None,
        "resource_metrics": "resource_metrics.csv",
        "resource_metrics_sha256": env.get("EXP_RESOURCE_METRICS_SHA") or None,
        "resource_summary": "resource_summary.json",
        "resource_summary_sha256": env.get("EXP_RESOURCE_SUMMARY_SHA") or None,
        "commands": "commands.log",
        "logs": run_logs,
    },
}
path = pathlib.Path(sys.argv[1])
temporary = path.with_suffix(".json.tmp")
temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
temporary.replace(path)
PY
}

process_group_alive() {
    local pgid="${1:-}"
    local stat_file record state _ppid process_group
    [ -n "$pgid" ] || return 1
    for stat_file in /proc/[0-9]*/stat; do
        IFS= read -r record 2>/dev/null < "$stat_file" || continue
        record="${record##*) }"
        read -r state _ppid process_group _ <<<"$record"
        [ "$process_group" = "$pgid" ] || continue
        case "$state" in
            Z|X) ;;
            *) return 0 ;;
        esac
    done
    return 1
}

stop_group() {
    local pid="${1:-}"
    local name="${2:-process}"
    [ -n "$pid" ] || return 0
    if process_group_alive "$pid"; then
        kill -INT -- "-$pid" 2>/dev/null || true
        for _ in $(seq 1 50); do
            process_group_alive "$pid" || break
            sleep 0.2
        done
        if process_group_alive "$pid"; then
            echo "Escalating $name cleanup to SIGTERM." >&2
            kill -TERM -- "-$pid" 2>/dev/null || true
            for _ in $(seq 1 25); do
                process_group_alive "$pid" || break
                sleep 0.2
            done
        fi
        if process_group_alive "$pid"; then
            echo "Escalating $name cleanup to SIGKILL." >&2
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    set +e
    if [ "$CLEANUP_COMPLETE" != "true" ]; then
        [ -z "$WATCHDOG_PID" ] || kill "$WATCHDOG_PID" 2>/dev/null || true
        stop_group "$PLAYER_PID" "rosbag player"
        stop_group "$RECORDER_PID" "rosbag recorder"
        stop_group "$LAUNCH_PID" "processing launch"
        touch "$STOP_FILE"
        if [ -n "$SAMPLER_PID" ]; then
            for _ in $(seq 1 20); do
                kill -0 "$SAMPLER_PID" 2>/dev/null || break
                sleep 0.1
            done
            stop_group "$SAMPLER_PID" "resource sampler"
        fi
        write_manifest "failed" "$status" || true
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for_node() {
    local expected="$1"
    local deadline=$((SECONDS + 90))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
        if ros2 node list --no-daemon 2>/dev/null | grep -Fxq "$expected"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_service() {
    local expected="$1"
    local deadline=$((SECONDS + 90))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$PLAYER_PID" 2>/dev/null || return 1
        if ros2 service list --no-daemon 2>/dev/null | grep -Fxq "$expected"; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_topic() {
    local topic="$1"
    local minimum_publishers="$2"
    local minimum_subscribers="$3"
    local deadline=$((SECONDS + 90))
    local info publishers subscribers
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return 1
        kill -0 "$RECORDER_PID" 2>/dev/null || return 1
        kill -0 "$PLAYER_PID" 2>/dev/null || return 1
        info="$(ros2 topic info --no-daemon "$topic" 2>/dev/null || true)"
        publishers="$(awk '/Publisher count:/{print $3}' <<<"$info")"
        subscribers="$(awk '/Subscription count:/{print $3}' <<<"$info")"
        if [ "${publishers:-0}" -ge "$minimum_publishers" ] \
            && [ "${subscribers:-0}" -ge "$minimum_subscribers" ]; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

validate_dlio_parameters() {
    python3 - "$DLIO_PARAMETERS_SNAPSHOT" <<'PY'
from pathlib import Path
import sys

import yaml

snapshot = Path(sys.argv[1])
payload = yaml.safe_load(snapshot.read_text(encoding="utf-8"))
try:
    parameters = payload["/dlio_odom_node"]["ros__parameters"]
except (KeyError, TypeError) as exc:
    print(f"Invalid D-LIO parameter snapshot {snapshot}: {exc}", file=sys.stderr)
    raise SystemExit(1)

expected = {
    "use_sim_time": True,
    "offline/replay": True,
    "offline/eventDrivenOdometry": True,
    "map/waitUntilMove": False,
    "publish/odom": True,
    "publish/pose": False,
    "publish/path": False,
    "publish/keyframes": False,
    "publish/deskewed": True,
    "publish/tf": False,
    "pointcloud/deskew": True,
    "frames/odom": "odom",
}

errors = []
for name, expected_value in expected.items():
    if name not in parameters:
        errors.append(f"D-LIO parameter {name} is missing from the live snapshot.")
        continue
    actual_value = parameters[name]
    if actual_value != expected_value or type(actual_value) is not type(expected_value):
        errors.append(
            f"D-LIO parameter {name} is {actual_value!r}, expected {expected_value!r}."
        )

if errors:
    print("\n".join(errors), file=sys.stderr)
    raise SystemExit(1)
PY
}

required_processing_nodes_alive() {
    local nodes
    nodes="$(ros2 node list --no-daemon 2>/dev/null)" || return 1
    grep -Fxq /dlio_odom_node <<<"$nodes"
}

bag_storage_signature() {
    python3 - "$RESULT_BAG" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
if not root.is_dir():
    print("0 0")
    raise SystemExit(0)

stats = []
for path in root.rglob("*"):
    try:
        if path.is_file():
            stats.append(path.stat())
    except FileNotFoundError:
        continue
print(sum(stat.st_size for stat in stats), max((stat.st_mtime_ns for stat in stats), default=0))
PY
}

wait_for_processing_quiescence() {
    local started_at="$SECONDS"
    local stable_since="$SECONDS"
    local previous_signature=""
    local signature current_size current_mtime elapsed stable_elapsed
    local node_misses=0

    DRAIN_OUTCOME="waiting"
    DRAIN_ELAPSED_SECONDS="0"
    echo "Waiting for result bag writes to remain unchanged for ${DRAIN_STABLE_SECONDS}s..."

    while [ $((SECONDS - started_at)) -lt "$DRAIN_TIMEOUT_SECONDS" ]; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            DRAIN_OUTCOME="processing_launch_exited"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Processing launch exited while waiting for quiescence." >&2
            return 1
        fi
        if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
            DRAIN_OUTCOME="recorder_exited"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Rosbag recorder exited while waiting for quiescence." >&2
            return 1
        fi
        if ! kill -0 "$SAMPLER_PID" 2>/dev/null; then
            DRAIN_OUTCOME="resource_sampler_exited"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Resource sampler exited while waiting for quiescence." >&2
            return 1
        fi

        if required_processing_nodes_alive; then
            node_misses=0
        else
            node_misses=$((node_misses + 1))
            if [ "$node_misses" -ge 3 ]; then
                DRAIN_OUTCOME="processing_node_missing"
                DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
                echo "Required processing node disappeared while waiting for quiescence." >&2
                return 1
            fi
        fi

        if ! signature="$(bag_storage_signature)"; then
            DRAIN_OUTCOME="storage_signature_failed"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Could not inspect result bag storage while waiting for quiescence." >&2
            return 1
        fi
        read -r current_size current_mtime <<<"$signature"
        if ! [[ "$current_size" =~ ^[0-9]+$ && "$current_mtime" =~ ^[0-9]+$ ]]; then
            DRAIN_OUTCOME="invalid_storage_signature"
            DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
            echo "Result bag storage signature is invalid: $signature" >&2
            return 1
        fi

        DRAIN_FINAL_SIZE_BYTES="$current_size"
        DRAIN_FINAL_MTIME_NS="$current_mtime"
        elapsed=$((SECONDS - started_at))
        DRAIN_ELAPSED_SECONDS="$elapsed"

        if [ -n "$previous_signature" ] && [ "$signature" = "$previous_signature" ]; then
            stable_elapsed=$((SECONDS - stable_since))
        else
            previous_signature="$signature"
            stable_since="$SECONDS"
            stable_elapsed=0
        fi

        if [ "$current_size" -gt 0 ] && [ "$stable_elapsed" -ge "$DRAIN_STABLE_SECONDS" ]; then
            DRAIN_OUTCOME="quiescent"
            echo "Result bag storage is quiescent after ${elapsed}s."
            return 0
        fi
        sleep "$DRAIN_POLL_SECONDS"
    done

    DRAIN_OUTCOME="timeout"
    DRAIN_ELAPSED_SECONDS="$((SECONDS - started_at))"
    echo "Result bag did not remain unchanged for ${DRAIN_STABLE_SECONDS}s within the ${DRAIN_TIMEOUT_SECONDS}s timeout." >&2
    return 1
}

validate_result_bag() {
    local metadata="$RESULT_BAG/metadata.yaml"
    [ -s "$metadata" ] || {
        echo "Finalized result bag metadata is missing or empty: $metadata" >&2
        return 1
    }

    echo "ros2 bag info:"
    ros2 bag info "$RESULT_BAG"
    echo
    echo "metadata and reader validation:"
    python3 - "$RESULT_BAG" <<'PY'
from pathlib import Path
import json
import sys

import rosbag2_py
import yaml

root = Path(sys.argv[1]).resolve()
metadata_path = root / "metadata.yaml"

try:
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
except (OSError, yaml.YAMLError) as exc:
    raise SystemExit(f"could not parse finalized metadata: {exc}") from exc

if not isinstance(document, dict):
    raise SystemExit("finalized metadata is not a mapping")
information = document.get("rosbag2_bagfile_information", document)
if not isinstance(information, dict):
    raise SystemExit("rosbag2_bagfile_information is not a mapping")

storage_id = information.get("storage_identifier")
if not isinstance(storage_id, str) or not storage_id:
    raise SystemExit("metadata has no storage identifier")

relative_paths = information.get("relative_file_paths")
if not isinstance(relative_paths, list) or not relative_paths:
    raise SystemExit("metadata has no storage files")
storage_files = []
for value in relative_paths:
    relative = Path(str(value))
    if relative.is_absolute() or ".." in relative.parts:
        raise SystemExit(f"unsafe storage path in metadata: {value!r}")
    storage_path = (root / relative).resolve()
    try:
        storage_path.relative_to(root)
    except ValueError as exc:
        raise SystemExit(f"storage path escapes result bag: {value!r}") from exc
    if not storage_path.is_file() or storage_path.stat().st_size <= 0:
        raise SystemExit(f"storage file is missing or empty: {relative}")
    storage_files.append(
        {"path": str(relative), "size_bytes": storage_path.stat().st_size}
    )

topics = information.get("topics_with_message_count")
if not isinstance(topics, list):
    raise SystemExit("metadata has no topic message counts")
counts = {}
types = {}
for item in topics:
    if not isinstance(item, dict):
        raise SystemExit("invalid topic message-count entry")
    topic_metadata = item.get("topic_metadata")
    if not isinstance(topic_metadata, dict):
        raise SystemExit("topic entry has no topic_metadata")
    name = topic_metadata.get("name")
    type_name = topic_metadata.get("type")
    if not isinstance(name, str) or not name:
        raise SystemExit("topic entry has no valid name")
    try:
        count = int(item.get("message_count"))
    except (TypeError, ValueError) as exc:
        raise SystemExit(f"invalid message count for {name}") from exc
    if count < 0:
        raise SystemExit(f"negative message count for {name}")
    counts[name] = counts.get(name, 0) + count
    types[name] = type_name

try:
    total_count = int(information.get("message_count"))
except (TypeError, ValueError) as exc:
    raise SystemExit("metadata has no valid total message count") from exc
if total_count <= 0:
    raise SystemExit("result bag contains no messages")
if total_count != sum(counts.values()):
    raise SystemExit(
        f"metadata total message count {total_count} does not match topic sum {sum(counts.values())}"
    )

expected_types = {
    "/dlio/odom_node/odom": "nav_msgs/msg/Odometry",
    "/dlio/odom_node/pointcloud/deskewed": "sensor_msgs/msg/PointCloud2",
}
required = list(expected_types)
missing = [name for name in required if counts.get(name, 0) <= 0]
if missing:
    raise SystemExit(
        "required result topics have no recorded messages: " + ", ".join(missing)
    )
wrong_types = {
    name: types.get(name)
    for name, expected_type in expected_types.items()
    if types.get(name) != expected_type
}
if wrong_types:
    raise SystemExit(f"required result topics have unexpected types: {wrong_types}")

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(root), storage_id=storage_id),
    rosbag2_py.ConverterOptions("", ""),
)
reader_topics = {entry.name for entry in reader.get_all_topics_and_types()}
missing_from_reader = sorted(set(required) - reader_topics)
if missing_from_reader:
    raise SystemExit(
        "required topics are not readable through rosbag2: "
        + ", ".join(missing_from_reader)
    )
if not reader.has_next():
    raise SystemExit("rosbag2 reader opened the result bag but found no messages")
first_topic, first_data, first_timestamp = reader.read_next()
if not first_data:
    raise SystemExit("the first serialized result message is empty")

print(
    json.dumps(
        {
            "storage_id": storage_id,
            "storage_files": storage_files,
            "total_messages": total_count,
            "required_topic_counts": {name: counts[name] for name in required},
            "required_topic_types": {name: types.get(name) for name in required},
            "first_readable_message": {
                "topic": first_topic,
                "timestamp_ns": first_timestamp,
                "serialized_bytes": len(first_data),
            },
        },
        indent=2,
        sort_keys=True,
    )
)
PY
}

validate_resource_artifacts() {
    python3 - "$METRICS_CSV" "$METRICS_SUMMARY" <<'PY'
from collections import Counter
from pathlib import Path
import csv
import json
import sys

csv_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
required = {"dlio", "player", "recorder"}

if not csv_path.is_file() or csv_path.stat().st_size <= 0:
    raise SystemExit(f"resource metrics CSV is missing or empty: {csv_path}")
if not summary_path.is_file() or summary_path.stat().st_size <= 0:
    raise SystemExit(f"resource summary is missing or empty: {summary_path}")

counts = Counter()
with csv_path.open(newline="", encoding="utf-8") as stream:
    reader = csv.DictReader(stream)
    required_columns = {"wall_time_s", "pid", "process", "cpu_time_s", "rss_kib"}
    missing_columns = required_columns - set(reader.fieldnames or ())
    if missing_columns:
        raise SystemExit(
            "resource metrics CSV is missing columns: "
            + ", ".join(sorted(missing_columns))
        )
    for line_number, row in enumerate(reader, start=2):
        label = row.get("process", "")
        if not label:
            raise SystemExit(f"resource metrics row {line_number} has no process label")
        try:
            wall_time = float(row["wall_time_s"])
            pid = int(row["pid"])
            cpu_time = float(row["cpu_time_s"])
            rss_kib = int(row["rss_kib"])
        except (TypeError, ValueError) as exc:
            raise SystemExit(
                f"resource metrics row {line_number} contains invalid numeric data"
            ) from exc
        if wall_time < 0 or pid <= 0 or cpu_time < 0 or rss_kib < 0:
            raise SystemExit(
                f"resource metrics row {line_number} contains out-of-range data"
            )
        counts[label] += 1

missing_rows = sorted(label for label in required if counts[label] <= 0)
if missing_rows:
    raise SystemExit(
        "resource metrics CSV has no samples for: " + ", ".join(missing_rows)
    )

try:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit(f"could not parse resource summary: {exc}") from exc
if not isinstance(summary, dict):
    raise SystemExit("resource summary is not an object")
try:
    interval = float(summary["interval_s"])
    elapsed = float(summary["elapsed_wall_time_s"])
except (KeyError, TypeError, ValueError) as exc:
    raise SystemExit("resource summary timing fields are invalid") from exc
if interval <= 0 or elapsed <= 0:
    raise SystemExit("resource summary timing fields must be positive")

processes = summary.get("processes")
if not isinstance(processes, dict):
    raise SystemExit("resource summary has no processes object")
validated = {}
for label in sorted(required):
    details = processes.get(label)
    if not isinstance(details, dict):
        raise SystemExit(f"resource summary has no entry for {label}")
    try:
        samples = int(details["samples"])
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit(f"resource summary sample count is invalid for {label}") from exc
    pids = details.get("pids")
    if samples <= 0 or not isinstance(pids, list) or not pids:
        raise SystemExit(f"resource summary has no usable samples for {label}")
    if samples != counts[label]:
        raise SystemExit(
            f"resource sample mismatch for {label}: CSV={counts[label]}, summary={samples}"
        )
    validated[label] = {"samples": samples, "pids": pids}

print(
    json.dumps(
        {
            "elapsed_wall_time_s": elapsed,
            "interval_s": interval,
            "processes": validated,
        },
        indent=2,
        sort_keys=True,
    )
)
PY
}

LAUNCH_CMD=(
    ros2 launch direct_lidar_inertial_odometry offline_dlio.launch.py
    "dlio_config:=$DLIO_CONFIG_SNAPSHOT"
    "params_config:=$PARAMS_CONFIG_SNAPSHOT"
    "offline_config:=$OFFLINE_CONFIG_SNAPSHOT"
)
RECORD_TOPICS=(
    /dlio/odom_node/odom
    /dlio/odom_node/pointcloud/deskewed
)
RECORDER_CMD=(ros2 bag record --use-sim-time -o "$RESULT_BAG" "${RECORD_TOPICS[@]}")
PLAYER_CMD=(
    ros2 bag play "$BAG" --clock --rate "$RATE" --start-paused
    --disable-keyboard-controls --start-offset "$START_OFFSET"
    --topics /points_raw /go2w/imu
)
ANALYZE_CMD=()
if [ "$ANALYZE" = "true" ]; then
    ANALYZE_CMD=(
        python3 "$ANALYZER_SNAPSHOT" analyze "$RESULT_BAG"
        --output-dir "$OUTPUT_DIR"
        --label baseline
        --voxel-size "$MAP_VOXEL_SIZE"
        --preview-max-points "$PREVIEW_MAX_POINTS"
        --plane-random-seed "$PLANE_RANDOM_SEED"
    )
fi
{
    printf 'launch:'; printf ' %q' "${LAUNCH_CMD[@]}"; printf '\n'
    printf 'record:'; printf ' %q' "${RECORDER_CMD[@]}"; printf '\n'
    printf 'play:'; printf ' %q' "${PLAYER_CMD[@]}"; printf '\n'
    if [ "$ANALYZE" = "true" ]; then
        printf 'analyze:'; printf ' %q' "${ANALYZE_CMD[@]}"; printf '\n'
    fi
} > "$OUTPUT_DIR/commands.log"

write_manifest "starting"

echo "Starting headless D-LIO in ROS domain $ROS_DOMAIN_ID"
echo "Bag: $BAG"
echo "Output: $OUTPUT_DIR"

setsid "${LAUNCH_CMD[@]}" > "$OUTPUT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
setsid "${RECORDER_CMD[@]}" > "$OUTPUT_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
setsid "${PLAYER_CMD[@]}" > "$OUTPUT_DIR/player.log" 2>&1 &
PLAYER_PID=$!

wait_for_node /dlio_odom_node || die "D-LIO did not become ready; see launch.log"
ros2 param dump --no-daemon --output-dir "$OUTPUT_DIR" /dlio_odom_node \
    > "$PARAMETER_DUMP_LOG" 2>&1
[ -s "$DLIO_PARAMETERS_SNAPSHOT" ] \
    || die "D-LIO parameter snapshot was not created"
validate_dlio_parameters \
    > "$PARAMETER_VALIDATION_LOG" 2>&1 \
    || {
        tail -n 40 "$PARAMETER_VALIDATION_LOG" >&2 || true
        die "D-LIO live parameters violate the offline artifact contract"
    }
DLIO_PARAMETERS_SHA256="$(sha256sum "$DLIO_PARAMETERS_SNAPSHOT" | awk '{print $1}')"
wait_for_service /rosbag2_player/resume || die "rosbag player resume service did not appear"

wait_for_topic /points_raw 1 1 || die "/points_raw endpoints did not become ready"
wait_for_topic /go2w/imu 1 1 || die "/go2w/imu endpoints did not become ready"
wait_for_topic /dlio/odom_node/odom 1 1 \
    || die "/dlio/odom_node/odom endpoints did not become ready"
wait_for_topic /dlio/odom_node/pointcloud/deskewed 1 1 \
    || die "/dlio/odom_node/pointcloud/deskewed endpoints did not become ready"

SAMPLER_CMD=(
    python3 "$SCRIPT_DIR/sample_process_metrics.py"
    --target "dlio:$LAUNCH_PID:dlio_odom_node"
    --target "player:$PLAYER_PID:*"
    --target "recorder:$RECORDER_PID:*"
    --output "$METRICS_CSV" --summary "$METRICS_SUMMARY"
    --stop-file "$STOP_FILE" --interval 1.0
)
setsid "${SAMPLER_CMD[@]}" > "$OUTPUT_DIR/metrics.log" 2>&1 &
SAMPLER_PID=$!

RESUME_TYPE="rosbag2_interfaces/srv/Resume"
ros2 service call /rosbag2_player/resume "$RESUME_TYPE" "{}" \
    > "$OUTPUT_DIR/resume.log" 2>&1
write_manifest "running"

if [ -n "$DURATION" ]; then
    WATCHDOG_WALL_SECONDS="$(python3 -c 'import sys; print(float(sys.argv[1]) / float(sys.argv[2]))' "$DURATION" "$RATE")"
    (
        sleep "$WATCHDOG_WALL_SECONDS"
        touch "$DURATION_MARKER"
        kill -INT -- "-$PLAYER_PID" 2>/dev/null || true
    ) &
    WATCHDOG_PID=$!
fi

PROCESSING_NODE_MISSES=0
while kill -0 "$PLAYER_PID" 2>/dev/null; do
    kill -0 "$LAUNCH_PID" 2>/dev/null \
        || die "processing launch exited during playback; see launch.log"
    kill -0 "$RECORDER_PID" 2>/dev/null \
        || die "rosbag recorder exited during playback; see recorder.log"
    kill -0 "$SAMPLER_PID" 2>/dev/null \
        || die "resource sampler exited during playback; see metrics.log"
    if required_processing_nodes_alive; then
        PROCESSING_NODE_MISSES=0
    else
        PROCESSING_NODE_MISSES=$((PROCESSING_NODE_MISSES + 1))
        [ "$PROCESSING_NODE_MISSES" -lt 3 ] \
            || die "required processing node disappeared during playback"
    fi
    sleep 1
done

set +e
wait "$PLAYER_PID"
PLAYER_STATUS=$?
set -e
if [ -n "$WATCHDOG_PID" ]; then
    kill "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
fi
if [ "$PLAYER_STATUS" -ne 0 ] && [ ! -f "$DURATION_MARKER" ]; then
    die "rosbag player exited with status $PLAYER_STATUS; see player.log"
fi
kill -0 "$LAUNCH_PID" 2>/dev/null || die "processing launch exited before drain"
kill -0 "$RECORDER_PID" 2>/dev/null || die "rosbag recorder exited before drain"
kill -0 "$SAMPLER_PID" 2>/dev/null || die "resource sampler exited before drain"
required_processing_nodes_alive || die "required processing node missing before drain"

write_manifest "draining"
if ! wait_for_processing_quiescence; then
    die "processing drain failed ($DRAIN_OUTCOME); result bag never became safely quiescent"
fi
write_manifest "drained"

stop_group "$RECORDER_PID" "rosbag recorder"
stop_group "$LAUNCH_PID" "processing launch"
touch "$STOP_FILE"
for _ in $(seq 1 30); do
    kill -0 "$SAMPLER_PID" 2>/dev/null || break
    sleep 0.1
done
stop_group "$SAMPLER_PID" "resource sampler"

write_manifest "validating"
if ! validate_result_bag > "$RESULT_BAG_VALIDATION_LOG" 2>&1; then
    tail -n 40 "$RESULT_BAG_VALIDATION_LOG" >&2 || true
    die "finalized result bag validation failed; see $RESULT_BAG_VALIDATION_LOG"
fi
RESULT_METADATA_SHA256="$(sha256sum "$RESULT_BAG/metadata.yaml" | awk '{print $1}')"

if ! validate_resource_artifacts > "$RESOURCE_VALIDATION_LOG" 2>&1; then
    tail -n 40 "$RESOURCE_VALIDATION_LOG" >&2 || true
    die "resource artifact validation failed; see $RESOURCE_VALIDATION_LOG"
fi
RESOURCE_METRICS_SHA256="$(sha256sum "$METRICS_CSV" | awk '{print $1}')"
RESOURCE_SUMMARY_SHA256="$(sha256sum "$METRICS_SUMMARY" | awk '{print $1}')"

if [ "$ANALYZE" = "true" ]; then
    write_manifest "analyzing"
    echo "Generating final map and trajectory artifacts..."
    if ! "${ANALYZE_CMD[@]}" > "$ANALYSIS_LOG" 2>&1; then
        tail -n 40 "$ANALYSIS_LOG" >&2 || true
        die "artifact analysis failed; see $ANALYSIS_LOG"
    fi
    for artifact in summary.json map_voxelized.pcd map_preview.pcd trajectory.csv; do
        [ -s "$OUTPUT_DIR/$artifact" ] \
            || die "artifact analysis did not create $artifact"
    done
fi

rm -f "$STOP_FILE" "$DURATION_MARKER"
write_manifest "completed" 0
CLEANUP_COMPLETE="true"
trap - EXIT INT TERM

echo "Experiment complete: $OUTPUT_DIR"
