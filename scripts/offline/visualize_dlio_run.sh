#!/usr/bin/env bash
# Display an analyzed offline D-LIO run without rerunning D-LIO.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PUBLISHER="$SCRIPT_DIR/publish_dlio_artifacts.py"
REPLAYER="$SCRIPT_DIR/replay_dlio_artifacts.py"
RVIZ_CONFIG="$SCRIPT_DIR/rviz/offline_result.rviz"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/visualize_dlio_run.sh RUN_DIR [options]

RUN_DIR may be a timestamped run directory or its bag-level parent. When a
parent is supplied, the newest completed run directly below it is selected.

Options:
  --dynamic     Grow the map/path while replaying saved D-LIO outputs
  --rate RATE   Dynamic result replay rate (default: 1.0)
  --no-rviz     Keep publishers running without RViz; this is not a one-shot check
  -h, --help    Show this help

Static mode publishes the frozen preview map and matching-frame trajectory.
Dynamic mode starts empty, incrementally voxelizes saved
/dlio/odom_node/pointcloud/deskewed
scans, and extends the path from saved /dlio/odom_node/odom. Neither mode runs D-LIO.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

manifest_completed() {
    python3 -c '
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    manifest = json.load(stream)
sys.exit(0 if manifest.get("state") == "completed" and manifest.get("exit_code") == 0 else 1)
' "$1"
}

resolve_run_dir() {
    local requested="$1"
    local candidate
    local -a candidates=()

    if [ -f "$requested/summary.json" ]; then
        printf '%s\n' "$requested"
        return 0
    fi

    shopt -s nullglob
    for candidate in "$requested"/*; do
        [ -d "$candidate" ] || continue
        [ -f "$candidate/summary.json" ] || continue
        [ -f "$candidate/manifest.json" ] || continue
        if manifest_completed "$candidate/manifest.json" 2>/dev/null; then
            candidates+=("$candidate")
        fi
    done
    shopt -u nullglob

    [ "${#candidates[@]}" -gt 0 ] \
        || die "summary.json does not exist in $requested or a completed child run"
    mapfile -t candidates < <(printf '%s\n' "${candidates[@]}" | sort -r)
    echo "Resolved latest completed run: ${candidates[0]}" >&2
    printf '%s\n' "${candidates[0]}"
}

RUN_DIR=""
DYNAMIC=false
RATE="1.0"
USE_RVIZ=true

while [ "$#" -gt 0 ]; do
    case "$1" in
        --dynamic)
            DYNAMIC=true
            shift
            ;;
        --rate)
            RATE="${2:?Error: --rate requires a value}"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            die "unknown option: $1"
            ;;
        *)
            [ -z "$RUN_DIR" ] || die "only one run directory may be supplied"
            RUN_DIR="$1"
            shift
            ;;
    esac
done

[ -n "$RUN_DIR" ] || die "an analyzed run directory is required"
python3 -c 'import math,sys; value=float(sys.argv[1]); sys.exit(not (math.isfinite(value) and value > 0.0))' "$RATE" \
    || die "--rate must be a finite number greater than zero"

RUN_DIR="$(realpath -m "$RUN_DIR")"
[ -d "$RUN_DIR" ] || die "run directory not found: $RUN_DIR"
RUN_DIR="$(resolve_run_dir "$RUN_DIR")"
[ -f "$PUBLISHER" ] || die "artifact publisher not found: $PUBLISHER"
[ -f "$REPLAYER" ] || die "dynamic replay publisher not found: $REPLAYER"
[ -f "$RVIZ_CONFIG" ] || die "RViz config not found: $RVIZ_CONFIG"

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
elif [ "${ROS_DISTRO:-}" != "humble" ]; then
    die "ROS 2 Humble is required (run inside the project container)"
fi

overlay_usable() (
    candidate="$1"
    install_root="$(realpath -m "$(dirname "$candidate")")"
    set +u
    # shellcheck source=/dev/null
    source "$candidate" >/dev/null 2>&1 || exit 1
    set -u
    prefix="$(ros2 pkg prefix direct_lidar_inertial_odometry 2>/dev/null)" || exit 1
    prefix="$(realpath -m "$prefix")"
    case "$prefix" in
        "$install_root"/*) exit 0 ;;
        *) exit 1 ;;
    esac
)

WORKSPACE_SETUP=""
for candidate in \
    "$REPO_ROOT/.devcontainer/offline_dlio/install/setup.bash" \
    "$REPO_ROOT/humble_ws/install/setup.bash"; do
    if [ -f "$candidate" ] && overlay_usable "$candidate"; then
        WORKSPACE_SETUP="$candidate"
        break
    fi
done
[ -n "$WORKSPACE_SETUP" ] \
    || die "no usable D-LIO workspace overlay was found; build the workspace"
set +u
# shellcheck source=/dev/null
source "$WORKSPACE_SETUP"
set -u

for command in python3 ros2 setsid; do
    command -v "$command" >/dev/null || die "required command not found: $command"
done
if [ "$USE_RVIZ" = true ]; then
    command -v rviz2 >/dev/null || die "rviz2 is not installed"
    if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
        die "no graphical display; --no-rviz keeps publishers running headlessly"
    fi
fi
if [ "$DYNAMIC" = true ]; then
    [ -f "$RUN_DIR/rosbag/metadata.yaml" ] \
        || die "dynamic result bag not found: $RUN_DIR/rosbag"
fi

if [ "$DYNAMIC" = true ]; then
    ARTIFACT_MAP_FRAME_ID="$(python3 "$REPLAYER" "$RUN_DIR" --print-frame-id)"
else
    ARTIFACT_MAP_FRAME_ID="$(
        python3 "$PUBLISHER" "$RUN_DIR" \
            --trajectory-topic /dlio/odom_node/odom --print-frame-id
    )"
fi
[ -n "$ARTIFACT_MAP_FRAME_ID" ] || die "validated artifact map frame is empty"
echo "Validated artifact map frame: $ARTIFACT_MAP_FRAME_ID"

ARTIFACT_PID=""
PLAYER_PID=""
CLEANED_UP=false

stop_group() {
    local pid="${1:-}"
    [ -n "$pid" ] || return 0
    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
    fi
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 25); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [ "$CLEANED_UP" != true ]; then
        stop_group "$PLAYER_PID"
        stop_group "$ARTIFACT_PID"
        CLEANED_UP=true
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo "Run: $RUN_DIR"
echo "Workspace: $WORKSPACE_SETUP"
if [ "$DYNAMIC" = true ]; then
    ARTIFACT_CMD=(python3 "$REPLAYER" "$RUN_DIR")
    echo "Starting growing map/path publisher from saved results..."
else
    ARTIFACT_CMD=(
        python3 "$PUBLISHER" "$RUN_DIR" --trajectory-topic /dlio/odom_node/odom
    )
    echo "Starting frozen map/path publisher..."
fi
setsid "${ARTIFACT_CMD[@]}" &
ARTIFACT_PID=$!
sleep 0.5
kill -0 "$ARTIFACT_PID" 2>/dev/null \
    || die "map/path publisher exited during startup"

if [ "$DYNAMIC" = true ]; then
    PLAYER_CMD=(
        ros2 bag play "$RUN_DIR/rosbag"
        --clock --rate "$RATE" --delay 2.0 --disable-keyboard-controls
        --topics /dlio/odom_node/pointcloud/deskewed /dlio/odom_node/odom
    )
    echo "Starting saved D-LIO result replay at ${RATE}x..."
    setsid "${PLAYER_CMD[@]}" &
    PLAYER_PID=$!
fi

if [ "$USE_RVIZ" = true ]; then
    RVIZ_CMD=(rviz2 -d "$RVIZ_CONFIG" -f "$ARTIFACT_MAP_FRAME_ID")
    if [ "$DYNAMIC" = true ]; then
        RVIZ_CMD+=(--ros-args -p use_sim_time:=true)
    fi
    echo "Starting RViz..."
    "${RVIZ_CMD[@]}"
elif [ "$DYNAMIC" = true ]; then
    wait "$PLAYER_PID"
    # Let the steady-clock update timer publish the final partial batch.
    sleep 1.2
else
    echo "Headless static publisher is running; press Ctrl-C to stop."
    wait "$ARTIFACT_PID"
fi
