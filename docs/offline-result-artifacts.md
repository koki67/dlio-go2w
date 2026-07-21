# Offline D-LIO result artifacts and visualization

This workflow processes a recorded Go2W/Hesai bag once without RViz, saves the
computed D-LIO outputs, builds deterministic map and trajectory artifacts, and
visualizes those frozen results later. It is separate from the interactive
`scripts/dlio/reconstruct_raw.sh` workflow and never starts sensor drivers.

The source bag is read-only. Processing consumes only `/points_raw` and
`/go2w/imu`.

## Data flow

```text
raw Pandar XT-16 bag
  /points_raw + /go2w/imu
        |
        | scripts/offline/run_dlio_offline.sh
        | headless D-LIO + result recording + resource sampling
        v
run directory
  rosbag/{/dlio/odom_node/odom,
          /dlio/odom_node/pointcloud/deskewed}
        |
        | deterministic post-processing
        v
  summary.json + trajectory.csv + voxelized/preview PCD maps
        |
        | scripts/offline/visualize_dlio_run.sh
        v
  static final map or growing replay of already-computed results
```

The runner starts source-bag playback paused, waits for D-LIO, the recorder,
and both input subscriptions, validates the live headless parameters, and only
then resumes playback. After the player exits, it waits for result writes to
become quiescent, finalizes and validates the result bag, and runs the analyzer.

## Build and storage setup

Rebuild the desktop D-LIO overlay after pulling the feature:

```bash
bash .devcontainer/postCreate.sh
```

The devcontainer maps the host directory
`/mnt/data1/experimental_data/dlio-go2w/results` to
`/mnt/dlio-go2w/results` and sets:

```bash
DLIO_RESULTS_ROOT=/mnt/dlio-go2w/results
```

Outside the devcontainer, the default is `<repository>/results`. Set
`DLIO_RESULTS_ROOT` or pass `--output` to override it. Result directories are
write-once: the runner refuses to mix a new run with existing files.

## Run headless D-LIO

From the repository root in the ROS 2 Humble desktop container:

GO2-W Hesai bags can contain isolated scans whose header or internal per-point
timestamps are several seconds out of order. Sanitize the source without
modifying it before a production run:

```bash
SOURCE=/mnt/go2w-experiment-recorder/bags/experiment_...
CLEAN=/mnt/dlio-go2w/results/derived-inputs/experiment_..._dlio_clean_points

python3 scripts/dlio/clean_raw_bag_timestamps.py \
  "$SOURCE" "$CLEAN" \
  --drop-point-time-offset-ms 200 \
  --json "${CLEAN}_report.json"
```

The report records every dropped scan. Use the cleaned path as `BAG`; the
original recorder bag remains read-only.

```bash
BAG=/mnt/dlio-go2w/results/derived-inputs/experiment_..._dlio_clean_points
OUT="${DLIO_RESULTS_ROOT:-$PWD/results}/dlio/example/baseline"

bash scripts/offline/run_dlio_offline.sh \
  "$BAG" --rate 1.0 --output "$OUT"
```

Useful options:

| Option | Purpose |
| --- | --- |
| `--start-offset SEC` | Start within the source bag |
| `--duration SEC` | Process an approximate bag duration for a smoke test |
| `--rate RATE` | Source playback multiplier; default `1.0` |
| `--domain-id ID` | Isolated ROS domain; default `77` |
| `--dlio-config YAML` | Calibration/intrinsics override |
| `--params-config YAML` | Runtime/tuning override |
| `--no-analyze` | Keep the frozen result bag without generating PCD/CSV files |
| `--map-voxel-size M` | Final map voxel edge; default `0.20` m |
| `--preview-max-points N` | RViz preview cap; default `500000` |
| `--plane-random-seed N` | Deterministic local-plane sample seed; default `7` |

A duration-limited run is useful for endpoint checks but is not a complete map.

## Headless D-LIO contract

The dedicated launch starts only `dlio_odom_node`. It does not start the D-LIO
map node, drivers, TF output, cumulative `nav_msgs/Path`, keyframe publishers,
or RViz. The offline override keeps only:

- `/dlio/odom_node/odom`, published once per processed scan;
- `/dlio/odom_node/pointcloud/deskewed`, already transformed into `odom`.

The offline override also sets `map/waitUntilMove: false` so the initial
stationary scans are retained; this changes only artifact publication, not the
odometry estimator or regular interactive profiles.

Regular launch profiles retain their previous publisher behavior because all
new output-control parameters default to enabled. Event-driven odometry is an
offline-only override; it lets result recording become quiescent after D-LIO
finishes its input queue.

The runner snapshots the calibration, runtime, offline, and live parameters,
plus hashes of the launch file, executable, source bag metadata, analyzer, and
result metadata.

## Generated artifacts

A successful analyzed run contains:

| Artifact | Purpose |
| --- | --- |
| `manifest.json` | Input, playback, hashes, process state, and artifact inventory |
| `dlio_calibration.yaml` | Exact calibration/intrinsics input |
| `dlio_params.yaml` | Exact runtime/tuning input |
| `dlio_offline.yaml` | Fixed headless output contract |
| `dlio_odom_node.yaml` | Live D-LIO parameter dump |
| `commands.log` | Shell-escaped launch, record, play, and analysis commands |
| `rosbag/` | Frozen odometry and odom-frame deskewed scans |
| `resource_metrics.csv` | Per-process CPU-time and RSS samples |
| `resource_summary.json` | Resource-sampling summary |
| `trajectory.csv` | Saved D-LIO base trajectory |
| `map_voxelized.pcd` | Voxelized accumulation of all finite deskewed points |
| `map_preview.pcd` | Deterministic bounded-size RViz map |
| `summary.json` | Trajectory, map, resource, provenance, and artifact hashes |
| `analysis.log` | Analyzer output or error details |

Both the map and trajectory preserve and validate their recorded frame IDs.
D-LIO publishes its deskewed scans in `odom`, so no synthetic display transform
or sensor-frame calibration is needed during visualization.

`map_voxelized.pcd` contains `x`, `y`, `z`, and `count`; `count` is the number
of registered points accumulated in each voxel. Voxel keys and preview samples
are deterministic for the same saved result bag and analysis settings.

## Visualize a completed result

Static mode publishes only the frozen `map_preview.pcd` and `trajectory.csv`:

```bash
ROS_DOMAIN_ID=78 bash scripts/offline/visualize_dlio_run.sh "$OUT"
```

You may also pass the bag-level parent directory. The viewer selects its newest
direct child whose manifest is `completed` with `exit_code: 0`:

```bash
bash scripts/offline/visualize_dlio_run.sh \
  /mnt/dlio-go2w/results/dlio/experiment_..._dlio_clean_points
```

It does not play a bag or run D-LIO. This is the preferred final-map view.

Dynamic mode starts with an empty map and path, replays only the two saved
result topics, publishes each newly occupied voxel once, and grows the path as
saved odometry arrives:

```bash
ROS_DOMAIN_ID=78 bash scripts/offline/visualize_dlio_run.sh \
  "$OUT" --dynamic --rate 2.0
```

The replay rate changes animation speed but cannot change the saved D-LIO
result. RViz retains incremental voxel batches instead of every repeated raw
point, and displays the current saved deskewed scan and robot pose separately.

Validate frozen artifacts without starting publishers or RViz:

```bash
python3 scripts/offline/publish_dlio_artifacts.py "$OUT" --validate-only
```

## Compare repeated runs

Runs from the same source bag and compatible analysis settings can be compared:

```bash
python3 scripts/offline/analyze_dlio_run.py compare \
  results/dlio/run-a results/dlio/run-b \
  --labels run-a run-b \
  --output results/dlio/comparison_summary.json
```

Without ground truth, trajectory differences are consistency diagnostics, not
absolute localization error.

## Troubleshooting

If the runner reports that no current overlay is usable, rebuild it. The
runner rejects a stale installed launch/config or an executable older than the
modified D-LIO source.

If RViz is blank, first run the publisher with `--validate-only`. Dynamic mode
also requires `RUN_DIR/rosbag/metadata.yaml`; static mode needs only analyzed
PCD/CSV/JSON artifacts.
