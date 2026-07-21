# dlio-go2w

## Table of contents

- [Attribution](#attribution)
- [Repository layout](#repository-layout)
- [Submodules](#submodules)
- [Setup](#setup)
- [Bag types](#bag-types)
- [Online D-LIO (robot)](#online-d-lio-robot)
  - [Desktop live RViz over WiFi](#desktop-live-rviz-over-wifi)
- [Record D-LIO Outputs (robot)](#record-d-lio-outputs-robot)
- [Record Raw Sensor Data (robot)](#record-raw-sensor-data-robot)
- [Diagnose Raw Bag Timestamps](#diagnose-raw-bag-timestamps)
- [Sanity-check and Clean LiDAR Frames](#sanity-check-and-clean-lidar-frames)
- [Replay D-LIO Outputs (robot or desktop)](#replay-d-lio-outputs-robot-or-desktop)
- [Reconstruct D-LIO From Raw Bag (robot or desktop)](#reconstruct-d-lio-from-raw-bag-robot-or-desktop)
- [Headless Offline Processing and Saved Results](#headless-offline-processing-and-saved-results)
- [Diagnose TF (desktop)](#diagnose-tf-desktop)
- [Catmux sessions](#catmux-sessions)
- [License](#license)

ROS 2 workspace for running **Direct LiDAR-Inertial Odometry (D-LIO)** on the Unitree GO2-W robot with a Hesai PandarXT-16 LiDAR.

## Attribution

The D-LIO algorithm is developed by the [VECTR Lab at UCLA](https://github.com/vectr-ucla):

> K. J. Chen, R. Nemiroff, and B. T. Lopez, "Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction," *2023 IEEE International Conference on Robotics and Automation (ICRA)*, London, UK, 2023.
> [[IEEE](https://ieeexplore.ieee.org/document/10160508)] [[arXiv](https://arxiv.org/abs/2203.03749)] [[GitHub](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)]

This workspace uses a GO2-W / Hesai-compatible fork at [koki67/direct_lidar_inertial_odometry](https://github.com/koki67/direct_lidar_inertial_odometry) (`feature/ros2` branch). The algorithm source and its MIT license are included verbatim as a Git submodule; see `humble_ws/src/direct_lidar_inertial_odometry/LICENSE`.

## Repository layout

```
dlio-go2w/
├── humble_ws/src/
│   ├── direct_lidar_inertial_odometry/  D-LIO algorithm (submodule)
│   ├── go2w-imu-publisher/              IMU republisher from GO2-W lowstate (submodule)
│   ├── go2w-hesai-lidar-driver/         Hesai PandarXT-16 ROS 2 driver (submodule)
│   └── unitree_ros2/                    Unitree DDS bindings + setup.sh (submodule)
├── docker/robot/                        ARM64 ROS 2 Humble image for the Jetson
├── .devcontainer/                       Desktop devcontainer (amd64, bag replay + reconstruction)
├── catmux/                              Robot-side tmux session definitions
├── scripts/dlio/                        Desktop offline scripts
├── scripts/offline/                     Headless runs, analysis, and saved-result viewing
├── scripts/diagnosis/                   TF and timestamp validation helpers
├── config/
│   ├── dlio/                            RViz config
│   └── sensor/                          Shared sensor calibration reference
└── bags/                                Bag storage (gitignored)
```

`config/sensor/go2w_calibration.yaml` is the single source of truth for extrinsic calibration values.

## Submodules

| Package | Repository | Branch | Purpose |
|---|---|---|---|
| `direct_lidar_inertial_odometry` | [koki67/direct_lidar_inertial_odometry](https://github.com/koki67/direct_lidar_inertial_odometry) | `feature/ros2` | D-LIO algorithm |
| `go2w-imu-publisher` | [koki67/go2w-imu-publisher](https://github.com/koki67/go2w-imu-publisher) | `main` | GO2-W IMU republisher |
| `go2w-hesai-lidar-driver` | [koki67/go2w-hesai-lidar-driver](https://github.com/koki67/go2w-hesai-lidar-driver) | `main` | Hesai XT16 ROS 2 driver |
| `unitree_ros2` | [koki67/unitree_ros2](https://github.com/koki67/unitree_ros2) | `master` | Unitree ROS 2 bindings & DDS |

Clone with submodules:
```bash
git clone --recurse-submodules git@github.com:koki67/dlio-go2w.git
```

Private-submodule setup (private repos):
- Ensure your GitHub SSH key is configured in your account and loaded in the agent.
- Clone and initialize submodules:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Setup

1. Build the robot Docker image (on the Jetson Orin NX):
   ```bash
   docker build -f docker/robot/Dockerfile -t dlio-go2w:latest .
   ```

2. Start the container:
   ```bash
   bash docker/robot/run.sh
   ```

3. Inside the container, build the workspace:
   ```bash
   cd /external/humble_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Bag types

This workspace distinguishes two types of bags with different use cases:

| Bag prefix | Contents | Use for |
|---|---|---|
| `dlio_YYYYMMDD_HHMMSS` | D-LIO output topics (odom, path, map, tf) | Fast replay — no algorithm needed |
| `raw_YYYYMMDD_HHMMSS` | Raw sensors only (`/go2w/imu` + `/points_raw`) | Re-running D-LIO to reconstruct outputs |

Replaying a `dlio_` bag shows previously computed results. Reconstructing from a `raw_` bag re-runs D-LIO, which lets you change parameters and get updated outputs.

## Online D-LIO (robot)

Complete [Setup](#setup) first, then:

```bash
catmux_create_session /external/catmux/online_dlio.yaml
```

This starts the IMU publisher, Hesai driver, and D-LIO node in a single tmux session. D-LIO uses the repository-standard Go2W TF by default: the URDF `imu` pose and the Hesai native-frame Rz(+90deg) LiDAR correction. CycloneDDS is enabled on both `eth0` and `wlan0` (when available), so D-LIO topics are visible over WiFi from the desktop.

### Desktop live RViz over WiFi

Open this repository in VS Code and reopen it in the devcontainer, then:
```bash
bash scripts/dlio/live_rviz.sh --iface enp97s0
```

Replace `enp97s0` with the actual desktop interface on the same subnet as the robot.

## Record D-LIO Outputs (robot)

Records the processed D-LIO topics for compact replay. No algorithm needed to replay this bag.

```bash
catmux_create_session /external/catmux/record_dlio.yaml
```

Recorded topics: `/dlio/odom_node/odom`, `/dlio/odom_node/path`, `/dlio/odom_node/keyframes`, `/dlio/odom_node/pointcloud/deskewed`, `/map`, `/tf`, `/tf_static`

Bags are saved to `/external/bags/dlio_YYYYMMDD_HHMMSS`.

## Record Raw Sensor Data (robot)

Records `/go2w/imu` and `/points_raw` only. Use this when you want to re-run D-LIO with different parameters later.

```bash
catmux_create_session /external/catmux/record_raw.yaml
```

Bags are saved to `/external/bags/raw_YYYYMMDD_HHMMSS`.

## Diagnose Raw Bag Timestamps

Use this after recording a `raw_` bag to inspect whether the GO2-W IMU and Hesai LiDAR timestamps are usable for D-LIO. The script reads the rosbag2 files directly; it does not run `ros2 bag play`, start D-LIO, or open RViz.

The default topics match `catmux/record_raw.yaml`:
- `/go2w/imu` from `go2w-imu-publisher`
- `/points_raw` from `go2w-hesai-lidar-driver`

Run it after sourcing ROS 2:
```bash
cd /home/user/ws/dlio-go2w
source /opt/ros/humble/setup.bash
python3 scripts/diagnosis/check_raw_bag_timestamps.py /external/bags/raw_YYYYMMDD_HHMMSS
```

Optional JSON output:
```bash
python3 scripts/diagnosis/check_raw_bag_timestamps.py /external/bags/raw_YYYYMMDD_HHMMSS --json timing_report.json
```

The report includes:
- message counts and time ranges for both topics
- `header.stamp` monotonicity, duplicate stamps, and estimated rates
- `bag_record_time - header.stamp`, which shows recording/transport delay from the bag's point of view
- nearest IMU sample gaps for each LiDAR frame
- Hesai `PointCloud2` per-point `timestamp` field checks used by D-LIO deskewing

This is an observation tool, not a pass/fail gate. Use the output to decide whether the IMU and LiDAR streams overlap in time, whether IMU stamps are monotonic, and whether a fixed LiDAR/IMU time offset should be estimated before offline reconstruction.

## Sanity-check and Clean LiDAR Frames

After `check_raw_bag_timestamps.py`, use the following tools to isolate bad LiDAR frames and create a sanitized copy of a raw bag.

### 1) Locate LiDAR timestamp anomalies

`scripts/diagnosis/locate_lidar_timestamp_anomalies.py` scans a raw bag for
non-increasing/duplicate LiDAR frames, unusual per-frame delays, and IMU alignment
outliers. It prints each flagged frame with nearby context so you can identify the
exact frames to drop.

```bash
source /opt/ros/humble/setup.bash
python3 scripts/diagnosis/locate_lidar_timestamp_anomalies.py /external/bags/raw_YYYYMMDD_HHMMSS
```

Optional JSON report:
```bash
python3 scripts/diagnosis/locate_lidar_timestamp_anomalies.py /external/bags/raw_YYYYMMDD_HHMMSS --json anomalies.json
```

### 2) Clean a raw bag by dropping bad LiDAR frames

`scripts/dlio/clean_raw_bag_timestamps.py` writes a new rosbag2 directory and drops
LiDAR frames flagged as anomalous. By default it drops non-increasing LiDAR
header timestamps; it can also inspect every Hesai per-point `timestamp` and
drop a scan whose internal point-time range is inconsistent even when the scan
header itself is monotonic. The source bag is never modified.

```bash
source /opt/ros/humble/setup.bash
python3 scripts/dlio/clean_raw_bag_timestamps.py /external/bags/raw_YYYYMMDD_HHMMSS /external/bags/raw_YYYYMMDD_HHMMSS_clean
```

For the GO2-W Hesai recordings used by the offline artifact workflow, enable
the per-point gate as well. A 200 ms threshold accepts the normal roughly
100 ms scan span while rejecting delayed points several seconds outside the
scan header:

```bash
python3 scripts/dlio/clean_raw_bag_timestamps.py \
  /mnt/go2w-experiment-recorder/bags/experiment_... \
  /mnt/dlio-go2w/results/derived-inputs/experiment_..._dlio_clean_points \
  --drop-point-time-offset-ms 200 \
  --json /mnt/dlio-go2w/results/derived-inputs/experiment_..._dlio_clean_points_report.json
```

Useful options:
- `--drop-record-delay-ms`: drop frames where `bag_time - header.stamp` is above a threshold.
- `--drop-lidar-outside-imu-range`: drop frames outside IMU header time range.
- `--drop-point-time-offset-ms`: drop frames whose minimum or maximum per-point
  timestamp differs from the PointCloud2 header by more than the threshold.
- `--keep-non-increasing`: keep non-increasing LiDAR header timestamps (off by default; they are dropped).
- `--trim-start-sec`: drop all messages before a bag-time offset.
- `--force`: overwrite output directory if it already exists.

Optional machine-readable report:
```bash
python3 scripts/dlio/clean_raw_bag_timestamps.py /external/bags/raw_... /external/bags/raw_..._clean --json clean_report.json
```

Use the output summary to confirm dropped frame count before running
`scripts/dlio/reconstruct_raw.sh` on the cleaned bag path.

## Rebase a Manually Split Raw Bag for D-LIO

When raw recording was deliberately stopped with `Ctrl+C` between rooms or
buildings and the resulting bags were subsequently merged, the gap is real in
both the IMU and LiDAR timelines. D-LIO integrates IMU time continuously, so a
multi-second gap can make it jump at the join even though the merged rosbag is
otherwise valid.

`scripts/dlio/rebase_raw_bag_timestamps.py` creates a **new, D-LIO-input-only**
bag with those long gaps compressed. It does not change the source bag. For
every later segment it shifts the rosbag playback timestamp, the IMU header,
the PointCloud2 header, and Hesai's absolute per-point `timestamp` field by
the same amount. Updating all four is necessary to keep D-LIO deskewing
consistent.

Use this only when each manual stop represents unrecorded time and the robot
was stationary (or was returned to the same pose) before recording resumed.
It does not estimate the unrecorded motion; if the robot moved during a stop,
reconstruct each segment separately and align the maps instead.

First inspect the proposed split points without writing anything:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/dlio/rebase_raw_bag_timestamps.py \
  bags/experiment_room1_room2_room3_merged \
  --dry-run
```

Then create the derived bag and validate it before reconstruction:

```bash
python3 scripts/dlio/rebase_raw_bag_timestamps.py \
  bags/experiment_room1_room2_room3_merged \
  bags/experiment_room1_room2_room3_merged_dlio_rebased

python3 scripts/diagnosis/check_raw_bag_timestamps.py \
  bags/experiment_room1_room2_room3_merged_dlio_rebased

python3 scripts/diagnosis/locate_lidar_timestamp_anomalies.py \
  bags/experiment_room1_room2_room3_merged_dlio_rebased \
  --imu-gap-threshold-ms 10

bash scripts/dlio/reconstruct_raw.sh \
  bags/experiment_room1_room2_room3_merged_dlio_rebased \
  --topics /points_raw /go2w/imu --rate 1.0
```

The default policy treats a combined IMU/LiDAR header gap greater than one
second as a recording boundary and leaves a 5 ms bridge. Customize it only
when the diagnostics show a different natural boundary:

- `--gap-threshold-sec`: gap that starts a new recording segment (default: `1.0`)
- `--bridge-gap-ms`: smallest time left between segments in both header and bag time (default: `5.0`)
- `--force`: replace an existing derived output bag
- `--json report.json`: retain the detected boundaries and applied offsets for provenance

The derived bag intentionally contains only `/go2w/imu` and `/points_raw`.
Use the original merged bag for other topics or for provenance; keep the
optional JSON report alongside the derived output.
## Replay D-LIO Outputs (robot or desktop)

**On robot** — edit the bag path in `catmux/playback_dlio.yaml`, then:
```bash
catmux_create_session /external/catmux/playback_dlio.yaml
```

**On desktop** — open the devcontainer, then:
```bash
bash scripts/dlio/playback.sh bags/dlio_YYYYMMDD_HHMMSS
```

RViz opens automatically. Close the window or press `Ctrl+C` to stop.

## Reconstruct D-LIO From Raw Bag (robot or desktop)

Re-runs the D-LIO algorithm on a raw sensor bag and visualizes the outputs in real time. The default TF profile is the repository-standard Go2W profile, so no `--tf-profile` option is needed for normal runs.

**On robot** — edit the bag path in `catmux/reconstruct_raw_dlio.yaml`, then:
```bash
catmux_create_session /external/catmux/reconstruct_raw_dlio.yaml
```

**On desktop** — open the devcontainer, then:
```bash
bash scripts/dlio/reconstruct_raw.sh bags/raw_YYYYMMDD_HHMMSS
```

The devcontainer mounts these host bag directories read-only and the offline
result directory read-write, so large artifacts remain outside the repository:

- `/mnt/data1/experimental_data/go2w-experiment-recorder/bags` at
  `/mnt/go2w-experiment-recorder/bags`
- `/mnt/data1/experimental_data/dlio-go2w/bags` at `/mnt/dlio-go2w/bags`
- `/mnt/data1/experimental_data/dlio-go2w/results` at `/mnt/dlio-go2w/results`

```bash
bash scripts/dlio/reconstruct_raw.sh /mnt/go2w-experiment-recorder/bags/raw_YYYYMMDD_HHMMSS
bash scripts/dlio/playback.sh /mnt/dlio-go2w/bags/dlio_YYYYMMDD_HHMMSS
```

After pulling this configuration change, run **Dev Containers: Rebuild and
Reopen in Container** once to apply the new mount.

## Headless Offline Processing and Saved Results

The reproducible offline workflow runs D-LIO once without RViz, records only
its odometry and odom-frame deskewed scans, then creates and visualizes the
completed map separately:

```bash
BAG=/mnt/go2w-experiment-recorder/bags/raw_YYYYMMDD_HHMMSS
OUT="${DLIO_RESULTS_ROOT:-$PWD/results}/dlio/example/baseline"

bash scripts/offline/run_dlio_offline.sh \
  "$BAG" --rate 1.0 --output "$OUT"
```

The runner consumes only `/points_raw` and `/go2w/imu`, starts playback paused
until all endpoints are ready, and generates `map_voxelized.pcd`,
`map_preview.pcd`, `trajectory.csv`, `summary.json`, provenance snapshots,
resource metrics, and a frozen two-topic result bag.

Inspect the final result without rerunning D-LIO:

```bash
bash scripts/offline/visualize_dlio_run.sh "$OUT"
```

Replay the saved result as a growing map and path:

```bash
bash scripts/offline/visualize_dlio_run.sh "$OUT" --dynamic --rate 2.0
```

Neither visualization mode runs the LIO algorithm. See
[the offline artifact workflow](docs/offline-result-artifacts.md) for the
storage contract, validation, comparison, and troubleshooting details.

## Diagnose TF (desktop)

`check_tf.sh` is a lightweight TF-only check for D-LIO frame alignment and extrinsics sanity.

It launches two static transforms from the standard `dlio.yaml` parameters:
- `base_link` -> `imu` (URDF pose, orientation aligned with `base_link`)
- `base_link` -> `hesai_lidar` (Hesai native-frame Rz(+90deg) correction)
and optional RViz visualization (`direct_lidar_inertial_odometry` check_tf config) to inspect the TF tree, including `/tf` and `/tf_static`.

Use it to catch broken/extrinsics-misconfigured calibrations before live runs.

```bash
bash scripts/diagnosis/check_tf.sh [--rviz false|true]
```

Flags:
- `--rviz true` (default): launch RViz for interactive frame inspection
- `--rviz false`: headless mode for CLI-only checks
- `--tf-profile legacy`: inspect the old pre-URDF-IMU profile

## Catmux sessions

| Session file | Purpose |
|---|---|
| `catmux/online_dlio.yaml` | Live D-LIO: IMU + LiDAR + SLAM |
| `catmux/record_dlio.yaml` | Record D-LIO output topics |
| `catmux/record_raw.yaml` | Record raw sensors for reconstruction |
| `catmux/playback_dlio.yaml` | Replay a recorded D-LIO bag |
| `catmux/reconstruct_raw_dlio.yaml` | Re-run D-LIO offline from a raw bag |

Create any session with `catmux_create_session /external/<path-to-yaml>`. Reconnect to a running session with `catmux attach`.

## License

The workspace configuration, scripts, and Dockerfiles in this repository are licensed under the MIT License — see [LICENSE](LICENSE).

The D-LIO algorithm (in `humble_ws/src/direct_lidar_inertial_odometry/`) carries its own MIT license by Kenny J. Chen, Ryan Nemiroff, and Brett T. Lopez — see `humble_ws/src/direct_lidar_inertial_odometry/LICENSE`.
