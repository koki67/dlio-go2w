# dlio-go2w

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

## Quick start: Online D-LIO (on robot)

Complete [Setup](#setup) first, then:

```bash
catmux_create_session /external/catmux/online_dlio.yaml
```

This starts the IMU publisher, Hesai driver, and D-LIO node in a single tmux session. CycloneDDS is enabled on both `eth0` and `wlan0` (when available), so D-LIO topics are visible over WiFi from the desktop.

### Desktop live RViz over WiFi

Open this repository in VS Code and reopen it in the devcontainer, then:
```bash
bash scripts/dlio/live_rviz.sh --iface enp97s0
```

Replace `enp97s0` with the actual desktop interface on the same subnet as the robot.

## Quick start: Record D-LIO outputs (on robot)

Records the processed D-LIO topics for compact replay. No algorithm needed to replay this bag.

```bash
catmux_create_session /external/catmux/record_dlio.yaml
```

Recorded topics: `/dlio/odom_node/odom`, `/dlio/odom_node/path`, `/dlio/odom_node/keyframes`, `/dlio/odom_node/pointcloud/deskewed`, `/map`, `/tf`, `/tf_static`

Bags are saved to `/external/bags/dlio_YYYYMMDD_HHMMSS`.

## Quick start: Record raw sensor data (on robot)

Records `/go2w/imu` and `/points_raw` only. Use this when you want to re-run D-LIO with different parameters later.

```bash
catmux_create_session /external/catmux/record_raw.yaml
```

Bags are saved to `/external/bags/raw_YYYYMMDD_HHMMSS`.

## Quick start: Replay D-LIO outputs (on robot or desktop)

**On robot** — edit the bag path in `catmux/playback_dlio.yaml`, then:
```bash
catmux_create_session /external/catmux/playback_dlio.yaml
```

**On desktop** — open the devcontainer, then:
```bash
bash scripts/dlio/playback.sh bags/dlio_YYYYMMDD_HHMMSS
```

RViz opens automatically. Close the window or press `Ctrl+C` to stop.

## Quick start: Reconstruct D-LIO from raw bag (on robot or desktop)

Re-runs the D-LIO algorithm on a raw sensor bag and visualizes the outputs in real time.

**On robot** — edit the bag path in `catmux/reconstruct_raw_dlio.yaml`, then:
```bash
catmux_create_session /external/catmux/reconstruct_raw_dlio.yaml
```

**On desktop** — open the devcontainer, then:
```bash
bash scripts/dlio/reconstruct_raw.sh bags/raw_YYYYMMDD_HHMMSS
```

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
