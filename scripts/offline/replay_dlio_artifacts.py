#!/usr/bin/env python3
"""Build a growing map and trajectory from a saved D-LIO result bag.

This node does not run D-LIO. It subscribes to the already-computed deskewed
cloud and odometry messages replayed from a result bag,
publishes each newly occupied map voxel once, and republishes the trajectory as
it grows.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from analyze_dlio_run import pointcloud2_xyz


CLOUD_TOPIC = "/dlio/odom_node/pointcloud/deskewed"
ODOMETRY_TOPIC = "/dlio/odom_node/odom"

EXPECTED_TOPICS = {
    CLOUD_TOPIC: "sensor_msgs/msg/PointCloud2",
    ODOMETRY_TOPIC: "nav_msgs/msg/Odometry",
}


class ReplayArtifactError(ValueError):
    """Raised when a saved result cannot support dynamic replay."""


@dataclass(frozen=True)
class DynamicRunConfig:
    run_dir: Path
    bag_dir: Path
    frame_id: str
    odometry_frame_id: str
    odometry_child_frame_id: str
    voxel_size_m: float
    topic_message_counts: Mapping[str, int]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ReplayArtifactError(f"{label} must be a JSON object")
    return value


def _normalize_frame_id(value: Any) -> str:
    return str(value or "").strip().lstrip("/")


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            return _mapping(json.load(stream), path.name)
    except FileNotFoundError as error:
        raise ReplayArtifactError(f"required artifact not found: {path}") from error
    except json.JSONDecodeError as error:
        raise ReplayArtifactError(f"invalid JSON artifact: {path}: {error}") from error


def load_dynamic_run(run_dir: Path | str) -> DynamicRunConfig:
    resolved = Path(run_dir).expanduser().resolve()
    if not resolved.is_dir():
        raise ReplayArtifactError(f"run directory not found: {resolved}")

    manifest_path = resolved / "manifest.json"
    if manifest_path.is_file():
        manifest = _load_json(manifest_path)
        if manifest.get("state") != "completed" or manifest.get("exit_code") != 0:
            raise ReplayArtifactError(
                "dynamic replay requires a completed run with exit_code 0"
            )

    bag_dir = resolved / "rosbag"
    if not (bag_dir / "metadata.yaml").is_file():
        raise ReplayArtifactError(f"result bag metadata not found: {bag_dir}")

    summary = _load_json(resolved / "summary.json")
    map_summary = _mapping(summary.get("map"), "summary.map")
    frame_id = _normalize_frame_id(map_summary.get("frame_id"))
    if not frame_id:
        raise ReplayArtifactError("summary.map.frame_id is empty")

    try:
        voxel_size_m = float(map_summary.get("voxel_size_m"))
    except (TypeError, ValueError) as error:
        raise ReplayArtifactError(
            "summary.map.voxel_size_m must be a finite positive number"
        ) from error
    if not math.isfinite(voxel_size_m) or voxel_size_m <= 0.0:
        raise ReplayArtifactError(
            "summary.map.voxel_size_m must be a finite positive number"
        )

    bag_summary = _mapping(summary.get("bag"), "summary.bag")
    trajectory_artifacts = _mapping(
        summary.get("trajectory_artifacts"),
        "summary.trajectory_artifacts",
    )
    odometry_artifact = _mapping(
        trajectory_artifacts.get(ODOMETRY_TOPIC),
        f"summary.trajectory_artifacts.{ODOMETRY_TOPIC}",
    )
    odometry_frame_id = _normalize_frame_id(odometry_artifact.get("frame_id"))
    odometry_child_frame_id = _normalize_frame_id(
        odometry_artifact.get("child_frame_id")
    )
    if not odometry_frame_id or not odometry_child_frame_id:
        raise ReplayArtifactError(
            f"summary {ODOMETRY_TOPIC} trajectory must provide frame_id and child_frame_id"
        )
    topic_types = _mapping(
        bag_summary.get("available_topic_types"),
        "summary.bag.available_topic_types",
    )
    raw_counts = _mapping(
        bag_summary.get("topic_message_counts"),
        "summary.bag.topic_message_counts",
    )
    counts: dict[str, int] = {}
    for topic, expected_type in EXPECTED_TOPICS.items():
        if topic_types.get(topic) != expected_type:
            raise ReplayArtifactError(
                f"{topic} must have type {expected_type}, got {topic_types.get(topic)!r}"
            )
        try:
            count = int(raw_counts.get(topic, 0))
        except (TypeError, ValueError) as error:
            raise ReplayArtifactError(f"invalid message count for {topic}") from error
        if count <= 0:
            raise ReplayArtifactError(f"result bag contains no {topic} messages")
        counts[topic] = count

    return DynamicRunConfig(
        run_dir=resolved,
        bag_dir=bag_dir,
        frame_id=frame_id,
        odometry_frame_id=odometry_frame_id,
        odometry_child_frame_id=odometry_child_frame_id,
        voxel_size_m=voxel_size_m,
        topic_message_counts=counts,
    )


class IncrementalVoxelMap:
    """Keep the first finite point observed in each occupied voxel."""

    def __init__(self, voxel_size_m: float) -> None:
        if not math.isfinite(voxel_size_m) or voxel_size_m <= 0.0:
            raise ValueError("voxel_size_m must be a finite positive number")
        self.voxel_size_m = float(voxel_size_m)
        self._seen: set[tuple[int, int, int]] = set()
        self._pending: list[np.ndarray] = []
        self.frame_count = 0
        self.input_point_count = 0
        self.finite_point_count = 0
        self.published_point_count = 0

    @property
    def voxel_count(self) -> int:
        return len(self._seen)

    @property
    def pending_point_count(self) -> int:
        return sum(points.shape[0] for points in self._pending)

    def add(self, points_xyz: np.ndarray) -> int:
        points = np.asarray(points_xyz, dtype=np.float64).reshape((-1, 3))
        self.frame_count += 1
        self.input_point_count += int(points.shape[0])
        finite = points[np.all(np.isfinite(points), axis=1)]
        self.finite_point_count += int(finite.shape[0])
        if finite.size == 0:
            return 0

        keys = np.floor(finite / self.voxel_size_m).astype(np.int64)
        _, first_indexes = np.unique(keys, axis=0, return_index=True)
        first_indexes.sort()

        new_indexes: list[int] = []
        for index in first_indexes:
            key_array = keys[index]
            key = (int(key_array[0]), int(key_array[1]), int(key_array[2]))
            if key in self._seen:
                continue
            self._seen.add(key)
            new_indexes.append(int(index))

        if new_indexes:
            new_points = finite[np.asarray(new_indexes, dtype=np.int64)].astype(
                np.float32, copy=True
            )
            self._pending.append(new_points)
        return len(new_indexes)

    def drain_pending(self) -> np.ndarray:
        if not self._pending:
            return np.empty((0, 3), dtype=np.float32)
        points = np.concatenate(self._pending, axis=0)
        self._pending.clear()
        self.published_point_count += int(points.shape[0])
        return points


def run_dynamic_replay(
    config: DynamicRunConfig,
    *,
    map_topic: str,
    path_topic: str,
    cloud_topic: str,
    odometry_topic: str,
    update_period_s: float,
) -> int:
    try:
        import rclpy
        from builtin_interfaces.msg import Time
        from nav_msgs.msg import Odometry, Path as PathMessage
        from rclpy.clock import Clock, ClockType
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import PointCloud2, PointField
        from geometry_msgs.msg import PoseStamped
    except ImportError as error:
        raise RuntimeError(
            "Dynamic replay requires a sourced ROS 2 Humble environment"
        ) from error

    class GrowingArtifactPublisher(Node):
        def __init__(self) -> None:
            super().__init__("dlio_growing_artifact_publisher")
            source_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_ALL,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            path_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._map_publisher = self.create_publisher(
                PointCloud2, map_topic, map_qos
            )
            self._path_publisher = self.create_publisher(
                PathMessage, path_topic, path_qos
            )
            self._cloud_subscription = self.create_subscription(
                PointCloud2, cloud_topic, self._cloud_callback, source_qos
            )
            self._odometry_subscription = self.create_subscription(
                Odometry, odometry_topic, self._odometry_callback, source_qos
            )
            self._voxels = IncrementalVoxelMap(config.voxel_size_m)
            self._path = PathMessage()
            self._path.header.frame_id = config.odometry_frame_id
            self._path_dirty = False
            self._latest_cloud_stamp = Time()
            self._update_count = 0
            steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._update_timer = self.create_timer(
                update_period_s, self._publish_updates, clock=steady_clock
            )

        def _cloud_callback(self, message: PointCloud2) -> None:
            frame_id = _normalize_frame_id(message.header.frame_id)
            if frame_id != config.frame_id:
                raise RuntimeError(
                    f"{cloud_topic} frame changed: expected {config.frame_id}, "
                    f"got {frame_id or '<empty>'}"
                )
            self._voxels.add(pointcloud2_xyz(message))
            self._latest_cloud_stamp = copy.deepcopy(message.header.stamp)

        def _odometry_callback(self, message: Odometry) -> None:
            frame_id = _normalize_frame_id(message.header.frame_id)
            child_frame_id = _normalize_frame_id(message.child_frame_id)
            if frame_id != config.odometry_frame_id:
                raise RuntimeError(
                    f"{odometry_topic} frame changed: expected "
                    f"{config.odometry_frame_id}, "
                    f"got {frame_id or '<empty>'}"
                )
            if child_frame_id != config.odometry_child_frame_id:
                raise RuntimeError(
                    f"{odometry_topic} child frame changed: expected "
                    f"{config.odometry_child_frame_id}, "
                    f"got {child_frame_id or '<empty>'}"
                )
            pose = PoseStamped()
            pose.header = copy.deepcopy(message.header)
            pose.header.frame_id = config.odometry_frame_id
            pose.pose = copy.deepcopy(message.pose.pose)
            self._path.poses.append(pose)
            self._path.header.stamp = copy.deepcopy(message.header.stamp)
            self._path_dirty = True

        def _cloud_message(self, points: np.ndarray) -> PointCloud2:
            packed = np.asarray(points, dtype="<f4").reshape((-1, 3))
            cloud = PointCloud2()
            cloud.header.stamp = copy.deepcopy(self._latest_cloud_stamp)
            cloud.header.frame_id = config.frame_id
            cloud.height = 1
            cloud.width = int(packed.shape[0])
            cloud.fields = [
                PointField(
                    name="x", offset=0, datatype=PointField.FLOAT32, count=1
                ),
                PointField(
                    name="y", offset=4, datatype=PointField.FLOAT32, count=1
                ),
                PointField(
                    name="z", offset=8, datatype=PointField.FLOAT32, count=1
                ),
            ]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = cloud.point_step * cloud.width
            cloud.data = packed.tobytes()
            cloud.is_dense = True
            return cloud

        def _publish_updates(self) -> None:
            published = False
            new_points = self._voxels.drain_pending()
            if new_points.size:
                self._map_publisher.publish(self._cloud_message(new_points))
                published = True
            if self._path_dirty:
                self._path_publisher.publish(self._path)
                self._path_dirty = False
                published = True
            if not published:
                return
            self._update_count += 1
            if self._update_count == 1 or self._update_count % 30 == 0:
                self.get_logger().info(
                    f"Growing replay: {self._voxels.frame_count} scans, "
                    f"{self._voxels.voxel_count} map voxels, "
                    f"{len(self._path.poses)} trajectory poses"
                )

    rclpy.init(args=None)
    node = GrowingArtifactPublisher()
    node.get_logger().info(
        f"Waiting for saved {cloud_topic} and {odometry_topic}; "
        f"voxel_size={config.voxel_size_m:g} m, frame={config.frame_id}"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="analyzed offline run directory")
    parser.add_argument("--map-topic", default="/offline/map")
    parser.add_argument("--path-topic", default="/offline/path")
    parser.add_argument("--cloud-topic", default=CLOUD_TOPIC)
    parser.add_argument("--odometry-topic", default=ODOMETRY_TOPIC)
    parser.add_argument(
        "--update-period",
        type=float,
        default=1.0,
        help="wall-clock seconds between growing map/path updates (default: 1.0)",
    )
    parser.add_argument(
        "--print-frame-id",
        action="store_true",
        help="validate the dynamic result bag and print its map frame",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if not math.isfinite(args.update_period) or args.update_period <= 0.0:
            raise ReplayArtifactError("--update-period must be finite and positive")
        config = load_dynamic_run(args.run_dir)
        if args.print_frame_id:
            print(config.frame_id)
            return 0
        print(
            f"Validated dynamic run {config.run_dir}: "
            f"clouds={config.topic_message_counts[CLOUD_TOPIC]}, "
            f"poses={config.topic_message_counts[ODOMETRY_TOPIC]}, "
            f"voxel_size={config.voxel_size_m:g} m, frame={config.frame_id}"
        )
        return run_dynamic_replay(
            config,
            map_topic=args.map_topic,
            path_topic=args.path_topic,
            cloud_topic=args.cloud_topic,
            odometry_topic=args.odometry_topic,
            update_period_s=args.update_period,
        )
    except (OSError, ReplayArtifactError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
