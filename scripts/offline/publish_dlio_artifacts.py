#!/usr/bin/env python3
"""Publish frozen D-LIO map and trajectory artifacts for RViz.

The publisher deliberately consumes only post-processed run artifacts.  It
does not replay the source sensors or run D-LIO again.  The map and path are
published with transient-local durability so RViz may start after this node.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO

import numpy as np


DLIO_ODOMETRY_TOPIC = "/dlio/odom_node/odom"


class ArtifactError(ValueError):
    """Raised when an offline run artifact is missing or malformed."""


@dataclass(frozen=True)
class PcdData:
    xyz: np.ndarray
    counts: np.ndarray
    data_format: str


@dataclass(frozen=True)
class TrajectoryData:
    positions: np.ndarray
    orientations_xyzw: np.ndarray
    frame_id: str
    child_frame_id: str


@dataclass(frozen=True)
class ArtifactBundle:
    run_dir: Path
    summary_path: Path
    map_path: Path
    trajectory_path: Path
    map_frame_id: str
    pcd: PcdData
    trajectory: TrajectoryData


def _normalize_frame_id(value: Any) -> str:
    return str(value or "").strip().lstrip("/")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validated_path(run_dir: Path, value: Any, description: str) -> Path:
    text = str(value or "").strip()
    if not text:
        raise ArtifactError(f"{description} path is missing from summary.json")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = run_dir / path
    path = path.resolve()
    if not path.is_file():
        raise ArtifactError(f"{description} does not exist: {path}")
    return path


def _validate_hash(path: Path, expected: Any, description: str) -> None:
    expected_text = str(expected or "").strip().lower()
    if not expected_text:
        return
    if len(expected_text) != 64 or any(
        character not in "0123456789abcdef" for character in expected_text
    ):
        raise ArtifactError(f"invalid SHA-256 for {description}: {expected}")
    actual = sha256_file(path)
    if actual != expected_text:
        raise ArtifactError(
            f"SHA-256 mismatch for {description}: expected {expected_text}, got {actual}"
        )


def _pcd_scalar_dtype(type_name: str, size: int) -> np.dtype[Any]:
    key = (type_name.upper(), int(size))
    scalar_types = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    try:
        return np.dtype(scalar_types[key])
    except KeyError as error:
        raise ArtifactError(
            f"unsupported PCD scalar TYPE/SIZE combination: {type_name}/{size}"
        ) from error


def _read_pcd_header(stream: BinaryIO) -> tuple[dict[str, list[str]], str]:
    header: dict[str, list[str]] = {}
    for _ in range(256):
        raw_line = stream.readline()
        if not raw_line:
            raise ArtifactError("PCD header ended before a DATA declaration")
        try:
            line = raw_line.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise ArtifactError("PCD header is not ASCII") from error
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        values = parts[1:]
        if key in header:
            raise ArtifactError(f"duplicate PCD header field: {key}")
        header[key] = values
        if key == "DATA":
            if len(values) != 1:
                raise ArtifactError("PCD DATA must have exactly one value")
            return header, values[0].lower()
    raise ArtifactError("PCD header is unreasonably long")


def _required_header_values(
    header: Mapping[str, list[str]], key: str
) -> list[str]:
    values = header.get(key)
    if not values:
        raise ArtifactError(f"PCD header is missing {key}")
    return values


def load_pcd(path: Path) -> PcdData:
    """Load an ASCII or uncompressed-binary PCD containing x/y/z/count."""

    with path.open("rb") as stream:
        header, data_format = _read_pcd_header(stream)
        fields = _required_header_values(header, "FIELDS")
        if len(fields) != len(set(fields)):
            raise ArtifactError("PCD FIELDS contains duplicate names")
        sizes = [int(value) for value in _required_header_values(header, "SIZE")]
        types = _required_header_values(header, "TYPE")
        raw_counts = header.get("COUNT") or ["1"] * len(fields)
        field_counts = [int(value) for value in raw_counts]
        if not (
            len(fields) == len(sizes) == len(types) == len(field_counts)
        ):
            raise ArtifactError("PCD FIELDS/SIZE/TYPE/COUNT lengths differ")
        if any(size <= 0 for size in sizes) or any(count <= 0 for count in field_counts):
            raise ArtifactError("PCD SIZE and COUNT values must be positive")
        for required in ("x", "y", "z", "count"):
            if required not in fields:
                raise ArtifactError(f"PCD is missing required field '{required}'")

        try:
            width = int(_required_header_values(header, "WIDTH")[0])
            height = int(_required_header_values(header, "HEIGHT")[0])
            points = int(_required_header_values(header, "POINTS")[0])
        except (IndexError, ValueError) as error:
            raise ArtifactError("PCD WIDTH/HEIGHT/POINTS must be integers") from error
        if width < 0 or height < 0 or points < 0 or width * height != points:
            raise ArtifactError("PCD WIDTH * HEIGHT must equal non-negative POINTS")

        scalar_dtypes = [
            _pcd_scalar_dtype(type_name, size)
            for type_name, size in zip(types, sizes)
        ]
        for required in ("x", "y", "z", "count"):
            if field_counts[fields.index(required)] != 1:
                raise ArtifactError(f"PCD field '{required}' must be scalar")

        if data_format == "binary":
            dtype_fields: list[tuple[Any, ...]] = []
            for name, scalar_dtype, field_count in zip(
                fields, scalar_dtypes, field_counts
            ):
                if field_count == 1:
                    dtype_fields.append((name, scalar_dtype))
                else:
                    dtype_fields.append((name, scalar_dtype, (field_count,)))
            dtype = np.dtype(dtype_fields, align=False)
            payload = stream.read()
            expected_bytes = points * dtype.itemsize
            if len(payload) != expected_bytes:
                raise ArtifactError(
                    "binary PCD payload size differs from POINTS * point_step: "
                    f"expected {expected_bytes}, got {len(payload)}"
                )
            records = np.frombuffer(payload, dtype=dtype, count=points)
            columns = {name: records[name] for name in fields}
        elif data_format == "ascii":
            try:
                payload_text = stream.read().decode("ascii")
            except UnicodeDecodeError as error:
                raise ArtifactError("ASCII PCD payload is not ASCII") from error
            values_per_point = sum(field_counts)
            values = np.fromstring(payload_text, dtype=np.float64, sep=" ")
            if values.size != points * values_per_point:
                raise ArtifactError(
                    "ASCII PCD value count differs from POINTS and COUNT fields"
                )
            matrix = values.reshape((points, values_per_point))
            columns: dict[str, np.ndarray] = {}
            offset = 0
            for name, field_count in zip(fields, field_counts):
                field_values = matrix[:, offset:offset + field_count]
                columns[name] = field_values[:, 0] if field_count == 1 else field_values
                offset += field_count
        else:
            raise ArtifactError(
                f"unsupported PCD DATA format '{data_format}'; use ascii or binary"
            )

    xyz = np.column_stack((columns["x"], columns["y"], columns["z"])).astype(
        np.float32, copy=False
    )
    raw_point_counts = np.asarray(columns["count"], dtype=np.float64)
    if xyz.shape != (points, 3) or raw_point_counts.shape != (points,):
        raise ArtifactError("PCD coordinate/count fields have unexpected shapes")
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(raw_point_counts)):
        raise ArtifactError("PCD contains non-finite coordinates or counts")
    rounded_counts = np.rint(raw_point_counts)
    if (
        np.any(raw_point_counts < 0.0)
        or np.any(raw_point_counts > np.iinfo(np.uint32).max)
        or not np.allclose(raw_point_counts, rounded_counts, rtol=0.0, atol=1.0e-6)
    ):
        raise ArtifactError("PCD count values must be uint32-compatible integers")
    return PcdData(
        xyz=np.ascontiguousarray(xyz),
        counts=np.ascontiguousarray(rounded_counts.astype(np.uint32)),
        data_format=data_format,
    )


def load_trajectory(
    path: Path,
    *,
    expected_frame_id: str = "",
    expected_child_frame_id: str = "",
) -> TrajectoryData:
    positions: list[tuple[float, float, float]] = []
    orientations: list[tuple[float, float, float, float]] = []
    frame_ids: set[str] = set()
    child_frame_ids: set[str] = set()
    required_columns = ("x", "y", "z", "qx", "qy", "qz", "qw")

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ArtifactError("trajectory CSV has no header")
        missing = [column for column in required_columns if column not in reader.fieldnames]
        if missing:
            raise ArtifactError(
                "trajectory CSV is missing columns: " + ", ".join(missing)
            )
        for line_number, row in enumerate(reader, start=2):
            try:
                position = tuple(float(row[column]) for column in required_columns[:3])
                quaternion = np.asarray(
                    [float(row[column]) for column in required_columns[3:]],
                    dtype=np.float64,
                )
            except (TypeError, ValueError) as error:
                raise ArtifactError(
                    f"trajectory CSV has a non-numeric pose at line {line_number}"
                ) from error
            if not all(math.isfinite(value) for value in position) or not np.all(
                np.isfinite(quaternion)
            ):
                raise ArtifactError(
                    f"trajectory CSV has a non-finite pose at line {line_number}"
                )
            norm = float(np.linalg.norm(quaternion))
            if norm <= 1.0e-12:
                raise ArtifactError(
                    f"trajectory CSV has a zero quaternion at line {line_number}"
                )
            quaternion /= norm
            positions.append(position)
            orientations.append(tuple(float(value) for value in quaternion))

            row_frame = _normalize_frame_id(row.get("frame_id"))
            row_child = _normalize_frame_id(row.get("child_frame_id"))
            if row_frame:
                frame_ids.add(row_frame)
            if row_child:
                child_frame_ids.add(row_child)

    if not positions:
        raise ArtifactError("trajectory CSV contains no poses")
    if len(frame_ids) > 1:
        raise ArtifactError(
            "trajectory CSV contains multiple frame IDs: " + ", ".join(sorted(frame_ids))
        )
    if len(child_frame_ids) > 1:
        raise ArtifactError(
            "trajectory CSV contains multiple child frame IDs: "
            + ", ".join(sorted(child_frame_ids))
        )

    expected_frame = _normalize_frame_id(expected_frame_id)
    expected_child = _normalize_frame_id(expected_child_frame_id)
    csv_frame = next(iter(frame_ids), "")
    csv_child = next(iter(child_frame_ids), "")
    if csv_frame and expected_frame and csv_frame != expected_frame:
        raise ArtifactError(
            f"trajectory frame mismatch: summary={expected_frame}, CSV={csv_frame}"
        )
    if csv_child and expected_child and csv_child != expected_child:
        raise ArtifactError(
            f"trajectory child frame mismatch: summary={expected_child}, CSV={csv_child}"
        )
    frame_id = csv_frame or expected_frame
    child_frame_id = csv_child or expected_child
    if not frame_id:
        raise ArtifactError("trajectory frame ID is absent from summary and CSV")

    return TrajectoryData(
        positions=np.asarray(positions, dtype=np.float64),
        orientations_xyzw=np.asarray(orientations, dtype=np.float64),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
    )


def _mapping_or_empty(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def load_artifact_bundle(
    run_dir: Path,
    *,
    trajectory_topic: str = DLIO_ODOMETRY_TOPIC,
    require_matching_frames: bool = True,
) -> ArtifactBundle:
    run_dir = run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        raise ArtifactError(f"run directory does not exist: {run_dir}")
    summary_path = run_dir / "summary.json"
    if not summary_path.is_file():
        raise ArtifactError(f"summary.json does not exist: {summary_path}")
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ArtifactError(f"invalid summary.json: {error}") from error
    if not isinstance(summary, Mapping):
        raise ArtifactError("summary.json root must be an object")

    map_summary = _mapping_or_empty(summary.get("map"))
    map_frame_id = _normalize_frame_id(map_summary.get("frame_id"))
    if not map_frame_id:
        raise ArtifactError("summary.json map.frame_id is missing")
    preview = _mapping_or_empty(map_summary.get("preview"))
    if preview.get("path"):
        map_path = _validated_path(run_dir, preview.get("path"), "preview PCD")
        map_hash = preview.get("sha256") or preview.get("pcd_sha256")
        expected_point_count = preview.get("point_count")
    else:
        map_path = _validated_path(run_dir, map_summary.get("pcd_path"), "map PCD")
        map_hash = (
            map_summary.get("pcd_sha256")
            or map_summary.get("sha256")
            or map_summary.get("preview_sha256")
        )
        expected_point_count = map_summary.get("voxel_count")
    _validate_hash(map_path, map_hash, "map PCD")
    pcd = load_pcd(map_path)
    if expected_point_count is not None:
        try:
            expected_count_int = int(expected_point_count)
        except (TypeError, ValueError) as error:
            raise ArtifactError("map point count in summary is not an integer") from error
        if expected_count_int != pcd.xyz.shape[0]:
            raise ArtifactError(
                "map point count mismatch: "
                f"summary={expected_count_int}, PCD={pcd.xyz.shape[0]}"
            )

    trajectory_artifacts = _mapping_or_empty(summary.get("trajectory_artifacts"))
    raw_trajectory = _mapping_or_empty(trajectory_artifacts.get(trajectory_topic))
    if raw_trajectory:
        trajectory_path = _validated_path(
            run_dir,
            raw_trajectory.get("path"),
            f"{trajectory_topic} trajectory CSV",
        )
        trajectory_frame = _normalize_frame_id(raw_trajectory.get("frame_id"))
        trajectory_child = _normalize_frame_id(raw_trajectory.get("child_frame_id"))
        trajectory_hash = raw_trajectory.get("sha256")
    else:
        if trajectory_topic != DLIO_ODOMETRY_TOPIC:
            raise ArtifactError(
                f"summary.json has no trajectory artifact for {trajectory_topic}"
            )
        trajectory_path = _validated_path(
            run_dir, "trajectory.csv", "D-LIO trajectory CSV"
        )
        trajectory_frame = map_frame_id
        trajectory_child = ""
        trajectory_hash = ""
    _validate_hash(
        trajectory_path, trajectory_hash, f"{trajectory_topic} trajectory CSV"
    )
    trajectory = load_trajectory(
        trajectory_path,
        expected_frame_id=trajectory_frame,
        expected_child_frame_id=trajectory_child,
    )
    if require_matching_frames and trajectory.frame_id != map_frame_id:
        raise ArtifactError(
            "map and trajectory frames differ: "
            f"map={map_frame_id}, trajectory={trajectory.frame_id}"
        )

    return ArtifactBundle(
        run_dir=run_dir,
        summary_path=summary_path,
        map_path=map_path,
        trajectory_path=trajectory_path,
        map_frame_id=map_frame_id,
        pcd=pcd,
        trajectory=trajectory,
    )


def _publish_bundle(bundle: ArtifactBundle, map_topic: str, path_topic: str) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as PathMessage
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import PointCloud2, PointField
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python packages are unavailable; source ROS 2 Humble first"
        ) from error

    class ArtifactPublisher(Node):
        def __init__(self) -> None:
            super().__init__("dlio_artifact_publisher")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._map_publisher = self.create_publisher(PointCloud2, map_topic, qos)
            self._path_publisher = self.create_publisher(PathMessage, path_topic, qos)
            self._published = False
            self._timer = self.create_timer(0.1, self._publish_once)

        def _publish_once(self) -> None:
            if self._published:
                return
            stamp = self.get_clock().now().to_msg()

            packed = np.empty(
                bundle.pcd.xyz.shape[0],
                dtype=np.dtype(
                    [
                        ("x", "<f4"),
                        ("y", "<f4"),
                        ("z", "<f4"),
                        ("count", "<u4"),
                    ]
                ),
            )
            packed["x"] = bundle.pcd.xyz[:, 0]
            packed["y"] = bundle.pcd.xyz[:, 1]
            packed["z"] = bundle.pcd.xyz[:, 2]
            packed["count"] = bundle.pcd.counts
            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = bundle.map_frame_id
            cloud.height = 1
            cloud.width = int(packed.shape[0])
            cloud.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="count", offset=12, datatype=PointField.UINT32, count=1),
            ]
            cloud.is_bigendian = False
            cloud.point_step = 16
            cloud.row_step = cloud.point_step * cloud.width
            cloud.data = packed.tobytes()
            cloud.is_dense = True

            path_message = PathMessage()
            path_message.header.stamp = stamp
            path_message.header.frame_id = bundle.trajectory.frame_id
            for position, orientation in zip(
                bundle.trajectory.positions,
                bundle.trajectory.orientations_xyzw,
            ):
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = bundle.trajectory.frame_id
                pose.pose.position.x = float(position[0])
                pose.pose.position.y = float(position[1])
                pose.pose.position.z = float(position[2])
                pose.pose.orientation.x = float(orientation[0])
                pose.pose.orientation.y = float(orientation[1])
                pose.pose.orientation.z = float(orientation[2])
                pose.pose.orientation.w = float(orientation[3])
                path_message.poses.append(pose)

            self._map_publisher.publish(cloud)
            self._path_publisher.publish(path_message)
            self._published = True
            self._timer.cancel()
            self.get_logger().info(
                f"Published {cloud.width} map points on {map_topic} and "
                f"{len(path_message.poses)} poses on {path_topic}; "
                f"map frame={bundle.map_frame_id}, "
                f"path frame={bundle.trajectory.frame_id}; "
                "keeping transient-local data alive"
            )

    rclpy.init(args=None)
    node = ArtifactPublisher()
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
    parser.add_argument(
        "--trajectory-topic",
        default=DLIO_ODOMETRY_TOPIC,
        help=f"saved trajectory to publish (default: {DLIO_ODOMETRY_TOPIC})",
    )
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument(
        "--validate-only",
        action="store_true",
        help="validate and summarize artifacts without starting a ROS node",
    )
    output_mode.add_argument(
        "--print-frame-id",
        action="store_true",
        help="validate artifacts and print only the normalized map frame ID",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        bundle = load_artifact_bundle(
            args.run_dir,
            trajectory_topic=args.trajectory_topic,
            require_matching_frames=True,
        )
        if args.print_frame_id:
            print(bundle.map_frame_id)
            return 0
        print(
            f"Validated run {bundle.run_dir}: "
            f"map={bundle.map_path.name} ({bundle.pcd.xyz.shape[0]} points, "
            f"{bundle.pcd.data_format}), trajectory={bundle.trajectory_path.name} "
            f"({bundle.trajectory.positions.shape[0]} poses), "
            f"map_frame={bundle.map_frame_id}, "
            f"trajectory_frame={bundle.trajectory.frame_id}"
        )
        if args.validate_only:
            return 0
        return _publish_bundle(bundle, args.map_topic, args.path_topic)
    except (ArtifactError, OSError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
