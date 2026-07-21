#!/usr/bin/env python3
"""Analyze offline D-LIO result bags and compare experiment runs.

The ROS imports are deliberately lazy so the numerical helpers and comparison
mode remain usable in a normal Python environment. Bag analysis must run in a
sourced ROS 2 Humble environment with rosbag2_py available.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


DLIO_ODOMETRY_TOPIC = "/dlio/odom_node/odom"
ODOMETRY_TOPICS = (DLIO_ODOMETRY_TOPIC,)
CLOUD_TOPIC = "/dlio/odom_node/pointcloud/deskewed"
DIAGNOSTIC_ARRAY_TYPE = "diagnostic_msgs/msg/DiagnosticArray"


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    bag_stamp_ns: int
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    frame_id: str = ""
    child_frame_id: str = ""


class FrameIdTracker:
    """Track one optional frame ID and reject ambiguous mixed-frame data."""

    def __init__(self, label: str) -> None:
        self.label = label
        self._frame_id = ""

    @property
    def frame_id(self) -> str:
        return self._frame_id

    def add(self, frame_id: Any) -> None:
        normalized = str(frame_id or "")
        if not normalized:
            return
        if not self._frame_id:
            self._frame_id = normalized
            return
        if normalized != self._frame_id:
            raise ValueError(
                f"{self.label} contains mixed nonempty frame IDs: "
                f"{self._frame_id!r} and {normalized!r}"
            )


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact_record(path: Path) -> dict[str, Any]:
    return {
        "path": path.name,
        "sha256": _sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def _canonical_json_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return _sha256_bytes(payload)


def _distribution(values: Sequence[float] | np.ndarray) -> dict[str, float | int | None]:
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95.0)),
        "max": float(np.max(array)),
    }


def _normalized_quaternions(quaternions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    norms = np.linalg.norm(quaternions, axis=1)
    valid = np.all(np.isfinite(quaternions), axis=1) & (norms > 1.0e-12)
    result = np.full_like(quaternions, np.nan, dtype=np.float64)
    result[valid] = quaternions[valid] / norms[valid, None]
    return result, valid


def trajectory_metrics(
    samples: Sequence[PoseSample],
    *,
    gap_threshold_s: float,
    translation_jump_threshold_m: float,
    orientation_jump_threshold_deg: float,
) -> dict[str, Any]:
    """Compute continuity and motion metrics without assuming ground truth."""
    if not samples:
        return {
            "sample_count": 0,
            "finite_sample_count": 0,
            "nonfinite_sample_count": 0,
            "duration_s": 0.0,
            "rate_hz": None,
            "non_monotonic_interval_count": 0,
            "gap_threshold_s": gap_threshold_s,
            "gap_count": 0,
            "gaps_s": _distribution([]),
            "path_length_m": 0.0,
            "terminal_displacement_m": 0.0,
            "translation_jump_threshold_m": translation_jump_threshold_m,
            "translation_jump_count": 0,
            "translation_steps_m": _distribution([]),
            "translation_jump_first_elapsed_s": None,
            "translation_max_step_elapsed_s": None,
            "orientation_jump_threshold_deg": orientation_jump_threshold_deg,
            "orientation_jump_count": 0,
            "orientation_steps_deg": _distribution([]),
            "orientation_jump_first_elapsed_s": None,
            "orientation_max_step_elapsed_s": None,
        }

    stamps_ns = np.asarray([sample.stamp_ns for sample in samples], dtype=np.int64)
    positions = np.asarray([sample.position for sample in samples], dtype=np.float64)
    quaternions = np.asarray(
        [sample.orientation_xyzw for sample in samples], dtype=np.float64
    )
    normalized_quaternions, quaternion_valid = _normalized_quaternions(quaternions)
    finite = np.all(np.isfinite(positions), axis=1) & quaternion_valid

    intervals_s = np.diff(stamps_ns).astype(np.float64) * 1.0e-9
    positive_intervals = intervals_s[intervals_s > 0.0]
    duration_s = float((np.max(stamps_ns) - np.min(stamps_ns)) * 1.0e-9)

    valid_positions = positions[finite]
    valid_quaternions = normalized_quaternions[finite]
    valid_stamps_ns = stamps_ns[finite]
    if valid_positions.shape[0] >= 2:
        translation_steps = np.linalg.norm(np.diff(valid_positions, axis=0), axis=1)
        step_elapsed_s = (
            valid_stamps_ns[1:] - stamps_ns[0]
        ).astype(np.float64) * 1.0e-9

        quaternion_dots = np.sum(valid_quaternions[:-1] * valid_quaternions[1:], axis=1)
        quaternion_dots = np.clip(np.abs(quaternion_dots), 0.0, 1.0)
        orientation_steps_deg = np.degrees(2.0 * np.arccos(quaternion_dots))
        path_length_m = float(np.sum(translation_steps))
        terminal_displacement_m = float(
            np.linalg.norm(valid_positions[-1] - valid_positions[0])
        )
    else:
        translation_steps = np.asarray([], dtype=np.float64)
        orientation_steps_deg = np.asarray([], dtype=np.float64)
        step_elapsed_s = np.asarray([], dtype=np.float64)
        path_length_m = 0.0
        terminal_displacement_m = 0.0

    gaps = positive_intervals[positive_intervals > gap_threshold_s]
    translation_jump_indices = np.flatnonzero(
        translation_steps > translation_jump_threshold_m
    )
    orientation_jump_indices = np.flatnonzero(
        orientation_steps_deg > orientation_jump_threshold_deg
    )
    return {
        "sample_count": len(samples),
        "finite_sample_count": int(np.count_nonzero(finite)),
        "nonfinite_sample_count": int(np.count_nonzero(~finite)),
        "duration_s": duration_s,
        "rate_hz": (
            float((len(samples) - 1) / duration_s)
            if len(samples) >= 2 and duration_s > 0.0
            else None
        ),
        "non_monotonic_interval_count": int(np.count_nonzero(intervals_s <= 0.0)),
        "gap_threshold_s": gap_threshold_s,
        "gap_count": int(gaps.size),
        "gaps_s": _distribution(gaps),
        "intervals_s": _distribution(positive_intervals),
        "path_length_m": path_length_m,
        "terminal_displacement_m": terminal_displacement_m,
        "translation_jump_threshold_m": translation_jump_threshold_m,
        "translation_jump_count": int(translation_jump_indices.size),
        "translation_jump_first_elapsed_s": (
            float(step_elapsed_s[translation_jump_indices[0]])
            if translation_jump_indices.size
            else None
        ),
        "translation_max_step_elapsed_s": (
            float(step_elapsed_s[int(np.argmax(translation_steps))])
            if translation_steps.size
            else None
        ),
        "translation_steps_m": _distribution(translation_steps),
        "orientation_jump_threshold_deg": orientation_jump_threshold_deg,
        "orientation_jump_count": int(orientation_jump_indices.size),
        "orientation_jump_first_elapsed_s": (
            float(step_elapsed_s[orientation_jump_indices[0]])
            if orientation_jump_indices.size
            else None
        ),
        "orientation_max_step_elapsed_s": (
            float(step_elapsed_s[int(np.argmax(orientation_steps_deg))])
            if orientation_steps_deg.size
            else None
        ),
        "orientation_steps_deg": _distribution(orientation_steps_deg),
    }


class VoxelAccumulator:
    """Streaming voxel moments for map export and local plane statistics."""

    # count, x, y, z, xx, xy, xz, yy, yz, zz
    MOMENT_COUNT = 10

    def __init__(self, voxel_size_m: float, chunk_points: int = 500_000) -> None:
        if voxel_size_m <= 0.0:
            raise ValueError("voxel_size_m must be positive")
        if chunk_points <= 0:
            raise ValueError("chunk_points must be positive")
        self.voxel_size_m = float(voxel_size_m)
        self.chunk_points = int(chunk_points)
        self._voxels: dict[tuple[int, int, int], np.ndarray] = {}
        self._pending: list[np.ndarray] = []
        self._pending_count = 0
        self.input_point_count = 0
        self.finite_point_count = 0
        self.frame_count = 0
        self.bounds_min = np.full(3, np.inf, dtype=np.float64)
        self.bounds_max = np.full(3, -np.inf, dtype=np.float64)

    def add(self, points_xyz: np.ndarray) -> None:
        points = np.asarray(points_xyz, dtype=np.float64).reshape((-1, 3))
        self.frame_count += 1
        self.input_point_count += int(points.shape[0])
        finite_points = points[np.all(np.isfinite(points), axis=1)]
        self.finite_point_count += int(finite_points.shape[0])
        if finite_points.size == 0:
            return
        self.bounds_min = np.minimum(self.bounds_min, np.min(finite_points, axis=0))
        self.bounds_max = np.maximum(self.bounds_max, np.max(finite_points, axis=0))
        self._pending.append(finite_points)
        self._pending_count += int(finite_points.shape[0])
        if self._pending_count >= self.chunk_points:
            self._flush()

    def _flush(self) -> None:
        if not self._pending:
            return
        points = np.concatenate(self._pending, axis=0)
        self._pending.clear()
        self._pending_count = 0
        keys = np.floor(points / self.voxel_size_m).astype(np.int64)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        counts = np.bincount(inverse).astype(np.float64)
        columns = [np.bincount(inverse, weights=points[:, i]) for i in range(3)]
        columns.extend(
            np.bincount(inverse, weights=points[:, i] * points[:, j])
            for i, j in ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))
        )
        moments = np.column_stack((counts, *columns))
        for key_array, values in zip(unique_keys, moments):
            key = (int(key_array[0]), int(key_array[1]), int(key_array[2]))
            current = self._voxels.get(key)
            if current is None:
                self._voxels[key] = values
            else:
                current += values

    @property
    def voxels(self) -> Mapping[tuple[int, int, int], np.ndarray]:
        self._flush()
        return self._voxels

    def sorted_centroids(self) -> tuple[np.ndarray, np.ndarray, list[tuple[int, int, int]]]:
        self._flush()
        keys = sorted(self._voxels)
        if not keys:
            return (
                np.empty((0, 3), dtype=np.float64),
                np.empty((0,), dtype=np.uint32),
                [],
            )
        counts = np.asarray([self._voxels[key][0] for key in keys], dtype=np.float64)
        centroids = np.asarray(
            [self._voxels[key][1:4] / self._voxels[key][0] for key in keys],
            dtype=np.float64,
        )
        return centroids, counts.astype(np.uint32), keys

    def local_plane_metrics(
        self,
        *,
        radius_m: float,
        min_points: int,
        max_samples: int,
        random_seed: int = 7,
    ) -> dict[str, Any]:
        self._flush()
        if radius_m <= 0.0:
            raise ValueError("radius_m must be positive")
        if min_points < 3:
            raise ValueError("min_points must be at least 3")
        keys = sorted(self._voxels)
        if not keys:
            return {
                "radius_m": radius_m,
                "minimum_points": min_points,
                "maximum_sample_count": max_samples,
                "random_seed": random_seed,
                "sampled_voxel_count": 0,
                "sampled_voxel_key_sha256": _canonical_json_sha256([]),
                "valid_neighborhood_count": 0,
                "plane_thickness_m": _distribution([]),
                "planarity": _distribution([]),
            }
        if max_samples > 0 and len(keys) > max_samples:
            rng = np.random.default_rng(random_seed)
            indexes = np.sort(rng.choice(len(keys), size=max_samples, replace=False))
            sample_keys = [keys[index] for index in indexes]
        else:
            sample_keys = keys

        voxel_radius = int(math.ceil(radius_m / self.voxel_size_m))
        offsets = [
            (dx, dy, dz)
            for dx in range(-voxel_radius, voxel_radius + 1)
            for dy in range(-voxel_radius, voxel_radius + 1)
            for dz in range(-voxel_radius, voxel_radius + 1)
        ]
        thicknesses: list[float] = []
        planarities: list[float] = []
        for key in sample_keys:
            center_values = self._voxels[key]
            center = center_values[1:4] / center_values[0]
            total = np.zeros(self.MOMENT_COUNT, dtype=np.float64)
            for dx, dy, dz in offsets:
                neighbor = self._voxels.get((key[0] + dx, key[1] + dy, key[2] + dz))
                if neighbor is None:
                    continue
                neighbor_center = neighbor[1:4] / neighbor[0]
                if np.linalg.norm(neighbor_center - center) <= radius_m:
                    total += neighbor
            count = total[0]
            if count < min_points:
                continue
            mean = total[1:4] / count
            second = np.asarray(
                [
                    [total[4], total[5], total[6]],
                    [total[5], total[7], total[8]],
                    [total[6], total[8], total[9]],
                ],
                dtype=np.float64,
            ) / count
            covariance = second - np.outer(mean, mean)
            eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0.0)
            largest = eigenvalues[2]
            if not np.all(np.isfinite(eigenvalues)) or largest <= 1.0e-15:
                continue
            thicknesses.append(float(math.sqrt(eigenvalues[0])))
            planarities.append(float((eigenvalues[1] - eigenvalues[0]) / largest))

        return {
            "radius_m": radius_m,
            "minimum_points": min_points,
            "maximum_sample_count": max_samples,
            "random_seed": random_seed,
            "sampled_voxel_count": len(sample_keys),
            "sampled_voxel_key_sha256": _canonical_json_sha256(sample_keys),
            "valid_neighborhood_count": len(thicknesses),
            "plane_thickness_m": _distribution(thicknesses),
            "planarity": _distribution(planarities),
            "definition": {
                "plane_thickness_m": "sqrt(smallest local covariance eigenvalue)",
                "planarity": "(middle - smallest eigenvalue) / largest eigenvalue",
            },
        }

    def summary(self, local_planes: Mapping[str, Any]) -> dict[str, Any]:
        centroids, _, keys = self.sorted_centroids()
        if centroids.size:
            bounds_min: list[float] | None = self.bounds_min.tolist()
            bounds_max: list[float] | None = self.bounds_max.tolist()
            extents = self.bounds_max - self.bounds_min
            bounds_extent: list[float] | None = extents.tolist()
            bounding_box_volume: float | None = float(np.prod(extents))
            xy_keys = {(key[0], key[1]) for key in keys}
        else:
            bounds_min = None
            bounds_max = None
            bounds_extent = None
            bounding_box_volume = None
            xy_keys = set()
        return {
            "frame_count": self.frame_count,
            "input_point_count": self.input_point_count,
            "finite_point_count": self.finite_point_count,
            "voxel_size_m": self.voxel_size_m,
            "voxel_count": len(keys),
            "occupied_volume_m3": len(keys) * self.voxel_size_m**3,
            "xy_occupied_cell_count": len(xy_keys),
            "xy_coverage_m2": len(xy_keys) * self.voxel_size_m**2,
            "bounds_min_m": bounds_min,
            "bounds_max_m": bounds_max,
            "bounds_extent_m": bounds_extent,
            "bounding_box_volume_m3": bounding_box_volume,
            "local_planes": dict(local_planes),
        }


def write_pcd(
    path: Path,
    centroids: np.ndarray,
    counts: np.ndarray,
    data_format: str,
) -> None:
    if data_format not in {"ascii", "binary"}:
        raise ValueError("PCD data format must be 'ascii' or 'binary'")
    point_count = int(centroids.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z count\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F U\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {point_count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {point_count}\n"
        f"DATA {data_format}\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    if data_format == "ascii":
        with path.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(header)
            for point, count in zip(centroids, counts):
                stream.write(
                    f"{point[0]:.9g} {point[1]:.9g} {point[2]:.9g} {int(count)}\n"
                )
        return
    payload = np.empty(
        point_count,
        dtype=np.dtype(
            [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("count", "<u4")]
        ),
    )
    payload["x"] = centroids[:, 0]
    payload["y"] = centroids[:, 1]
    payload["z"] = centroids[:, 2]
    payload["count"] = counts
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(payload.tobytes())


def pointcloud2_xyz(message: Any) -> np.ndarray:
    """Extract XYZ from PointCloud2, including organized clouds with row padding."""
    fields = {field.name: field for field in message.fields}
    missing = {"x", "y", "z"} - fields.keys()
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(sorted(missing))}")
    ros_to_numpy = {
        1: "i1",
        2: "u1",
        3: "i2",
        4: "u2",
        5: "i4",
        6: "u4",
        7: "f4",
        8: "f8",
    }
    endian = ">" if message.is_bigendian else "<"
    names = []
    formats = []
    offsets = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype not in ros_to_numpy or field.count != 1:
            raise ValueError(f"Unsupported PointCloud2 {name} field datatype/count")
        names.append(name)
        formats.append(endian + ros_to_numpy[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": message.point_step,
        }
    )
    row_bytes = message.width * message.point_step
    raw = memoryview(message.data)
    if message.height <= 1 or message.row_step == row_bytes:
        records = np.frombuffer(raw, dtype=dtype, count=message.width * message.height)
    else:
        rows = [
            np.frombuffer(
                raw[row * message.row_step:row * message.row_step + row_bytes],
                dtype=dtype,
                count=message.width,
            )
            for row in range(message.height)
        ]
        records = np.concatenate(rows) if rows else np.empty((0,), dtype=dtype)
    return np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64, copy=False
    )


def _message_stamp_ns(message: Any, bag_stamp_ns: int) -> int:
    try:
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return stamp_ns if stamp_ns != 0 else int(bag_stamp_ns)
    except AttributeError:
        return int(bag_stamp_ns)


def _message_frame_id(message: Any) -> str:
    header = getattr(message, "header", None)
    return str(getattr(header, "frame_id", "") or "")


def _pose_sample(message: Any, bag_stamp_ns: int) -> PoseSample:
    pose = message.pose.pose
    return PoseSample(
        stamp_ns=_message_stamp_ns(message, bag_stamp_ns),
        bag_stamp_ns=int(bag_stamp_ns),
        position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        orientation_xyzw=(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
        frame_id=_message_frame_id(message),
        child_frame_id=str(getattr(message, "child_frame_id", "") or ""),
    )


class DiagnosticCollector:
    def __init__(self) -> None:
        self.array_count = 0
        self.status_count = 0
        self.level_counts: Counter[str] = Counter()
        self.message_counts: Counter[str] = Counter()
        self.numeric_values: dict[str, list[float]] = defaultdict(list)
        self.last_values: dict[str, str] = {}

    def add(self, topic: str, message: Any) -> None:
        self.array_count += 1
        for status in message.status:
            self.status_count += 1
            name = status.name or "<unnamed>"
            raw_level = status.level
            if isinstance(raw_level, (bytes, bytearray, memoryview)):
                level = raw_level[0] if len(raw_level) == 1 else -1
            else:
                level = int(raw_level)
            level_name = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}.get(level, str(level))
            self.level_counts[level_name] += 1
            if status.message:
                self.message_counts[f"{name}: {status.message}"] += 1
            for value in status.values:
                key = f"{topic}/{name}/{value.key}"
                text = str(value.value)
                self.last_values[key] = text
                try:
                    numeric = float(text)
                except ValueError:
                    continue
                if math.isfinite(numeric):
                    self.numeric_values[key].append(numeric)

    def summary(self) -> dict[str, Any]:
        return {
            "array_count": self.array_count,
            "status_count": self.status_count,
            "level_counts": dict(sorted(self.level_counts.items())),
            "message_counts": dict(sorted(self.message_counts.items())),
            "numeric_values": {
                key: {**_distribution(values), "last": values[-1]}
                for key, values in sorted(self.numeric_values.items())
            },
            "last_values": dict(sorted(self.last_values.items())),
        }


def resource_metrics(run_dir: Path) -> dict[str, Any]:
    """Read runner resource artifacts without making them mandatory."""
    summary_path = run_dir / "resource_summary.json"
    provided_summary: Any = None
    if summary_path.is_file():
        try:
            provided_summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            provided_summary = {"read_error": str(error)}

    csv_path = run_dir / "resource_metrics.csv"
    if not csv_path.is_file():
        return {
            "available": provided_summary is not None,
            "resource_metrics_csv": None,
            "runner_summary": provided_summary,
            "processes": {},
        }

    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    with csv_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                groups[row["process"]].append(
                    {
                        "wall_time_s": float(row["wall_time_s"]),
                        "pid": int(row["pid"]),
                        "cpu_time_s": float(row["cpu_time_s"]),
                        "rss_kib": float(row["rss_kib"]),
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue

    process_summaries: dict[str, Any] = {}
    all_wall_times: list[float] = []
    for process, rows in sorted(groups.items()):
        all_wall_times.extend(row["wall_time_s"] for row in rows)
        cpu_delta_s = 0.0
        by_pid: dict[int, list[dict[str, Any]]] = defaultdict(list)
        for row in rows:
            by_pid[row["pid"]].append(row)
        for pid_rows in by_pid.values():
            cpu_times = [row["cpu_time_s"] for row in pid_rows]
            cpu_delta_s += max(0.0, max(cpu_times) - min(cpu_times))
        wall_span_s = max(row["wall_time_s"] for row in rows) - min(
            row["wall_time_s"] for row in rows
        )
        process_summaries[process] = {
            "sample_count": len(rows),
            "pid_count": len(by_pid),
            "wall_span_s": wall_span_s,
            "cpu_delta_s": cpu_delta_s,
            "average_cpu_cores": cpu_delta_s / wall_span_s if wall_span_s > 0.0 else None,
            "peak_rss_bytes": int(max(row["rss_kib"] for row in rows) * 1024.0),
        }
    wall_span_s = max(all_wall_times) - min(all_wall_times) if all_wall_times else 0.0
    return {
        "available": bool(process_summaries) or provided_summary is not None,
        "resource_metrics_csv": csv_path.name,
        "wall_span_s": wall_span_s,
        "runner_summary": provided_summary,
        "processes": process_summaries,
    }


def _write_trajectory_csv(path: Path, topic: str, samples: Sequence[PoseSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    origin_ns = samples[0].stamp_ns if samples else 0
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "topic",
                "timestamp_ns",
                "elapsed_s",
                "bag_timestamp_ns",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "frame_id",
                "child_frame_id",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    topic,
                    sample.stamp_ns,
                    f"{(sample.stamp_ns - origin_ns) * 1.0e-9:.9f}",
                    sample.bag_stamp_ns,
                    *sample.position,
                    *sample.orientation_xyzw,
                    sample.frame_id,
                    sample.child_frame_id,
                ]
            )


def _write_trajectory_artifacts(
    output_dir: Path,
    trajectories: Mapping[str, Sequence[PoseSample]],
    primary_topic: str,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    primary_samples = trajectories[primary_topic]
    primary_path = output_dir / "trajectory.csv"
    _write_trajectory_csv(primary_path, primary_topic, primary_samples)
    primary_record = _artifact_record(primary_path)

    artifacts: dict[str, dict[str, Any]] = {}
    for topic, samples in trajectories.items():
        if not samples:
            continue
        if topic == primary_topic:
            path = primary_path
        else:
            safe_topic = topic.strip("/").replace("/", "_") or "root"
            path = output_dir / f"trajectory_{safe_topic}.csv"
        if path != primary_path:
            _write_trajectory_csv(path, topic, samples)
        frame_ids = sorted({sample.frame_id for sample in samples if sample.frame_id})
        child_frame_ids = sorted(
            {sample.child_frame_id for sample in samples if sample.child_frame_id}
        )
        artifacts[topic] = {
            **_artifact_record(path),
            "frame_id": frame_ids[0] if len(frame_ids) == 1 else "",
            "child_frame_id": child_frame_ids[0] if len(child_frame_ids) == 1 else "",
            "frame_ids": frame_ids,
            "child_frame_ids": child_frame_ids,
        }
    return primary_record, artifacts


def _stable_preview(
    centroids: np.ndarray, counts: np.ndarray, max_points: int
) -> tuple[np.ndarray, np.ndarray]:
    """Select evenly spaced entries from already sorted voxel centroids."""
    if max_points <= 0:
        raise ValueError("preview_max_points must be positive")
    point_count = int(centroids.shape[0])
    if point_count <= max_points:
        return centroids, counts
    if max_points == 1:
        indexes = np.asarray([(point_count - 1) // 2], dtype=np.int64)
    else:
        indexes = (
            np.arange(max_points, dtype=np.int64) * (point_count - 1)
        ) // (max_points - 1)
    return centroids[indexes], counts[indexes]


def analyze_bag(args: argparse.Namespace) -> int:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "Bag analysis requires a sourced ROS 2 Humble environment "
            "with rosbag2_py, rclpy, and rosidl_runtime_py"
        ) from error

    bag_path = Path(args.result_bag).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        metadata.name: metadata.type for metadata in reader.get_all_topics_and_types()
    }
    selected_diagnostic_topics = {
        topic for topic, type_name in topic_types.items() if type_name == DIAGNOSTIC_ARRAY_TYPE
    }
    selected_diagnostic_topics.update(args.diagnostics_topic)
    selected_topics = set(ODOMETRY_TOPICS) | {CLOUD_TOPIC} | selected_diagnostic_topics
    message_classes = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in selected_topics
    }

    trajectories: dict[str, list[PoseSample]] = {topic: [] for topic in ODOMETRY_TOPICS}
    voxel_map = VoxelAccumulator(args.voxel_size, args.voxel_chunk_points)
    cloud_frames = FrameIdTracker(CLOUD_TOPIC)
    diagnostics = DiagnosticCollector()
    topic_message_counts: Counter[str] = Counter()
    selected_message_count = 0
    pointcloud_parse_error_count = 0

    while reader.has_next():
        topic, serialized, bag_stamp_ns = reader.read_next()
        message_class = message_classes.get(topic)
        if message_class is None:
            continue
        selected_message_count += 1
        topic_message_counts[topic] += 1
        message = deserialize_message(serialized, message_class)
        if topic in trajectories:
            trajectories[topic].append(_pose_sample(message, bag_stamp_ns))
        elif topic == CLOUD_TOPIC:
            cloud_frames.add(_message_frame_id(message))
            try:
                voxel_map.add(pointcloud2_xyz(message))
            except ValueError as error:
                pointcloud_parse_error_count += 1
                if pointcloud_parse_error_count <= 3:
                    print(f"warning: skipping malformed {CLOUD_TOPIC}: {error}", file=sys.stderr)
        elif topic in selected_diagnostic_topics:
            diagnostics.add(topic, message)

    primary_topic = next(
        (topic for topic in ODOMETRY_TOPICS if trajectories[topic]), ODOMETRY_TOPICS[0]
    )
    primary_samples = trajectories[primary_topic]
    if not primary_samples:
        raise RuntimeError(f"result bag contains no {DLIO_ODOMETRY_TOPIC} samples")
    if voxel_map.frame_count == 0 or voxel_map.finite_point_count == 0:
        raise RuntimeError(
            f"result bag contains no finite {CLOUD_TOPIC} map points"
        )
    trajectory_record, trajectory_artifacts = _write_trajectory_artifacts(
        output_dir, trajectories, primary_topic
    )
    trajectory_path = output_dir / trajectory_record["path"]

    local_planes = voxel_map.local_plane_metrics(
        radius_m=args.plane_radius,
        min_points=args.plane_min_points,
        max_samples=args.plane_max_samples,
        random_seed=args.plane_random_seed,
    )
    centroids, counts, _ = voxel_map.sorted_centroids()
    pcd_path = output_dir / "map_voxelized.pcd"
    write_pcd(pcd_path, centroids, counts, args.pcd_format)
    preview_centroids, preview_counts = _stable_preview(
        centroids, counts, args.preview_max_points
    )
    preview_path = output_dir / "map_preview.pcd"
    write_pcd(preview_path, preview_centroids, preview_counts, args.pcd_format)
    map_summary = voxel_map.summary(local_planes)
    pcd_record = _artifact_record(pcd_path)
    preview_record = _artifact_record(preview_path)
    map_summary.update(
        {
            "topic": CLOUD_TOPIC,
            "frame_id": cloud_frames.frame_id,
            "pointcloud_parse_error_count": pointcloud_parse_error_count,
            "pcd_path": pcd_path.name,
            "pcd_sha256": pcd_record["sha256"],
            "pcd_size_bytes": pcd_record["size_bytes"],
            "pcd_data_format": args.pcd_format,
            "pcd_fields": ["x", "y", "z", "count"],
            "voxel_chunk_points": args.voxel_chunk_points,
            "preview": {
                **preview_record,
                "point_count": int(preview_centroids.shape[0]),
                "maximum_point_count": args.preview_max_points,
                "selection": (
                    "stable evenly spaced indexes over sorted voxel centroids"
                ),
            },
        }
    )

    trajectory_summaries = {
        topic: trajectory_metrics(
            samples,
            gap_threshold_s=args.gap_threshold,
            translation_jump_threshold_m=args.translation_jump_threshold,
            orientation_jump_threshold_deg=args.orientation_jump_threshold_deg,
        )
        for topic, samples in trajectories.items()
    }
    analysis_parameters = {
        "storage_id": args.storage_id,
        "diagnostics_topics": sorted(selected_diagnostic_topics),
        "voxel_size_m": args.voxel_size,
        "voxel_chunk_points": args.voxel_chunk_points,
        "plane_radius_m": args.plane_radius,
        "plane_minimum_points": args.plane_min_points,
        "plane_maximum_samples": args.plane_max_samples,
        "plane_random_seed": args.plane_random_seed,
        "pcd_data_format": args.pcd_format,
        "preview_max_points": args.preview_max_points,
        "trajectory_gap_s": args.gap_threshold,
        "translation_jump_m": args.translation_jump_threshold,
        "orientation_jump_deg": args.orientation_jump_threshold_deg,
    }
    metadata_path = bag_path / "metadata.yaml"
    metadata_record = (
        _artifact_record(metadata_path) if metadata_path.is_file() else None
    )
    artifact_hashes = {
        trajectory_record["path"]: trajectory_record["sha256"],
        pcd_record["path"]: pcd_record["sha256"],
        preview_record["path"]: preview_record["sha256"],
    }
    for artifact in trajectory_artifacts.values():
        artifact_hashes[artifact["path"]] = artifact["sha256"]

    summary = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "analysis_kind": "relative offline odometry/map diagnostics; no ground truth",
        "run_label": args.label or output_dir.name,
        "source_bag": str(bag_path),
        "primary_trajectory_topic": primary_topic,
        "trajectory_csv": trajectory_path.name,
        "trajectory_csv_sha256": trajectory_record["sha256"],
        "trajectory_csv_size_bytes": trajectory_record["size_bytes"],
        "trajectory_csvs": {
            topic: artifact["path"]
            for topic, artifact in trajectory_artifacts.items()
        },
        "trajectory_artifacts": trajectory_artifacts,
        "artifact_hashes": dict(sorted(artifact_hashes.items())),
        "analysis_parameters": analysis_parameters,
        "analysis_parameters_sha256": _canonical_json_sha256(analysis_parameters),
        "trajectory": trajectory_summaries[primary_topic],
        "trajectories": trajectory_summaries,
        "map": map_summary,
        "diagnostics": {
            "topics": sorted(selected_diagnostic_topics),
            **diagnostics.summary(),
        },
        "resources": resource_metrics(output_dir),
        "bag": {
            "storage_id": args.storage_id,
            "selected_message_count": selected_message_count,
            "topic_message_counts": dict(sorted(topic_message_counts.items())),
            "available_topic_types": dict(sorted(topic_types.items())),
            "metadata_path": (
                metadata_record["path"] if metadata_record is not None else None
            ),
            "metadata_sha256": (
                metadata_record["sha256"] if metadata_record is not None else None
            ),
        },
        "thresholds": {
            "trajectory_gap_s": args.gap_threshold,
            "translation_jump_m": args.translation_jump_threshold,
            "orientation_jump_deg": args.orientation_jump_threshold_deg,
        },
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {trajectory_path}")
    print(f"wrote {pcd_path}")
    print(f"wrote {preview_path}")
    print(f"wrote {summary_path}")
    return 0


def _load_trajectory_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    timestamps: list[int] = []
    positions: list[tuple[float, float, float]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                timestamps.append(int(row["timestamp_ns"]))
                positions.append((float(row["x"]), float(row["y"]), float(row["z"])))
            except (KeyError, TypeError, ValueError):
                continue
    return (
        np.asarray(timestamps, dtype=np.int64),
        np.asarray(positions, dtype=np.float64).reshape((-1, 3)),
    )


def trajectory_divergence(
    reference_timestamps_ns: np.ndarray,
    reference_positions: np.ndarray,
    candidate_timestamps_ns: np.ndarray,
    candidate_positions: np.ndarray,
) -> dict[str, Any]:
    """Position difference at common times; this is not an accuracy metric."""
    if reference_timestamps_ns.size < 2 or candidate_timestamps_ns.size == 0:
        return {"common_sample_count": 0, "translation_difference_m": _distribution([])}
    reference_order = np.argsort(reference_timestamps_ns, kind="stable")
    reference_timestamps_ns = reference_timestamps_ns[reference_order]
    reference_positions = reference_positions[reference_order]
    unique_mask = np.concatenate(([True], np.diff(reference_timestamps_ns) > 0))
    reference_timestamps_ns = reference_timestamps_ns[unique_mask]
    reference_positions = reference_positions[unique_mask]
    common = (
        (candidate_timestamps_ns >= reference_timestamps_ns[0])
        & (candidate_timestamps_ns <= reference_timestamps_ns[-1])
        & np.all(np.isfinite(candidate_positions), axis=1)
    )
    candidate_times = candidate_timestamps_ns[common]
    candidate_common_positions = candidate_positions[common]
    if candidate_times.size == 0:
        return {"common_sample_count": 0, "translation_difference_m": _distribution([])}
    relative_reference_time = (
        reference_timestamps_ns - reference_timestamps_ns[0]
    ).astype(np.float64)
    relative_candidate_time = (candidate_times - reference_timestamps_ns[0]).astype(np.float64)
    interpolated = np.column_stack(
        [
            np.interp(
                relative_candidate_time,
                relative_reference_time,
                reference_positions[:, axis],
            )
            for axis in range(3)
        ]
    )
    differences = np.linalg.norm(candidate_common_positions - interpolated, axis=1)
    return {
        "common_sample_count": int(candidate_times.size),
        "common_start_timestamp_ns": int(candidate_times[0]),
        "common_end_timestamp_ns": int(candidate_times[-1]),
        "translation_difference_m": _distribution(differences),
        "terminal_translation_difference_m": float(differences[-1]),
        "interpretation": "difference from the first D-LIO run, not ground-truth error",
    }


def _nested(data: Mapping[str, Any], path: str) -> Any:
    current: Any = data
    for key in path.split("."):
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


def _required_nested(data: Mapping[str, Any], path: str) -> Any:
    current: Any = data
    for key in path.split("."):
        if not isinstance(current, Mapping) or key not in current:
            raise ValueError(f"comparison input is missing required field '{path}'")
        current = current[key]
    return current


def _comparison_invariant_value(
    summary: Mapping[str, Any],
    *,
    run_label: str,
    canonical_path: str,
    accepted_paths: Sequence[str],
) -> Any:
    """Resolve one required invariant across additive summary schema variants."""
    found: dict[str, Any] = {}
    for path in accepted_paths:
        current: Any = summary
        for key in path.split("."):
            if not isinstance(current, Mapping) or key not in current:
                break
            current = current[key]
        else:
            if current is not None:
                found[path] = current

    invariant_name = f"summary.{canonical_path}"
    if not found:
        accepted = ", ".join(f"'{path}'" for path in accepted_paths)
        raise ValueError(
            f"run '{run_label}' is missing comparison invariant "
            f"'{invariant_name}'; expected one of: {accepted}"
        )
    first_value = next(iter(found.values()))
    if any(value != first_value for value in found.values()):
        raise ValueError(
            f"run '{run_label}' has conflicting fields for comparison invariant "
            f"'{invariant_name}': {found}"
        )
    return first_value


def _validate_comparison_runs(runs: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if len(runs) < 2:
        raise ValueError("compare mode requires at least two runs")

    for run in runs:
        label = run["label"]
        manifest = run["manifest"]
        summary = run["summary"]
        if manifest.get("state") != "completed" or manifest.get("exit_code") != 0:
            raise ValueError(f"run '{label}' does not have a successful completed manifest")
        if _required_nested(summary, "trajectory.sample_count") < 2:
            raise ValueError(f"run '{label}' has fewer than two trajectory samples")
        if _required_nested(summary, "trajectory.finite_sample_count") < 2:
            raise ValueError(f"run '{label}' has fewer than two finite trajectory samples")
        if _required_nested(summary, "map.frame_count") < 1:
            raise ValueError(f"run '{label}' contains no registered cloud frames")
        if _required_nested(summary, "map.finite_point_count") < 1:
            raise ValueError(f"run '{label}' contains no finite registered cloud points")

    invariant_paths = {
        "manifest": [
            "bag.path",
            "bag.metadata_sha256",
            "playback.topics",
            "playback.start_offset_s",
            "playback.duration_s",
            "playback.rate",
        ],
        "summary": [
            "primary_trajectory_topic",
            "thresholds",
            "map.voxel_size_m",
            "map.pcd_data_format",
            "map.local_planes.radius_m",
            "map.local_planes.minimum_points",
        ],
    }
    validated: dict[str, Any] = {}
    for namespace, paths in invariant_paths.items():
        for path in paths:
            values = [_required_nested(run[namespace], path) for run in runs]
            if any(value != values[0] for value in values[1:]):
                labelled = {
                    run["label"]: value for run, value in zip(runs, values)
                }
                raise ValueError(
                    f"comparison invariant '{namespace}.{path}' differs: {labelled}"
                )
            validated[f"{namespace}.{path}"] = values[0]
    additive_summary_invariants = {
        "map.voxel_chunk_points": (
            "map.voxel_chunk_points",
            "analysis_parameters.voxel_chunk_points",
        ),
        "map.local_planes.maximum_sample_count": (
            "map.local_planes.maximum_sample_count",
            "analysis_parameters.plane_maximum_samples",
        ),
        "map.local_planes.random_seed": (
            "map.local_planes.random_seed",
            "analysis_parameters.plane_random_seed",
        ),
        "map.preview.maximum_point_count": (
            "map.preview.maximum_point_count",
            "analysis_parameters.preview_max_points",
        ),
    }
    for canonical_path, accepted_paths in additive_summary_invariants.items():
        values = [
            _comparison_invariant_value(
                run["summary"],
                run_label=run["label"],
                canonical_path=canonical_path,
                accepted_paths=accepted_paths,
            )
            for run in runs
        ]
        if any(value != values[0] for value in values[1:]):
            labelled = {run["label"]: value for run, value in zip(runs, values)}
            raise ValueError(
                f"comparison invariant 'summary.{canonical_path}' differs: {labelled}"
            )
        validated[f"summary.{canonical_path}"] = values[0]

    return validated


def compare_runs(args: argparse.Namespace) -> int:
    run_dirs = [Path(path).expanduser().resolve() for path in args.run_dirs]
    labels = args.labels if args.labels else [path.name for path in run_dirs]
    if len(labels) != len(run_dirs):
        raise ValueError("--labels must contain exactly one label per run directory")
    runs: list[dict[str, Any]] = []
    trajectories: list[tuple[np.ndarray, np.ndarray]] = []
    for label, run_dir in zip(labels, run_dirs):
        summary_path = run_dir / "summary.json"
        trajectory_path = run_dir / "trajectory.csv"
        manifest_path = run_dir / "manifest.json"
        if (
            not summary_path.is_file()
            or not trajectory_path.is_file()
            or not manifest_path.is_file()
        ):
            raise FileNotFoundError(
                f"{run_dir} must contain manifest.json, summary.json, and trajectory.csv"
            )
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        runs.append(
            {
                "label": label,
                "run_dir": str(run_dir),
                "summary": summary,
                "manifest": manifest,
            }
        )
        trajectories.append(_load_trajectory_csv(trajectory_path))

    validated_invariants = _validate_comparison_runs(runs)
    for run, trajectory in zip(runs, trajectories):
        expected_count = _required_nested(run["summary"], "trajectory.sample_count")
        actual_count = int(trajectory[0].size)
        if actual_count != expected_count:
            raise ValueError(
                f"run '{run['label']}' trajectory.csv count differs from summary.json"
            )
    reference_times, reference_positions = trajectories[0]
    divergence = {
        labels[index]: trajectory_divergence(
            reference_times,
            reference_positions,
            trajectories[index][0],
            trajectories[index][1],
        )
        for index in range(1, len(runs))
    }
    metric_paths = [
        "trajectory.sample_count",
        "trajectory.duration_s",
        "trajectory.rate_hz",
        "trajectory.gap_count",
        "trajectory.gaps_s.max",
        "trajectory.nonfinite_sample_count",
        "trajectory.translation_jump_count",
        "trajectory.translation_jump_first_elapsed_s",
        "trajectory.translation_max_step_elapsed_s",
        "trajectory.translation_steps_m.max",
        "trajectory.orientation_jump_count",
        "trajectory.orientation_jump_first_elapsed_s",
        "trajectory.orientation_max_step_elapsed_s",
        "trajectory.orientation_steps_deg.max",
        "trajectory.path_length_m",
        "trajectory.terminal_displacement_m",
        "map.input_point_count",
        "map.voxel_count",
        "map.occupied_volume_m3",
        "map.xy_coverage_m2",
        "map.local_planes.plane_thickness_m.median",
        "map.local_planes.plane_thickness_m.p95",
        "map.local_planes.planarity.median",
    ]
    table = [
        {
            "metric": metric,
            **{run["label"]: _nested(run["summary"], metric) for run in runs},
        }
        for metric in metric_paths
    ]
    process_names = sorted(
        {
            process
            for run in runs
            for process in _nested(run["summary"], "resources.processes") or {}
        }
    )
    for process in process_names:
        for field in ("average_cpu_cores", "peak_rss_bytes"):
            metric = f"resources.processes.{process}.{field}"
            table.append(
                {
                    "metric": metric,
                    **{run["label"]: _nested(run["summary"], metric) for run in runs},
                }
            )

    comparison = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "reference_run": labels[0],
        "reference_role": "first D-LIO comparison run; not ground truth",
        "runs": [
            {
                "label": run["label"],
                "run_dir": run["run_dir"],
                "source_bag": run["summary"].get("source_bag"),
                "primary_trajectory_topic": run["summary"].get(
                    "primary_trajectory_topic"
                ),
            }
            for run in runs
        ],
        "validated_invariants": validated_invariants,
        "metrics": table,
        "reference_trajectory_divergence": divergence,
    }
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = output_path.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["metric", *labels])
        writer.writeheader()
        writer.writerows(table)
    print(f"wrote {output_path}")
    print(f"wrote {csv_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    analyze = subparsers.add_parser("analyze", help="analyze one rosbag2 result bag")
    analyze.add_argument("result_bag", help="rosbag2 directory containing result topics")
    analyze.add_argument("--output-dir", required=True, help="directory for generated artifacts")
    analyze.add_argument("--label", help="run label stored in summary.json")
    analyze.add_argument("--storage-id", default="sqlite3", help="rosbag2 storage plugin")
    analyze.add_argument(
        "--diagnostics-topic",
        action="append",
        default=[],
        help="extra DiagnosticArray topic (all typed DiagnosticArray topics are automatic)",
    )
    analyze.add_argument("--voxel-size", type=float, default=0.20, help="map voxel edge in m")
    analyze.add_argument(
        "--voxel-chunk-points", type=int, default=500_000, help="streaming reduction chunk"
    )
    analyze.add_argument(
        "--plane-radius", type=float, default=0.60, help="local covariance radius in m"
    )
    analyze.add_argument("--plane-min-points", type=int, default=30)
    analyze.add_argument("--plane-max-samples", type=int, default=5_000)
    analyze.add_argument(
        "--plane-random-seed", type=int, default=7, help="deterministic plane sample seed"
    )
    analyze.add_argument("--pcd-format", choices=("ascii", "binary"), default="binary")
    analyze.add_argument(
        "--preview-max-points", type=int, default=500_000, help="map preview point cap"
    )
    analyze.add_argument("--gap-threshold", type=float, default=0.20, help="odometry gap in s")
    analyze.add_argument(
        "--translation-jump-threshold", type=float, default=1.0, help="step jump in m"
    )
    analyze.add_argument("--orientation-jump-threshold-deg", type=float, default=15.0)
    analyze.set_defaults(function=analyze_bag)

    compare = subparsers.add_parser("compare", help="compare analyzed run directories")
    compare.add_argument(
        "run_dirs",
        nargs="+",
        help="run directories, with the comparison reference first",
    )
    compare.add_argument("--labels", nargs="+", help="labels corresponding to run_dirs")
    compare.add_argument(
        "--output", default="comparison_summary.json", help="comparison JSON output path"
    )
    compare.set_defaults(function=compare_runs)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.function(args))
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
