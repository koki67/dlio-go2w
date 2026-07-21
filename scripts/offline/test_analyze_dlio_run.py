#!/usr/bin/env python3

import csv
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_dlio_run import (  # noqa: E402
    FrameIdTracker,
    PoseSample,
    _pose_sample,
    _sha256_file,
    _stable_preview,
    _validate_comparison_runs,
    _write_trajectory_artifacts,
    build_parser,
    DiagnosticCollector,
    VoxelAccumulator,
    pointcloud2_xyz,
    resource_metrics,
    trajectory_divergence,
    trajectory_metrics,
    write_pcd,
)


def _pose(stamp_ns, x, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return PoseSample(stamp_ns, stamp_ns + 10, (x, 0.0, 0.0), quaternion)


def test_trajectory_metrics_reports_gaps_jumps_and_path():
    samples = [
        _pose(0, 0.0),
        _pose(100_000_000, 0.2),
        _pose(600_000_000, 2.0, (0.0, 0.0, 1.0, 0.0)),
    ]
    metrics = trajectory_metrics(
        samples,
        gap_threshold_s=0.2,
        translation_jump_threshold_m=1.0,
        orientation_jump_threshold_deg=15.0,
    )

    assert metrics["sample_count"] == 3
    assert metrics["gap_count"] == 1
    assert metrics["translation_jump_count"] == 1
    assert metrics["orientation_jump_count"] == 1
    assert np.isclose(metrics["translation_jump_first_elapsed_s"], 0.6)
    assert np.isclose(metrics["translation_max_step_elapsed_s"], 0.6)
    assert np.isclose(metrics["orientation_jump_first_elapsed_s"], 0.6)
    assert np.isclose(metrics["orientation_max_step_elapsed_s"], 0.6)
    assert np.isclose(metrics["path_length_m"], 2.0)
    assert np.isclose(metrics["terminal_displacement_m"], 2.0)


def test_quaternion_sign_flip_is_not_an_orientation_jump():
    metrics = trajectory_metrics(
        [_pose(0, 0.0), _pose(100_000_000, 0.0, (0.0, 0.0, 0.0, -1.0))],
        gap_threshold_s=0.2,
        translation_jump_threshold_m=1.0,
        orientation_jump_threshold_deg=1.0,
    )

    assert metrics["orientation_jump_count"] == 0
    assert np.isclose(metrics["orientation_steps_deg"]["max"], 0.0)


def test_voxel_accumulator_centroids_and_plane_statistics():
    accumulator = VoxelAccumulator(0.5, chunk_points=8)
    x, y = np.meshgrid(np.linspace(-1.0, 1.0, 9), np.linspace(-1.0, 1.0, 9))
    plane = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    accumulator.add(plane[:40])
    accumulator.add(plane[40:])

    centroids, counts, keys = accumulator.sorted_centroids()
    local = accumulator.local_plane_metrics(
        radius_m=0.8, min_points=5, max_samples=100, random_seed=1
    )

    assert len(keys) == centroids.shape[0] == counts.shape[0]
    assert int(np.sum(counts)) == plane.shape[0]
    assert local["valid_neighborhood_count"] > 0
    assert np.isclose(local["plane_thickness_m"]["max"], 0.0, atol=1.0e-9)
    assert local["planarity"]["median"] > 0.0


def test_write_pcd_supports_ascii_and_binary(tmp_path):
    points = np.asarray([[1.0, 2.0, 3.0], [-1.0, 0.5, 4.0]])
    counts = np.asarray([2, 7], dtype=np.uint32)
    ascii_path = tmp_path / "ascii.pcd"
    binary_path = tmp_path / "binary.pcd"

    write_pcd(ascii_path, points, counts, "ascii")
    write_pcd(binary_path, points, counts, "binary")

    ascii_data = ascii_path.read_text(encoding="utf-8")
    assert "DATA ascii\n" in ascii_data
    assert "1 2 3 2\n" in ascii_data
    binary_data = binary_path.read_bytes()
    marker = b"DATA binary\n"
    assert marker in binary_data
    assert len(binary_data.split(marker, 1)[1]) == 2 * 16


class _Field:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class _PointCloud:
    pass


def test_pointcloud2_xyz_handles_organized_row_padding():
    message = _PointCloud()
    message.fields = [_Field("x", 0), _Field("y", 4), _Field("z", 8)]
    message.is_bigendian = False
    message.point_step = 12
    message.width = 2
    message.height = 2
    message.row_step = 28

    row1 = struct.pack("<ffffff", 1, 2, 3, 4, 5, 6) + b"pad!"
    row2 = struct.pack("<ffffff", 7, 8, 9, 10, 11, 12) + b"pad!"
    message.data = row1 + row2

    points = pointcloud2_xyz(message)

    np.testing.assert_allclose(
        points,
        np.asarray([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]),
    )


def test_diagnostic_collector_handles_humble_uint8_bytes():
    collector = DiagnosticCollector()
    collector.add(
        "/diagnostics",
        SimpleNamespace(
            status=[
                SimpleNamespace(
                    name="processor",
                    level=b"\x01",
                    message="idle flush",
                    values=[],
                )
            ]
        ),
    )
    assert collector.summary()["level_counts"] == {"WARN": 1}


def _comparison_run(label, rate=1.0):
    return {
        "label": label,
        "manifest": {
            "state": "completed",
            "exit_code": 0,
            "bag": {"path": "/bag", "metadata_sha256": "abc"},
            "playback": {
                "topics": ["/points_raw", "/go2w/imu"],
                "start_offset_s": 0.0,
                "duration_s": None,
                "rate": rate,
            },
        },
        "summary": {
            "primary_trajectory_topic": "/dlio/odom_node/odom",
            "thresholds": {
                "trajectory_gap_s": 0.2,
                "translation_jump_m": 1.0,
                "orientation_jump_deg": 15.0,
            },
            "trajectory": {"sample_count": 3, "finite_sample_count": 3},
            "map": {
                "frame_count": 3,
                "finite_point_count": 10,
                "voxel_size_m": 0.2,
                "voxel_chunk_points": 500_000,
                "pcd_data_format": "binary",
                "local_planes": {
                    "radius_m": 0.6,
                    "minimum_points": 20,
                    "maximum_sample_count": 5_000,
                    "random_seed": 7,
                },
                "preview": {"maximum_point_count": 500_000},
            },
        },
    }


def test_comparison_validation_requires_matching_replay_and_analysis_settings():
    baseline = _comparison_run("baseline")
    candidate = _comparison_run("candidate")

    validated = _validate_comparison_runs([baseline, candidate])
    assert validated["summary.map.voxel_chunk_points"] == 500_000
    assert (
        validated["summary.map.local_planes.maximum_sample_count"] == 5_000
    )
    assert validated["summary.map.local_planes.random_seed"] == 7
    assert (
        validated["summary.map.preview.maximum_point_count"] == 500_000
    )
    assert validated["manifest.playback.rate"] == 1.0

    candidate["manifest"]["playback"]["rate"] = 2.0
    with pytest.raises(ValueError, match="manifest.playback.rate"):
        _validate_comparison_runs([baseline, candidate])


@pytest.mark.parametrize(
    ("path", "different_value", "canonical_path"),
    [
        (("map", "voxel_chunk_points"), 1_000, "summary.map.voxel_chunk_points"),
        (
            ("map", "local_planes", "maximum_sample_count"),
            100,
            "summary.map.local_planes.maximum_sample_count",
        ),
        (("map", "local_planes", "random_seed"), 99, "summary.map.local_planes.random_seed"),
        (
            ("map", "preview", "maximum_point_count"),
            1_000,
            "summary.map.preview.maximum_point_count",
        ),
    ],
)
def test_comparison_rejects_different_deterministic_analysis_settings(
    path, different_value, canonical_path
):
    baseline = _comparison_run("baseline")
    candidate = _comparison_run("candidate")

    current = candidate["summary"]
    for key in path[:-1]:
        current = current[key]
    current[path[-1]] = different_value

    with pytest.raises(
        ValueError,
        match=canonical_path.replace(".", r"\."),
    ):
        _validate_comparison_runs([baseline, candidate])


def test_comparison_accepts_analysis_parameter_fallback_fields():
    baseline = _comparison_run("baseline")
    candidate = _comparison_run("candidate")

    candidate_summary = candidate["summary"]
    candidate_summary["analysis_parameters"] = {
        "voxel_chunk_points": 500_000,
        "plane_maximum_samples": 5_000,
        "plane_random_seed": 7,
        "preview_max_points": 500_000,
    }
    map_summary = candidate_summary["map"]
    map_summary.pop("voxel_chunk_points")
    map_summary["local_planes"].pop("maximum_sample_count")
    map_summary["local_planes"].pop("random_seed")
    map_summary["preview"].pop("maximum_point_count")

    validated = _validate_comparison_runs([baseline, candidate])
    assert validated["summary.map.voxel_chunk_points"] == 500_000


def test_comparison_reports_missing_new_invariant_field():
    legacy = _comparison_run("legacy")
    candidate = _comparison_run("candidate")
    legacy["summary"]["map"].pop("voxel_chunk_points")

    with pytest.raises(
        ValueError,
        match=r"missing comparison invariant 'summary\.map\.voxel_chunk_points'",
    ):
        _validate_comparison_runs([legacy, candidate])


def test_comparison_reports_conflicting_fallback_fields():
    baseline = _comparison_run("baseline")
    candidate = _comparison_run("candidate")
    candidate["summary"]["analysis_parameters"] = {"voxel_chunk_points": 1_000}

    with pytest.raises(
        ValueError,
        match=r"conflicting fields.*summary\.map\.voxel_chunk_points",
    ):
        _validate_comparison_runs([baseline, candidate])


def test_trajectory_divergence_interpolates_reference():
    reference_times = np.asarray([0, 1_000_000_000, 2_000_000_000], dtype=np.int64)
    reference_positions = np.asarray([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=float)
    candidate_times = np.asarray([500_000_000, 1_500_000_000], dtype=np.int64)
    candidate_positions = np.asarray([[0.5, 1, 0], [1.5, 1, 0]], dtype=float)

    divergence = trajectory_divergence(
        reference_times, reference_positions, candidate_times, candidate_positions
    )

    assert divergence["common_sample_count"] == 2
    assert np.isclose(divergence["translation_difference_m"]["mean"], 1.0)


def test_resource_metrics_derives_cpu_and_peak_rss(tmp_path):
    csv_path = tmp_path / "resource_metrics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["wall_time_s", "pid", "process", "cpu_time_s", "rss_kib"],
        )
        writer.writeheader()
        writer.writerows(
            [
                {
                    "wall_time_s": 0,
                    "pid": 10,
                    "process": "dlio",
                    "cpu_time_s": 2,
                    "rss_kib": 100,
                },
                {
                    "wall_time_s": 2,
                    "pid": 10,
                    "process": "dlio",
                    "cpu_time_s": 5,
                    "rss_kib": 120,
                },
            ]
        )
    (tmp_path / "resource_summary.json").write_text(
        json.dumps({"runner": "ok"}), encoding="utf-8"
    )

    result = resource_metrics(tmp_path)

    assert result["available"]
    assert np.isclose(result["processes"]["dlio"]["average_cpu_cores"], 1.5)
    assert result["processes"]["dlio"]["peak_rss_bytes"] == 120 * 1024


def test_plane_metrics_are_chunk_and_insertion_order_independent():
    axis = np.arange(-12, 13, dtype=np.float64) / 8.0
    x, y = np.meshgrid(axis, axis)
    plane = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))

    first = VoxelAccumulator(0.25, chunk_points=37)
    first.add(plane)

    second = VoxelAccumulator(0.25, chunk_points=11)
    for start in range(0, plane.shape[0], 17):
        second.add(plane[::-1][start:start + 17])

    settings = {
        "radius_m": 0.55,
        "min_points": 5,
        "max_samples": 80,
        "random_seed": 19,
    }
    first_metrics = first.local_plane_metrics(**settings)
    second_metrics = second.local_plane_metrics(**settings)

    assert first_metrics["sampled_voxel_key_sha256"] == second_metrics[
        "sampled_voxel_key_sha256"
    ]
    assert first_metrics["sampled_voxel_count"] == 80
    assert first_metrics["valid_neighborhood_count"] == second_metrics[
        "valid_neighborhood_count"
    ]
    for metric in ("plane_thickness_m", "planarity"):
        for statistic in ("count", "mean", "median", "p95", "max"):
            left = first_metrics[metric][statistic]
            right = second_metrics[metric][statistic]
            if left is None or right is None:
                assert left == right
            else:
                assert np.isclose(left, right, equal_nan=True)


def test_frame_tracking_rejects_mixed_cloud_frames_and_pose_keeps_frames():
    tracker = FrameIdTracker("/dlio/odom_node/pointcloud/deskewed")
    tracker.add("")
    tracker.add("camera_init")
    tracker.add("camera_init")
    assert tracker.frame_id == "camera_init"
    with pytest.raises(ValueError, match="mixed nonempty frame IDs"):
        tracker.add("map")

    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2), frame_id="map"),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )
    sample = _pose_sample(message, 123)

    assert sample.frame_id == "map"
    assert sample.child_frame_id == "base_link"


def test_dlio_trajectory_artifact_uses_primary_csv(tmp_path):
    odom_sample = PoseSample(
        stamp_ns=1,
        bag_stamp_ns=2,
        position=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id="camera_init",
        child_frame_id="body",
    )
    topic = "/dlio/odom_node/odom"
    primary_record, artifacts = _write_trajectory_artifacts(
        tmp_path,
        {topic: [odom_sample]},
        topic,
    )

    assert primary_record["path"] == "trajectory.csv"
    assert set(artifacts) == {topic}
    assert artifacts[topic]["path"] == "trajectory.csv"
    assert artifacts[topic]["frame_id"] == "camera_init"
    assert artifacts[topic]["child_frame_id"] == "body"
    assert primary_record["sha256"] == _sha256_file(tmp_path / "trajectory.csv")

    with (tmp_path / "trajectory.csv").open(newline="") as stream:
        primary_rows = list(csv.DictReader(stream))
    assert primary_rows[0]["topic"] == topic
    assert primary_rows[0]["frame_id"] == "camera_init"
    assert primary_rows[0]["child_frame_id"] == "body"


def test_map_preview_is_deterministic_evenly_spaced_and_capped(tmp_path):
    values = np.arange(23, dtype=np.float64)
    centroids = np.column_stack((values, values * 2.0, values * -0.5))
    counts = np.arange(1, 24, dtype=np.uint32)

    first_points, first_counts = _stable_preview(centroids, counts, 7)
    second_points, second_counts = _stable_preview(centroids, counts, 7)

    assert first_points.shape == (7, 3)
    np.testing.assert_array_equal(first_points, second_points)
    np.testing.assert_array_equal(first_counts, second_counts)
    np.testing.assert_array_equal(first_points[0], centroids[0])
    np.testing.assert_array_equal(first_points[-1], centroids[-1])

    first_path = tmp_path / "first.pcd"
    second_path = tmp_path / "second.pcd"
    write_pcd(first_path, first_points, first_counts, "binary")
    write_pcd(second_path, second_points, second_counts, "binary")
    assert _sha256_file(first_path) == _sha256_file(second_path)

    single_points, single_counts = _stable_preview(centroids, counts, 1)
    np.testing.assert_array_equal(single_points[0], centroids[11])
    np.testing.assert_array_equal(single_counts, counts[[11]])
    with pytest.raises(ValueError, match="preview_max_points"):
        _stable_preview(centroids, counts, 0)


def test_analyze_cli_has_deterministic_artifact_defaults():
    args = build_parser().parse_args(["analyze", "bag", "--output-dir", "out"])

    assert args.plane_random_seed == 7
    assert args.preview_max_points == 500_000
