#!/usr/bin/env python3

import csv
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from publish_dlio_artifacts import (  # noqa: E402
    ArtifactError,
    load_artifact_bundle,
    load_pcd,
    load_trajectory,
    main,
    sha256_file,
)


def _pcd_header(point_count, data_format):
    return (
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


def _write_ascii_pcd(path, rows):
    payload = "".join(
        f"{x:.9g} {y:.9g} {z:.9g} {count}\n" for x, y, z, count in rows
    )
    path.write_text(_pcd_header(len(rows), "ascii") + payload, encoding="ascii")


def _write_binary_pcd(path, rows):
    records = np.asarray(
        rows,
        dtype=np.dtype(
            [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("count", "<u4")]
        ),
    )
    path.write_bytes(
        _pcd_header(len(records), "binary").encode("ascii") + records.tobytes()
    )


def _write_trajectory(path, frame_id="camera_init", child_frame_id="body"):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "topic",
                "timestamp_ns",
                "frame_id",
                "child_frame_id",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "topic": "/dlio/odom_node/odom",
                "timestamp_ns": 100,
                "frame_id": frame_id,
                "child_frame_id": child_frame_id,
                "x": 0,
                "y": 1,
                "z": 2,
                "qx": 0,
                "qy": 0,
                "qz": 0,
                "qw": 2,
            }
        )
        writer.writerow(
            {
                "topic": "/dlio/odom_node/odom",
                "timestamp_ns": 200,
                "frame_id": frame_id,
                "child_frame_id": child_frame_id,
                "x": 3,
                "y": 4,
                "z": 5,
                "qx": 0,
                "qy": 0,
                "qz": 1,
                "qw": 0,
            }
        )


def test_load_ascii_and_binary_pcd(tmp_path):
    rows = [(1.0, 2.0, 3.0, 4), (-1.5, 0.25, 7.0, 9)]
    ascii_path = tmp_path / "ascii.pcd"
    binary_path = tmp_path / "binary.pcd"
    _write_ascii_pcd(ascii_path, rows)
    _write_binary_pcd(binary_path, rows)

    ascii_cloud = load_pcd(ascii_path)
    binary_cloud = load_pcd(binary_path)

    expected_xyz = np.asarray([row[:3] for row in rows], dtype=np.float32)
    expected_counts = np.asarray([row[3] for row in rows], dtype=np.uint32)
    np.testing.assert_allclose(ascii_cloud.xyz, expected_xyz)
    np.testing.assert_allclose(binary_cloud.xyz, expected_xyz)
    np.testing.assert_array_equal(ascii_cloud.counts, expected_counts)
    np.testing.assert_array_equal(binary_cloud.counts, expected_counts)
    assert ascii_cloud.data_format == "ascii"
    assert binary_cloud.data_format == "binary"


def test_load_pcd_rejects_nonfinite_and_truncated_payload(tmp_path):
    nonfinite = tmp_path / "nonfinite.pcd"
    nonfinite.write_text(
        _pcd_header(1, "ascii") + "nan 0 0 1\n", encoding="ascii"
    )
    with pytest.raises(ArtifactError, match="non-finite"):
        load_pcd(nonfinite)

    truncated = tmp_path / "truncated.pcd"
    truncated.write_bytes(_pcd_header(1, "binary").encode("ascii") + b"too short")
    with pytest.raises(ArtifactError, match="payload size"):
        load_pcd(truncated)


def test_load_trajectory_validates_frames_and_normalizes_quaternions(tmp_path):
    path = tmp_path / "trajectory.csv"
    _write_trajectory(path)

    trajectory = load_trajectory(
        path, expected_frame_id="/camera_init", expected_child_frame_id="body"
    )

    assert trajectory.frame_id == "camera_init"
    assert trajectory.child_frame_id == "body"
    assert trajectory.positions.shape == (2, 3)
    np.testing.assert_allclose(
        np.linalg.norm(trajectory.orientations_xyzw, axis=1), np.ones(2)
    )

    with pytest.raises(ArtifactError, match="frame mismatch"):
        load_trajectory(path, expected_frame_id="odom")


def test_load_bundle_prefers_preview_and_validates_hashes(tmp_path):
    map_path = tmp_path / "map_preview.pcd"
    trajectory_path = tmp_path / "trajectory.csv"
    _write_binary_pcd(map_path, [(1.0, 2.0, 3.0, 5), (4.0, 5.0, 6.0, 7)])
    _write_trajectory(trajectory_path)
    summary = {
        "map": {
            "frame_id": "camera_init",
            "pcd_path": "unused_full_map.pcd",
            "preview": {
                "path": map_path.name,
                "point_count": 2,
                "sha256": sha256_file(map_path),
            },
        },
        "trajectory_artifacts": {
            "/dlio/odom_node/odom": {
                "path": trajectory_path.name,
                "frame_id": "camera_init",
                "child_frame_id": "body",
                "sha256": sha256_file(trajectory_path),
            }
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")

    bundle = load_artifact_bundle(tmp_path)

    assert bundle.map_path == map_path
    assert bundle.trajectory_path == trajectory_path
    assert bundle.map_frame_id == "camera_init"
    assert bundle.pcd.xyz.shape == (2, 3)
    assert bundle.trajectory.positions.shape == (2, 3)

    summary["map"]["preview"]["sha256"] = "0" * 64
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    with pytest.raises(ArtifactError, match="SHA-256 mismatch"):
        load_artifact_bundle(tmp_path)


def test_load_bundle_supports_full_map_and_trajectory_fallback(tmp_path):
    map_path = tmp_path / "map_voxelized.pcd"
    trajectory_path = tmp_path / "trajectory.csv"
    _write_ascii_pcd(map_path, [(0.0, 0.0, 0.0, 1)])
    _write_trajectory(trajectory_path)
    summary = {
        "map": {
            "frame_id": "camera_init",
            "pcd_path": map_path.name,
            "voxel_count": 1,
            "pcd_sha256": sha256_file(map_path),
        }
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")

    bundle = load_artifact_bundle(tmp_path)

    assert bundle.map_path == map_path
    assert bundle.trajectory_path == trajectory_path


def test_load_bundle_rejects_map_trajectory_frame_mismatch(tmp_path):
    map_path = tmp_path / "map_voxelized.pcd"
    trajectory_path = tmp_path / "trajectory.csv"
    _write_ascii_pcd(map_path, [(0.0, 0.0, 0.0, 1)])
    _write_trajectory(trajectory_path, frame_id="odom")
    summary = {
        "map": {"frame_id": "camera_init", "pcd_path": map_path.name},
        "trajectory_artifacts": {
            "/dlio/odom_node/odom": {
                "path": trajectory_path.name,
                "frame_id": "odom",
                "child_frame_id": "body",
            }
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")

    with pytest.raises(ArtifactError, match="map and trajectory frames differ"):
        load_artifact_bundle(tmp_path)


def test_load_bundle_allows_base_aligned_odom_trajectory(tmp_path):
    map_path = tmp_path / "map_voxelized.pcd"
    trajectory_path = tmp_path / "trajectory.csv"
    _write_ascii_pcd(map_path, [(0.0, 0.0, 0.0, 1)])
    _write_trajectory(
        trajectory_path, frame_id="odom", child_frame_id="base_link"
    )
    summary = {
        "map": {"frame_id": "camera_init", "pcd_path": map_path.name},
        "trajectory_artifacts": {
            "/odom": {
                "path": trajectory_path.name,
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "sha256": sha256_file(trajectory_path),
            }
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")

    bundle = load_artifact_bundle(
        tmp_path,
        trajectory_topic="/odom",
        require_matching_frames=False,
    )

    assert bundle.map_frame_id == "camera_init"
    assert bundle.trajectory.frame_id == "odom"
    assert bundle.trajectory.child_frame_id == "base_link"


def test_print_frame_id_outputs_only_normalized_validated_frame(tmp_path, capsys):
    map_path = tmp_path / "map_preview.pcd"
    trajectory_path = tmp_path / "trajectory.csv"
    _write_ascii_pcd(map_path, [(0.0, 0.0, 0.0, 1)])
    _write_trajectory(trajectory_path, frame_id="/map")
    summary = {
        "map": {
            "frame_id": "/map",
            "preview": {
                "path": map_path.name,
                "point_count": 1,
                "sha256": sha256_file(map_path),
            },
        },
        "trajectory_artifacts": {
            "/dlio/odom_node/odom": {
                "path": trajectory_path.name,
                "frame_id": "/map",
                "child_frame_id": "body",
                "sha256": sha256_file(trajectory_path),
            }
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")

    assert main([str(tmp_path), "--print-frame-id"]) == 0
    captured = capsys.readouterr()
    assert captured.out == "map\n"
    assert captured.err == ""
