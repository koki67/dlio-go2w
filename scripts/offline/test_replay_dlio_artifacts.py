#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay_dlio_artifacts import (  # noqa: E402
    IncrementalVoxelMap,
    ReplayArtifactError,
    load_dynamic_run,
    main,
)


def _write_dynamic_run(tmp_path: Path) -> Path:
    bag_dir = tmp_path / "rosbag"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (tmp_path / "manifest.json").write_text(
        json.dumps({"state": "completed", "exit_code": 0}), encoding="utf-8"
    )
    summary = {
        "map": {"frame_id": "/camera_init", "voxel_size_m": 0.2},
        "bag": {
            "available_topic_types": {
                "/dlio/odom_node/pointcloud/deskewed": "sensor_msgs/msg/PointCloud2",
                "/dlio/odom_node/odom": "nav_msgs/msg/Odometry",
            },
            "topic_message_counts": {
                "/dlio/odom_node/pointcloud/deskewed": 12,
                "/dlio/odom_node/odom": 12,
            },
        },
        "trajectory_artifacts": {
            "/dlio/odom_node/odom": {
                "path": "trajectory.csv",
                "frame_id": "odom",
                "child_frame_id": "base_link",
            }
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    return tmp_path


def test_incremental_voxel_map_publishes_each_voxel_once():
    accumulator = IncrementalVoxelMap(0.2)
    first = np.asarray(
        [
            [0.01, 0.01, 0.01],
            [0.19, 0.19, 0.19],
            [0.20, 0.0, 0.0],
            [-0.01, 0.0, 0.0],
            [np.nan, 1.0, 2.0],
        ]
    )

    assert accumulator.add(first) == 3
    assert accumulator.frame_count == 1
    assert accumulator.input_point_count == 5
    assert accumulator.finite_point_count == 4
    assert accumulator.voxel_count == 3
    assert accumulator.pending_point_count == 3

    published = accumulator.drain_pending()
    np.testing.assert_allclose(
        published,
        np.asarray(
            [
                [0.01, 0.01, 0.01],
                [0.20, 0.0, 0.0],
                [-0.01, 0.0, 0.0],
            ],
            dtype=np.float32,
        ),
    )
    assert accumulator.published_point_count == 3
    assert accumulator.pending_point_count == 0

    assert accumulator.add(first[:4]) == 0
    assert accumulator.drain_pending().shape == (0, 3)
    assert accumulator.voxel_count == 3


def test_load_dynamic_run_validates_saved_topics_and_normalizes_frame(tmp_path):
    run_dir = _write_dynamic_run(tmp_path)

    config = load_dynamic_run(run_dir)

    assert config.run_dir == run_dir
    assert config.bag_dir == run_dir / "rosbag"
    assert config.frame_id == "camera_init"
    assert config.odometry_frame_id == "odom"
    assert config.odometry_child_frame_id == "base_link"
    assert config.voxel_size_m == pytest.approx(0.2)
    assert config.topic_message_counts == {
        "/dlio/odom_node/pointcloud/deskewed": 12,
        "/dlio/odom_node/odom": 12,
    }


def test_load_dynamic_run_rejects_incomplete_or_incompatible_results(tmp_path):
    run_dir = _write_dynamic_run(tmp_path)
    (run_dir / "manifest.json").write_text(
        json.dumps({"state": "failed", "exit_code": 1}), encoding="utf-8"
    )
    with pytest.raises(ReplayArtifactError, match="completed run"):
        load_dynamic_run(run_dir)

    (run_dir / "manifest.json").write_text(
        json.dumps({"state": "completed", "exit_code": 0}), encoding="utf-8"
    )
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    summary["bag"]["available_topic_types"]["/dlio/odom_node/odom"] = "std_msgs/msg/String"
    (run_dir / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    with pytest.raises(ReplayArtifactError, match="/odom must have type"):
        load_dynamic_run(run_dir)


def test_print_frame_id_does_not_start_ros(tmp_path, capsys):
    run_dir = _write_dynamic_run(tmp_path)

    assert main([str(run_dir), "--print-frame-id"]) == 0
    captured = capsys.readouterr()
    assert captured.out == "camera_init\n"
    assert captured.err == ""


def test_update_period_must_be_positive(tmp_path, capsys):
    run_dir = _write_dynamic_run(tmp_path)

    assert main([str(run_dir), "--update-period", "0"]) == 2
    assert "--update-period must be finite and positive" in capsys.readouterr().err
