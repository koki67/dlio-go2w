from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = (
    REPO_ROOT
    / "humble_ws"
    / "src"
    / "direct_lidar_inertial_odometry"
    / "cfg"
    / "offline.yaml"
)


def test_offline_config_persists_only_replay_artifact_outputs():
    document = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    parameters = document["/**"]["ros__parameters"]

    assert parameters == {
        "offline/eventDrivenOdometry": True,
        "map/waitUntilMove": False,
        "publish/odom": True,
        "publish/pose": False,
        "publish/path": False,
        "publish/keyframes": False,
        "publish/deskewed": True,
        "publish/tf": False,
        "dlio/verbose": False,
    }
