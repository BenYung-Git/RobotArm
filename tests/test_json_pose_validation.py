"""Test: JSON pose files are validated correctly.

Critical: missing or malformed JSON must NOT be silently replaced with
defaults — the rule is fail-closed.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from robotarm.utils.pose_loader import PoseLoadError, load_pose

POSES_DIR = Path("/tmp/robotarm_demo/data/poses")


def test_home_loads_correctly():
    joints = load_pose(POSES_DIR / "home.json")
    assert len(joints) == 6
    assert all(j == 0.0 for j in joints)


def test_pose_a_loads_correctly():
    joints = load_pose(POSES_DIR / "pose_a.json")
    assert len(joints) == 6
    assert all(isinstance(j, float) for j in joints)
    # Pose A: shoulder_pan = +π/2
    assert abs(joints[0] - math.pi / 2) < 1e-6


def test_pose_b_loads_correctly():
    joints = load_pose(POSES_DIR / "pose_b.json")
    assert len(joints) == 6
    assert abs(joints[0] - (-math.pi / 2)) < 1e-6


def test_missing_file_raises(tmp_path):
    """Missing file MUST raise; no silent default fabrication."""
    fake = tmp_path / "no_such_pose.json"
    with pytest.raises(PoseLoadError) as excinfo:
        load_pose(fake)
    msg = str(excinfo.value).lower()
    assert "not found" in msg
    assert "refusing" in msg or "fabricate" in msg


def test_malformed_json_raises(tmp_path):
    f = tmp_path / "bad.json"
    f.write_text("{ this is not json", encoding="utf-8")
    with pytest.raises(PoseLoadError) as excinfo:
        load_pose(f)
    assert "json" in str(excinfo.value).lower()


def test_missing_joints_rad_raises(tmp_path):
    f = tmp_path / "no_joints.json"
    f.write_text(json.dumps({"name": "NoJoints"}), encoding="utf-8")
    with pytest.raises(PoseLoadError) as excinfo:
        load_pose(f)
    assert "joints_rad" in str(excinfo.value)


def test_wrong_length_raises(tmp_path):
    f = tmp_path / "short.json"
    f.write_text(json.dumps({"joints_rad": [0.0, 0.0, 0.0]}), encoding="utf-8")
    with pytest.raises(PoseLoadError):
        load_pose(f)


def test_non_finite_value_raises(tmp_path):
    f = tmp_path / "nan.json"
    f.write_text(json.dumps({"joints_rad": [float("nan")] + [0.0] * 5}), encoding="utf-8")
    # json.dumps writes NaN as NaN (not standard) - need allow_nan=True default
    # python's json accepts it but parseable; load_pose should reject
    with pytest.raises(PoseLoadError):
        load_pose(f)


def test_string_value_raises(tmp_path):
    f = tmp_path / "str.json"
    f.write_text(json.dumps({"joints_rad": ["abc"] + [0.0] * 5}), encoding="utf-8")
    with pytest.raises(PoseLoadError):
        load_pose(f)


def test_bool_value_rejected_as_joint(tmp_path):
    f = tmp_path / "bool.json"
    f.write_text(json.dumps({"joints_rad": [True] + [0.0] * 5}), encoding="utf-8")
    with pytest.raises(PoseLoadError):
        load_pose(f)


def test_all_poses_within_ur5_joint_limits():
    """Sanity check: every pose must be within the UR5 joint limits.

    UR5 limits (from URDF):
      - shoulder_pan/lift, wrist_1/2/3: ±2π
      - elbow: ±π
    """
    TWO_PI = 2 * math.pi
    LIMITS = [(-TWO_PI, TWO_PI), (-TWO_PI, TWO_PI), (-math.pi, math.pi),
              (-TWO_PI, TWO_PI), (-TWO_PI, TWO_PI), (-TWO_PI, TWO_PI)]

    for f in POSES_DIR.glob("*.json"):
        joints = load_pose(f)
        for i, (v, (lo, hi)) in enumerate(zip(joints, LIMITS)):
            assert lo <= v <= hi, f"{f.name}: joint {i} = {v} outside [{lo}, {hi}]"
