"""Test: joint limits are enforced.

Uses a real PyBullet DIRECT connection. No GUI / no network.
"""
from __future__ import annotations

import math

import pytest

from robotarm.core.virtual_robot import VirtualRobot
from robotarm.exceptions import JointLimitError


@pytest.fixture
def robot():
    pybullet = pytest.importorskip("pybullet")
    r = VirtualRobot(robot_id="test_ur5")
    r.connect(target="direct", gui=False)
    yield r
    r.disconnect()


def test_zero_pose_is_accepted(robot):
    robot.moveJ([0.0] * 6)
    state = robot.get_state()
    for j in state.joints:
        assert abs(j) < 1e-3


def test_within_limits_accepted(robot):
    # All joints near mid-range
    pose = [0.1, -0.2, 0.3, -0.1, 0.2, -0.05]
    robot.moveJ(pose)
    state = robot.get_state()
    for actual, target in zip(state.joints, pose):
        assert abs(actual - target) < 0.01


def test_shoulder_pan_above_limit_rejected(robot):
    too_big = list(robot.joint_targets)
    too_big[0] = math.pi * 2 + 1.0  # > +2π
    with pytest.raises(JointLimitError) as excinfo:
        robot.moveJ(too_big)
    assert "shoulder_pan_joint" in str(excinfo.value)


def test_shoulder_pan_below_limit_rejected(robot):
    too_small = list(robot.joint_targets)
    too_small[0] = -math.pi * 2 - 1.0
    with pytest.raises(JointLimitError):
        robot.moveJ(too_small)


def test_elbow_above_pi_rejected(robot):
    # elbow index = 2, limit ±π
    bad = [0.0] * 6
    bad[2] = math.pi + 0.1
    with pytest.raises(JointLimitError) as excinfo:
        robot.moveJ(bad)
    assert "elbow_joint" in str(excinfo.value)


def test_elbow_below_neg_pi_rejected(robot):
    bad = [0.0] * 6
    bad[2] = -math.pi - 0.1
    with pytest.raises(JointLimitError):
        robot.moveJ(bad)


def test_non_finite_target_rejected(robot):
    with pytest.raises(Exception):  # InvalidPoseError
        robot.moveJ([float("nan")] + [0.0] * 5)
    with pytest.raises(Exception):
        robot.moveJ([float("inf")] + [0.0] * 5)


def test_wrong_length_rejected(robot):
    with pytest.raises(Exception):  # InvalidPoseError
        robot.moveJ([0.0] * 5)  # only 5
