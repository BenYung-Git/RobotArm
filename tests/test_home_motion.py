"""Test: move_to_home() drives joints to zero (within tolerance)."""
from __future__ import annotations

import pytest

from robotarm.core.virtual_robot import VirtualRobot


@pytest.fixture
def robot():
    pybullet = pytest.importorskip("pybullet")
    r = VirtualRobot(robot_id="test_home")
    r.connect(target="direct", gui=False)
    # First move to a non-zero pose
    r.moveJ([0.5, -0.5, 1.0, -0.3, 0.7, -0.2])
    yield r
    r.disconnect()


def test_move_to_home_brings_joints_near_zero(robot):
    robot.move_to_home()
    state = robot.get_state()
    for j in state.joints:
        assert abs(j) < 1e-2, f"Joint {j} not near zero after move_to_home()"


def test_move_to_home_actually_runs(robot):
    # Confirm move is not silently a no-op
    pre = robot.get_state().joints
    assert any(abs(p) > 0.1 for p in pre), "Test setup failed: pre-home pose should be non-zero"
    robot.move_to_home()
    post = robot.get_state().joints
    for p in post:
        assert abs(p) < 1e-2
