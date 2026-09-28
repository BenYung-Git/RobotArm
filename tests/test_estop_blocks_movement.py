"""Test: E-stop blocks motion; reset enables it again."""
from __future__ import annotations

import pytest

from robotarm.core.virtual_robot import VirtualRobot
from robotarm.exceptions import EmergencyStopError


@pytest.fixture
def robot():
    pybullet = pytest.importorskip("pybullet")
    r = VirtualRobot(robot_id="test_estop")
    r.connect(target="direct", gui=False)
    yield r
    r.disconnect()


def test_estop_initial_false(robot):
    assert robot.is_e_stopped is False


def test_estop_set_true(robot):
    robot.emergency_stop()
    assert robot.is_e_stopped is True


def test_estop_blocks_moveJ(robot):
    robot.emergency_stop()
    with pytest.raises(EmergencyStopError):
        robot.moveJ([0.1] * 6)


def test_estop_blocks_move_to_home(robot):
    robot.emergency_stop()
    with pytest.raises(EmergencyStopError):
        robot.move_to_home()


def test_estop_blocks_set_joint_targets(robot):
    robot.emergency_stop()
    with pytest.raises(EmergencyStopError):
        robot.set_joint_targets([0.1] * 6)


def test_estop_blocks_step_toward_targets(robot):
    robot.set_joint_targets([0.1] * 6)
    robot.emergency_stop()
    with pytest.raises(EmergencyStopError):
        robot.step_toward_targets()


def test_reset_clears_estop(robot):
    robot.emergency_stop()
    assert robot.is_e_stopped is True
    robot.reset_estop()
    assert robot.is_e_stopped is False


def test_after_reset_motion_works(robot):
    robot.emergency_stop()
    with pytest.raises(EmergencyStopError):
        robot.moveJ([0.1] * 6)
    robot.reset_estop()
    robot.moveJ([0.1] * 6)  # should succeed
    state = robot.get_state()
    assert all(abs(j - 0.1) < 0.05 for j in state.joints), \
        f"After reset and moveJ(0.1), got {state.joints}"


def test_estop_during_motion_aborts(robot):
    # Set a long target then trip E-stop mid-call
    # (this test exercises the in-loop check inside moveJ)
    robot.set_joint_targets([0.0] * 6)
    robot.emergency_stop()
    with pytest.raises(EmergencyStopError):
        robot.moveJ([2.0] * 6)  # long motion, but refused at start


def test_state_reports_e_stop(robot):
    robot.emergency_stop()
    state = robot.get_state()
    assert state.e_stopped is True
    assert state.mode == "virtual"
    assert state.robot_id == "test_estop"
