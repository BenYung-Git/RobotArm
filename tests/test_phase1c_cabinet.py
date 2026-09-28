"""Phase 1C cabinet / status-light / safety-gate / reset / job-sequence tests.

Covers:
- Cabinet status_light mapping (all 5 states)
- Cabinet API input update via /api/v1/virtual-io/inputs/<signal>
- Output write rejected from API (read-only from seed)
- Session-only VirtualIO — no JSON write-back
- Safety gate backend motion blocking
- E-stop / protective-stop / fault job cancellation
- Reset does NOT auto-restart
- Demo sequence Home -> Pose A -> Pose B -> Home
- /api/v1/status complete payload
- Static import scan (no socket / requests / vendor SDK)
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path

import pytest

# WebRobot lives in robotarm.core — no PyBullet dependency.
from robotarm.core.web_robot import WebRobot
from robotarm.core.virtual_controller_cabinet import VirtualControllerCabinet
from robotarm.core.tick_driver import TickDriver
from robotarm.core.cabinet_state import (
    CabinetState, SafetyState, motion_blocked, job_start_allowed,
)
from robotarm.core.status_light import compute_status_light
from robotarm.exceptions import (
    CabinetStateError,
    JobAlreadyRunningError,
    JobValidationError,
    VirtualInterlockError,
    VirtualSafetyStopError,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA = PROJECT_ROOT / "data"
POSES = DATA / "poses"
IO_SEED = DATA / "virtual_io" / "default.json"
JOB_SPEC = DATA / "jobs" / "demo_sequence.json"


@pytest.fixture
def tick_driver():
    """Per-test TickDriver running the global ROBOT if present, else a fresh
    WebRobot. Tests that exercise motion must use this fixture so the arm
    actually interpolates toward targets."""
    yield None  # placeholder — per-test usage below


def _make_cabinet() -> VirtualControllerCabinet:
    """Helper: build a fresh cabinet with the project's real seed/spec files."""
    robot = WebRobot(robot_id="test_cabinet")
    return VirtualControllerCabinet(
        robot=robot,
        cabinet_id="test_cabinet_01",
        poses_dir=POSES,
        io_seed_path=IO_SEED,
        job_spec_path=JOB_SPEC,
        motion_timeout_s=10.0,
    )


@pytest.fixture
def cabinet_with_tick():
    """Cabinet + a 60Hz tick driver. Stops the driver on teardown."""
    cab = _make_cabinet()
    driver = TickDriver(cab.robot, hz=60.0)
    driver.start()
    yield cab
    driver.stop(timeout=2.0)


# ---------------------------------------------------------------------------
# status_light mapping
# ---------------------------------------------------------------------------

def test_status_light_estop_is_blinking_red() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.ESTOP,
        safety_state=SafetyState.ESTOP,
        virtual_inputs={},
        required_inputs=None,
    )
    assert sl.color == "red"
    assert sl.is_blinking is True
    assert "急停" in sl.label_zh
    assert "E-stop" in sl.label_en


def test_status_light_protective_stop_red_static() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.PROTECTIVE_STOP,
        safety_state=SafetyState.PROTECTIVE_STOP,
        virtual_inputs={"safety_gate_closed": True},
        required_inputs=None,
    )
    assert sl.color == "red"
    assert sl.is_blinking is False
    assert sl.label_en == "Virtual Fault / Motion Blocked"


def test_status_light_fault_red_static() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.FAULT,
        safety_state=SafetyState.FAULT,
        virtual_inputs={},
        required_inputs=None,
    )
    assert sl.color == "red"
    assert sl.is_blinking is False


def test_status_light_offline_is_gray() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.OFFLINE,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={},
        required_inputs=None,
    )
    assert sl.color == "gray"
    assert sl.is_blinking is False


def test_status_light_ready_green() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": True,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "green"
    assert sl.is_blinking is False


def test_status_light_running_green() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.RUNNING,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": True,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "green"
    assert sl.label_en == "Virtual Cycle Running"
    assert "DEMO" not in sl.reason_en  # no job_id when none passed


def test_status_light_missing_required_input_is_yellow() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": False,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "yellow"
    assert sl.label_en == "Virtual Warning / Waiting"
    assert sl.reason_en == "Fixture Not Clamped"
    assert sl.reason_zh == "治具未夾緊"


# ---------------------------------------------------------------------------
# Phase 1C final spec — Yellow vs Safety Gate separation (2026-09-16).
# ---------------------------------------------------------------------------

def test_yellow_fixture_not_clamped_when_gate_closed() -> None:
    """``fixture_clamped=False``, ``safety_gate_closed=True``, READY →
    Yellow, reason="Fixture Not Clamped" (NOT safety-gate)."""
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": False,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "yellow"
    assert sl.is_blinking is False
    assert sl.label_en == "Virtual Warning / Waiting"
    assert sl.label_zh == "等待條件完成"
    assert sl.reason_en == "Fixture Not Clamped"
    assert sl.reason_zh == "治具未夾緊"
    # SAFETY rule: safety gate MUST NOT appear in the Yellow reason.
    assert "Safety Gate" not in sl.reason_en
    assert "safety_gate" not in sl.reason_en.lower()
    assert "Safety Gate" not in sl.label_en


def test_yellow_part_not_present() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": False, "fixture_clamped": True,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "yellow"
    assert sl.reason_en == "Part Not Present"
    assert sl.reason_zh == "工件不存在"


def test_yellow_welder_not_ready() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": True,
                        "welder_ready": False, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "yellow"
    assert sl.reason_en == "Welder Not Ready"
    assert sl.reason_zh == "焊機未就緒"


def test_safety_gate_open_never_produces_yellow() -> None:
    """Even with ALL readiness inputs satisfied except the gate, an open
    safety gate MUST NOT produce Yellow. The status_light must surface it
    as PROTECTIVE_STOP (Red), because the gate is a SAFETY condition."""
    # Open gate alone, no readiness info required — should not be Yellow.
    sl = compute_status_light(
        cabinet_state=CabinetState.PROTECTIVE_STOP,
        safety_state=SafetyState.PROTECTIVE_STOP,
        virtual_inputs={"safety_gate_closed": False},
        required_inputs=None,
    )
    assert sl.color == "red"
    assert sl.label_en == "Virtual Fault / Motion Blocked"
    assert sl.reason_en == "Virtual Safety Gate Open"
    assert sl.reason_zh == "虛擬安全門未關閉"


def test_safety_gate_open_with_other_inputs_red_not_yellow() -> None:
    """Even if fixture_clamped is also False alongside an open gate, the
    gate still wins — Red, "Virtual Safety Gate Open", NOT Yellow."""
    sl = compute_status_light(
        cabinet_state=CabinetState.PROTECTIVE_STOP,
        safety_state=SafetyState.PROTECTIVE_STOP,
        virtual_inputs={"part_present": True, "fixture_clamped": False,
                        "welder_ready": True, "safety_gate_closed": False},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "red"
    assert sl.reason_en == "Virtual Safety Gate Open"
    assert "Fixture" not in sl.reason_en


def test_running_state_label_is_virtual_cycle_running_not_ready() -> None:
    """RUNNING must NEVER show "Virtual System Ready"."""
    sl = compute_status_light(
        cabinet_state=CabinetState.RUNNING,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": True,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
        active_job_id="DEMO_SEQUENCE_001",
        current_step=2,
        total_steps=4,
    )
    assert sl.color == "green"
    assert sl.is_blinking is False
    assert sl.label_en == "Virtual Cycle Running"
    assert sl.label_zh == "虛擬工序執行中"
    # CRITICAL: must NOT show "Virtual System Ready"
    assert "System Ready" not in sl.label_en
    assert "虛擬系統就緒" not in sl.label_zh
    # Reason must include job_id and step/total
    assert "DEMO_SEQUENCE_001" in sl.reason_en
    assert "step 2 / 4" in sl.reason_en
    assert "DEMO_SEQUENCE_001" in sl.reason_zh
    assert "第 2 / 4" in sl.reason_zh


def test_ready_state_label_is_virtual_system_ready() -> None:
    """READY + all readiness inputs true → "Virtual System Ready"."""
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": True,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present", "fixture_clamped",
                         "welder_ready", "safety_gate_closed"],
    )
    assert sl.color == "green"
    assert sl.is_blinking is False
    assert sl.label_en == "Virtual System Ready"
    assert sl.label_zh == "虛擬系統就緒"
    # MUST NOT show "Virtual Cycle Running"
    assert "Cycle Running" not in sl.label_en
    assert "工序執行中" not in sl.label_zh


def test_offline_label_is_offline_disabled() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.OFFLINE,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={},
        required_inputs=None,
    )
    assert sl.color == "gray"
    assert sl.label_en == "Offline / Disabled"
    assert sl.label_zh == "離線／停用"


def test_status_light_to_dict_round_trip() -> None:
    sl = compute_status_light(
        cabinet_state=CabinetState.READY,
        safety_state=SafetyState.NORMAL,
        virtual_inputs={"part_present": True, "fixture_clamped": True,
                        "welder_ready": True, "safety_gate_closed": True},
        required_inputs=["part_present"],
    )
    d = sl.to_dict()
    for k in ("color", "is_blinking", "label_zh", "label_en",
              "reason_zh", "reason_en"):
        assert k in d


# ---------------------------------------------------------------------------
# Cabinet mediation — virtual I/O API input update + session-only no write-back
# ---------------------------------------------------------------------------

def test_cabinet_set_input_updates_in_memory() -> None:
    cab = _make_cabinet()
    cab.set_input("fixture_clamped", False)
    s = cab.get_status()
    assert s.virtual_inputs["fixture_clamped"] is False
    # Non-safety readiness input → Yellow (not Red), even though gate closed
    assert s.status_light.color == "yellow"
    assert s.status_light.reason_en == "Fixture Not Clamped"


def test_cabinet_set_input_does_not_write_back_to_seed() -> None:
    cab = _make_cabinet()
    before = json.loads(IO_SEED.read_text())
    cab.set_input("welder_ready", False)
    after = json.loads(IO_SEED.read_text())
    assert before == after, "seed file on disk must NOT change"


def test_cabinet_set_output_does_not_write_back_to_seed() -> None:
    cab = _make_cabinet()
    before = json.loads(IO_SEED.read_text())
    cab.io.set_output("stack_light_red", True)
    after = json.loads(IO_SEED.read_text())
    assert before == after


def test_cabinet_set_input_unknown_signal_rejected() -> None:
    cab = _make_cabinet()
    with pytest.raises(JobValidationError):
        cab.set_input("foo", True)


def test_cabinet_set_input_non_bool_rejected() -> None:
    cab = _make_cabinet()
    with pytest.raises(ValueError):
        cab.set_input("part_present", 1)


# ---------------------------------------------------------------------------
# Safety gate backend motion blocking
# ---------------------------------------------------------------------------

def test_motion_blocked_in_estop() -> None:
    cab = _make_cabinet()
    cab.trigger_estop()
    assert motion_blocked(cab.state) is True
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("home")


def test_motion_blocked_in_fault() -> None:
    cab = _make_cabinet()
    cab.trigger_fault("TEST_FAULT", "test")
    assert cab.state == CabinetState.FAULT
    assert motion_blocked(cab.state) is True
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("home")


def test_motion_blocked_in_protective_stop() -> None:
    cab = _make_cabinet()
    cab.trigger_protective_stop()
    assert cab.state == CabinetState.PROTECTIVE_STOP
    assert motion_blocked(cab.state) is True
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("home")


def test_motion_allowed_in_ready() -> None:
    cab = _make_cabinet()
    assert cab.state == CabinetState.READY
    assert motion_blocked(cab.state) is False
    cab.move_to("home")
    assert True  # no exception


# ---------------------------------------------------------------------------
# Safety gate enforcement — gate open MUST trigger PROTECTIVE_STOP and
# reject motion + job start. (Phase 1C final spec §2, 2026-09-16.)
# ---------------------------------------------------------------------------

def test_safety_gate_open_rejects_motion_with_protective_stop() -> None:
    """Opening the safety gate during READY → cabinet auto-transitions to
    PROTECTIVE_STOP and refuses every motion command."""
    cab = _make_cabinet()
    assert cab.state == CabinetState.READY
    cab.set_input("safety_gate_closed", False)
    # Move the arm — this triggers _require_motion_allowed_locked which
    # auto-transitions to PROTECTIVE_STOP and rejects.
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("home")
    assert cab.state == CabinetState.PROTECTIVE_STOP
    s = cab.get_status()
    # Status light must be Red with "Virtual Safety Gate Open" reason.
    assert s.status_light.color == "red"
    assert s.status_light.reason_en == "Virtual Safety Gate Open"
    assert s.status_light.reason_zh == "虛擬安全門未關閉"


def test_safety_gate_open_rejects_pose_a_and_pose_b() -> None:
    cab = _make_cabinet()
    cab.set_input("safety_gate_closed", False)
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("pose_a")
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("pose_b")
    with pytest.raises(VirtualSafetyStopError):
        cab.move_home()
    assert cab.state == CabinetState.PROTECTIVE_STOP


def test_safety_gate_open_rejects_set_joint_targets() -> None:
    cab = _make_cabinet()
    cab.set_input("safety_gate_closed", False)
    with pytest.raises(VirtualSafetyStopError):
        cab.set_joint_targets([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert cab.state == CabinetState.PROTECTIVE_STOP


def test_safety_gate_open_rejects_start_job() -> None:
    """start_job() must auto-PROTECTIVE_STOP + refuse when gate is open."""
    cab = _make_cabinet()
    cab.set_input("safety_gate_closed", False)
    with pytest.raises(VirtualSafetyStopError):
        cab.start_job()
    assert cab.state == CabinetState.PROTECTIVE_STOP
    s = cab.get_status()
    assert s.active_job_id is None
    assert s.status_light.color == "red"
    assert s.status_light.reason_en == "Virtual Safety Gate Open"


def test_safety_gate_open_during_running_triggers_protective_stop() -> None:
    """If the gate opens WHILE a job is RUNNING, the cabinet must
    auto-PROTECTIVE_STOP (via the motion-allowed check) and refuse further
    motion. The job is cancelled."""
    cab = _make_cabinet()  # no tick — job will hit timeout but that's OK
    cab.start_job()
    assert cab.state == CabinetState.RUNNING
    cab.set_input("safety_gate_closed", False)
    # Next motion attempt trips the safety auto-stop.
    with pytest.raises(VirtualSafetyStopError):
        cab.move_to("home")
    assert cab.state == CabinetState.PROTECTIVE_STOP
    s = cab.get_status()
    assert s.status_light.color == "red"
    assert s.status_light.reason_en == "Virtual Safety Gate Open"


def test_close_gate_and_reset_restores_ready() -> None:
    cab = _make_cabinet()
    cab.set_input("safety_gate_closed", False)
    with pytest.raises(VirtualSafetyStopError):
        cab.start_job()
    assert cab.state == CabinetState.PROTECTIVE_STOP
    # Close the gate, reset the protective stop, and the cabinet is back
    # to READY (with all readiness inputs satisfied → Green Ready).
    cab.set_input("safety_gate_closed", True)
    cab.reset_protective_stop()
    assert cab.state == CabinetState.READY
    s = cab.get_status()
    assert s.status_light.color == "green"
    assert s.status_light.label_en == "Virtual System Ready"


# ---------------------------------------------------------------------------
# RUNNING status_light integration — backend reflects job + step context.
# ---------------------------------------------------------------------------

def test_running_status_light_reflects_job_and_step(
        cabinet_with_tick) -> None:
    """While the job is RUNNING, status_light must show
    "Virtual Cycle Running" with job_id and step / total — never
    "Virtual System Ready"."""
    cab = cabinet_with_tick
    cab.start_job()
    # Poll quickly — we only need to see at least one RUNNING snapshot.
    seen_running = False
    deadline = time.time() + 5
    while time.time() < deadline:
        s = cab.get_status()
        if s.cabinet_state == CabinetState.RUNNING:
            seen_running = True
            assert s.status_light.color == "green"
            assert s.status_light.label_en == "Virtual Cycle Running"
            assert s.status_light.label_zh == "虛擬工序執行中"
            assert "System Ready" not in s.status_light.label_en
            assert "虛擬系統就緒" not in s.status_light.label_zh
            assert "DEMO_SEQUENCE_001" in s.status_light.reason_en
            assert "DEMO_SEQUENCE_001" in s.status_light.reason_zh
            # Step/total present (1-based display)
            assert "/ 4" in s.status_light.reason_en
            assert "/ 4" in s.status_light.reason_zh
            break
        time.sleep(0.05)
    assert seen_running, "never observed RUNNING state within 5s"


def test_job_start_blocked_when_estopped() -> None:
    cab = _make_cabinet()
    cab.trigger_estop()
    assert job_start_allowed(cab.state) is False
    with pytest.raises(CabinetStateError):
        cab.start_job()


def test_job_start_blocked_when_fault() -> None:
    cab = _make_cabinet()
    cab.trigger_fault("X", "x")
    with pytest.raises(CabinetStateError):
        cab.start_job()


# ---------------------------------------------------------------------------
# E-stop / protective stop / fault job cancellation
# ---------------------------------------------------------------------------

def test_estop_cancels_running_job(cabinet_with_tick) -> None:
    """Start a job, immediately trigger E-stop — runner must be cancelled."""
    cab = cabinet_with_tick
    cab.start_job()
    assert cab.state == CabinetState.RUNNING
    assert cab.runner is not None
    cab.trigger_estop()
    assert cab.state == CabinetState.ESTOP
    # give the runner thread a moment to wind down
    time.sleep(0.05)
    assert cab.runner is None or cab.runner.is_running is False


def test_fault_cancels_running_job(cabinet_with_tick) -> None:
    cab = cabinet_with_tick
    cab.start_job()
    cab.trigger_fault("X", "x")
    assert cab.state == CabinetState.FAULT
    time.sleep(0.05)
    assert cab.runner is None or cab.runner.is_running is False


def test_protective_stop_cancels_running_job(cabinet_with_tick) -> None:
    cab = cabinet_with_tick
    cab.start_job()
    cab.trigger_protective_stop()
    assert cab.state == CabinetState.PROTECTIVE_STOP
    time.sleep(0.05)
    assert cab.runner is None or cab.runner.is_running is False


# ---------------------------------------------------------------------------
# Reset does NOT auto-restart
# ---------------------------------------------------------------------------

def test_reset_estop_does_not_restart_job(cabinet_with_tick) -> None:
    cab = cabinet_with_tick
    cab.start_job()
    cab.trigger_estop()
    cab.reset_estop()
    assert cab.state == CabinetState.READY
    assert cab.runner is None or cab.runner.is_running is False
    s = cab.get_status()
    assert s.active_job_id is None


def test_reset_fault_does_not_restart_job(cabinet_with_tick) -> None:
    cab = cabinet_with_tick
    cab.start_job()
    cab.trigger_fault("X", "x")
    cab.reset_fault()
    assert cab.state == CabinetState.READY
    s = cab.get_status()
    assert s.active_job_id is None
    assert s.fault_code is None
    assert s.fault_message is None


def test_reset_protective_stop_does_not_restart_job(cabinet_with_tick) -> None:
    cab = cabinet_with_tick
    cab.start_job()
    cab.trigger_protective_stop()
    cab.reset_protective_stop()
    assert cab.state == CabinetState.READY
    s = cab.get_status()
    assert s.active_job_id is None


# ---------------------------------------------------------------------------
# Demo sequence: Home -> Pose A -> Pose B -> Home
# ---------------------------------------------------------------------------

def test_demo_sequence_steps_match_spec() -> None:
    """Spec: step_01=home, step_02=pose_a, step_03=pose_b, step_04=home."""
    spec = json.loads(JOB_SPEC.read_text())
    steps = spec["steps"]
    actions = [(s["step_id"], s["action"], s.get("pose")) for s in steps]
    assert actions == [
        ("step_01", "move_pose", "home"),
        ("step_02", "move_pose", "pose_a"),
        ("step_03", "move_pose", "pose_b"),
        ("step_04", "move_pose", "home"),
    ]


def test_home_pose_is_all_zero() -> None:
    """Single source of truth for Home pose."""
    home = json.loads((POSES / "home.json").read_text())
    assert home["joints_rad"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_pose_a_b_distinct_from_home() -> None:
    home = json.loads((POSES / "home.json").read_text())
    pose_a = json.loads((POSES / "pose_a.json").read_text())
    pose_b = json.loads((POSES / "pose_b.json").read_text())
    assert home["joints_rad"] != pose_a["joints_rad"]
    assert home["joints_rad"] != pose_b["joints_rad"]
    assert pose_a["joints_rad"] != pose_b["joints_rad"]


def test_no_pose_c_file_exists() -> None:
    """Sanity: there must NOT be a pose_c.json on disk."""
    assert not (POSES / "pose_c.json").exists()


def test_demo_sequence_runs_to_completion(cabinet_with_tick) -> None:
    """End-to-end: start job, wait for completion, verify back at home.

    We crank down motion_timeout_s by using a longer timeout for stability,
    but we trust the tick thread to actually move the arm. With WebRobot at
    max_step_rad=0.05 and a 60Hz tick, each pose transition takes ~1s.
    """
    cab = cabinet_with_tick
    jid = cab.start_job()
    assert jid == "DEMO_SEQUENCE_001"
    deadline = time.time() + 30
    while time.time() < deadline:
        prog = cab.current_job_progress()
        if prog and prog.get("completed") is True:
            break
        time.sleep(0.1)
    final = cab.current_job_progress()
    assert final is not None
    assert final.get("completed") is True
    assert final.get("current_step_index") == final.get("total_steps")
    assert cab.state == CabinetState.READY
    # The robot's joint targets should now equal the home pose (last step
    # is step_04=home). Read directly from the robot.
    home_targets = [0.0] * 6
    actual_targets = list(cab.robot.joint_targets)
    assert all(abs(a - h) < 1e-6
               for a, h in zip(actual_targets, home_targets))


# ---------------------------------------------------------------------------
# Race condition: stale callbacks from a cancelled/superseded runner must
# NOT mutate the cabinet state after reset.
# ---------------------------------------------------------------------------

def test_stale_callback_after_cancel_is_dropped(cabinet_with_tick) -> None:
    """After trigger_estop cancels the runner, any late callback from that
    runner must be dropped — the cabinet state must remain READY after reset.
    """
    cab = cabinet_with_tick
    cab.start_job()
    rid_before = cab._runner_run_id
    gen_before = cab._runner_generation
    assert rid_before is not None
    cab.trigger_estop()
    cab.reset_estop()
    # Cabinet should be READY with NO runner / NO identity
    assert cab.state == CabinetState.READY
    assert cab._runner is None
    assert cab._runner_run_id is None
    assert cab._runner_generation is None
    assert cab._fault_code is None
    assert cab._fault_message is None
    # Now simulate a stale callback from the cancelled runner — cabinet must
    # reject it (no exception, state unchanged, no runner reinstalled).
    cab._on_runner_complete(rid_before, gen_before)
    assert cab.state == CabinetState.READY
    assert cab._runner is None
    # And a stale fault callback from a never-installed runner:
    cab._on_runner_fault("VIRTUAL_MOTION_TIMEOUT", "stale",
                         99999, 99999)
    assert cab.state == CabinetState.READY
    assert cab._fault_code is None


def test_stale_callback_after_reset_estop_does_not_re_overwrite_fault(
        cabinet_with_tick) -> None:
    """Specifically test the VIRTUAL_MOTION_TIMEOUT race: after fault
    + reset_fault, a late timeout from the dead runner must NOT restore
    the fault_code."""
    cab = cabinet_with_tick
    cab.start_job()
    rid_before = cab._runner_run_id
    gen_before = cab._runner_generation
    cab.trigger_fault("X", "x")
    assert cab.state == CabinetState.FAULT
    assert cab._fault_code == "X"
    cab.reset_fault()
    assert cab.state == CabinetState.READY
    assert cab._fault_code is None
    # Simulate late timeout callback from the now-dead runner:
    cab._on_runner_fault("VIRTUAL_MOTION_TIMEOUT", "stale late timeout",
                         rid_before, gen_before)
    # The cabinet state must NOT have re-entered FAULT, and the fault_code
    # must NOT have been overwritten with the stale code.
    assert cab.state == CabinetState.READY
    assert cab._fault_code is None


def test_completed_progress_preserved_after_natural_completion(
        cabinet_with_tick) -> None:
    """After natural completion, the final progress (including completed=True)
    must still be queryable via current_job_progress()."""
    cab = cabinet_with_tick
    cab.start_job()
    deadline = time.time() + 30
    while time.time() < deadline:
        prog = cab.current_job_progress()
        if prog and prog.get("completed") is True:
            break
        time.sleep(0.1)
    final = cab.current_job_progress()
    assert final is not None
    assert final["completed"] is True
    assert final["total_steps"] == 4
    # Active job ID is gone, but the progress snapshot lives on:
    assert cab.state == CabinetState.READY
    s = cab.get_status()
    assert s.active_job_id is None


# ---------------------------------------------------------------------------
# /api/v1/status complete payload
# ---------------------------------------------------------------------------

@pytest.fixture
def flask_client():
    """Spin up a Flask test client for /api/v1/* routes. Function-scope so
    tests don't share mutable state (the cabinet is a singleton inside the
    module)."""
    # Import the app module — this instantiates CABINET, ROBOT, etc.
    from web import app as flask_app_module
    flask_app_module.app.config["TESTING"] = True
    flask_app_module._ensure_tick_thread()
    cab = flask_app_module.CABINET
    # Try each safety reset so we leave the cabinet in READY.
    for reset_fn_name in ("reset_estop", "reset_fault",
                            "reset_protective_stop"):
        try:
            getattr(cab, reset_fn_name)()
        except Exception:
            pass
    # Restore IO inputs to seed defaults (all True for the demo seed).
    for sig in ("part_present", "fixture_clamped",
                "welder_ready", "safety_gate_closed"):
        try:
            cab.set_input(sig, True)
        except Exception:
            pass
    with flask_app_module.app.test_client() as c:
        yield c
        # Teardown: clear any active job and reset all safety states.
        try:
            cab.reset_estop()
            cab.reset_fault()
            cab.reset_protective_stop()
            for sig in ("part_present", "fixture_clamped",
                        "welder_ready", "safety_gate_closed"):
                cab.set_input(sig, True)
        except Exception:
            pass


def test_api_health_ok(flask_client) -> None:
    r = flask_client.get("/api/health")
    assert r.status_code == 200
    j = r.get_json()
    assert j["ok"] is True


def test_api_v1_status_complete_payload(flask_client) -> None:
    r = flask_client.get("/api/v1/status")
    assert r.status_code == 200
    j = r.get_json()
    assert j["ok"] is True
    s = j["status"]
    # Required fields per brief §7.1
    for f in ("mode", "cabinet_type", "cabinet_id", "cabinet_state",
              "controller_connection", "real_robot_control",
              "motion_state", "safety_state",
              "active_job_id", "current_step_index", "total_steps",
              "fault", "virtual_inputs", "virtual_outputs",
              "status_light", "updated_at",
              "joint_positions_deg", "target_positions_deg",
              "tcp_estimate_m"):
        assert f in s, f"missing field: {f}"
    # status_light sub-payload complete
    sl = s["status_light"]
    for f in ("color", "is_blinking", "label_zh", "label_en",
              "reason_zh", "reason_en"):
        assert f in sl, f"missing status_light.{f}"
    # Brand-neutral defaults
    assert s["mode"] == "virtual_only"
    assert s["cabinet_type"] == "virtual_controller_cabinet"
    assert s["real_robot_control"] == "disabled"
    assert s["controller_connection"] == "not_configured"


def test_api_v1_status_light_standalone(flask_client) -> None:
    r = flask_client.get("/api/v1/status-light")
    assert r.status_code == 200
    j = r.get_json()
    assert "status_light" in j
    assert "cabinet_state" in j
    sl = j["status_light"]
    assert sl["color"] in {"gray", "green", "yellow", "red"}


def test_api_v1_cabinet_status_contains_status_light(flask_client) -> None:
    r = flask_client.get("/api/v1/cabinet/status")
    assert r.status_code == 200
    j = r.get_json()
    assert "status_light" in j["status"]


def test_api_input_update_via_endpoint(flask_client) -> None:
    r = flask_client.post("/api/v1/virtual-io/inputs/safety_gate_closed",
                          json={"value": False})
    assert r.status_code == 200
    # Safety gate is a SAFETY input → triggers PROTECTIVE_STOP (Red, "Virtual
    # Safety Gate Open"), not Yellow. Yellow is reserved for non-safety
    # readiness inputs (part/fixture/welder).
    r2 = flask_client.get("/api/v1/status-light")
    sl = r2.get_json()["status_light"]
    assert sl["color"] == "red"
    assert sl["reason_en"] == "Virtual Safety Gate Open"
    # Restore — but first reset the protective stop so the cabinet returns
    # to READY before we flip the input back.
    flask_client.post("/api/v1/cabinet/reset-protective-stop")
    flask_client.post("/api/v1/virtual-io/inputs/safety_gate_closed",
                      json={"value": True})


def test_api_input_non_bool_rejected(flask_client) -> None:
    r = flask_client.post("/api/v1/virtual-io/inputs/part_present",
                          json={"value": "true"})
    assert r.status_code == 400


def test_api_input_unknown_signal_rejected(flask_client) -> None:
    r = flask_client.post("/api/v1/virtual-io/inputs/bogus_signal",
                          json={"value": True})
    assert r.status_code == 400


def test_api_no_write_endpoint_for_outputs(flask_client) -> None:
    """Outputs are read-only from the API surface — only inputs have a
    POST endpoint. Sanity-check that there's no /api/v1/virtual-io/outputs/*
    that mutates state."""
    # Try POSTing to a fictitious outputs endpoint: should 404.
    r = flask_client.post("/api/v1/virtual-io/outputs/stack_light_red",
                          json={"value": True})
    assert r.status_code == 404


# ---------------------------------------------------------------------------
# Static import scan — no socket / requests / vendor SDK
# ---------------------------------------------------------------------------

BANNED_IMPORT_RE = re.compile(
    r"^\s*(?:from|import)\s+(socket|requests|urllib(?:\.\w+)?|http\.client|"
    r"serial|pymodbus|opcua|urx|rokae|jaka|asyncua|pycomm3)\b",
    re.MULTILINE,
)


def _scan_source_dir(rel: str) -> list[tuple[Path, int, str]]:
    """Return (path, lineno, line) for any banned import in dir."""
    root = PROJECT_ROOT / rel
    hits: list[tuple[Path, int, str]] = []
    for p in root.rglob("*.py"):
        if "__pycache__" in p.parts:
            continue
        try:
            text = p.read_text(encoding="utf-8")
        except Exception:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            if BANNED_IMPORT_RE.search(line):
                hits.append((p.relative_to(PROJECT_ROOT), i, line.strip()))
    return hits


def test_no_banned_imports_robotarm() -> None:
    hits = _scan_source_dir("robotarm")
    assert not hits, f"banned imports in robotarm/: {hits}"


def test_no_banned_imports_web() -> None:
    hits = _scan_source_dir("web")
    assert not hits, f"banned imports in web/: {hits}"


def test_no_banned_imports_tests() -> None:
    hits = _scan_source_dir("tests")
    assert not hits, f"banned imports in tests/: {hits}"


# ---------------------------------------------------------------------------
# Outbound network at runtime — connect() must reject non-loopback
# ---------------------------------------------------------------------------

def test_webrobot_connect_rejects_non_loopback() -> None:
    """The demo robot refuses any non-loopback target."""
    r = WebRobot(robot_id="test_conn")
    with pytest.raises(Exception):
        r.connect(target="192.168.1.100:30004")
