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
    """WAITING cabinet + missing non-safety readiness input -> Yellow.

    Updated 2026-09-16: WAITING is now a real CabinetState. The
    status_light logic for "yellow because readiness is missing" is
    driven by ``cabinet_state == WAITING``, not by inspecting the
    required_inputs while READY. The cabinet's
    ``_re_evaluate_idle_readiness_locked`` is the single source of
    truth for the READY <-> WAITING flip.
    """
    sl = compute_status_light(
        cabinet_state=CabinetState.WAITING,
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
    """``fixture_clamped=False``, ``safety_gate_closed=True``, WAITING →
    Yellow, reason="Fixture Not Clamped" (NOT safety-gate).

    Updated 2026-09-16: cabinet state is now WAITING (not READY) when
    non-safety readiness is missing. The safety gate remains closed and
    is intentionally not a Yellow reason.
    """
    sl = compute_status_light(
        cabinet_state=CabinetState.WAITING,
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
        cabinet_state=CabinetState.WAITING,
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
        cabinet_state=CabinetState.WAITING,
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

# ===========================================================================
# 2026-09-16 Bug-01: WAITING state synchronization
# ===========================================================================
#
# Bug spec: when a non-safety readiness input (part_present /
# fixture_clamped / welder_ready) is False and no E-stop / Fault /
# Protective Stop is active, the cabinet state MUST be WAITING (not
# READY) and the status light MUST be yellow. Motion / job start must
# be blocked in WAITING. Safety priority ESTOP > FAULT/PROT > WAITING
# > READY must be preserved at all times.
#
# These tests do NOT touch the real Flask server. They exercise the
# VirtualControllerCabinet directly with the project seed files.


# ---------------------------------------------------------------------------
# Bug-01 — WAITING state from non-safety readiness changes
# ---------------------------------------------------------------------------

def test_workpiece_absent_sets_waiting_state(cabinet_with_tick) -> None:
    """part_present=False -> cabinet state = WAITING, status yellow,
    cycle_running false, motion blocked."""
    cab = cabinet_with_tick
    assert cab.state == CabinetState.READY
    cab.set_input("part_present", False)
    assert cab.state == CabinetState.WAITING
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.WAITING
    assert s.status_light.color == "yellow"
    assert s.virtual_outputs.get("cycle_running") is False
    # motion must be blocked
    with pytest.raises(VirtualSafetyStopError):
        cab.move_home()
    # job start must be blocked (VirtualInterlockError per D2)
    with pytest.raises(VirtualInterlockError):
        cab.start_job()


def test_fixture_not_clamped_sets_waiting_state(cabinet_with_tick) -> None:
    """fixture_clamped=False -> cabinet state = WAITING."""
    cab = cabinet_with_tick
    cab.set_input("fixture_clamped", False)
    assert cab.state == CabinetState.WAITING
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.WAITING
    assert s.status_light.color == "yellow"
    assert s.status_light.reason_en == "Fixture Not Clamped"
    assert s.status_light.reason_zh == "治具未夾緊"
    with pytest.raises(VirtualSafetyStopError):
        cab.move_home()
    with pytest.raises(VirtualInterlockError):
        cab.start_job()


def test_welder_not_ready_sets_waiting_state(cabinet_with_tick) -> None:
    """welder_ready=False -> cabinet state = WAITING."""
    cab = cabinet_with_tick
    cab.set_input("welder_ready", False)
    assert cab.state == CabinetState.WAITING
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.WAITING
    assert s.status_light.color == "yellow"
    assert s.status_light.reason_en == "Welder Not Ready"
    with pytest.raises(VirtualSafetyStopError):
        cab.move_home()
    with pytest.raises(VirtualInterlockError):
        cab.start_job()


def test_waiting_state_blocks_demo_job_or_motion(cabinet_with_tick) -> None:
    """WAITING -> start_job raises VirtualInterlockError with missing
    inputs list; move_home / set_targets raise VirtualSafetyStopError."""
    cab = cabinet_with_tick
    cab.set_input("fixture_clamped", False)
    cab.set_input("welder_ready", False)
    assert cab.state == CabinetState.WAITING
    # start_job must raise VirtualInterlockError, not CabinetStateError.
    with pytest.raises(VirtualInterlockError) as ei:
        cab.start_job()
    missing = ei.value.missing
    assert "fixture_clamped" in missing
    assert "welder_ready" in missing
    # move_home must raise VirtualSafetyStopError naming the missing input.
    with pytest.raises(VirtualSafetyStopError) as em:
        cab.move_home()
    assert "waiting for" in str(em.value).lower()
    # set_joint_targets must also be blocked.
    with pytest.raises(VirtualSafetyStopError):
        cab.set_joint_targets([0.0] * 6)


def test_all_inputs_restored_returns_to_ready(cabinet_with_tick) -> None:
    """Three non-safety readiness inputs all True -> WAITING flips back
    to READY (single source of truth: cabinet's re-eval helper)."""
    cab = cabinet_with_tick
    cab.set_input("part_present", False)
    assert cab.state == CabinetState.WAITING
    cab.set_input("part_present", True)
    assert cab.state == CabinetState.READY
    # restore order does not matter
    cab.set_input("fixture_clamped", False)
    cab.set_input("welder_ready", False)
    assert cab.state == CabinetState.WAITING
    cab.set_input("fixture_clamped", True)
    # still waiting because welder_ready still missing
    assert cab.state == CabinetState.WAITING
    cab.set_input("welder_ready", True)
    assert cab.state == CabinetState.READY


# ---------------------------------------------------------------------------
# Bug-01 — Safety priority must beat WAITING
# ---------------------------------------------------------------------------

def test_estop_beats_waiting(cabinet_with_tick) -> None:
    """WAITING + trigger_estop -> ESTOP (priority over WAITING)."""
    cab = cabinet_with_tick
    cab.set_input("fixture_clamped", False)
    assert cab.state == CabinetState.WAITING
    cab.trigger_estop()
    assert cab.state == CabinetState.ESTOP
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.ESTOP
    assert s.status_light.color == "red"
    assert s.status_light.is_blinking is True


def test_fault_beats_waiting(cabinet_with_tick) -> None:
    """WAITING + trigger_fault -> FAULT."""
    cab = cabinet_with_tick
    cab.set_input("welder_ready", False)
    assert cab.state == CabinetState.WAITING
    cab.trigger_fault("VIRTUAL_FAULT", "test")
    assert cab.state == CabinetState.FAULT
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.FAULT
    assert s.status_light.color == "red"
    assert s.status_light.is_blinking is False


def test_safety_gate_open_beats_waiting(cabinet_with_tick) -> None:
    """WAITING + safety_gate_closed=False -> PROTECTIVE_STOP (gate is a
    SAFETY input and must NEVER produce WAITING)."""
    cab = cabinet_with_tick
    cab.set_input("welder_ready", False)
    assert cab.state == CabinetState.WAITING
    cab.set_input("safety_gate_closed", False)
    assert cab.state == CabinetState.PROTECTIVE_STOP
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.PROTECTIVE_STOP
    assert s.status_light.color == "red"
    assert s.status_light.reason_en == "Virtual Safety Gate Open"


def test_protective_stop_requires_explicit_reset(cabinet_with_tick) -> None:
    """WAITING -> PROTECTIVE_STOP (gate open) -> close gate -> cabinet
    stays in PROTECTIVE_STOP (NOT auto-reset)."""
    cab = cabinet_with_tick
    cab.set_input("welder_ready", False)
    cab.set_input("safety_gate_closed", False)
    assert cab.state == CabinetState.PROTECTIVE_STOP
    cab.set_input("safety_gate_closed", True)
    # Cabinet must NOT auto-reset.
    assert cab.state == CabinetState.PROTECTIVE_STOP
    # Operator must press reset_protective_stop.
    cab.reset_protective_stop()
    # After reset: re-eval checks readiness — welder_ready is still off,
    # so cabinet should land on WAITING, not READY (per D3).
    assert cab.state == CabinetState.WAITING


# ---------------------------------------------------------------------------
# Bug-01 — D3: safety resets re-evaluate readiness
# ---------------------------------------------------------------------------

def test_reset_estop_re_evaluates_waiting_readiness(cabinet_with_tick) -> None:
    """READY -> WAITING (fixture off) -> ESTOP -> reset_estop.
    Re-eval must land on WAITING (fixture still off), not READY."""
    cab = cabinet_with_tick
    cab.set_input("fixture_clamped", False)
    assert cab.state == CabinetState.WAITING
    cab.trigger_estop()
    assert cab.state == CabinetState.ESTOP
    cab.reset_estop()
    # Re-eval: fixture still off -> WAITING (NOT READY).
    assert cab.state == CabinetState.WAITING
    # Now restore fixture -> cabinet should flip back to READY.
    cab.set_input("fixture_clamped", True)
    assert cab.state == CabinetState.READY


def test_reset_fault_re_evaluates_waiting_readiness(cabinet_with_tick) -> None:
    """READY -> WAITING (welder off) -> FAULT -> reset_fault.
    Re-eval must land on WAITING, not READY."""
    cab = cabinet_with_tick
    cab.set_input("welder_ready", False)
    assert cab.state == CabinetState.WAITING
    cab.trigger_fault("VIRTUAL_FAULT", "test")
    assert cab.state == CabinetState.FAULT
    cab.reset_fault()
    # Re-eval: welder still off -> WAITING.
    assert cab.state == CabinetState.WAITING
    cab.set_input("welder_ready", True)
    assert cab.state == CabinetState.READY


def test_reset_protective_stop_re_evaluates_waiting_readiness(
        cabinet_with_tick) -> None:
    """READY -> WAITING (part off) -> trigger_protective_stop ->
    reset_protective_stop -> re-eval lands on WAITING (part still off)."""
    cab = cabinet_with_tick
    cab.set_input("part_present", False)
    assert cab.state == CabinetState.WAITING
    cab.trigger_protective_stop()
    assert cab.state == CabinetState.PROTECTIVE_STOP
    cab.reset_protective_stop()
    # Re-eval: part still off -> WAITING (NOT READY).
    assert cab.state == CabinetState.WAITING
    cab.set_input("part_present", True)
    assert cab.state == CabinetState.READY


def test_reset_estop_when_all_ready_returns_to_ready(cabinet_with_tick) -> None:
    """If readiness is fully restored by the time we reset, cabinet
    returns to READY (not WAITING)."""
    cab = cabinet_with_tick
    cab.set_input("fixture_clamped", False)
    assert cab.state == CabinetState.WAITING
    cab.trigger_estop()
    assert cab.state == CabinetState.ESTOP
    # Restore readiness while in ESTOP.
    cab.set_input("fixture_clamped", True)
    # Still ESTOP — readiness re-eval does not happen during safety state.
    assert cab.state == CabinetState.ESTOP
    cab.reset_estop()
    # Now re-eval runs — readiness is satisfied -> READY.
    assert cab.state == CabinetState.READY


# ---------------------------------------------------------------------------
# Bug-01 — WAITING <-> status-light single source of truth
# ---------------------------------------------------------------------------

def test_status_light_does_not_drive_cabinet_state(cabinet_with_tick) -> None:
    """Regression: status_light is purely a function of cabinet_state.
    It must NEVER mutate cabinet state. Setting fixture_clamped=False
    transitions cabinet state directly; status light just renders."""
    cab = cabinet_with_tick
    cab.set_input("fixture_clamped", False)
    # State went to WAITING because the cabinet's re-eval helper
    # decided so — NOT because the status light told it to.
    assert cab.state == CabinetState.WAITING
    # If we manually set the state back to READY while the input is
    # still missing, the next set_input call should snap it back to
    # WAITING — confirming the re-eval helper is the only source of
    # truth for the flip.
    cab._state = CabinetState.READY  # direct mutation, no helper
    cab.set_input("part_present", True)  # any non-safety input triggers re-eval
    assert cab.state == CabinetState.WAITING


# ===========================================================================
# 2026-09-16 Bug-02: Demo Job completion result retention
# ===========================================================================
#
# Bug spec: after a demo job completes naturally, the UI must continue to
# show the job's last result (job_id, progress, current_step, result,
# elapsed) instead of clearing it to "—". The cabinet must end up in
# READY (or WAITING if readiness is now missing), cycle_running must be
# False, and a successful subsequent job must replace the prior result.
# A rejected new job must NOT clear the prior terminal result.


def _wait_for_completion(cab, timeout_s: float = 30.0) -> dict:
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        last = cab.current_job_progress()
        if last and last.get("completed") is True:
            return last
        time.sleep(0.05)
    raise AssertionError(f"Job did not complete within {timeout_s}s; last={last}")


def test_completed_demo_job_retains_last_result(cabinet_with_tick) -> None:
    """After natural completion, CabinetStatus.last_job_result is set
    with the right schema (job_id, completed, result, success, etc.)."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    s = cab.get_status()
    assert s.last_job_result is not None
    r = s.last_job_result
    assert r["job_id"] == "DEMO_SEQUENCE_001"
    assert r["completed"] is True
    assert r["result"] == "completed"
    assert r["success"] is True
    assert r["current_step"] == "Completed"
    assert r["current_step_zh"] == "已完成"


def test_completed_demo_job_reports_final_progress(cabinet_with_tick) -> None:
    """Final progress is 4 / 4 (job has 4 steps)."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    r = cab.get_status().last_job_result
    assert r["total_steps"] == 4
    assert r["current_step_index"] == 4
    # elapsed_s must be present and >= 0
    assert isinstance(r["elapsed_s"], (int, float))
    assert r["elapsed_s"] >= 0
    assert isinstance(r["finished_at"], (int, float))


def test_completed_demo_job_returns_cabinet_to_ready_or_waiting(
        cabinet_with_tick) -> None:
    """After natural completion, cabinet state is READY (if readiness
    is still satisfied) or WAITING (if readiness became unsatisfied
    during the job). In this baseline case readiness stays satisfied."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    assert cab.state == CabinetState.READY
    s = cab.get_status()
    assert s.cabinet_state == CabinetState.READY


def test_completed_demo_job_sets_cycle_running_false(cabinet_with_tick) -> None:
    """After completion, virtual output cycle_running must be False."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    s = cab.get_status()
    assert s.virtual_outputs.get("cycle_running") is False
    assert s.virtual_outputs.get("stack_light_green") is True
    assert s.virtual_outputs.get("stack_light_red") is False


def test_starting_new_job_replaces_previous_terminal_result(
        cabinet_with_tick) -> None:
    """After the first job completes and a SECOND job starts successfully,
    the new active_job_id appears and last_job_result is cleared (because
    the UI now renders the active job)."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    assert cab.get_status().last_job_result is not None
    # Start a second job.
    cab.set_input("fixture_clamped", False)  # simulate readiness going missing then restored
    cab.set_input("fixture_clamped", True)
    jid = cab.start_job()
    assert jid == "DEMO_SEQUENCE_001"
    # While the new job is running, last_job_result must be cleared (UI
    # now shows active job, not terminal result).
    s = cab.get_status()
    assert s.last_job_result is None
    # Wait for it to complete to clean up.
    _wait_for_completion(cab)


def test_rejected_new_job_does_not_clear_previous_terminal_result(
        cabinet_with_tick) -> None:
    """If start_job() is rejected (e.g. WAITING or interlock fail), the
    previous terminal result must NOT be cleared."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    prior = cab.get_status().last_job_result
    assert prior is not None

    # Force cabinet into WAITING (by clearing a readiness input).
    cab.set_input("fixture_clamped", False)
    # start_job must be rejected with VirtualInterlockError.
    with pytest.raises(VirtualInterlockError):
        cab.start_job()
    # last_job_result must still be the previous terminal result.
    s = cab.get_status()
    assert s.last_job_result is not None
    assert s.last_job_result["job_id"] == prior["job_id"]
    assert s.last_job_result["completed"] is True
    # Also test the gate-open reject path.
    cab.set_input("fixture_clamped", True)
    assert cab.state == CabinetState.READY
    cab.set_input("safety_gate_closed", False)
    with pytest.raises(VirtualSafetyStopError):
        cab.start_job()
    s = cab.get_status()
    assert s.last_job_result is not None
    assert s.last_job_result["job_id"] == prior["job_id"]


def test_status_payload_exposes_last_job_result_after_completion(
        cabinet_with_tick) -> None:
    """After completion, /api/v1/status payload includes last_job_result
    with the documented schema."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    s = cab.get_status().to_status_payload()
    assert "last_job_result" in s
    r = s["last_job_result"]
    assert r is not None
    assert r["job_id"] == "DEMO_SEQUENCE_001"
    assert r["completed"] is True
    assert r["result"] == "completed"
    assert r["success"] is True
    assert r["current_step_index"] == 4
    assert r["total_steps"] == 4
    assert r["elapsed_s"] >= 0


def test_last_job_result_cancelled_on_protective_stop_during_run(
        cabinet_with_tick) -> None:
    """If a running job is cancelled by a protective stop, last_job_result
    must record a 'cancelled' result with success=False (NOT 'completed'
    / success=True)."""
    cab = cabinet_with_tick
    cab.start_job()
    # Wait for it to actually start moving.
    time.sleep(0.2)
    cab.trigger_protective_stop()
    # Wait for the runner thread to terminate and the cabinet callback
    # to fire.
    deadline = time.time() + 3.0
    while time.time() < deadline:
        r = cab.get_status().last_job_result
        if r is not None:
            break
        time.sleep(0.05)
    r = cab.get_status().last_job_result
    assert r is not None
    assert r["job_id"] == "DEMO_SEQUENCE_001"
    assert r["completed"] is True
    assert r["success"] is False
    # The result must be 'cancelled' (NOT 'completed'); it could also be
    # 'failed' if the cancel_reason happened to start with "fault:".
    assert r["result"] in ("cancelled", "failed")


def test_post_completion_readiness_loss_lands_in_waiting(
        cabinet_with_tick) -> None:
    """After a job completes, if a non-safety readiness input becomes
    False, the cabinet must transition to WAITING (not stay in READY)."""
    cab = cabinet_with_tick
    cab.start_job()
    _wait_for_completion(cab)
    assert cab.state == CabinetState.READY
    cab.set_input("part_present", False)
    # post-completion, re-eval must flip READY -> WAITING.
    assert cab.state == CabinetState.WAITING
    s = cab.get_status()
    assert s.last_job_result is not None
    assert s.last_job_result["success"] is True
