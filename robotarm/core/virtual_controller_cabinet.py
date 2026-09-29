"""VirtualControllerCabinet — the ONLY layer allowed to command the arm.

Phase 1C brief §2.1: "UI → API → RobotArm service → VirtualControllerCabinet
→ VirtualRobot".

Cabinet responsibilities:

* Hold the state machine (CabinetState + SafetyState + MotionState).
* Hold VirtualIO (session-only).
* Hold the active JobRunner (zero or one at a time).
* Gate every motion / IO write / job start behind state checks.
* Translate robot / job events back into cabinet state updates.

This module never imports socket, requests, etc. See ``safety_guard`` for
the static scan that enforces that.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from robotarm.core.cabinet_state import (
    CabinetState,
    MotionState,
    SafetyState,
    job_start_allowed,
    motion_blocked,
    next_state,
    safety_state_of,
)
from robotarm.core.job_runner import JobRunner, JobSpec
from robotarm.core.status_light import StatusLight, compute_status_light
from robotarm.core.virtual_io import VirtualIO
from robotarm.exceptions import (
    CabinetStateError,
    JobAlreadyRunningError,
    JobValidationError,
    VirtualFaultError,
    VirtualInterlockError,
    VirtualSafetyStopError,
)
from robotarm.utils.pose_loader import load_pose

LOG = logging.getLogger("robotarm.cabinet")


# ---------------------------------------------------------------------------
# CabinetStatus
# ---------------------------------------------------------------------------


#: Non-safety readiness inputs. A missing value here flips the cabinet
# between READY and WAITING (never ESTOP/PROTECTIVE_STOP). The safety
# gate is intentionally NOT in this list — gate-open is always
# PROTECTIVE_STOP, never WAITING.
_NON_SAFETY_READINESS_SIGNALS = frozenset({
    "part_present", "fixture_clamped", "welder_ready",
})


@dataclass
class CabinetStatus:
    cabinet_id: str
    mode: str                       # always "virtual_only"
    cabinet_type: str               # always "virtual_controller_cabinet"
    cabinet_state: CabinetState
    real_robot_control: str         # always "disabled"
    controller_connection: str      # always "not_configured"
    safety_state: SafetyState
    motion_state: MotionState
    active_job_id: str | None
    current_step_index: int | None  # 1-based for UI per brief §3.2
    total_steps: int
    current_step_id: str | None
    current_step_name: str | None
    fault_code: str | None
    fault_message: str | None
    virtual_inputs: dict[str, bool]
    virtual_outputs: dict[str, bool]
    status_light: StatusLight
    updated_at: float
    #: Last terminal job result. None if no job has ever completed, OR
    #: the last completed job has been replaced by a new active job.
    #: Schema:
    #:   {
    #:     "job_id": "DEMO_SEQUENCE_001",
    #:     "current_step_index": 4,
    #:     "total_steps": 4,
    #:     "current_step": "Completed" | "Cancelled" | "Failed: <code>",
    #:     "completed": true,
    #:     "result": "completed" | "cancelled" | "failed",
    #:     "success": true | false,
    #:     "elapsed_s": 1.234,
    #:     "finished_at": <unix timestamp>,
    #:   }
    last_job_result: dict[str, Any] | None = None

    def to_status_payload(self) -> dict[str, Any]:
        """Return the JSON status payload for /api/v1/status + /api/v1/cabinet/status."""
        return {
            "mode": self.mode,
            "cabinet_type": self.cabinet_type,
            "cabinet_id": self.cabinet_id,
            "cabinet_state": self.cabinet_state.value,
            "controller_connection": self.controller_connection,
            "real_robot_control": self.real_robot_control,
            "motion_state": self.motion_state.value,
            "safety_state": self.safety_state.value,
            "active_job_id": self.active_job_id,
            "current_step_index": self.current_step_index,
            "total_steps": self.total_steps,
            "current_step_id": self.current_step_id,
            "current_step_name": self.current_step_name,
            "fault": (
                {"code": self.fault_code, "message": self.fault_message}
                if self.fault_code else None
            ),
            "virtual_inputs": dict(self.virtual_inputs),
            "virtual_outputs": dict(self.virtual_outputs),
            "status_light": self.status_light.to_dict(),
            "updated_at": self.updated_at,
            "last_job_result": self.last_job_result,
        }


# ---------------------------------------------------------------------------
# VirtualControllerCabinet
# ---------------------------------------------------------------------------


class VirtualControllerCabinet:
    """Top-level mediator between UI/routes and VirtualRobot.

    Construction:

        cabinet = VirtualControllerCabinet(
            robot=web_robot,
            poses_dir=Path("data/poses"),
            io_seed_path=Path("data/virtual_io/default.json"),
            job_spec_path=Path("data/jobs/demo_sequence.json"),
        )

    Thread safety: a single ``threading.RLock`` guards all mutations. The
    Flask dev server is single-threaded by default, but JobRunner runs the
    job in a worker thread, so a lock is required.
    """

    def __init__(
        self,
        robot,
        *,
        cabinet_id: str = "virtual_cabinet_01",
        poses_dir: str | Path = "data/poses",
        io_seed_path: str | Path = "data/virtual_io/default.json",
        job_spec_path: str | Path = "data/jobs/demo_sequence.json",
        motion_timeout_s: float = 10.0,
    ) -> None:
        self.cabinet_id = cabinet_id
        self.robot = robot
        self.poses_dir = Path(poses_dir)
        self.io_seed_path = Path(io_seed_path)
        self.job_spec_path = Path(job_spec_path)
        self.motion_timeout_s = float(motion_timeout_s)

        # ---- state ----------------------------------------------------
        self._state: CabinetState = CabinetState.OFFLINE
        self._fault_code: str | None = None
        self._fault_message: str | None = None

        # ---- IO (session only; no disk writes) ------------------------
        # Raises FileNotFoundError / JobValidationError if seed is bad.
        self._io: VirtualIO = VirtualIO.from_seed_file(self.io_seed_path)

        # ---- Job runner (created on demand) ---------------------------
        self._runner: JobRunner | None = None
        self._runner_lock = threading.RLock()
        # Identity of the currently-installed runner. Callbacks from any
        # other runner (older or never-installed) are dropped — this is
        # the race-condition guard for "reset followed by stale timeout".
        self._runner_run_id: int | None = None
        self._runner_generation: int | None = None
        # Last progress snapshot from the runner — kept after termination
        # so callers can still inspect the final state.
        self._last_completed_progress: dict[str, Any] | None = None
        # Last TERMINAL job result exposed in CabinetStatus.last_job_result.
        # Schema is documented on the CabinetStatus dataclass above. Built
        # from ``_last_completed_progress`` at terminal-event time and from
        # the active runner's progress when applicable.
        self._last_job_result: dict[str, Any] | None = None

        # ---- Required inputs for status_light (cached from job spec) -
        try:
            from robotarm.core.job_runner import JobSpec as _JS
            _spec = _JS.from_json_file(self.job_spec_path)
            self._required_inputs: list[str] = list(_spec.required_inputs)
        except Exception:
            # If spec can't be loaded, status_light will just skip the
            # interlock check (returning green when not blocked).
            self._required_inputs = []

        # ---- Bring robot online and self-init ------------------------
        self.robot.connect()
        # After init we expect to be READY.
        self._state = next_state(self._state, "init")

    # ==================================================================
    # Read-only inspection
    # ==================================================================

    @property
    def state(self) -> CabinetState:
        return self._state

    @property
    def io(self) -> VirtualIO:
        return self._io

    @property
    def runner(self) -> JobRunner | None:
        return self._runner

    def get_status(
        self,
        *,
        joint_positions_rad: tuple[float, ...] | None = None,
        joint_targets_rad: tuple[float, ...] | None = None,
        tcp_position: tuple[float, float, float] | None = None,
    ) -> CabinetStatus:
        """Build a snapshot of the cabinet status + live robot info.

        ``joint_positions_rad``, ``joint_targets_rad``, ``tcp_position``
        are passed in by the Flask layer because the robot itself knows
        those (and they are mutated by the 60Hz tick thread outside this
        object's lock).
        """
        with self._runner_lock:
            active_id = self._runner.job.job_id if self._runner else None
            cur_step_idx = (
                self._runner.progress.current_step_index
                if self._runner else None
            )
            cur_step_id = (
                self._runner.progress.current_step_id
                if self._runner else None
            )
            cur_step_name = (
                self._runner.progress.current_step_name
                if self._runner else None
            )
            total = (
                self._runner.progress.total_steps if self._runner else 0
            )
            motion = self._derive_motion_state(
                active_id, joint_positions_rad, joint_targets_rad
            )
            required_inputs = self._load_required_inputs_locked()
            light = compute_status_light(
                cabinet_state=self._state,
                safety_state=safety_state_of(self._state),
                virtual_inputs=dict(self._io.inputs),
                required_inputs=required_inputs,
                motion_blocked_reason=self._fault_message,
                # Running-progress context so the RUNNING state can show
                # "Virtual Cycle Running" with job_id + step/total.
                active_job_id=active_id,
                current_step=(
                    (cur_step_idx + 1) if cur_step_idx is not None else None
                ),
                total_steps=total if total else None,
            )

        return CabinetStatus(
            cabinet_id=self.cabinet_id,
            mode="virtual_only",
            cabinet_type="virtual_controller_cabinet",
            cabinet_state=self._state,
            real_robot_control="disabled",
            controller_connection="not_configured",
            safety_state=safety_state_of(self._state),
            motion_state=motion,
            active_job_id=active_id,
            current_step_index=(cur_step_idx + 1) if cur_step_idx is not None else None,
            total_steps=total,
            current_step_id=cur_step_id,
            current_step_name=cur_step_name,
            fault_code=self._fault_code,
            fault_message=self._fault_message,
            virtual_inputs=dict(self._io.inputs),
            virtual_outputs=dict(self._io.outputs),
            status_light=light,
            updated_at=time.time(),
            last_job_result=(dict(self._last_job_result)
                             if self._last_job_result is not None else None),
        )

    def get_full_status_payload(
        self,
        *,
        joint_positions_rad: tuple[float, ...] | None = None,
        joint_targets_rad: tuple[float, ...] | None = None,
        tcp_position: tuple[float, float, float] | None = None,
    ) -> dict[str, Any]:
        """Combined payload for /api/v1/status including arm + joint info."""
        s = self.get_status(
            joint_positions_rad=joint_positions_rad,
            joint_targets_rad=joint_targets_rad,
            tcp_position=tcp_position,
        )
        d = s.to_status_payload()
        # Brief §7.1 additional fields
        d["joint_positions_deg"] = [
            round(j * 57.29577951308232, 3)
            for j in (joint_positions_rad or ())
        ]
        d["target_positions_deg"] = [
            round(j * 57.29577951308232, 3)
            for j in (joint_targets_rad or ())
        ]
        # Brief §7.1 names this `tcp_estimate_m` to make the approximation
        # explicit. None if not provided.
        d["tcp_estimate_m"] = (
            [round(v, 4) for v in tcp_position]
            if tcp_position is not None else None
        )
        return d

    def _derive_motion_state(
        self,
        active_job_id: str | None,
        joints: tuple[float, ...] | None,
        targets: tuple[float, ...] | None,
    ) -> MotionState:
        if motion_blocked(self._state):
            return MotionState.STOPPED
        if active_job_id is not None and joints and targets:
            return MotionState.MOVING if joints != targets else MotionState.IDLE
        # No active job — derive from delta if known
        if joints and targets and joints != targets:
            return MotionState.MOVING
        return MotionState.IDLE

    # ==================================================================
    # Safety triggers
    # ==================================================================

    def trigger_estop(self, *, source: str = "api") -> CabinetState:
        """Trip the virtual E-stop. Cancels any active job."""
        return self._apply_safety_trigger("trigger_estop", "estop", source=source)

    def trigger_protective_stop(self, *, source: str = "api") -> CabinetState:
        return self._apply_safety_trigger(
            "trigger_protective_stop", "protective_stop", source=source
        )

    def trigger_fault(self, code: str, message: str = "") -> CabinetState:
        """Inject a virtual fault (e.g. from job timeout)."""
        with self._runner_lock:
            old = self._state
            self._state = next_state(self._state, "trigger_fault")
            self._fault_code = code
            self._fault_message = message
            self._cancel_active_job_locked(reason=f"fault:{code}")
            self._set_outputs_locked(red=True, green=False, cycle=False)
        # also reflect in arm's internal E-stop flag
        try:
            self.robot.emergency_stop()
        except Exception:
            pass
        LOG.warning("Cabinet %s: fault %s (%s -> %s)",
                    self.cabinet_id, code, old.value, self._state.value)
        return self._state

    def _apply_safety_trigger(
        self, trigger: str, label: str, *, source: str
    ) -> CabinetState:
        with self._runner_lock:
            old = self._state
            try:
                self._state = next_state(self._state, trigger)
            except CabinetStateError as e:
                # Allowed from any non-OFFLINE state per brief §3.1; if the
                # transition table doesn't list it from OFFLINE that's fine.
                if old == CabinetState.OFFLINE:
                    LOG.warning("Cabinet safety trigger %s ignored in OFFLINE", label)
                    return old
                raise
            self._cancel_active_job_locked(reason=label)
            self._set_outputs_locked(red=True, green=False, cycle=False)
        # Reflect into the robot's internal E-stop flag too (legacy parity).
        try:
            self.robot.emergency_stop()
        except Exception:
            pass
        LOG.warning("Cabinet %s: %s triggered from %s (%s -> %s)",
                    self.cabinet_id, label, source, old.value, self._state.value)
        return self._state

    # ==================================================================
    # Resets
    # ==================================================================

    def reset_estop(self) -> CabinetState:
        return self._reset_safety("ESTOP", "reset_estop", set_green=True)

    def reset_protective_stop(self) -> CabinetState:
        return self._reset_safety("PROTECTIVE_STOP", "reset_protective_stop",
                                  set_green=True)

    def reset_fault(self) -> CabinetState:
        with self._runner_lock:
            self._fault_code = None
            self._fault_message = None
        return self._reset_safety("FAULT", "reset_fault", set_green=True)

    def _reset_safety(
        self, from_state_name: str, trigger: str, *, set_green: bool
    ) -> CabinetState:
        with self._runner_lock:
            if self._state.value != from_state_name:
                raise CabinetStateError(
                    f"Cannot {trigger}: cabinet is in {self._state.value}, "
                    f"expected {from_state_name}"
                )
            old = self._state
            self._state = next_state(self._state, trigger)
            # D3 readiness re-eval: a safety reset must NOT skip the
            # readiness check. If any non-safety readiness input is still
            # missing, cabinet returns to WAITING — not directly to READY.
            self._re_evaluate_idle_readiness_locked()
            if set_green:
                self._set_outputs_locked(red=False, green=True, cycle=False)
        # Reflect in the robot too.
        try:
            self.robot.reset_estop()
        except Exception:
            pass
        LOG.info("Cabinet %s: %s reset (%s -> %s)",
                 self.cabinet_id, from_state_name, old.value, self._state.value)
        return self._state

    # ==================================================================
    # Motion commands (mediated)
    # ==================================================================

    def move_to(self, pose_name: str) -> CabinetState:
        """Set joint targets to ``data/poses/<pose_name>.json``.

        Raises:
            VirtualSafetyStopError: if cabinet blocks motion.
            JobAlreadyRunningError: if a job is RUNNING.
            FileNotFoundError: if the pose file is missing.
            JobValidationError: if the pose file is invalid.
        """
        with self._runner_lock:
            self._require_motion_allowed_locked()
            path = self.poses_dir / f"{pose_name}.json"
            joints = load_pose(path)        # raises clearly on bad JSON
            self.robot.set_joint_targets(joints)
            self._set_outputs_locked(cycle=False)
        return self._state

    def set_joint_targets(self, joints: list[float]) -> CabinetState:
        """Raw joint target setting (sliders). Mediated by cabinet state."""
        with self._runner_lock:
            self._require_motion_allowed_locked()
            self.robot.set_joint_targets(joints)
            self._set_outputs_locked(cycle=False)
        return self._state

    def move_home(self) -> CabinetState:
        return self.move_to("home")

    # ==================================================================
    # Virtual I/O
    # ==================================================================

    def set_input(self, signal: str, value: bool) -> None:
        """Update a virtual input. No whitelist here (delegated to VirtualIO).

        SAFETY rule: setting ``safety_gate_closed`` to False while the
        cabinet is READY/RUNNING auto-triggers PROTECTIVE_STOP. Setting it
        to True does NOT auto-reset (operator must press reset button).

        READINESS rule: setting one of the non-safety readiness inputs
        (``part_present`` / ``fixture_clamped`` / ``welder_ready``) does
        NOT directly mutate cabinet state. Instead,
        ``_re_evaluate_idle_readiness_locked`` is called so the cabinet
        flips between READY and WAITING in a single source-of-truth
        helper. The re-eval only takes effect at idle (READY/WAITING);
        during RUNNING the runner drives state directly.
        """
        with self._runner_lock:
            self._io.set_input(signal, value)
            # READINESS: non-safety readiness input changed — re-evaluate.
            if signal in _NON_SAFETY_READINESS_SIGNALS:
                self._re_evaluate_idle_readiness_locked()
            # SAFETY: gate opened during READY/RUNNING → PROTECTIVE_STOP.
            if (signal == "safety_gate_closed" and value is False
                    and self._state in (CabinetState.READY,
                                        CabinetState.WAITING,
                                        CabinetState.RUNNING)):
                self._state = next_state(self._state,
                                          "trigger_protective_stop")
                self._set_outputs_locked(red=True, green=False, cycle=False)
                # Cancel any running job too.
                if self._runner is not None and self._runner.is_running:
                    self._runner.cancel("safety_gate_open")
                    try:
                        self._runner.join(timeout=1.5)
                    except Exception:
                        pass
                    # Snapshot BEFORE clearing identity (see also
                    # _cancel_active_job_locked — same ownership rule).
                    if self._runner is not None:
                        self._last_completed_progress = (
                            self._runner.progress.snapshot()
                        )
                        # Build the public last_job_result so the UI
                        # can render the cancellation reason.
                        self._last_job_result = (
                            self._build_last_job_result_from_progress_locked(
                                self._last_completed_progress
                            )
                        )
                    self._runner = None
                    self._runner_run_id = None
                    self._runner_generation = None
                try:
                    self.robot.emergency_stop()
                except Exception:
                    pass
                LOG.warning(
                    "Cabinet %s: virtual safety gate opened; auto-triggered "
                    "PROTECTIVE_STOP", self.cabinet_id,
                )
        LOG.info("Cabinet %s: virtual input %s = %s",
                 self.cabinet_id, signal, value)

    # ==================================================================
    # Jobs
    # ==================================================================

    def start_job(self, job_id: str | None = None) -> str:
        """Start the demo job. Validates state, IO, then launches JobRunner.

        Returns the started ``job_id``. Raises on any preflight failure.

        SAFETY rule: ``safety_gate_closed=False`` is a virtual safety
        condition — not a readiness waiting condition. If the gate is open
        at job start, the cabinet auto-transitions to PROTECTIVE_STOP and
        rejects the start (caller must close the gate and reset).
        """
        with self._runner_lock:
            # SAFETY: gate open -> PROTECTIVE_STOP + reject start.
            if self._io.inputs.get("safety_gate_closed") is False:
                if self._state in (CabinetState.READY, CabinetState.RUNNING):
                    self._state = next_state(
                        self._state, "trigger_protective_stop"
                    )
                    self._set_outputs_locked(
                        red=True, green=False, cycle=False
                    )
                raise VirtualSafetyStopError(
                    "Cannot start job: virtual safety gate is open. "
                    "Close the gate and reset the protective stop."
                )
            if self._state == CabinetState.WAITING:
                # D2: WAITING → start_job must raise VirtualInterlockError
                # (not CabinetStateError) so the API layer can return the
                # missing-inputs list, matching the interlock error contract.
                missing = self._non_safety_readiness_missing_locked()
                raise VirtualInterlockError(missing)
            if not job_start_allowed(self._state):
                raise CabinetStateError(
                    f"Cannot start job: cabinet is in {self._state.value}, "
                    f"must be READY"
                )
            if self._runner is not None and self._runner.is_running:
                raise JobAlreadyRunningError(
                    f"Job {self._runner.job.job_id} is already running"
                )

            spec = JobSpec.from_json_file(self.job_spec_path)
            if job_id is not None and spec.job_id != job_id:
                raise JobValidationError(
                    f"Requested job_id {job_id!r} does not match file "
                    f"job_id {spec.job_id!r}"
                )

            # Interlock check (readiness only — gate is already enforced above)
            missing = self._io.check_interlock(spec.required_inputs)
            if missing:
                raise VirtualInterlockError(missing)

            # Build runner with hooks that flip cabinet state.
            self._runner = JobRunner(
                job=spec,
                robot=self.robot,
                poses_dir=self.poses_dir,
                motion_timeout_s=self.motion_timeout_s,
                on_safety_stop=self._on_runner_safety_stop,
                on_fault=self._on_runner_fault,
                on_complete=self._on_runner_complete,
            )
            self._state = next_state(self._state, "start_job")
            self._set_outputs_locked(red=False, green=False, cycle=True)
            # Start FIRST (bumps the runner's generation), then install
            # the identity so callbacks from THIS run match exactly.
            self._runner.start()
            self._runner_run_id = self._runner.run_id
            self._runner_generation = self._runner.generation
            # D6: only now — after the new job has been successfully
            # accepted AND started — do we clear the previous terminal
            # result. If start_job() had raised before this point (gate
            # open, WAITING, ESTOP, FAULT, PROTECTIVE_STOP, interlock
            # missing, already-running), the previous last_job_result
            # would have been preserved.
            self._last_job_result = None
            self._last_completed_progress = None
        LOG.info("Cabinet %s: started job %s (run_id=%s, gen=%d)",
                 self.cabinet_id, spec.job_id,
                 self._runner.run_id, self._runner.generation)
        return spec.job_id

    def current_job_progress(self) -> dict[str, Any] | None:
        """Return JobProgress.snapshot(), or the last completed snapshot if
        the runner has terminated (so callers can see final state)."""
        with self._runner_lock:
            if self._runner is not None:
                return self._runner.progress.snapshot()
            return self._last_completed_progress

    # ==================================================================
    # Runner callbacks (called from JobRunner thread)
    # ==================================================================

    def _on_runner_complete(self, run_id: int, gen: int) -> None:
        # Ownership check: drop callbacks from runners that are no longer
        # installed (e.g. superseded by a reset or a new job).
        if run_id != self._runner_run_id or gen != self._runner_generation:
            LOG.info("Cabinet %s: dropping stale _on_runner_complete "
                     "(run_id=%s gen=%d, current=%s/%s)",
                     self.cabinet_id, run_id, gen,
                     self._runner_run_id, self._runner_generation)
            return
        with self._runner_lock:
            # Capture the final progress snapshot before clearing the runner
            # reference, so callers that poll after completion can still see
            # the last known step state.
            self._last_completed_progress: dict[str, Any] | None = (
                self._runner.progress.snapshot() if self._runner else None
            )
            # Build the public last_job_result from the terminal progress.
            # This is what the UI renders when active_job_id is None.
            if self._last_completed_progress is not None:
                self._last_job_result = (
                    self._build_last_job_result_from_progress_locked(
                        self._last_completed_progress
                    )
                )
            if self._state == CabinetState.RUNNING:
                self._state = next_state(self._state, "job_complete")
            # Post-completion readiness re-eval: if any non-safety readiness
            # input is still missing, cabinet should land in WAITING instead
            # of READY. (Safety inputs unchanged — gate-open would already
            # have triggered PROTECTIVE_STOP via set_input().)
            self._re_evaluate_idle_readiness_locked()
            self._set_outputs_locked(cycle=False, green=True)
            # Clear the runner reference AND its identity — terminal state.
            self._runner = None
            self._runner_run_id = None
            self._runner_generation = None

    def _on_runner_fault(self, code: str, msg: str,
                         run_id: int, gen: int) -> None:
        # Ownership check.
        if run_id != self._runner_run_id or gen != self._runner_generation:
            LOG.info("Cabinet %s: dropping stale _on_runner_fault %s "
                     "(run_id=%s gen=%d, current=%s/%s)",
                     self.cabinet_id, code, run_id, gen,
                     self._runner_run_id, self._runner_generation)
            return
        # Re-enter cabinet state machine. If we're already in a safety state
        # (FAULT/ESTOP/PROTECTIVE_STOP), we just record the fault info —
        # we don't try to transition (illegal).
        with self._runner_lock:
            self._last_completed_progress: dict[str, Any] | None = (
                self._runner.progress.snapshot() if self._runner else None
            )
            # Build the public last_job_result so the UI shows the failure.
            if self._last_completed_progress is not None:
                self._last_job_result = (
                    self._build_last_job_result_from_progress_locked(
                        self._last_completed_progress
                    )
                )
            old = self._state
            if old == CabinetState.RUNNING:
                # Standard fault path: record and transition.
                self._fault_code = code
                self._fault_message = msg
                self._state = next_state(self._state, "job_failed")
                self._set_outputs_locked(red=True, green=False, cycle=False)
            elif old in (CabinetState.READY, CabinetState.WAITING):
                # Fault from READY/WAITING (e.g. while idle) — trigger_fault
                # is allowed from both.
                self._fault_code = code
                self._fault_message = msg
                self._state = next_state(self._state, "trigger_fault")
                self._set_outputs_locked(red=True, green=False, cycle=False)
            else:
                # Already in ESTOP / PROTECTIVE_STOP / FAULT — DON'T overwrite
                # fault_code (it may have been cleared by a reset). Just log.
                LOG.info("Cabinet %s: runner fault %s while in %s; "
                         "skipping record (cabinet already in safety state)",
                         self.cabinet_id, code, old.value)
            # Clear the runner reference — terminal state.
            self._runner = None
            self._runner_run_id = None
            self._runner_generation = None

    def _on_runner_safety_stop(self, reason: str,
                                run_id: int, gen: int) -> None:
        # Ownership check.
        if run_id != self._runner_run_id or gen != self._runner_generation:
            LOG.info("Cabinet %s: dropping stale _on_runner_safety_stop %s "
                     "(run_id=%s gen=%d, current=%s/%s)",
                     self.cabinet_id, reason, run_id, gen,
                     self._runner_run_id, self._runner_generation)
            return
        with self._runner_lock:
            old = self._state
            if old in (CabinetState.ESTOP, CabinetState.PROTECTIVE_STOP,
                       CabinetState.FAULT, CabinetState.OFFLINE):
                return  # already in a terminal/locked state
            trigger = ("trigger_protective_stop"
                       if reason == "protective_stop"
                       else "trigger_estop")
            self._state = next_state(self._state, trigger)
            self._set_outputs_locked(red=True, green=False, cycle=False)

    # ==================================================================
    # Internal helpers
    # ==================================================================

    def _require_motion_allowed_locked(self) -> None:
        """Caller must hold ``_runner_lock``.

        SAFETY rule: ``safety_gate_closed=False`` is treated as a virtual
        safety condition, not a readiness waiting condition. If the gate
        opens during READY/RUNNING we auto-transition to PROTECTIVE_STOP
        and refuse the motion.

        Order: gate check FIRST, then job-running check, then motion block.
        """
        # 1. SAFETY gate — check first so a gate-open during RUNNING also
        #    trips the safety stop (cancel the job too).
        if (self._io.inputs.get("safety_gate_closed") is False
                and self._state in (CabinetState.READY, CabinetState.RUNNING)):
            self._state = next_state(self._state, "trigger_protective_stop")
            self._set_outputs_locked(red=True, green=False, cycle=False)
            # Cancel any running job so its callbacks stop firing.
            if self._runner is not None and self._runner.is_running:
                self._runner.cancel("safety_gate_open")
                try:
                    self._runner.join(timeout=1.5)
                except Exception:
                    pass
                # Snapshot BEFORE clearing identity (see also
                # _cancel_active_job_locked — same ownership rule).
                if self._runner is not None:
                    self._last_completed_progress = (
                        self._runner.progress.snapshot()
                    )
                    self._last_job_result = (
                        self._build_last_job_result_from_progress_locked(
                            self._last_completed_progress
                        )
                    )
                self._runner = None
                self._runner_run_id = None
                self._runner_generation = None
            try:
                self.robot.emergency_stop()
            except Exception:
                pass
            raise VirtualSafetyStopError(
                "Motion blocked: virtual safety gate is open."
            )
        # 2. Job already running?
        if self._runner is not None and self._runner.is_running:
            raise JobAlreadyRunningError(
                f"Job {self._runner.job.job_id} is running; wait or cancel."
            )
        # 3. Cabinet-level motion block?
        if motion_blocked(self._state):
            # WAITING falls in here too — distinct error message naming the
            # missing readiness input(s) so the UI / API can present a
            # human-readable reason (per D2 spec).
            if self._state == CabinetState.WAITING:
                missing = self._non_safety_readiness_missing_locked()
                first = missing[0] if missing else "readiness"
                raise VirtualSafetyStopError(
                    f"Motion blocked: waiting for {first}."
                )
            raise VirtualSafetyStopError(
                f"Motion blocked: cabinet is in {self._state.value}"
            )
        if self._state not in (CabinetState.READY, CabinetState.RUNNING):
            raise CabinetStateError(
                f"Motion not allowed in cabinet state {self._state.value}"
            )

    def _load_required_inputs_locked(self) -> list[str]:
        """Caller must hold ``_runner_lock``. Return the cached required-input
        list loaded from the job spec file at construction time."""
        return list(self._required_inputs)

    def _cancel_active_job_locked(self, reason: str) -> None:
        if self._runner is not None and self._runner.is_running:
            self._runner.cancel(reason)
            # Wait briefly for the runner thread to terminate.
            try:
                self._runner.join(timeout=1.5)
            except Exception:
                pass
            # Snapshot the last progress BEFORE clearing the runner — this
            # keeps the final progress queryable but makes active_job_id
            # return None (since _runner is now None).
            #
            # IMPORTANT: take the snapshot even if the thread is still
            # technically alive. After this block we are about to clear
            # _runner / _runner_run_id, so any subsequent callback from
            # this runner will be dropped by the ownership check. The
            # snapshot taken here is therefore the LAST authoritative
            # source for this job's terminal state.
            if self._runner is not None:
                self._last_completed_progress = (
                    self._runner.progress.snapshot()
                )
                self._last_job_result = (
                    self._build_last_job_result_from_progress_locked(
                        self._last_completed_progress
                    )
                )
            # Clear the runner reference — terminal state. Identity is also
            # cleared so any in-flight callbacks from the dead runner get
            # dropped by the ownership check.
            self._runner = None
            self._runner_run_id = None
            self._runner_generation = None

    def _set_outputs_locked(
        self,
        *,
        red: bool | None = None,
        green: bool | None = None,
        cycle: bool | None = None,
    ) -> None:
        """Update virtual outputs by name. None = leave unchanged."""
        if red is not None:
            self._io.set_output("stack_light_red", red)
        if green is not None:
            self._io.set_output("stack_light_green", green)
        if cycle is not None:
            self._io.set_output("cycle_running", cycle)

    # ==================================================================
    # Readiness re-evaluation (READY <-> WAITING)
    # ==================================================================

    def _non_safety_readiness_missing_locked(self) -> list[str]:
        """Return the list of non-safety readiness inputs that are False
        (or missing). Caller must hold ``_runner_lock``.

        Empty list means all non-safety readiness inputs are satisfied.
        Safety gate is intentionally NOT checked here — gate-open is
        always PROTECTIVE_STOP, never WAITING.
        """
        missing: list[str] = []
        for sig in _NON_SAFETY_READINESS_SIGNALS:
            v = self._io.inputs.get(sig)
            if v is False:
                missing.append(sig)
        return missing

    def _re_evaluate_idle_readiness_locked(self) -> None:
        """Re-evaluate READY <-> WAITING for idle cabinet states.

        Caller must hold ``_runner_lock``. This helper is the SINGLE
        source of truth for the READY/WAITING flip. Status light must
        never be used to mutate cabinet state.

        Behavior by current cabinet state:
        - OFFLINE / RUNNING / ESTOP / PROTECTIVE_STOP / FAULT: NO-OP.
          Safety states keep priority; readiness is irrelevant while
          a safety condition is latched. After a safety RESET the
          caller (e.g. ``_reset_safety``) re-invokes this helper.
        - READY: if any non-safety readiness input is missing, transition
          to WAITING via ``readiness_lost``.
        - WAITING: if all non-safety readiness inputs are satisfied,
          transition to READY via ``readiness_restored``.

        Readiness inputs are only consulted here at idle (READY/WAITING)
        time. During RUNNING, the runner drives cabinet state directly.
        """
        if self._state not in (CabinetState.READY, CabinetState.WAITING):
            return
        missing = self._non_safety_readiness_missing_locked()
        if self._state == CabinetState.READY and missing:
            self._state = next_state(self._state, "readiness_lost")
            LOG.info("Cabinet %s: readiness lost (%s) -> WAITING",
                     self.cabinet_id, ",".join(missing))
        elif self._state == CabinetState.WAITING and not missing:
            self._state = next_state(self._state, "readiness_restored")
            LOG.info("Cabinet %s: readiness restored -> READY",
                     self.cabinet_id)

    # ==================================================================
    # Last-job-result builder
    # ==================================================================

    def _build_last_job_result_from_progress_locked(
        self, prog: dict[str, Any]
    ) -> dict[str, Any]:
        """Convert a ``JobProgress.snapshot()`` into the public
        ``last_job_result`` schema. Caller must hold ``_runner_lock``.

        Mapping rules:
        - ``completed=True, cancelled=False, fault_code absent``
          -> result="completed", success=True, current_step="Completed"
        - ``completed=True, cancelled=True, cancel_reason starts with
          "fault:"``
          -> result="failed", success=False, current_step="Failed: <code>"
        - ``completed=True, cancelled=True`` (other reasons)
          -> result="cancelled", success=False, current_step="Cancelled"
        """
        completed = bool(prog.get("completed"))
        cancelled = bool(prog.get("cancelled"))
        cancel_reason = prog.get("cancel_reason")
        cur_idx = prog.get("current_step_index")
        total = prog.get("total_steps")
        elapsed = prog.get("elapsed_s")
        finished_at = prog.get("finished_at")

        if completed and not cancelled:
            result = "completed"
            success = True
            current_step_en = "Completed"
            current_step_zh = "已完成"
        elif completed and cancelled and isinstance(cancel_reason, str) \
                and cancel_reason.startswith("fault:"):
            # cancel_reason format from JobRunner._fault is "fault:<code>"
            code = cancel_reason.split(":", 1)[1] if ":" in cancel_reason \
                else cancel_reason
            result = "failed"
            success = False
            current_step_en = f"Failed: {code}"
            current_step_zh = f"失敗:{code}"
        elif completed and cancelled:
            result = "cancelled"
            success = False
            current_step_en = "Cancelled"
            current_step_zh = "已取消"
        else:
            # Should not normally happen — runner always sets one of
            # completed/cancelled before terminating. Defensive default:
            result = "cancelled"
            success = False
            current_step_en = "Cancelled"
            current_step_zh = "已取消"

        return {
            "job_id": prog.get("job_id"),
            "current_step_index": cur_idx,
            "total_steps": total,
            "current_step": current_step_en,
            "current_step_zh": current_step_zh,
            "completed": completed,
            "result": result,
            "success": success,
            "elapsed_s": elapsed,
            "finished_at": finished_at,
        }

    # ==================================================================
    # Compatibility for the Phase 1B legacy direct-UI routes
    # ==================================================================

    def emergency_stop_compat(self) -> CabinetState:
        """Legacy /api/emergency_stop — forwards to cabinet E-stop."""
        return self.trigger_estop(source="legacy-route")

    def reset_estop_compat(self) -> CabinetState:
        """Legacy /api/reset_estop — forwards to cabinet reset."""
        return self.reset_estop()

    def move_home_compat(self) -> CabinetState:
        return self.move_home()

    def load_pose_compat(self, pose_name: str) -> CabinetState:
        return self.move_to(pose_name)
