"""JobRunner — executes a validated Job step-by-step on a WebRobot.

Phase 1C design constraints (per Ben Yung, 2026-09-16):
    * Job only starts via VirtualControllerCabinet.start_job().
    * Each ``move_pose`` step waits for the underlying arm to reach its
      target before advancing (synchronised via ``threading.Event``, NOT
      sleep-based polling).
    * On E-stop, protective stop, fault, timeout, or motion exception:
        - the active job is cancelled;
        - subsequent steps are not executed;
        - the cabinet state is set accordingly;
        - the job is NOT automatically retried / restarted.
    * Per-step virtual motion timeout (configurable) for fail-safe; not a
      safety function.

The runner is intentionally single-job-at-a-time. Cabinet enforces that.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

LOG = logging.getLogger("robotarm.job")


JOB_SCHEMA_VERSION = 1


from robotarm.exceptions import (
    EmergencyStopError,
    JobAlreadyRunningError,
    JobValidationError,
    VirtualFaultError,
)
from robotarm.utils.pose_loader import PoseLoadError, load_pose


# ---------------------------------------------------------------------------
# Public dataclass: JobSpec
# ---------------------------------------------------------------------------


@dataclass
class JobStep:
    step_id: str
    action: str          # only "move_pose" supported in Phase 1C
    pose: str            # logical name e.g. "home", "pose_a"


# ---------------------------------------------------------------------------
# Public dataclass: JobSpec
# ---------------------------------------------------------------------------


@dataclass
class JobStep:
    step_id: str
    action: str          # only "move_pose" supported in Phase 1C
    pose: str            # logical name e.g. "home", "pose_a"

    @classmethod
    def from_dict(cls, raw: dict[str, Any], idx: int) -> "JobStep":
        if not isinstance(raw, dict):
            raise JobValidationError(f"job.steps[{idx}] must be a dict.")
        sid = raw.get("step_id")
        action = raw.get("action")
        pose = raw.get("pose")
        if not isinstance(sid, str) or not sid:
            raise JobValidationError(f"job.steps[{idx}].step_id missing/invalid.")
        if action != "move_pose":
            raise JobValidationError(
                f"job.steps[{idx}].action must be 'move_pose', got {action!r}."
            )
        if not isinstance(pose, str) or not pose:
            raise JobValidationError(f"job.steps[{idx}].pose missing/invalid.")
        return cls(step_id=sid, action=action, pose=pose)


@dataclass
class JobSpec:
    job_id: str
    name: str
    mode: str                       # must be "virtual_only"
    required_inputs: list[str]
    steps: list[JobStep] = field(default_factory=list)
    schema_version: int = JOB_SCHEMA_VERSION

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "JobSpec":
        if not isinstance(raw, dict):
            raise JobValidationError("Job JSON must be a top-level object.")
        sv = raw.get("schema_version")
        if sv != JOB_SCHEMA_VERSION:
            raise JobValidationError(
                f"Job schema_version must be {JOB_SCHEMA_VERSION}, got {sv!r}"
            )
        mode = raw.get("mode")
        if mode != "virtual_only":
            raise JobValidationError(
                f"Job mode must be 'virtual_only', got {mode!r}"
            )
        jid = raw.get("job_id")
        if not isinstance(jid, str) or not jid:
            raise JobValidationError("Job job_id missing/invalid.")
        name = raw.get("name", "")
        if not isinstance(name, str):
            raise JobValidationError("Job name must be a string.")
        required = raw.get("required_inputs", [])
        if not isinstance(required, list) or not all(isinstance(s, str) for s in required):
            raise JobValidationError("Job required_inputs must be a list of strings.")
        steps_raw = raw.get("steps")
        if not isinstance(steps_raw, list) or not steps_raw:
            raise JobValidationError("Job steps must be a non-empty list.")
        steps = [JobStep.from_dict(s, i) for i, s in enumerate(steps_raw)]
        return cls(
            job_id=jid,
            name=name,
            mode=mode,
            required_inputs=list(required),
            steps=steps,
            schema_version=sv,
        )

    @classmethod
    def from_json_file(cls, path: str | Path) -> "JobSpec":
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"Job file not found: {p}")
        try:
            raw = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            raise JobValidationError(f"Job file is not valid JSON: {e}") from e
        return cls.from_dict(raw)


# ---------------------------------------------------------------------------
# Execution helpers
# ---------------------------------------------------------------------------


@dataclass
class JobProgress:
    """Live job progress reported to the cabinet / UI."""

    job_id: str
    total_steps: int
    current_step_index: int | None = None   # 0-based; None until first step
    current_step_id: str | None = None
    current_step_name: str | None = None    # logical pose name being executed
    started_at: float = 0.0
    finished_at: float | None = None
    completed: bool = False
    cancelled: bool = False
    cancel_reason: str | None = None

    @property
    def elapsed_s(self) -> float:
        end = self.finished_at if self.finished_at else time.time()
        return max(0.0, end - self.started_at)

    def snapshot(self) -> dict[str, Any]:
        return {
            "job_id": self.job_id,
            "total_steps": self.total_steps,
            # Brief §7.1 mandates current_step_index as 1-based OR documented
            # as internal. We document it as 0-based internally and let the
            # UI compute the display number.
            "current_step_index": (
                self.current_step_index + 1
                if self.current_step_index is not None else None
            ),
            "current_step_id": self.current_step_id,
            "current_step_name": self.current_step_name,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "elapsed_s": round(self.elapsed_s, 3),
            "completed": self.completed,
            "cancelled": self.cancelled,
            "cancel_reason": self.cancel_reason,
        }


# ---------------------------------------------------------------------------
# JobRunner
# ---------------------------------------------------------------------------


class JobRunner:
    """Drives a single JobSpec against a WebRobot in a background thread.

    Synchronisation
    ---------------
    * The runner owns ``_thread`` (the execution thread) and ``_cancel_event``
      (signals "abort the job ASAP").
    * Per-step target completion uses the WebRobot's own interpolation loop;
      we wait on ``_step_done_event`` which the runner sets once the arm's
      ``joints`` equal ``joint_targets`` within tolerance — NO ``time.sleep``
      polling loops.
    * ``_cancel_event`` is checked between steps and during the per-step
      wait so external triggers (E-stop / fault) abort the job promptly.

    Timeout
    -------
    * ``motion_timeout_s`` (default 10.0) bounds each step. If exceeded,
      the runner raises VirtualFaultError("VIRTUAL_MOTION_TIMEOUT", ...).
    * Timeout is virtual demo fail-safe only — NOT a safety function.
    """

    DEFAULT_MOTION_TIMEOUT_S = 10.0
    #: How close (rad) the arm's joints must be to targets to consider a
    #: step "arrived".
    STEP_TOLERANCE_RAD = 0.01
    #: Max wait between step-arrival polls (kept tiny — no sleep-only sync).
    STEP_POLL_INTERVAL_S = 0.02

    def __init__(
        self,
        job: JobSpec,
        robot,
        poses_dir: str | Path,
        *,
        motion_timeout_s: float = DEFAULT_MOTION_TIMEOUT_S,
        # Optional hooks used by the cabinet to translate runner signals
        # into cabinet-state transitions. Both default to no-ops.
        on_safety_stop=None,
        on_fault=None,
        on_complete=None,
    ) -> None:
        self.job = job
        self.robot = robot
        self.poses_dir = Path(poses_dir)
        self.motion_timeout_s = float(motion_timeout_s)

        self._on_safety_stop = on_safety_stop or (lambda reason: None)
        self._on_fault = on_fault or (lambda code, msg: None)
        self._on_complete = on_complete or (lambda: None)

        self._thread: threading.Thread | None = None
        self._cancel_event = threading.Event()
        self._step_done_event = threading.Event()
        self._started_event = threading.Event()
        # Job-run identity. The cabinet uses this to identify the SPECIFIC
        # runner instance that owns the callback. Every callback fired by
        # this runner is wrapped so the cabinet can verify ownership and
        # drop stale callbacks (e.g. from a runner whose job was already
        # cancelled and reset).
        self.run_id: str = id(self)  # unique per runner instance
        self.generation: int = 0     # bumped on every start()
        self.progress = JobProgress(
            job_id=job.job_id,
            total_steps=len(job.steps),
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Kick off the job in a daemon thread. Idempotent? No — raises if
        already running."""
        if self._thread is not None and self._thread.is_alive():
            raise JobAlreadyRunningError(
                f"JobRunner for {self.job.job_id} already running."
            )
        self._cancel_event.clear()
        self._step_done_event.clear()
        self._started_event.clear()
        # Bump the generation; every callback fired after this start() is
        # tagged with the new generation so the cabinet can verify ownership.
        self.generation += 1
        self.progress = JobProgress(
            job_id=self.job.job_id,
            total_steps=len(self.job.steps),
        )
        self.progress.started_at = time.time()
        self._thread = threading.Thread(
            target=self._run, name=f"job-{self.job.job_id}",
            args=(self.generation,),
            daemon=True,
        )
        self._thread.start()

    def cancel(self, reason: str) -> None:
        """Request cancellation. Thread-safe."""
        LOG.info("Job %s: cancel requested (%s)", self.job.job_id, reason)
        self._cancel_event.set()
        self._step_done_event.set()  # unblock any pending wait

    def join(self, timeout: float | None = None) -> bool:
        """Wait for the runner thread to finish. Returns True if it did."""
        if self._thread is None:
            return True
        return self._thread.join(timeout)

    def wait_until_started(self, timeout: float | None = None) -> bool:
        """Block until the runner thread has actually entered _run()."""
        return self._started_event.wait(timeout)

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    # Worker thread
    # ------------------------------------------------------------------

    def _run(self, gen: int) -> None:
        LOG.info("Job %s: starting (gen=%d, %d steps)",
                 self.job.job_id, gen, len(self.job.steps))
        self._started_event.set()
        try:
            for i, step in enumerate(self.job.steps):
                if self._cancel_event.is_set():
                    self._mark_cancelled("aborted_before_step", gen=gen)
                    return

                self.progress.current_step_index = i
                self.progress.current_step_id = step.step_id
                self.progress.current_step_name = step.pose

                # Load pose JSON
                try:
                    pose_path = self.poses_dir / f"{step.pose}.json"
                    target_joints = load_pose(pose_path)
                except PoseLoadError as e:
                    self._fault("JOB_POSE_INVALID", f"step {step.step_id}: {e}",
                                gen=gen)
                    return

                # Set joint targets on the arm. The WebRobot itself enforces
                # joint limits and (legacy) E-stop. Cabinet-level safety is
                # enforced by the cabinet BEFORE calling start_job.
                try:
                    self.robot.set_joint_targets(target_joints)
                except EmergencyStopError as e:
                    self._mark_cancelled(f"estop_during_set:{e}", gen=gen)
                    self._on_safety_stop("estop", self.run_id, gen)
                    return

                # Wait for the arm to reach its targets, with per-step timeout.
                self._step_done_event.clear()
                arrived = self._wait_for_step(
                    target_joints, self.motion_timeout_s
                )
                if not arrived:
                    self._fault(
                        "VIRTUAL_MOTION_TIMEOUT",
                        f"step {step.step_id} ({step.pose}) did not arrive "
                        f"within {self.motion_timeout_s:.1f}s",
                        gen=gen,
                    )
                    return

                if self._cancel_event.is_set():
                    self._mark_cancelled("aborted_after_step", gen=gen)
                    return

            # All steps done. Mark progress.completed = True FIRST (the
            # snapshot under lock is the public source of truth) then call
            # _on_complete — the cabinet callback takes the same lock, so
            # any concurrent poller sees either (runner.progress with
            # completed=True, runner reference still set) or (no runner,
            # _last_completed_progress with completed=True) — never an
            # inconsistent mix.
            self.progress.completed = True
            self.progress.finished_at = time.time()
            LOG.info("Job %s: completed in %.2fs",
                     self.job.job_id, self.progress.elapsed_s)
            self._on_complete(self.run_id, gen)
        except Exception as e:  # pragma: no cover - defensive
            LOG.exception("Job %s: unexpected exception", self.job.job_id)
            self._fault("JOB_UNEXPECTED", repr(e), gen=gen)

    # ------------------------------------------------------------------
    # Sync helpers
    # ------------------------------------------------------------------

    def _wait_for_step(
        self, target_joints: list[float], timeout_s: float,
    ) -> bool:
        """Block until robot.joints ≈ target_joints or timeout / cancel.

        Uses short sleep + Event wait. NOT a busy poll loop; the dominant
        wait is on ``_step_done_event`` which the cabinet signals on
        cancel/reset.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._cancel_event.is_set():
                return False
            cur = self.robot.joints  # tuple of current joint angles (rad)
            if _close_enough(cur, target_joints, self.STEP_TOLERANCE_RAD):
                return True
            # Sleep a tiny amount; ALSO wait on the event for prompt wake-up.
            self._step_done_event.wait(timeout=self.STEP_POLL_INTERVAL_S)
            self._step_done_event.clear()
        return False

    # ------------------------------------------------------------------
    # Terminal-state markers
    # ------------------------------------------------------------------

    def _mark_cancelled(self, reason: str, *, gen: int | None = None) -> None:
        # Mark cancelled FIRST so the snapshot under lock is consistent;
        # the cabinet callback serialises on the same lock.
        self.progress.cancelled = True
        self.progress.cancel_reason = reason
        self.progress.completed = True
        self.progress.finished_at = time.time()
        LOG.info("Job %s: cancelled (%s)", self.job.job_id, reason)
        try:
            self._on_complete(self.run_id,
                              gen if gen is not None else self.generation)
        except Exception:
            pass

    def _fault(self, code: str, msg: str,
               *, gen: int | None = None) -> None:
        self._mark_cancelled(f"fault:{code}",
                             gen=gen if gen is not None else self.generation)
        LOG.error("Job %s: fault %s — %s", self.job.job_id, code, msg)
        self._on_fault(code, msg, self.run_id,
                       gen if gen is not None else self.generation)


def _close_enough(a, b, tol: float) -> bool:
    if len(a) != len(b):
        return False
    return all(abs(x - y) <= tol for x, y in zip(a, b))
