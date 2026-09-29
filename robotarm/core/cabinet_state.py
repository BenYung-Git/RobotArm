"""Cabinet state machine for the Virtual Controller Cabinet.

The cabinet mediates ALL motion, E-stop, fault, and job execution. UI never
talks to the arm directly; every action must traverse a cabinet state
transition.

States (from the Phase 1C brief §3.1 + 2026-09-16 bug fix):

    OFFLINE          Not yet initialised
    READY            Initialised, idle, all readiness inputs satisfied
    WAITING          Initialised, idle, BUT one or more non-safety readiness
                     inputs (part_present / fixture_clamped / welder_ready)
                     are NOT satisfied. Motion and job start are blocked.
                     Safety gate is closed (gate-open is PROTECTIVE_STOP).
    RUNNING          A job is executing
    ESTOP            Virtual E-stop active — motion blocked
    PROTECTIVE_STOP  Virtual protective stop — motion blocked
    FAULT            Virtual fault — motion blocked until reset

Legal transitions:

    OFFLINE          -> READY                            (init)
    READY            -> RUNNING                          (start_job accepted)
    RUNNING          -> READY | WAITING                  (job_complete;
                                                         re-evaluate readiness)
    READY            -> WAITING                          (readiness_lost)
    WAITING          -> READY                            (readiness_restored)
    WAITING          -> ESTOP | PROTECTIVE_STOP | FAULT  (safety triggers)
    any              -> ESTOP                            (trigger_estop)
    any              -> PROTECTIVE_STOP                  (trigger_protective_stop)
    READY | RUNNING  -> FAULT                            (trigger_fault, timeout)
    ESTOP            -> READY | WAITING                  (reset_estop;
                                                         re-evaluate readiness)
    PROTECTIVE_STOP  -> READY | WAITING                  (reset_protective_stop;
                                                         re-evaluate readiness)
    FAULT            -> READY | WAITING                  (reset_fault;
                                                         re-evaluate readiness)

Safety priority order (highest to lowest):
    ESTOP > FAULT / PROTECTIVE_STOP > WAITING > READY

Resets (ESTOP / FAULT / PROTECTIVE_STOP) DO NOT skip the readiness check.
After a safety reset, if any non-safety readiness input is still missing,
the cabinet returns to WAITING — not directly to READY.

WAITING never arises from a safety input (safety_gate_closed=False is
always PROTECTIVE_STOP, never WAITING). The cabinet enforces this in
``VirtualControllerCabinet.set_input`` and in
``_re_evaluate_idle_readiness_locked``.

Illegal transitions raise CabinetStateError.

This module is pure data + rules — no I/O, no threading, no network.
"""
from __future__ import annotations

from enum import Enum


class CabinetState(str, Enum):
    """Virtual cabinet states. ``str`` mixin keeps them JSON-serialisable."""

    OFFLINE = "OFFLINE"
    READY = "READY"
    WAITING = "WAITING"
    RUNNING = "RUNNING"
    ESTOP = "ESTOP"
    PROTECTIVE_STOP = "PROTECTIVE_STOP"
    FAULT = "FAULT"


class SafetyState(str, Enum):
    """Subset of cabinet state that describes safety-relevant conditions.

    Mapped from CabinetState by ``safety_state_of()`` for the UI / API.

    Note: WAITING is not a safety state — it's a normal-state sub-condition.
    Readiness waiting is non-safety and maps to ``NORMAL``.
    """

    NORMAL = "normal"
    ESTOP = "estop"
    PROTECTIVE_STOP = "protective_stop"
    FAULT = "fault"


class MotionState(str, Enum):
    """High-level motion status independent of cabinet state."""

    IDLE = "idle"
    MOVING = "moving"
    STOPPED = "stopped"


# ---------------------------------------------------------------------------
# Transition table
# ---------------------------------------------------------------------------

#: frozenset of states that block motion / job start.
# WAITING is included: a cabinet in WAITING has unsatisfied non-safety
# readiness inputs and must NOT move or start a job.
_MOTION_BLOCKED = frozenset(
    {CabinetState.ESTOP, CabinetState.PROTECTIVE_STOP, CabinetState.FAULT,
     CabinetState.WAITING}
)

#: frozenset of states in which a new job may be started.
# WAITING is NOT allowed — readiness must be re-satisfied first.
_JOB_START_ALLOWED = frozenset({CabinetState.READY})


# Transition table: { from_state: { trigger: to_state } }
TRANSITIONS: dict[CabinetState, dict[str, CabinetState]] = {
    CabinetState.OFFLINE: {
        "init": CabinetState.READY,
    },
    CabinetState.READY: {
        "start_job": CabinetState.RUNNING,
        "trigger_estop": CabinetState.ESTOP,
        "trigger_protective_stop": CabinetState.PROTECTIVE_STOP,
        "trigger_fault": CabinetState.FAULT,
        "readiness_lost": CabinetState.WAITING,
        # ``job_complete`` is also accepted from RUNNING; from READY it would
        # be a no-op transition back to READY which we model by NOT listing
        # it (callers use ``next_state`` and that raises on illegal).
    },
    CabinetState.WAITING: {
        "readiness_restored": CabinetState.READY,
        "trigger_estop": CabinetState.ESTOP,
        "trigger_protective_stop": CabinetState.PROTECTIVE_STOP,
        "trigger_fault": CabinetState.FAULT,
        # Note: NO start_job trigger from WAITING — readiness must be
        # restored first. ``_JOB_START_ALLOWED`` enforces this independently.
        # Note: NO direct reset_*  -> READY from WAITING. The cabinet
        # applies safety resets through a separate re-evaluation helper
        # which may land on READY or WAITING depending on current inputs.
    },
    CabinetState.RUNNING: {
        "job_complete": CabinetState.READY,   # post-completion re-eval may flip to WAITING via set_input()
        "job_failed": CabinetState.FAULT,
        "trigger_estop": CabinetState.ESTOP,
        "trigger_protective_stop": CabinetState.PROTECTIVE_STOP,
        "trigger_fault": CabinetState.FAULT,
    },
    CabinetState.ESTOP: {
        "reset_estop": CabinetState.READY,    # post-reset re-eval may flip to WAITING via set_input()
    },
    CabinetState.PROTECTIVE_STOP: {
        "reset_protective_stop": CabinetState.READY,  # ditto
    },
    CabinetState.FAULT: {
        "reset_fault": CabinetState.READY,    # ditto
    },
}


def can_transition(current: CabinetState, trigger: str) -> bool:
    """Return True iff ``current --trigger--> ?`` is a legal transition."""
    return trigger in TRANSITIONS.get(current, {})


def next_state(current: CabinetState, trigger: str) -> CabinetState:
    """Return the state after applying ``trigger`` from ``current``.

    Raises:
        CabinetStateError: if the transition is illegal.
    """
    # Import here to avoid a circular dependency at module load.
    from robotarm.exceptions import CabinetStateError

    targets = TRANSITIONS.get(current, {})
    if trigger not in targets:
        raise CabinetStateError(
            f"Illegal transition: {current.value} cannot accept trigger "
            f"{trigger!r}. Allowed triggers: {sorted(targets.keys()) or '∅'}."
        )
    return targets[trigger]


def safety_state_of(cabinet: CabinetState) -> SafetyState:
    """Map a cabinet state to its UI-facing safety state.

    WAITING is a *normal* (non-safety) condition. The UI shows it via
    CabinetState == "WAITING" + Status Light color = yellow, but
    ``safety_state`` stays "normal" so the safety-priority surfaces
    (e.g. E-Stop banner) are not confused.
    """
    return {
        CabinetState.OFFLINE: SafetyState.NORMAL,
        CabinetState.READY: SafetyState.NORMAL,
        CabinetState.WAITING: SafetyState.NORMAL,
        CabinetState.RUNNING: SafetyState.NORMAL,
        CabinetState.ESTOP: SafetyState.ESTOP,
        CabinetState.PROTECTIVE_STOP: SafetyState.PROTECTIVE_STOP,
        CabinetState.FAULT: SafetyState.FAULT,
    }[cabinet]


def motion_blocked(cabinet: CabinetState) -> bool:
    """True if the cabinet refuses motion / job start in this state."""
    return cabinet in _MOTION_BLOCKED


def job_start_allowed(cabinet: CabinetState) -> bool:
    """True if a new job may be started in this state."""
    return cabinet in _JOB_START_ALLOWED
