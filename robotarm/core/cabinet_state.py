"""Cabinet state machine for the Virtual Controller Cabinet.

The cabinet mediates ALL motion, E-stop, fault, and job execution. UI never
talks to the arm directly; every action must traverse a cabinet state
transition.

States (from the Phase 1C brief §3.1):

    OFFLINE          Not yet initialised
    READY            Initialised, idle, accepting jobs
    RUNNING          A job is executing
    ESTOP            Virtual E-stop active — motion blocked
    PROTECTIVE_STOP  Virtual protective stop — motion blocked
    FAULT            Virtual fault — motion blocked until reset

Legal transitions (any = any state):

    OFFLINE          -> READY                    (init)
    READY            -> RUNNING                  (start_job accepted)
    RUNNING          -> READY                    (job completed)
    any              -> ESTOP                    (trigger_estop)
    any              -> PROTECTIVE_STOP          (trigger_protective_stop)
    READY | RUNNING  -> FAULT                    (trigger_fault, timeout)
    ESTOP            -> READY                    (reset_estop)
    PROTECTIVE_STOP  -> READY                    (reset_protective_stop)
    FAULT            -> READY                    (reset_fault)

Illegal transitions raise CabinetStateError.

This module is pure data + rules — no I/O, no threading, no network.
"""
from __future__ import annotations

from enum import Enum


class CabinetState(str, Enum):
    """Virtual cabinet states. ``str`` mixin keeps them JSON-serialisable."""

    OFFLINE = "OFFLINE"
    READY = "READY"
    RUNNING = "RUNNING"
    ESTOP = "ESTOP"
    PROTECTIVE_STOP = "PROTECTIVE_STOP"
    FAULT = "FAULT"


class SafetyState(str, Enum):
    """Subset of cabinet state that describes safety-relevant conditions.

    Mapped from CabinetState by ``safety_state_of()`` for the UI / API.
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

#: frozenset of states that block motion / job start
_MOTION_BLOCKED = frozenset(
    {CabinetState.ESTOP, CabinetState.PROTECTIVE_STOP, CabinetState.FAULT}
)

#: frozenset of states in which a new job may be started
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
        # In Phase 1B the legacy API already permitted triggering estop from
        # an implicit "running-ish" state; we keep the same surface by
        # allowing safety triggers from any non-OFFLINE state below.
    },
    CabinetState.RUNNING: {
        "job_complete": CabinetState.READY,
        "job_failed": CabinetState.FAULT,
        "trigger_estop": CabinetState.ESTOP,
        "trigger_protective_stop": CabinetState.PROTECTIVE_STOP,
        "trigger_fault": CabinetState.FAULT,
    },
    CabinetState.ESTOP: {
        "reset_estop": CabinetState.READY,
    },
    CabinetState.PROTECTIVE_STOP: {
        "reset_protective_stop": CabinetState.READY,
    },
    CabinetState.FAULT: {
        "reset_fault": CabinetState.READY,
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
    """Map a cabinet state to its UI-facing safety state."""
    return {
        CabinetState.OFFLINE: SafetyState.NORMAL,
        CabinetState.READY: SafetyState.NORMAL,
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
