"""Backend-computed Status Light for the Virtual Cabinet.

The Status Light is the UI-facing semantic summary of cabinet + IO + safety
state. **The UI must render what the backend returns — it MUST NOT derive its
own colour, label, or blink.**

Every state includes:

    color       one of {"gray", "green", "yellow", "red"}
    is_blinking bool
    label_zh    str  (Chinese label)
    label_en    str  (English label)
    reason_zh   str  (Chinese explanation)
    reason_en   str  (English explanation)

The Status Light is **brand-neutral** and explicitly disclaims association with
ROKAE, JAKA, or any real controller. The UI is responsible for displaying the
disclaimer text; this module just supplies the state.

Mapping (per Phase 1C final spec, 2026-09-16 + 2026-09-16 bug fix):

    CabinetState.ESTOP                       -> red, blinking, "Virtual E-stop Active"
    CabinetState.PROTECTIVE_STOP             -> red, static,    "Virtual Fault / Motion Blocked"
        - If safety_gate_closed is False:    -> "Virtual Safety Gate Open"
        - Else:                              -> actual protective-stop reason
    CabinetState.FAULT                       -> red, static,    "Virtual Fault / Motion Blocked"
        - reason: virtual fault reason
    CabinetState.OFFLINE                     -> gray, static,   "Offline / Disabled"
    CabinetState.WAITING                     -> yellow, static, "Virtual Warning / Waiting"
        - reason: names the FIRST missing non-safety readiness input
          (part_present / fixture_clamped / welder_ready).
          SAFETY rule: safety_gate_closed is NEVER a yellow reason.
    CabinetState.READY + all readiness inputs true -> green, static, "Virtual System Ready"
    CabinetState.RUNNING                     -> green, static,
        "Virtual Cycle Running", reason includes job_id + current/total steps.

Safety rule: ``safety_gate_closed=False`` is a SAFETY condition, not a
readiness condition. It can never produce a Yellow "Waiting" state — only
Red PROTECTIVE_STOP. The UI / tests must verify this distinction.

WAITING rule: WAITING is a cabinet state, not just a status-light color.
This module's status-light logic for WAITING reads from the (already
computed) cabinet_state passed in by the caller — it does NOT mutate
cabinet state. The cabinet's ``_re_evaluate_idle_readiness_locked``
helper is the single source of truth for READY <-> WAITING flips.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal, Optional

from robotarm.core.cabinet_state import (
    CabinetState,
    SafetyState,
)

Color = Literal["gray", "green", "yellow", "red"]


# ---------------------------------------------------------------------------
# Per-input Yellow reasons (non-safety readiness only).
# Order matters: the FIRST missing signal in ``required_inputs`` wins so the
# reason is deterministic. ``safety_gate_closed`` is intentionally NOT in this
# list — it is a SAFETY input and must never produce Yellow.
# ---------------------------------------------------------------------------

_YELLOW_REASONS: dict[str, tuple[str, str]] = {
    "part_present":   ("工件不存在",   "Part Not Present"),
    "fixture_clamped": ("治具未夾緊",  "Fixture Not Clamped"),
    "welder_ready":   ("焊機未就緒",   "Welder Not Ready"),
}


@dataclass(frozen=True)
class StatusLight:
    """Immutable backend-computed status light payload."""

    color: str            # gray | green | yellow | red
    is_blinking: bool
    label_zh: str
    label_en: str
    reason_zh: str
    reason_en: str

    def to_dict(self) -> dict[str, Any]:
        """JSON-serialisable dict for the API payload."""
        return {
            "color": self.color,
            "is_blinking": self.is_blinking,
            "label_zh": self.label_zh,
            "label_en": self.label_en,
            "reason_zh": self.reason_zh,
            "reason_en": self.reason_en,
        }


def _gray(reason_zh: str, reason_en: str) -> StatusLight:
    return StatusLight(
        color="gray",
        is_blinking=False,
        label_zh="離線／停用",
        label_en="Offline / Disabled",
        reason_zh=reason_zh,
        reason_en=reason_en,
    )


def _green(label_zh: str, label_en: str,
           reason_zh: str, reason_en: str) -> StatusLight:
    return StatusLight(
        color="green",
        is_blinking=False,
        label_zh=label_zh,
        label_en=label_en,
        reason_zh=reason_zh,
        reason_en=reason_en,
    )


def _yellow(reason_zh: str, reason_en: str) -> StatusLight:
    return StatusLight(
        color="yellow",
        is_blinking=False,
        label_zh="等待條件完成",
        label_en="Virtual Warning / Waiting",
        reason_zh=reason_zh,
        reason_en=reason_en,
    )


def _red(reason_zh: str, reason_en: str) -> StatusLight:
    return StatusLight(
        color="red",
        is_blinking=False,
        label_zh="虛擬故障／運動已封鎖",
        label_en="Virtual Fault / Motion Blocked",
        reason_zh=reason_zh,
        reason_en=reason_en,
    )


def _blinking_red(reason_zh: str, reason_en: str) -> StatusLight:
    return StatusLight(
        color="red",
        is_blinking=True,
        label_zh="虛擬急停已觸發",
        label_en="Virtual E-stop Active",
        reason_zh=reason_zh,
        reason_en=reason_en,
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compute_status_light(
    *,
    cabinet_state: CabinetState,
    safety_state: SafetyState,
    virtual_inputs: dict[str, bool],
    required_inputs: list[str] | None = None,
    motion_blocked_reason: str | None = None,
    # Running progress — only used when cabinet_state == RUNNING.
    active_job_id: Optional[str] = None,
    current_step: Optional[int] = None,
    total_steps: Optional[int] = None,
) -> StatusLight:
    """Pure function: derive the StatusLight from cabinet state + IO.

    Rules (priority top-down):

        1. ESTOP                       -> blinking_red, "Virtual E-stop Active"
        2. PROTECTIVE_STOP             -> red, "Virtual Fault / Motion Blocked"
           a. safety_gate_closed=False  -> reason "Virtual Safety Gate Open"
           b. other missing readiness   -> reason names the input
           c. otherwise                 -> motion_blocked_reason or generic
        3. FAULT                       -> red, "Virtual Fault / Motion Blocked"
                                          (reason = motion_blocked_reason)
        4. OFFLINE                     -> gray, "Offline / Disabled"
        5. WAITING                     -> yellow, "Virtual Warning / Waiting"
                                          (reason = first missing non-safety
                                          readiness input; safety_gate is
                                          never a yellow reason — gate-open
                                          is PROTECTIVE_STOP, handled above.)
        6. READY + all readiness true  -> green, "Virtual System Ready"
        7. RUNNING                     -> green, "Virtual Cycle Running"
                                          (reason = job_id + step/total)
        8. otherwise (fallback)        -> gray

    Note: WAITING is a CabinetState — the cabinet's
    ``_re_evaluate_idle_readiness_locked`` helper decides whether to
    transition into or out of it. This function merely derives the
    status-light from the cabinet state passed in. It MUST NOT mutate
    cabinet state.
    """
    # 1. E-stop wins
    if cabinet_state == CabinetState.ESTOP or safety_state == SafetyState.ESTOP:
        return _blinking_red(
            reason_zh="虛擬急停已觸發；所有運動已被封鎖。",
            reason_en="Virtual E-stop is active. All motion is blocked.",
        )

    # 2. Protective stop — SAFETY rule: safety_gate_closed=False is the
    # primary safety stop. Other missing readiness inputs only produce
    # protective-stop when the cabinet was already in that state (and the
    # gate is closed but something else tripped it).
    if (cabinet_state == CabinetState.PROTECTIVE_STOP
            or safety_state == SafetyState.PROTECTIVE_STOP):
        # Safety gate is THE safety condition. If it's open, that's the
        # reason — regardless of any other missing readiness input.
        if virtual_inputs.get("safety_gate_closed") is False:
            return _red(
                reason_zh="虛擬安全門未關閉",
                reason_en="Virtual Safety Gate Open",
            )
        # Gate is closed; surface the actual protective-stop reason.
        reason_zh: Optional[str] = motion_blocked_reason
        reason_en: Optional[str] = motion_blocked_reason
        if reason_zh is None:
            reason_zh = "虛擬防護停止已觸發；運動已封鎖。"
            reason_en = "Virtual protective stop active; motion blocked."
        return _red(reason_zh=reason_zh, reason_en=reason_en)

    # 3. Fault
    if cabinet_state == CabinetState.FAULT or safety_state == SafetyState.FAULT:
        return _red(
            reason_zh=(motion_blocked_reason
                       or "虛擬故障已記錄；運動已封鎖，等待重置。"),
            reason_en=(motion_blocked_reason
                       or "Virtual fault recorded; motion blocked, "
                          "reset required."),
        )

    # 4. Offline
    if cabinet_state == CabinetState.OFFLINE:
        return _gray(
            reason_zh="虛擬控制器尚未初始化。",
            reason_en="Virtual cabinet offline or disabled.",
        )

    # 5. WAITING -> yellow "Virtual Warning / Waiting".
    #    The cabinet's _re_evaluate_idle_readiness_locked decided this
    #    cabinet is in WAITING; we only render the colour + reason.
    #    Reason picks the FIRST missing non-safety readiness input
    #    (safety_gate_closed is intentionally never a yellow reason).
    if cabinet_state == CabinetState.WAITING and required_inputs:
        for sig in required_inputs:
            if sig == "safety_gate_closed":
                continue  # SAFETY — never Yellow
            v = virtual_inputs.get(sig)
            if v is None or v is False:
                zh, en = _YELLOW_REASONS.get(
                    sig, (f"等待輸入:{sig}", f"Waiting on input: {sig}")
                )
                return _yellow(reason_zh=zh, reason_en=en)
        # Fallback if all required_inputs satisfied but cabinet somehow
        # still in WAITING (should not happen — single source of truth is
        # the cabinet's re-eval helper). Surface a generic yellow.
        return _yellow(
            reason_zh="等待條件完成",
            reason_en="Virtual Warning / Waiting",
        )

    # 6. READY + all readiness true -> Green "Virtual System Ready"
    if cabinet_state == CabinetState.READY:
        return _green(
            label_zh="虛擬系統就緒",
            label_en="Virtual System Ready",
            reason_zh="虛擬系統就緒，所有 readiness 條件已滿足。",
            reason_en=("All virtual readiness conditions are satisfied."),
        )

    # 7. RUNNING -> Green "Virtual Cycle Running" with job + step context.
    if cabinet_state == CabinetState.RUNNING:
        job = active_job_id or "—"
        cur = current_step if current_step is not None else "—"
        tot = total_steps if total_steps is not None else "—"
        return _green(
            label_zh="虛擬工序執行中",
            label_en="Virtual Cycle Running",
            reason_zh=f"正在執行 {job}，第 {cur} / {tot} 步",
            reason_en=f"Running {job}, step {cur} / {tot}",
        )

    # 8. Fallback
    return _gray(
        reason_zh="虛擬系統狀態未知。",
        reason_en="Virtual system state is unknown.",
    )
