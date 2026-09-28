"""Virtual I/O — in-memory simulation signals for the cabinet interlock.

Phase 1C rule (per Ben Yung, 2026-09-16):
    "Virtual I/O 只作記憶體 session simulation, 不回寫 JSON."

This module therefore:
  * loads a SEED configuration from ``data/virtual_io/default.json`` ONCE
    at construction time;
  * holds the current state purely in memory;
  * exposes setters that ONLY mutate the in-memory dict;
  * never writes back to disk.

The seed file is required. If it is missing, malformed, or has a wrong
schema, construction raises — never silently fabricates data.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from robotarm.exceptions import JobValidationError


SCHEMA_VERSION = 1

# Signal names allowed to be mutated at runtime (whitelist).
# Anything else in the seed is treated as read-only reference info.
# (In Phase 1C, only inputs are user-toggleable; outputs are managed by
# the cabinet/job runner.)
_ALLOWED_INPUT_SIGNALS: frozenset[str] = frozenset({
    "part_present",
    "fixture_clamped",
    "welder_ready",
    "safety_gate_closed",
})

_REQUIRED_INPUT_SIGNALS: frozenset[str] = frozenset({
    "part_present",
    "fixture_clamped",
    "welder_ready",
    "safety_gate_closed",
})


@dataclass
class VirtualIO:
    """In-memory virtual I/O bundle.

    ``inputs`` and ``outputs`` are plain dicts of ``str -> bool``. The
    object is intentionally NOT thread-safe at the dict level — the
    Flask single-threaded dev server plus the per-job ``threading.Lock``
    in the cabinet is sufficient for a demo. Tests that need concurrency
    should construct their own lock.
    """

    inputs: dict[str, bool] = field(default_factory=dict)
    outputs: dict[str, bool] = field(default_factory=dict)
    mode: str = "virtual_only"
    schema_version: int = SCHEMA_VERSION

    # ------------------------------------------------------------------
    # Loading (one-shot, seed-only)
    # ------------------------------------------------------------------

    @classmethod
    def from_seed_file(cls, path: str | Path) -> "VirtualIO":
        """Load and validate a seed I/O configuration from ``path``.

        Raises:
            FileNotFoundError: if the file does not exist.
            JobValidationError: on any validation failure (malformed JSON,
                wrong schema_version, missing keys, non-boolean values,
                unknown signal names).
        """
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"Virtual I/O seed file not found: {p}")

        try:
            raw = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            raise JobValidationError(
                f"Virtual I/O seed file is not valid JSON: {e}"
            ) from e

        if not isinstance(raw, dict):
            raise JobValidationError(
                "Virtual I/O seed file must contain a JSON object at top level."
            )

        # schema_version
        sv = raw.get("schema_version")
        if sv != SCHEMA_VERSION:
            raise JobValidationError(
                f"Virtual I/O seed schema_version must be {SCHEMA_VERSION}, "
                f"got {sv!r}"
            )

        mode = raw.get("mode")
        if mode != "virtual_only":
            raise JobValidationError(
                f"Virtual I/O seed mode must be 'virtual_only', got {mode!r}"
            )

        # inputs / outputs
        inputs_raw = raw.get("inputs")
        outputs_raw = raw.get("outputs")
        if not isinstance(inputs_raw, dict):
            raise JobValidationError("Virtual I/O seed 'inputs' must be a dict.")
        if not isinstance(outputs_raw, dict):
            raise JobValidationError("Virtual I/O seed 'outputs' must be a dict.")

        inputs = _validate_signal_dict(
            inputs_raw, where="inputs", allow_names=_ALLOWED_INPUT_SIGNALS,
        )
        # outputs are read-only from the seed; we still validate types.
        outputs = _validate_signal_dict(
            outputs_raw, where="outputs", allow_names=None,
        )

        # All required inputs must be present.
        missing_required = [s for s in _REQUIRED_INPUT_SIGNALS if s not in inputs]
        if missing_required:
            raise JobValidationError(
                "Virtual I/O seed missing required inputs: "
                + ", ".join(missing_required)
            )

        return cls(
            inputs=inputs,
            outputs=outputs,
            mode=mode,
            schema_version=sv,
        )

    # ------------------------------------------------------------------
    # Interlock check
    # ------------------------------------------------------------------

    def check_interlock(self, required: list[str]) -> list[str]:
        """Return the list of required inputs that are NOT satisfied.

        "Satisfied" means the input exists AND its value is True.

        An empty return value means the interlock passes.
        """
        missing: list[str] = []
        for sig in required:
            val = self.inputs.get(sig)
            if val is None:
                missing.append(f"{sig}(missing)")
            elif val is not True:
                missing.append(f"{sig}={val}")
        return missing

    # ------------------------------------------------------------------
    # Mutation (input toggles only)
    # ------------------------------------------------------------------

    def set_input(self, signal: str, value: bool) -> None:
        """Set a single input. Only whitelisted input names are accepted.

        Raises:
            JobValidationError: if ``signal`` is not in the whitelist.
            ValueError: if ``value`` is not strictly a Python ``bool``.
        """
        if signal not in _ALLOWED_INPUT_SIGNALS:
            raise JobValidationError(
                f"Cannot set virtual input {signal!r}: not in whitelist "
                f"{sorted(_ALLOWED_INPUT_SIGNALS)}"
            )
        if not isinstance(value, bool):
            # Reject int 0/1, str 'true', etc. — strict booleans only.
            raise ValueError(
                f"Virtual input {signal!r} value must be bool, got {type(value).__name__}"
            )
        self.inputs[signal] = value

    def set_output(self, signal: str, value: bool) -> None:
        """Set an output (cabinet-internal use). No whitelist; type-checked."""
        if not isinstance(value, bool):
            raise ValueError(
                f"Virtual output {signal!r} value must be bool, got {type(value).__name__}"
            )
        self.outputs[signal] = value

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        """Return a JSON-safe dict representation for the status payload."""
        return {
            "inputs": dict(self.inputs),
            "outputs": dict(self.outputs),
            "mode": self.mode,
            "schema_version": self.schema_version,
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _validate_signal_dict(
    raw: dict[str, Any],
    *,
    where: str,
    allow_names: frozenset[str] | None,
) -> dict[str, bool]:
    """Validate a {signal: bool} dict.

    - Reject non-string keys.
    - Reject non-boolean values.
    - If ``allow_names`` is provided, reject keys outside that set.
    """
    out: dict[str, bool] = {}
    for k, v in raw.items():
        if not isinstance(k, str):
            raise JobValidationError(
                f"Virtual I/O {where} key must be a string, got {type(k).__name__}"
            )
        if allow_names is not None and k not in allow_names:
            raise JobValidationError(
                f"Virtual I/O {where} contains unknown signal {k!r}; "
                f"allowed: {sorted(allow_names)}"
            )
        # Strict bool: reject int/float/str.
        if not isinstance(v, bool):
            raise JobValidationError(
                f"Virtual I/O {where}[{k!r}] must be a JSON boolean, "
                f"got {type(v).__name__} ({v!r})"
            )
        out[k] = v
    return out
