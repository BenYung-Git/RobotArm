"""Load a 6-DOF pose definition from JSON.

Expected JSON format::

    {
        "name": "Home",
        "description": "All-zero joint pose",
        "joints_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

The function enforces:
- File exists (does NOT silently create defaults).
- File parses as JSON.
- joints_rad is a list of 6 finite numbers.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any


class PoseLoadError(Exception):
    """Raised when a pose JSON file is missing or malformed."""


_REQUIRED_KEYS = ("joints_rad",)


def load_pose(path: str | os.PathLike) -> tuple[float, ...]:
    """Load a 6-tuple of joint angles (radians) from a JSON file.

    Raises ``PoseLoadError`` if the file is missing, malformed, or
    contains out-of-range / non-finite values.
    """
    p = Path(path)
    if not p.exists():
        raise PoseLoadError(
            f"Pose file not found: {p!s}. "
            f"Refusing to fabricate default pose data; create the file first."
        )

    try:
        raw = p.read_text(encoding="utf-8")
    except OSError as e:
        raise PoseLoadError(f"Cannot read pose file {p!s}: {e}") from e

    try:
        data: Any = json.loads(raw)
    except json.JSONDecodeError as e:
        raise PoseLoadError(f"Pose file {p!s} is not valid JSON: {e}") from e

    if not isinstance(data, dict):
        raise PoseLoadError(
            f"Pose file {p!s}: expected JSON object at top level, got {type(data).__name__}"
        )

    for key in _REQUIRED_KEYS:
        if key not in data:
            raise PoseLoadError(f"Pose file {p!s}: missing required key {key!r}")

    joints_raw = data["joints_rad"]
    if not isinstance(joints_raw, list):
        raise PoseLoadError(
            f"Pose file {p!s}: 'joints_rad' must be a list, got {type(joints_raw).__name__}"
        )
    if len(joints_raw) != 6:
        raise PoseLoadError(
            f"Pose file {p!s}: 'joints_rad' must have 6 elements, got {len(joints_raw)}"
        )

    joints: list[float] = []
    for i, v in enumerate(joints_raw):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise PoseLoadError(
                f"Pose file {p!s}: joints_rad[{i}] must be a number, got {type(v).__name__}"
            )
        fv = float(v)
        if not math.isfinite(fv):
            raise PoseLoadError(
                f"Pose file {p!s}: joints_rad[{i}] = {fv!r} is not finite"
            )
        joints.append(fv)

    return tuple(joints)
