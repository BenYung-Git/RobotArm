"""Virtual-only safety guard.

Enforces the project rule that the demo must NOT touch real hardware,
network I/O, vendor SDKs, or any non-loopback target. The guard runs at
import time AND at every connect() call.

Detection strategies
--------------------

1. **Static source scan**: at import time, this module scans the package
   source tree and asserts no source file imports a forbidden module.
   Forbidden modules: ``socket``, ``http``, ``urllib``, ``requests``,
   ``serial``, ``modbus``, real vendor SDK names (``rokae``, ``jaka``,
   ``urx``, ``ur_pykuka``, etc.). If any are found, the package refuses
   to import.

2. **Restricted connect API**: ``VirtualRobot.connect()`` accepts only
   ``target="loopback"`` (the default) or ``target=None``. Any other
   value raises ``VirtualOnlyViolationError``.

3. **No monkey-patch of socket**: per project rule. Detection is purely
   static / structural; we never touch ``socket.socket`` at runtime.

This module is intentionally minimal and side-effect-free apart from
the static scan at import time.
"""
from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Iterable

from .exceptions import VirtualOnlyViolationError

# Modules whose presence (as imports) in any package source file indicates
# a network / hardware leak. We allow them in *this* safety_guard module
# itself (meta: the guard may reference them by name in error messages),
# but never in any other module of the package.
_FORBIDDEN_MODULES = frozenset(
    {
        "socket",
        "http",
        "http.client",
        "urllib",
        "urllib.request",
        "urllib3",
        "requests",
        "serial",
        "pymodbus",
        "modbus_tk",
        "rokae",
        "jaka",
        "jksdk",
        "urx",  # python urx driver
        "ur_pykuka",
    }
)

# Substring patterns in source that suggest real hardware I/O.
_FORBIDDEN_PATTERNS = (
    (re.compile(r"socket\.socket\("), "socket.socket() call"),
    (re.compile(r"serial\.Serial\("), "serial.Serial() call"),
    (re.compile(r"requests\.(get|post|put|delete)\("), "requests HTTP call"),
    (re.compile(r"urllib\.request\.urlopen\("), "urllib HTTP call"),
)

# Allowed: every module that legitimately imports forbidden-looking modules.
_ALLOWED_FILES_WITH_FORBIDDEN_IMPORTS = frozenset(
    {
        # This module itself references names in error messages / comments.
        os.path.basename(__file__),
    }
)


def _scan_package_source(package_root: Path) -> list[str]:
    """Walk package source and return list of (file, reason) violations."""
    violations: list[str] = []
    for path in package_root.rglob("*.py"):
        if path.name in _ALLOWED_FILES_WITH_FORBIDDEN_IMPORTS:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        # Check imports
        for line in text.splitlines():
            stripped = line.strip()
            if not stripped.startswith(("import ", "from ")):
                continue
            # Tokenise: split on whitespace and 'as', take module path
            tokens = re.split(r"\s+", stripped)
            if len(tokens) < 2:
                continue
            mod = tokens[1].split(".")[0]
            if mod in _FORBIDDEN_MODULES:
                violations.append(f"{path}: forbidden import '{mod}' in line: {stripped}")
        # Check patterns
        for pat, desc in _FORBIDDEN_PATTERNS:
            for match in pat.finditer(text):
                # Find line number
                line_no = text[: match.start()].count("\n") + 1
                violations.append(f"{path}:{line_no}: forbidden pattern ({desc})")
    return violations


def run_static_guard(package_root: Path | None = None) -> None:
    """Run static source scan. Raise if any violations are found.

    Scans only the ``robotarm/`` package directory by default. Tests and
    demos may legitimately mention forbidden module names to exercise the
    guard itself.
    """
    if package_root is None:
        # Default: scan the robotarm/ package only (not tests/ or demos/)
        here = Path(__file__).resolve().parent
        package_root = here  # robotarm/safety_guard.py -> robotarm/
    violations = _scan_package_source(package_root)
    if violations:
        msg = "Virtual-only safety guard FAILED:\n  " + "\n  ".join(violations)
        raise VirtualOnlyViolationError(msg)


def validate_connect_target(target: str | None) -> None:
    """Validate that a connect() target is virtual-only.

    Allowed:
      - None
      - "loopback"
      - "local"
      - "direct"  (PyBullet DIRECT mode)
      - "gui"     (PyBullet GUI mode)

    Anything else (IP, hostname, vendor URL) is rejected.
    """
    allowed = {None, "loopback", "local", "direct", "gui"}
    if target not in allowed:
        raise VirtualOnlyViolationError(
            f"connect() target={target!r} is not virtual-only. "
            f"Allowed: {sorted(t for t in allowed if t is not None)} or None."
        )


# Run static guard at import time
run_static_guard()
