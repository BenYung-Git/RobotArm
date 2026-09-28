"""Test: virtual-only safety guard rejects forbidden targets / non-loopback."""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from robotarm.core.virtual_robot import VirtualRobot
from robotarm.exceptions import VirtualOnlyViolationError


PACKAGE_ROOT = Path("/tmp/robotarm_demo/robotarm")


def test_static_guard_runs_at_import():
    """Importing the package triggers the static source scan."""
    # If safety_guard fails on import, importing VirtualRobot would have failed
    from robotarm import safety_guard  # noqa: F401
    # Run it explicitly against the robotarm/ package directory
    safety_guard.run_static_guard(PACKAGE_ROOT)


def test_static_guard_flags_socket_import():
    """A source file containing 'import socket' should be flagged."""
    # Create a temp source file with a forbidden import
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        fake_pkg = Path(td) / "fake_pkg"
        fake_pkg.mkdir()
        bad = fake_pkg / "leak.py"
        bad.write_text("import socket\n")
        with pytest.raises(VirtualOnlyViolationError) as excinfo:
            from robotarm.safety_guard import run_static_guard
            run_static_guard(fake_pkg)
        assert "socket" in str(excinfo.value)


def test_static_guard_flags_requests_import():
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        fake_pkg = Path(td) / "fake_pkg2"
        fake_pkg.mkdir()
        bad = fake_pkg / "http_call.py"
        bad.write_text("import requests\n")
        with pytest.raises(VirtualOnlyViolationError) as excinfo:
            from robotarm.safety_guard import run_static_guard
            run_static_guard(fake_pkg)
        assert "requests" in str(excinfo.value)


def test_static_guard_flags_socket_pattern():
    """Even without import, calling socket.socket() should be flagged."""
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        fake_pkg = Path(td) / "fake_pkg3"
        fake_pkg.mkdir()
        bad = fake_pkg / "leak2.py"
        bad.write_text("s = socket.socket(AF_INET, SOCK_STREAM)\n")
        with pytest.raises(VirtualOnlyViolationError) as excinfo:
            from robotarm.safety_guard import run_static_guard
            run_static_guard(fake_pkg)
        assert "socket" in str(excinfo.value).lower()


def test_validate_connect_target_accepts_allowed():
    from robotarm.safety_guard import validate_connect_target
    for t in (None, "loopback", "local", "direct", "gui"):
        validate_connect_target(t)  # should NOT raise


def test_validate_connect_target_rejects_ip():
    from robotarm.safety_guard import validate_connect_target
    for t in ("192.168.1.100", "10.0.0.1", "robot.local", "http://controller"):
        with pytest.raises(VirtualOnlyViolationError):
            validate_connect_target(t)


def test_robot_connect_rejects_non_loopback_target():
    """Connecting with a real IP must raise VirtualOnlyViolationError."""
    pybullet = pytest.importorskip("pybullet")
    r = VirtualRobot(robot_id="safety_test")
    with pytest.raises(VirtualOnlyViolationError):
        r.connect(target="192.168.1.100")


def test_robot_connect_accepts_loopback():
    pybullet = pytest.importorskip("pybullet")
    r = VirtualRobot(robot_id="safety_test_ok")
    try:
        r.connect(target="loopback")
        assert r.is_connected
    finally:
        r.disconnect()


def test_no_real_sdk_imports_in_package():
    """Walk all package files; ensure no real SDK imports anywhere."""
    forbidden = {"rokae", "jaka", "jksdk", "urx", "ur_pykuka",
                 "requests", "serial"}
    for path in PACKAGE_ROOT.rglob("*.py"):
        text = path.read_text()
        for line in text.splitlines():
            stripped = line.strip()
            if not stripped.startswith(("import ", "from ")):
                continue
            tokens = re.split(r"\s+", stripped)
            if len(tokens) < 2:
                continue
            mod = tokens[1].split(".")[0]
            if mod in forbidden:
                pytest.fail(f"Forbidden import {mod!r} in {path}: {stripped}")
