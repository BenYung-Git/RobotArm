"""Test: URDF asset exists locally, loads in PyBullet, has 6 controllable revolute joints.

This test never touches the network — it asserts that the local bundle
in /tmp/robotarm_demo/assets/ur5/ is intact and loadable.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

ASSETS = Path("/tmp/robotarm_demo/assets/ur5")
URDF = ASSETS / "urdf" / "ur5.standalone.urdf"


def test_assets_directory_exists():
    assert ASSETS.is_dir(), f"Asset dir missing: {ASSETS}"


def test_standalone_urdf_exists():
    assert URDF.is_file(), f"Standalone URDF missing: {URDF}"


def test_manifest_exists():
    manifest = ASSETS.parent / "ASSET_MANIFEST.md"
    assert manifest.is_file(), f"ASSET_MANIFEST.md missing"
    assert "UR5" in manifest.read_text()


def test_mesh_files_present():
    for sub in ("visual", "collision"):
        d = ASSETS / "meshes" / "ur5" / sub
        assert d.is_dir(), f"Mesh dir missing: {d}"
        # 7 expected pieces for UR5 (base, shoulder, upperarm, forearm, wrist1/2/3)
        files = list(d.glob("*"))
        assert len(files) >= 7, f"Too few meshes in {d}: got {len(files)}"


def test_urdf_loads_in_pybullet_with_6_revolute_joints():
    """Connect PyBullet in DIRECT mode (no GUI, no network) and verify joint discovery."""
    pybullet = pytest.importorskip("pybullet")

    client = pybullet.connect(pybullet.DIRECT)
    try:
        pybullet.setAdditionalSearchPath(str(ASSETS))
        body = pybullet.loadURDF(str(URDF))
        n = pybullet.getNumJoints(body)
        assert n > 0, "URDF loaded but has no joints"

        revolute = []
        for i in range(n):
            info = pybullet.getJointInfo(body, i)
            name = info[1].decode()
            jtype = info[2]
            lo, hi = info[8], info[9]
            if jtype == pybullet.JOINT_REVOLUTE and lo < hi:
                revolute.append((i, name, lo, hi))

        assert len(revolute) == 6, (
            f"Expected exactly 6 controllable revolute joints, got {len(revolute)}: "
            f"{[n for _, n, _, _ in revolute]}"
        )
        # Verify expected joint names
        names = [n for _, n, _, _ in revolute]
        assert "shoulder_pan_joint" in names
        assert "elbow_joint" in names
        assert "wrist_1_joint" in names

        # Verify elbow has the tighter ±π limit
        elbow = next(r for r in revolute if r[1] == "elbow_joint")
        import math
        assert abs(elbow[2] - (-math.pi)) < 1e-3
        assert abs(elbow[3] - math.pi) < 1e-3
    finally:
        pybullet.disconnect(client)


def test_no_network_during_runtime():
    """Assert no remote URL is fetched at runtime — URDF paths must all be local."""
    text = URDF.read_text()
    # After setup-time rewriting, all mesh refs should be file:// or local relative paths
    # We should NOT see package:// anymore
    assert "package://" not in text, (
        "URDF still contains package:// references; mesh paths not rewritten for local use"
    )
    # And every mesh reference should be either file:// or relative
    for line in text.splitlines():
        if "filename=" in line:
            assert ("file://" in line) or ("/" not in line.split('"')[1] or line.split('"')[1].startswith("/")), (
                f"Suspicious mesh path in URDF: {line.strip()}"
            )
