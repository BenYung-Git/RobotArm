"""Minimal VirtualRobot API surface for Phase 1 demo.

Provides the subset promised in the SDD:
  - connect()            — connect to local PyBullet simulator only
  - disconnect()         — close simulator
  - moveJ(joints)        — joint-space motion (smooth interpolation)
  - move_to_home()       — convenience wrapper
  - get_state()          — return RobotState snapshot
  - emergency_stop()     — set fail-closed flag
  - reset_estop()        — clear flag
  - set_joint_targets()  — for slider-driven control (slider writes
                            target, main loop interpolates)

Safety
------
- Joint limits are checked against the URDF-discovered limits.
- Emergency stop is fail-closed: any motion attempt while E-stopped
  raises ``EmergencyStopError``.
- The safety_guard module validates the connect target.
- PyBullet connection mode is always ``DIRECT`` (no network).
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Iterable

from ..exceptions import (
    EmergencyStopError,
    InvalidPoseError,
    JointLimitError,
    NotConnectedError,
    RobotAlreadyConnectedError,
    URDFAssetError,
)
from .robot_spec import RobotSpec, UR5_SPEC
from .robot_state import RobotState

DEFAULT_URDF = "/tmp/robotarm_demo/assets/ur5/urdf/ur5.standalone.urdf"

# Motion interpolation parameters
DEFAULT_STEP_HZ = 100.0  # control loop tick
DEFAULT_MAX_STEP_RAD = 0.2  # max per-tick joint delta (smooth motion)


class VirtualRobot:
    """6-DOF virtual robotic arm driven by local PyBullet simulator.

    Parameters
    ----------
    robot_id:
        Logical identifier (e.g. "ur5_demo").
    urdf_path:
        Absolute path to a local standalone URDF. Default points to the
        bundle downloaded in ``/tmp/robotarm_demo/assets/ur5/``.
    spec:
        Optional RobotSpec. If None, defaults to UR5_SPEC.
    """

    def __init__(
        self,
        robot_id: str = "ur5_demo",
        urdf_path: str = DEFAULT_URDF,
        spec: RobotSpec | None = None,
    ) -> None:
        self.robot_id = robot_id
        self._urdf_path = urdf_path
        self._spec = spec or UR5_SPEC

        # Connection state
        self._client: int | None = None
        self._body_id: int | None = None
        self._connected = False
        self._e_stopped = False

        # Joint target (where the controller is moving TO).
        # 6-tuple of radians; populated by moveJ / move_to_home / sliders.
        self._joint_targets: tuple[float, ...] = tuple(0.0 for _ in range(self._spec.dof))

        # Cached EE link index (resolved after connect)
        self._ee_link_index_cache: int | None = None

        # Initialise safety_guard by referencing it (ensures guard ran at
        # import time and at first connect).
        from .. import safety_guard  # noqa: F401  (import for side effects)

    # --- Properties --------------------------------------------------

    @property
    def spec(self) -> RobotSpec:
        return self._spec

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_e_stopped(self) -> bool:
        return self._e_stopped

    @property
    def joint_targets(self) -> tuple[float, ...]:
        return self._joint_targets

    # --- Connection --------------------------------------------------

    def connect(self, target: str | None = "loopback", gui: bool = False) -> bool:
        """Connect to local PyBullet simulator.

        ``target`` must be one of: None, "loopback", "local", "direct",
        "gui". Anything else (IPs, hostnames, vendor URLs) is rejected
        by the safety guard.
        """
        # Defer import to allow tests to mock pybullet.
        from .. import safety_guard

        safety_guard.validate_connect_target(target)

        if self._connected:
            raise RobotAlreadyConnectedError(
                f"Robot {self.robot_id} is already connected"
            )

        if not os.path.isfile(self._urdf_path):
            raise URDFAssetError(
                f"URDF not found at {self._urdf_path!r}. "
                f"Run the install setup to download the asset bundle."
            )

        import pybullet as p
        import pybullet_data  # noqa: F401  (verify pkg available)

        # Map target -> pybullet connect mode
        if target == "gui":
            mode = p.GUI
        else:
            # loopback / local / direct / None → all map to DIRECT
            mode = p.DIRECT

        self._client = p.connect(mode)
        if self._client < 0:
            raise URDFAssetError(f"pyullet.connect({mode}) returned {self._client}")

        # Set the search path so mesh filenames can resolve if relative.
        search_path = str(Path(self._urdf_path).parent.parent)
        p.setAdditionalSearchPath(search_path)
        p.setGravity(0, 0, -9.81)

        # Load URDF
        try:
            self._body_id = p.loadURDF(self._urdf_path)
        except Exception as e:  # pybullet.error
            p.disconnect(self._client)
            self._client = None
            raise URDFAssetError(f"Failed to load URDF {self._urdf_path!r}: {e}")

        # Reset all joints to zero
        for idx in self._spec.controllable_joint_indices:
            p.resetJointState(self._body_id, idx, 0.0)

        # Step the simulation once so joint states are committed
        for _ in range(10):
            p.stepSimulation()

        self._connected = True
        self._e_stopped = False
        return True

    def disconnect(self) -> None:
        """Disconnect from simulator."""
        if not self._connected:
            return
        import pybullet as p

        if self._client is not None:
            try:
                p.disconnect(self._client)
            except Exception:
                pass
        self._client = None
        self._body_id = None
        self._connected = False

    # --- E-stop ------------------------------------------------------

    def emergency_stop(self) -> None:
        """Set E-stop. All motion will be refused until reset."""
        self._e_stopped = True

    def reset_estop(self) -> None:
        """Clear E-stop."""
        self._e_stopped = False

    # --- Motion ------------------------------------------------------

    def _check_connected(self) -> None:
        if not self._connected:
            raise NotConnectedError(
                f"Robot {self.robot_id} is not connected. Call connect() first."
            )

    def _check_estop(self) -> None:
        if self._e_stopped:
            raise EmergencyStopError(
                f"E-stop is active on {self.robot_id}. Call reset_estop() to clear."
            )

    def _validate_targets(self, joints: Iterable[float]) -> tuple[float, ...]:
        targets = tuple(float(j) for j in joints)
        if len(targets) != self._spec.dof:
            raise InvalidPoseError(
                f"Expected {self._spec.dof} joint values, got {len(targets)}"
            )
        for i, (val, lo, hi, name) in enumerate(
            zip(
                targets,
                self._spec.joint_lower_limits_rad,
                self._spec.joint_upper_limits_rad,
                self._spec.controllable_joint_names,
            )
        ):
            if not math.isfinite(val):
                raise InvalidPoseError(
                    f"Joint {i} ({name}) target {val!r} is not finite"
                )
            if val < lo or val > hi:
                raise JointLimitError(i, name, val, lo, hi)
        return targets

    def moveJ(self, joints: Iterable[float], steps: int | None = None) -> None:
        """Smooth joint-space motion to the given target.

        ``joints`` is a 6-tuple of radians. The motion is interpolated
        across ``steps`` control ticks (default: derived from max joint
        delta / DEFAULT_MAX_STEP_RAD).
        """
        self._check_connected()
        self._check_estop()
        targets = self._validate_targets(joints)

        import pybullet as p

        current = self._get_joint_positions()
        if steps is None:
            max_delta = max(abs(t - c) for t, c in zip(targets, current))
            steps = max(1, int(math.ceil(max_delta / DEFAULT_MAX_STEP_RAD)))

        # Interpolate
        for s in range(1, steps + 1):
            alpha = s / steps
            interp = tuple(
                current[i] + (targets[i] - current[i]) * alpha for i in range(self._spec.dof)
            )
            self._apply_targets(interp)
            p.stepSimulation()
            if self._e_stopped:
                # If E-stop triggered mid-motion, stop interpolation
                raise EmergencyStopError(
                    "E-stop triggered during moveJ — motion aborted"
                )

        self._joint_targets = targets

    def set_joint_targets(self, joints: Iterable[float]) -> None:
        """Set targets without interpolation (used by sliders in GUI tick).

        Validates limits and E-stop. Does NOT call stepSimulation;
        the GUI loop is responsible for stepping and interpolating.
        """
        self._check_connected()
        self._check_estop()
        targets = self._validate_targets(joints)
        self._joint_targets = targets

    def step_toward_targets(self, max_step_rad: float = DEFAULT_MAX_STEP_RAD) -> bool:
        """Take one interpolation step toward current joint targets.

        Returns True if motion is still in progress, False if at target.
        Caller must call ``pybullet.stepSimulation()`` after.
        """
        self._check_connected()
        if self._e_stopped:
            raise EmergencyStopError(
                "step_toward_targets refused: E-stop is active"
            )
        current = self._get_joint_positions()
        interp = []
        moved = False
        for cur, tgt in zip(current, self._joint_targets):
            delta = tgt - cur
            if abs(delta) <= max_step_rad:
                interp.append(tgt)
            else:
                interp.append(cur + math.copysign(max_step_rad, delta))
                moved = True
        self._apply_targets(tuple(interp))
        return moved

    def move_to_home(self, steps: int | None = None) -> None:
        """Convenience: move to all-zero joint pose (UR5 'home')."""
        self.moveJ([0.0] * self._spec.dof, steps=steps)

    def _apply_targets(self, joints: tuple[float, ...]) -> None:
        """Write joint positions directly to PyBullet."""
        import pybullet as p

        assert self._body_id is not None
        for idx, val in zip(self._spec.controllable_joint_indices, joints):
            p.resetJointState(self._body_id, idx, val)

    def _get_joint_positions(self) -> tuple[float, ...]:
        """Read current joint positions (radians) for controllable joints."""
        import pybullet as p

        assert self._body_id is not None
        result = []
        for idx in self._spec.controllable_joint_indices:
            state = p.getJointState(self._body_id, idx)
            result.append(state[0])  # position
        return tuple(result)

    # --- State -------------------------------------------------------

    def get_state(self) -> RobotState:
        """Return current state snapshot. Does not require connection."""
        joints = (
            self._get_joint_positions()
            if self._connected
            else tuple(0.0 for _ in range(self._spec.dof))
        )

        if self._connected:
            import pybullet as p

            assert self._body_id is not None
            ee_idx = self._ee_link_index()
            link_state = p.getLinkState(self._body_id, ee_idx)
            tcp_pos = tuple(link_state[0])
            tcp_orn = tuple(link_state[1])
        else:
            tcp_pos = (0.0, 0.0, 0.0)
            tcp_orn = (0.0, 0.0, 0.0, 1.0)

        return RobotState(
            robot_id=self.robot_id,
            mode="virtual",
            joints=joints,
            tcp_position=tcp_pos,
            tcp_orientation=tcp_orn,
            connected=self._connected,
            e_stopped=self._e_stopped,
        )

    # --- Context manager ---------------------------------------------

    def __enter__(self) -> "VirtualRobot":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # --- EE link index lookup ----------------------------------------

    def _ee_link_index(self) -> int:
        """Resolve EE link name to PyBullet link index.

        Cached on the VirtualRobot instance after first resolution.
        """
        if self._ee_link_index_cache is not None:
            return self._ee_link_index_cache
        import pybullet as p

        assert self._body_id is not None
        n = p.getNumJoints(self._body_id)
        for j in range(n):
            info = p.getJointInfo(self._body_id, j)
            link_name = info[12].decode("utf-8") if info[12] else ""
            if link_name == self._spec.ee_link_name:
                self._ee_link_index_cache = j
                return j
        # If not found, use the last link index (= n)
        self._ee_link_index_cache = n
        return n
