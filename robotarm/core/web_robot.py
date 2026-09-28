"""WebRobot — in-memory 6-DOF arm simulator for the web demo.

Reuses the VirtualRobot API surface (connect / moveJ / set_joint_targets /
move_to_home / get_state / emergency_stop / reset_estop) but backs it
with a pure-Python joint state list and a simplified forward-kinematics
proxy for TCP position. No PyBullet, no network, no real hardware.

The intent is NOT a physically accurate UR5 model; it is a deterministic,
responsive, network-friendly state store that the web UI can drive and
display joint angles + an approximate TCP position.
"""
from __future__ import annotations

import math

from ..core.robot_spec import RobotSpec, UR5_SPEC
from ..core.robot_state import RobotState


# Approximate link-length factors for the UR5 (in metres), tuned so the
# "TCP xyz" display shows plausible values for typical demo poses. These
# are NOT official UR5 DH parameters — they are visual proxies only.
_LINK_LENGTHS = (
    0.0,   # base offset (m)
    0.425, # shoulder offset
    0.392, # upper arm
    0.094, # forearm (offset to elbow)
    0.094, # wrist 1
    0.082, # wrist 2 (small)
)


class WebRobot:
    """Pure-Python 6-DOF virtual arm, mirroring VirtualRobot API.

    Joint state and limits are real (read from RobotSpec); TCP is a
    lightweight kinematic approximation suitable for the web display.
    """

    def __init__(
        self,
        robot_id: str = "ur5_web_demo",
        spec=None,
        max_step_rad: float | None = None,
    ) -> None:
        self.robot_id = robot_id
        self._spec = spec or UR5_SPEC

        # State
        self._connected = False
        self._e_stopped = False
        self._joints: list[float] = [0.0] * self._spec.dof
        self._joint_targets: tuple[float, ...] = tuple(0.0 for _ in range(self._spec.dof))
        # For the animation loop: a target interpolation step is
        # Max joint radians moved per ``tick()``. Default ~3 deg/tick at 60Hz
        # (= 3 rad/s). Overridable for slow-motion demos via constructor.
        import os as _os
        if max_step_rad is None:
            _env = _os.environ.get("ROBOTARM_MAX_STEP_RAD")
            self._max_step_rad = float(_env) if _env else 0.05
        else:
            self._max_step_rad = float(max_step_rad)

    # --- API contract mirroring VirtualRobot -------------------------

    def connect(self) -> bool:
        self._connected = True
        self._e_stopped = False
        return True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_e_stopped(self) -> bool:
        return self._e_stopped

    @property
    def joint_targets(self) -> tuple[float, ...]:
        return self._joint_targets

    @property
    def joints(self) -> tuple[float, ...]:
        """Current joint angles (rad). Read-only view for status / sync."""
        return tuple(self._joints)

    @property
    def spec(self) -> RobotSpec:
        return self._spec

    def emergency_stop(self) -> None:
        self._e_stopped = True

    def reset_estop(self) -> None:
        self._e_stopped = False

    def _validate(self, joints) -> tuple[float, ...]:
        targets = tuple(float(j) for j in joints)
        if len(targets) != self._spec.dof:
            from ..exceptions import InvalidPoseError
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
                from ..exceptions import InvalidPoseError
                raise InvalidPoseError(f"Joint {i} ({name}) target {val!r} not finite")
            if val < lo or val > hi:
                from ..exceptions import JointLimitError
                raise JointLimitError(i, name, val, lo, hi)
        return targets

    def set_joint_targets(self, joints) -> None:
        from ..exceptions import NotConnectedError, EmergencyStopError
        if not self._connected:
            raise NotConnectedError(f"Robot {self.robot_id} not connected")
        if self._e_stopped:
            raise EmergencyStopError(f"E-stop active on {self.robot_id}")
        self._joint_targets = self._validate(joints)

    def moveJ(self, joints, steps: int | None = None) -> None:
        """Snap motion: just set targets and tick to completion.

        ``steps`` is accepted for API parity with VirtualRobot; the web
        loop uses ``tick()`` to animate smoothly regardless.
        """
        self.set_joint_targets(joints)
        # Optionally snap to target immediately if steps is small / zero
        if steps == 0:
            self._joints = list(self._joint_targets)

    def move_to_home(self, steps: int | None = None) -> None:
        self.moveJ([0.0] * self._spec.dof, steps=steps)

    def tick(self) -> bool:
        """Advance one interpolation step toward joint targets.

        Returns True if motion is still in progress.
        Raises EmergencyStopError if E-stop is set.
        """
        from ..exceptions import EmergencyStopError
        if self._e_stopped:
            raise EmergencyStopError(f"Cannot tick: E-stop active on {self.robot_id}")
        moved = False
        for i in range(self._spec.dof):
            cur = self._joints[i]
            tgt = self._joint_targets[i]
            delta = tgt - cur
            if abs(delta) <= self._max_step_rad:
                self._joints[i] = tgt
            else:
                self._joints[i] = cur + math.copysign(self._max_step_rad, delta)
                moved = True
        return moved

    def get_state(self) -> RobotState:
        tcp_pos, tcp_orn = self._approx_fk()
        return RobotState(
            robot_id=self.robot_id,
            mode="virtual",
            joints=tuple(self._joints),
            tcp_position=tcp_pos,
            tcp_orientation=tcp_orn,
            connected=self._connected,
            e_stopped=self._e_stopped,
        )

    # --- Lightweight forward-kinematics proxy -----------------------

    def _approx_fk(self) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Crude planar-style reach estimate.

        Returns ``((x, y, z), (qx, qy, qz, qw))``. Not a real FK — purely
        for the web status display. Treat the numbers as "approximate
        reach envelope" rather than authoritative pose.
        """
        # Project to X-Z plane using shoulder_pan for direction, plus
        # shoulder_lift / elbow / wrist contributions for reach length.
        if len(self._joints) < 6:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

        j0, j1, j2, j3, j4, j5 = self._joints[:6]
        base_h = _LINK_LENGTHS[0]
        L1 = _LINK_LENGTHS[1]
        L2 = _LINK_LENGTHS[2]
        L3 = _LINK_LENGTHS[3]
        L4 = _LINK_LENGTHS[4]
        L5 = _LINK_LENGTHS[5]

        # Planar arm angle in shoulder plane (j1 + j2 + j3)
        planar = j1 + j2 + j3
        # Reach envelope in X-Z (ignoring wrist for simplicity)
        r = base_h + L1 + L2 * math.cos(j2) + L3 * math.cos(planar)
        x = r * math.cos(j0)
        y = r * math.sin(j0)
        z = base_h + L1 + L2 * math.sin(j2) + L3 * math.sin(planar)
        # Quaternion: identity for the demo display
        return ((x, y, z), (0.0, 0.0, 0.0, 1.0))
