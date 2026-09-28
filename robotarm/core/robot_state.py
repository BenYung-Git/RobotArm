"""Snapshot of robot state returned by ``VirtualRobot.get_state()``."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotState:
    """Immutable snapshot of the virtual robot at one instant."""

    robot_id: str
    mode: str  # always "virtual"
    joints: tuple[float, ...]  # 6 joint angles in radians
    tcp_position: tuple[float, ...]  # (x, y, z) in meters
    tcp_orientation: tuple[float, ...]  # (qx, qy, qz, qw) unit quaternion
    connected: bool
    e_stopped: bool

    @property
    def joints_deg(self) -> tuple[float, ...]:
        return tuple(j * 57.29577951308232 for j in self.joints)
