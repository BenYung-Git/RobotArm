"""Robot specification dataclass and built-in UR5 spec."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotSpec:
    """Static specification for a 6-DOF virtual arm.

    Joint limits are populated from the URDF at install/setup time;
    this dataclass is what application code reads at runtime.
    """

    name: str  # e.g. "UR5"
    brand: str  # e.g. "Universal Robots"
    dof: int
    controllable_joint_indices: tuple[int, ...]
    controllable_joint_names: tuple[str, ...]
    joint_lower_limits_rad: tuple[float, ...]
    joint_upper_limits_rad: tuple[float, ...]
    # End-effector link name (for TCP pose queries)
    ee_link_name: str

    def __post_init__(self) -> None:
        n = len(self.controllable_joint_indices)
        assert len(self.controllable_joint_names) == n, "joint names length mismatch"
        assert len(self.joint_lower_limits_rad) == n, "joint lower limits length mismatch"
        assert len(self.joint_upper_limits_rad) == n, "joint upper limits length mismatch"
        assert self.dof == n, f"dof={self.dof} != controllable joints={n}"


# Discovered at install time from URDF `ur5.standalone.urdf`.
# Values are read from PyBullet at startup (see virtual_robot.discover_spec);
# these defaults are an install-time snapshot used only when PyBullet is
# not yet connected.
UR5_SPEC = RobotSpec(
    name="UR5",
    brand="Universal Robots",
    dof=6,
    controllable_joint_indices=(1, 2, 3, 4, 5, 6),
    controllable_joint_names=(
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
    joint_lower_limits_rad=(
        -2.0 * 3.141592653589793,  # -2π
        -2.0 * 3.141592653589793,
        -1.0 * 3.141592653589793,  # -π
        -2.0 * 3.141592653589793,
        -2.0 * 3.141592653589793,
        -2.0 * 3.141592653589793,
    ),
    joint_upper_limits_rad=(
        2.0 * 3.141592653589793,
        2.0 * 3.141592653589793,
        1.0 * 3.141592653589793,
        2.0 * 3.141592653589793,
        2.0 * 3.141592653589793,
        2.0 * 3.141592653589793,
    ),
    ee_link_name="tool0",
)
