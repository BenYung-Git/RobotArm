"""Custom exceptions for RobotArm virtual demo.

All motion-related failures raise a subclass of ``RobotArmError`` so the
GUI and tests can discriminate reason cleanly.
"""


class RobotArmError(Exception):
    """Base class for all RobotArm errors."""


class VirtualOnlyViolationError(RobotArmError):
    """Raised when the safety guard detects a non-virtual access pattern."""


class NotConnectedError(RobotArmError):
    """Raised when an action requires an active simulator connection."""


class RobotAlreadyConnectedError(RobotArmError):
    """Raised when connect() is called twice without disconnect()."""


class JointLimitError(RobotArmError):
    """Raised when a joint target is outside the allowed range."""

    def __init__(self, joint_index: int, joint_name: str, value: float, lower: float, upper: float):
        self.joint_index = joint_index
        self.joint_name = joint_name
        self.value = value
        self.lower = lower
        self.upper = upper
        super().__init__(
            f"Joint {joint_index} ({joint_name}): target {value:.4f} rad outside "
            f"[{lower:.4f}, {upper:.4f}] rad"
        )


class EmergencyStopError(RobotArmError):
    """Raised when motion is attempted while E-stop is active."""


class InvalidPoseError(RobotArmError):
    """Raised when a pose definition is malformed (wrong length, non-finite, etc.)."""


class URDFAssetError(RobotArmError):
    """Raised when the local URDF asset is missing or unreadable."""


# ---------------------------------------------------------------------------
# Phase 1C — Virtual Controller Cabinet exceptions
# ---------------------------------------------------------------------------


class CabinetStateError(RobotArmError):
    """Raised on illegal cabinet state transition."""


class JobAlreadyRunningError(RobotArmError):
    """Raised when ``start_job`` is called while another job is RUNNING."""


class JobValidationError(RobotArmError):
    """Raised when a job JSON fails schema / pose / mode validation."""


class VirtualInterlockError(RobotArmError):
    """Raised when a required Virtual I/O input is not in its run-state."""

    def __init__(self, missing: list[str]):
        self.missing = list(missing)
        super().__init__(
            "Virtual interlock not satisfied; missing/invalid inputs: "
            + ", ".join(missing)
        )


class VirtualSafetyStopError(RobotArmError):
    """Raised when motion / job step is attempted during E-stop / prot. stop."""


class VirtualFaultError(RobotArmError):
    """Raised on a virtual fault (timeout, motion error, etc.)."""

    def __init__(self, code: str, message: str = ""):
        self.code = code
        self.message = message
        super().__init__(f"Virtual fault [{code}]: {message}")
