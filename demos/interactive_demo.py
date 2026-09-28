"""PyBullet GUI interactive demo for the UR5 virtual arm.

Run with:
    python demos/interactive_demo.py

Controls (in PyBullet GUI side panel + keyboard):
    - 6 joint sliders   : drag to set joint targets
    - Home button       : move to home pose
    - Pose A button     : load and move to pose_a.json
    - Pose B button     : load and move to pose_b.json
    - E-Stop button     : stop all motion; sliders no longer move arm
    - Reset E-Stop      : re-enable motion

The status panel (printed to stdout) shows:
    mode, robot_id, 6 joint angles, TCP position, E-stop state.

Model disclaimer
----------------
UR5 in this demo is a stand-in virtual model. It is NOT a digital
twin of ROKAE CR7, JAKA Zu7, or any production hardware.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

# Ensure repo root is on sys.path when run as a script
_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

# IMPORTANT: use the venv pybullet. If not on path, give a clear error.
try:
    import pybullet as p
    import pybullet_data  # noqa: F401
except ImportError as e:
    sys.stderr.write(
        "ERROR: pybullet not found. Install with:\n"
        "    /home/ubuntu/docRoute/venv/bin/pip install pybullet\n"
        f"Original error: {e}\n"
    )
    sys.exit(1)

from robotarm.core.virtual_robot import VirtualRobot  # noqa: E402
from robotarm.utils.pose_loader import PoseLoadError, load_pose  # noqa: E402

POSES_DIR = _ROOT / "data" / "poses"


def rad_to_deg(r: float) -> float:
    return r * 57.29577951308232


def print_state(state, joint_targets) -> None:
    """Print live status to stdout (overwriting previous line)."""
    joints_str = " ".join(f"{rad_to_deg(j):+7.2f}°" for j in state.joints)
    targets_str = " ".join(f"{rad_to_deg(t):+7.2f}°" for t in joint_targets)
    tcp = state.tcp_position
    line = (
        f"mode={state.mode} id={state.robot_id} "
        f"estop={'YES' if state.e_stopped else 'no '} "
        f"conn={'YES' if state.connected else 'no '} | "
        f"joints [deg]: {joints_str} | "
        f"targets [deg]: {targets_str} | "
        f"TCP xyz=({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}) m"
    )
    # Clear line + write
    sys.stdout.write("\r" + " " * 200 + "\r" + line)
    sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Run in DIRECT mode (no PyBullet window). Useful for headless tests.",
    )
    parser.add_argument(
        "--robot-id", default="ur5_demo", help="Logical robot id (default: ur5_demo)"
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="Number of GUI ticks to run (0 = infinite). For tests.",
    )
    args = parser.parse_args()

    robot = VirtualRobot(robot_id=args.robot_id)

    print("Connecting to PyBullet (local only)...", flush=True)
    target = "loopback" if args.no_gui else "gui"
    robot.connect(target=target, gui=not args.no_gui)
    print(f"Connected. Joints: {robot.spec.controllable_joint_names}", flush=True)
    print(f"Limits (rad): {robot.spec.joint_lower_limits_rad} .. {robot.spec.joint_upper_limits_rad}", flush=True)

    # Discover EE link index now (caches on spec)
    ee_idx = robot._ee_link_index()
    print(f"EE link index = {ee_idx} (link name: {robot.spec.ee_link_name})", flush=True)

    # Build sliders + buttons
    slider_ids: list[int] = []
    if not args.no_gui:
        for i, name in enumerate(robot.spec.controllable_joint_names):
            lo = math.degrees(robot.spec.joint_lower_limits_rad[i])
            hi = math.degrees(robot.spec.joint_upper_limits_rad[i])
            slider_ids.append(
                p.addUserDebugParameter(
                    paramName=f"[{i}] {name}", rangeMin=lo, rangeMax=hi, startValue=0.0
                )
            )
        # Buttons (rangeMin == rangeMax means button)
        btn_home = p.addUserDebugParameter("Home", 1, 0, 0)
        btn_pose_a = p.addUserDebugParameter("Pose A", 1, 0, 0)
        btn_pose_b = p.addUserDebugParameter("Pose B", 1, 0, 0)
        btn_estop = p.addUserDebugParameter("Virtual E-STOP", 1, 0, 0)
        btn_reset = p.addUserDebugParameter("Reset E-Stop", 1, 0, 0)
    else:
        btn_home = btn_pose_a = btn_pose_b = btn_estop = btn_reset = -1

    print("\n=== RobotArm Phase 1 Virtual Demo ===")
    print("Drag sliders to control each joint. Click buttons for pose / e-stop.")
    print("Press Ctrl-C to quit.\n")

    prev_btn = {btn_home: False, btn_pose_a: False, btn_pose_b: False,
                btn_estop: False, btn_reset: False}

    cycle = 0
    try:
        while True:
            # Read slider values
            new_targets = list(robot.joint_targets)
            for i, sid in enumerate(slider_ids):
                val_deg = p.readUserDebugParameter(sid)
                new_targets[i] = math.radians(val_deg)

            # Try to set targets (may raise EmergencyStopError / JointLimitError)
            try:
                robot.set_joint_targets(new_targets)
            except Exception as e:
                # During E-stop, sliders are blocked; reflect by syncing sliders back
                # to last good targets.
                if args.no_gui:
                    print(f"\n[refused] {e}")
                else:
                    # Sync sliders back
                    for i, sid in enumerate(slider_ids):
                        lo = math.degrees(robot.spec.joint_lower_limits_rad[i])
                        hi = math.degrees(robot.spec.joint_upper_limits_rad[i])
                        cur_deg = max(lo, min(hi, math.degrees(robot.joint_targets[i])))
                        p.addUserDebugParameter(
                            paramName=f"[{i}] {robot.spec.controllable_joint_names[i]}",
                            rangeMin=lo, rangeMax=hi, startValue=cur_deg,
                        )
                        # Note: addUserDebugParameter re-adds; pybullet has no
                        # in-place update. We just leave the slider state alone;
                        # the user's next drag will be validated again.

            # Step the simulator with smooth motion
            if not robot.is_e_stopped:
                moved = robot.step_toward_targets()
                p.stepSimulation()

            # Read buttons (only fires on click event)
            for btn_id, fname in (
                (btn_home, "home.json"),
                (btn_pose_a, "pose_a.json"),
                (btn_pose_b, "pose_b.json"),
            ):
                if btn_id < 0:
                    continue
                cur = p.readUserDebugParameter(btn_id)
                clicked = cur != prev_btn.get(btn_id, False)
                prev_btn[btn_id] = cur
                if clicked:
                    if robot.is_e_stopped:
                        print(f"\n[refused] cannot load {fname}: E-stop is active")
                        continue
                    path = POSES_DIR / fname
                    try:
                        joints = load_pose(path)
                    except PoseLoadError as e:
                        print(f"\n[error] {e}")
                        continue
                    print(f"\n[action] loading {fname}: {joints}")
                    robot.set_joint_targets(joints)

            if btn_estop >= 0:
                cur = p.readUserDebugParameter(btn_estop)
                clicked = cur != prev_btn.get(btn_estop, False)
                prev_btn[btn_estop] = cur
                if clicked:
                    robot.emergency_stop()
                    print("\n[EMERGENCY STOP] all motion refused; sliders blocked.")

            if btn_reset >= 0:
                cur = p.readUserDebugParameter(btn_reset)
                clicked = cur != prev_btn.get(btn_reset, False)
                prev_btn[btn_reset] = cur
                if clicked:
                    robot.reset_estop()
                    print("\n[reset] E-stop cleared; motion re-enabled.")

            # Print state line
            state = robot.get_state()
            print_state(state, robot.joint_targets)

            cycle += 1
            if args.cycles and cycle >= args.cycles:
                break

            time.sleep(1.0 / 60.0)

    except KeyboardInterrupt:
        print("\n[exit] Ctrl-C received")

    finally:
        robot.disconnect()
        print("\nDisconnected.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
