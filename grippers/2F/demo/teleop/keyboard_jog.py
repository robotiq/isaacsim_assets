#!/usr/bin/env python3
"""
Direct keyboard teleop for the UR5e + 2F-85 in Isaac Sim.

No MoveIt, no Servo, no URDF, no IK — just per-joint deltas published
straight to /joint_command. Holds the latest target while you press keys
and republishes at 30 Hz so the action graph keeps tracking.

Controls:
  1..6   select arm joint  (shoulder_pan ... wrist_3)
  7      select gripper (drives finger_joint only)
  +/=    nudge selected joint forward (+0.02 rad per key)
  -/_    nudge selected joint backward
  h      go home pose
  o      open gripper
  c      close gripper
  q      quit

Run after `source /opt/ros/humble/setup.bash` in a terminal where you
want keyboard focus. Uses raw tty; Ctrl-C restores.
"""
import math
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER_JOINTS = [
    # With the Physx_Loop variant of the 2F-85, only finger_joint is driven.
    # The other finger DOFs follow from the 4-bar linkage / mimic constraints.
    "finger_joint",
]

HOME_ARM = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

ARM_STEP = 0.02   # rad per nudge
GRIP_STEP = 0.04
GRIP_OPEN = 0.0
GRIP_CLOSED = 0.78


class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def getch(self, timeout=0.05):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ""


class JogNode(Node):
    def __init__(self):
        super().__init__("keyboard_jog")
        self.pub = self.create_publisher(JointState, "/joint_command", 10)
        self.arm = list(HOME_ARM)
        self.grip = GRIP_OPEN
        self.selected = 0  # 0..5 arm joints, 6 = gripper
        self.timer = self.create_timer(1 / 30.0, self.publish_target)

    def publish_target(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + GRIPPER_JOINTS
        msg.position = self.arm + [self.grip] * len(GRIPPER_JOINTS)
        self.pub.publish(msg)

    def label(self):
        return "gripper" if self.selected == 6 else ARM_JOINTS[self.selected]

    def value(self):
        return self.grip if self.selected == 6 else self.arm[self.selected]

    def nudge(self, sign):
        if self.selected == 6:
            self.grip = max(GRIP_OPEN, min(GRIP_CLOSED, self.grip + sign * GRIP_STEP))
        else:
            self.arm[self.selected] += sign * ARM_STEP


def main():
    rclpy.init()
    node = JogNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(__doc__)
    print(f"selected: {node.label()}  value: {node.value():+.3f}")

    try:
        with KeyReader() as kb:
            while rclpy.ok():
                k = kb.getch()
                if not k:
                    continue
                if k in ("q", "\x03"):
                    break
                if k in "123456":
                    node.selected = int(k) - 1
                elif k == "7":
                    node.selected = 6
                elif k in ("+", "="):
                    node.nudge(+1)
                elif k in ("-", "_"):
                    node.nudge(-1)
                elif k == "h":
                    node.arm = list(HOME_ARM)
                elif k == "o":
                    node.grip = GRIP_OPEN
                elif k == "c":
                    node.grip = GRIP_CLOSED
                print(f"\rselected: {node.label():28s} value: {node.value():+.3f}   ", end="", flush=True)
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
