#!/usr/bin/env python3
"""
Minimal ROS 2 publisher for driving a UR5e + Robotiq 2F-85 in Isaac Sim 6
via the ROS2SubscribeJointState action-graph node.

Usage (in a shell with /opt/ros/humble sourced, *not* the shell that
launched Isaac Sim):

    python3 send_joint_command.py home
    python3 send_joint_command.py ready
    python3 send_joint_command.py open
    python3 send_joint_command.py close
    python3 send_joint_command.py wave        # continuous demo
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Robotiq 2F-85: the driven joint in most URDFs / Isaac USDs.
# If your USD exposes a different name (e.g. "robotiq_85_left_knuckle_joint"),
# change it here.
GRIPPER_JOINT = "finger_joint"

# 2F-85 driven-joint travel: 0 rad (open) -> ~0.8 rad (closed).
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.78

POSES = {
    "home":  [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0],
    "ready": [0.0, -1.2, 1.2, -1.57, -1.57, 0.0],
}


class JointCommander(Node):
    def __init__(self):
        super().__init__("isaac_joint_commander")
        self.pub = self.create_publisher(JointState, "/joint_command", 10)

    def send(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.pub.publish(msg)
        self.get_logger().info(f"sent {dict(zip(names, positions))}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    cmd = sys.argv[1]

    rclpy.init()
    node = JointCommander()
    # Give the publisher a moment to connect before the first message.
    time.sleep(0.5)

    try:
        if cmd in POSES:
            # Named-pose commands also open the gripper — these are recovery
            # / "back to a known state" moves, not motions to preserve a grip.
            node.send(UR5E_JOINTS + [GRIPPER_JOINT],
                      POSES[cmd] + [GRIPPER_OPEN])
            time.sleep(0.5)
        elif cmd == "open":
            node.send([GRIPPER_JOINT], [GRIPPER_OPEN])
            time.sleep(0.5)
        elif cmd == "close":
            node.send([GRIPPER_JOINT], [GRIPPER_CLOSED])
            time.sleep(0.5)
        elif cmd == "wave":
            t0 = time.time()
            base = POSES["ready"][:]
            while rclpy.ok() and time.time() - t0 < 20.0:
                t = time.time() - t0
                q = base[:]
                q[0] = 0.6 * math.sin(0.5 * t)        # shoulder_pan
                q[3] = -1.57 + 0.4 * math.sin(1.0 * t)  # wrist_1
                node.send(UR5E_JOINTS, q)
                rclpy.spin_once(node, timeout_sec=0.05)
        else:
            print(f"unknown command: {cmd}")
            print(__doc__)
            sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
