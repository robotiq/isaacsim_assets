#!/usr/bin/env python3
"""
Filter /joint_states (14 joints from Isaac Sim — 6 arm + 8 finger) down to the 6 UR5e arm joints
and republish on /joint_states_arm. MoveIt Servo subscribes to the filtered
topic so the 2F-85 finger joints don't trip its CurrentStateMonitor.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
}


class ArmFilter(Node):
    def __init__(self):
        super().__init__("joint_states_arm_filter")
        self.pub = self.create_publisher(JointState, "/joint_states_arm", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.on_msg, 10)

    def on_msg(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        for i, name in enumerate(msg.name):
            if name in ARM_JOINTS:
                out.name.append(name)
                if i < len(msg.position):
                    out.position.append(msg.position[i])
                if i < len(msg.velocity):
                    out.velocity.append(msg.velocity[i])
                if i < len(msg.effort):
                    out.effort.append(msg.effort[i])
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ArmFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
