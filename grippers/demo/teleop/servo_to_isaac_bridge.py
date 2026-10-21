#!/usr/bin/env python3
"""
Bridge MoveIt Servo's controller-style output to Isaac Sim's /joint_command.

Servo (default UR config) publishes std_msgs/Float64MultiArray on
/forward_position_controller/commands — 6 doubles in arm joint order.
This node attaches joint names and republishes as sensor_msgs/JointState
on /joint_command, which the action graph subscribes to.

Drops messages containing NaN/inf or values outside the UR5e joint limits
(prevents Servo's singularity-NaN from wrapping the wrist drives).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_BOUNDS = [
    (-2 * math.pi, 2 * math.pi),
    (-math.pi,     2 * math.pi),
    (-math.pi,     math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
]


class ServoToIsaacBridge(Node):
    def __init__(self):
        super().__init__("servo_to_isaac_bridge")
        self.pub = self.create_publisher(JointState, "/joint_command", 10)
        self.create_subscription(
            Float64MultiArray, "/forward_position_controller/commands",
            self.on_cmd, 10,
        )
        self.dropped = 0
        self.get_logger().info(
            "bridging /forward_position_controller/commands -> /joint_command"
        )

    def on_cmd(self, msg: Float64MultiArray):
        if len(msg.data) != len(UR5E_JOINTS):
            self.get_logger().warn(
                f"expected {len(UR5E_JOINTS)} positions, got {len(msg.data)}; dropping"
            )
            return
        for i, v in enumerate(msg.data):
            lo, hi = JOINT_BOUNDS[i]
            if math.isnan(v) or math.isinf(v) or v < lo or v > hi:
                self.dropped += 1
                if self.dropped % 50 == 1:
                    self.get_logger().warn(
                        f"dropped command: {UR5E_JOINTS[i]}={v!r} out of [{lo:.2f}, {hi:.2f}] "
                        f"(total dropped: {self.dropped})"
                    )
                return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = UR5E_JOINTS
        out.position = list(msg.data)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ServoToIsaacBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
