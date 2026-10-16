"""ROS 2 side of test_gripper_ros2.py: run by the test in a clean shell with the system ROS
sourced (never inside Isaac, whose Python cannot load the system rclpy).

Publishes what topic_based_ros2_control sends to a simulator (the driven joint plus
mimic joints the asset does not have), for `--seconds`, and prints one JSON line with
the last JointState it received back.
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

parser = argparse.ArgumentParser()
parser.add_argument("--command-topic", default="/sim/joint_commands")
parser.add_argument("--state-topic", default="/sim/joint_states")
parser.add_argument("--joint", required=True)
parser.add_argument("--target", type=float, required=True)
parser.add_argument("--mimic", nargs="*", default=[])
parser.add_argument("--seconds", type=float, default=10.0)
args = parser.parse_args()

rclpy.init()
node = Node("gripper_ros2_test_probe")
pub = node.create_publisher(JointState, args.command_topic, 10)
received = []
node.create_subscription(JointState, args.state_topic, received.append, 10)

command = JointState()
command.name = [args.joint, *args.mimic]
command.position = [args.target] + [-args.target] * len(args.mimic)
end = time.time() + args.seconds
while time.time() < end:
    pub.publish(command)
    rclpy.spin_once(node, timeout_sec=0.02)

last = received[-1] if received else None
print(json.dumps({
    "messages": len(received),
    "name": list(last.name) if last else [],
    "position": list(last.position) if last else [],
    "velocity": list(last.velocity) if last else [],
}))
