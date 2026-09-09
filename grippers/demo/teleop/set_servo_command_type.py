#!/usr/bin/env python3
"""Manually select MoveIt Servo's active command type.

    python3 set_servo_command_type.py [twist|joint_jog|pose]

A debugging aid only — nothing launches this. The teleop frontends each select
their own command type through compat.switch_command_type(), because only the
frontend knows whether it is sending twists or joint jogs. Reach for this when
poking at Servo by hand, or to unstick a session where some other publisher
left it in the wrong mode.

On Humble this is a no-op: that Servo accepts TwistStamped and JointJog at any
time and offers no switch_command_type service.
"""
import os
import sys

import rclpy
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _servo_compat as compat  # noqa: E402


def main():
    want = (sys.argv[1] if len(sys.argv) > 1 else "twist").lower()

    if not compat.supports_command_type_switch():
        print("this Servo has no switch_command_type service; nothing to do")
        return 0

    rclpy.init()
    node = Node("set_servo_command_type")
    try:
        return 0 if compat.switch_command_type(node, want) else 1
    except ValueError as exc:
        print(exc)
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
