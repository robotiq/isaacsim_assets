"""
Launch joy_node + gamepad_teleop together. Run this alongside the
ur5e_servo.launch.py (Servo stack) — it does NOT start Servo itself.

  Terminal A:  ros2 launch .../ur5e_servo.launch.py
  Terminal B:  ros2 launch .../gamepad.launch.py

Both joy_node and gamepad_teleop run with use_sim_time=true so message
stamps land inside Servo's stale-message window. autorepeat_rate keeps /joy
flowing at 30 Hz while a stick is held — without it Servo halts after
incoming_command_timeout (0.1 s) of stick-still. That timeout applies on both
Humble and Jazzy; on Humble it is paired with low_latency_mode, on Jazzy with
the halt_all_joints_* flags (see _servo_compat.apply_idle_protection).

gamepad_teleop selects Servo's TWIST command type itself at startup — on Jazzy
that is required before Servo accepts anything, on Humble it is a no-op.
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    sim_time = {"use_sim_time": True}

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            sim_time,
            {
                "device_id": 0,            # /dev/input/js0
                "deadzone": 0.05,          # joy_node-side deadzone (we do our own too)
                "autorepeat_rate": 30.0,   # republish /joy at 30 Hz while held
                "coalesce_interval_ms": 1,
            },
        ],
    )

    teleop = ExecuteProcess(
        cmd=[
            "python3", os.path.join(here, "gamepad_teleop.py"),
            "--ros-args", "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    return LaunchDescription([joy_node, teleop])
