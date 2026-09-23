"""
One-shot launcher: brings up the full teleop stack (Servo + bridge + filter
+ robot_state_publisher, then joy_node + gamepad mapper) in one command.

  ros2 launch teleop.launch.py

Prerequisites (same as the individual launches):
  - Isaac Sim already running with the /World/ROS_JointControl action graph
    built and simulation playing (so /clock and /joint_states are live).
  - A DualShock 4 / DualSense controller connected on /dev/input/js0.

Design:
  - Includes ur5e_servo.launch.py immediately.
  - Includes gamepad.launch.py 10 s later so /servo_node/start_servo has
    time to be advertised. Without this delay gamepad_teleop.py warns
    "start_servo failed" on the first attempt and you'd need to trigger
    the resync manually (Circle button).
  - Includes haptics.launch.py 12 s later: injects the gripper contact-force
    plot into Isaac and starts the DualSense R2 trigger force-feedback node.
    Best-effort — if Isaac/the controller aren't ready it just logs and the
    rest of the stack is unaffected.

If you'd rather run the two halves separately (e.g. to leave the gamepad
off while another consumer sends twists), invoke ur5e_servo.launch.py and
gamepad.launch.py individually — nothing here depends on this wrapper.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, "ur5e_servo.launch.py"))
    )

    gamepad = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(here, "gamepad.launch.py"))
            )
        ],
    )

    # The contact plot redraws an omni.ui Plot every frame inside Isaac and is
    # the most expensive thing this launch adds to the sim process. Set
    # TELEOP_HAPTICS=0 to leave it (and the R2 rumble) out -- useful both when
    # you want the frames back and when bisecting a frame-rate problem, since
    # the rest of the stack is pure ROS and costs Isaac almost nothing.
    haptics_enabled = os.environ.get("TELEOP_HAPTICS", "1") != "0"

    haptics = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(here, "haptics.launch.py"))
            )
        ],
    )

    actions = [servo, gamepad]
    if haptics_enabled:
        actions.append(haptics)
    else:
        print("[teleop.launch] haptics DISABLED (TELEOP_HAPTICS=0): "
              "no contact-force plot, no R2 rumble")
    return LaunchDescription(actions)
