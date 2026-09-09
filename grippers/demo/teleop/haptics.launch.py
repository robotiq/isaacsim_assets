"""
DualSense haptic loop that pairs with the teleop stack. Two pieces:

  1. Injects the gripper pad contact-force plot into the running Isaac session
     (../mcp_bridge/make_contact_plot.py). That injection also starts the UDP
     force broadcast on 127.0.0.1:8770.
  2. Runs trigger_force_feedback.py, which reads that UDP stream and drives the
     DualSense R2 adaptive-trigger resistance from the average of the two pad
     forces.

Included automatically by teleop.launch.py. Can also be launched on its own:

    ros2 launch haptics.launch.py
    ros2 launch haptics.launch.py full_force_n:=100   # softer full-scale

Prerequisites:
  - Isaac up with the MCP extension (TCP 8766) and the teleop scene loaded.
  - pydualsense + hidapi installed for the system python3 (see requirements.txt).
  - Read/write access to the DualSense hidraw node (uaccess udev rule).

The plot injection is one-shot and needs Isaac already running. If you restart
Isaac, re-run this launch (or just ../mcp_bridge/make_contact_plot.py) to
re-inject; the long-running trigger node keeps running and reconnects to the
fresh UDP stream on its own.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    plot_script = os.path.join(here, "..", "mcp_bridge", "make_contact_plot.py")
    haptic_script = os.path.join(here, "trigger_force_feedback.py")

    full_force_n = LaunchConfiguration("full_force_n")

    return LaunchDescription([
        DeclareLaunchArgument(
            "full_force_n",
            default_value="200",
            description="average pad force (N) mapped to full R2 stiffness",
        ),
        # One-shot: inject the plot + UDP force stream into the live Isaac session.
        # Exits immediately after injecting; a failure here (e.g. Isaac not up)
        # does not stop the teleop stack.
        ExecuteProcess(
            cmd=["python3", plot_script],
            output="screen",
        ),
        # Long-running: read the UDP force stream and drive the R2 trigger.
        ExecuteProcess(
            cmd=["python3", haptic_script, "--full-force-N", full_force_n],
            output="screen",
        ),
    ])
