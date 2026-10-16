"""The simulated gripper answers the robotiq/ros stack like the real one.

For each 2F gripper and physics variant (PhysX and Newton): load it, build the ROS 2
graph of grippers/ros2/build_gripper_graph.py, and from a separate ROS 2 process send the
JointState topic_based_ros2_control sends to a simulator. Check the driven joint reaches
the commanded position and that the state topic reports exactly that one joint, under
its URDF name.

Needs a system ROS 2 (Humble or Jazzy under /opt/ros); skipped without one. The probe
runs outside Isaac, in a clean environment, as Isaac's Python cannot load the system rclpy.
"""

from __future__ import annotations

import glob
import json
import runpy
import subprocess
import sys
from pathlib import Path

import omni.timeline
import pytest

from _scene import REPO, load_gripper, step

PROBE = Path(__file__).with_name("_ros_probe.py")
BUILDER = REPO / "grippers" / "ros2" / "build_gripper_graph.py"

TARGET = 0.5  # rad, well inside the 0 - 0.8 travel
TOLERANCE = 0.05
SECONDS = 12.0  # wall time the probe publishes; the app is stepped meanwhile

# (gripper, Physics variant) -> (model of the builder, the ROS joint name, mimic joints the plugin also sends)
_85 = ("2f_85", "robotiq_85_left_knuckle_joint",
       ["robotiq_85_right_knuckle_joint", "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"])
_140 = ("2f_140", "finger_joint",
        ["left_inner_knuckle_joint", "left_inner_finger_joint", "right_outer_knuckle_joint",
         "right_inner_knuckle_joint", "right_inner_finger_joint"])
CASES = {
    ("Robotiq_2F_85", "Physx_compliant"): _85,
    ("Robotiq_2F_85", "Physx_parallel_grip"): _85,
    ("Robotiq_2F_85", "Newton_compliant"): _85,
    ("Robotiq_2F_85", "Newton_parallel_grip"): _85,
    ("Robotiq_2F_140", "Physx_compliant"): _140,
    ("Robotiq_2F_140", "Physx_parallel_grip"): _140,
    ("Robotiq_2F_140", "Newton_compliant"): _140,
    ("Robotiq_2F_140", "Newton_parallel_grip"): _140,
}


def _ros_setup() -> str | None:
    setups = sorted(glob.glob("/opt/ros/*/setup.bash"))
    return setups[-1] if setups else None


@pytest.mark.skipif(_ros_setup() is None, reason="no system ROS 2 under /opt/ros")
@pytest.mark.parametrize(("gripper", "physics"), list(CASES), ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_ros2(simulation_app, view, gripper: str, physics: str) -> None:
    model, ros_joint, mimics = CASES[(gripper, physics)]
    scene = load_gripper(simulation_app, gripper, physics, view)
    step(simulation_app, 30)

    # The builder is a script for Isaac's Script Editor: run it as one.
    namespace = runpy.run_path(str(BUILDER))
    namespace["build_graph"](model=model)
    step(simulation_app, 10)

    probe = subprocess.Popen(
        ["env", "-i", f"HOME={Path.home()}", "PATH=/usr/bin:/bin", "bash", "-c",
         f"source {_ros_setup()} && exec python3 {PROBE} --joint {ros_joint} --target {TARGET} "
         f"--mimic {' '.join(mimics)} --seconds {SECONDS}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    while probe.poll() is None:
        step(simulation_app, 5)
    out, err = probe.communicate()
    omni.timeline.get_timeline_interface().stop()
    assert probe.returncode == 0, f"probe failed:\n{err}"

    state = json.loads(out.strip().splitlines()[-1])
    print(f"{gripper} {physics}: {state}")
    assert state["messages"] > 0, "nothing published on the state topic"
    assert state["name"] == [ros_joint], f"state topic reports {state['name']}, expected only {[ros_joint]}"
    assert abs(state["position"][0] - TARGET) < TOLERANCE, f"{ros_joint}={state['position'][0]}, commanded {TARGET}"
    assert len(state["velocity"]) == 1, "no velocity published"
