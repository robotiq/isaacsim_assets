"""The simulated gripper answers the robotiq/robotiq-ros stack like the real one.

For each 2F gripper and the Hand-E C10, and each physics variant (PhysX and Newton): load it, build the ROS 2
graph of grippers/ros2/build_gripper_graph.py, and from a separate ROS 2 process send the
JointState topic_based_ros2_control sends to a simulator (on ros <= 1.3, robotiq/robotiq-ros#118:
the mimic joints named, a position for the driven one only). Check the driven joint reaches
the commanded position and that the state topic reports exactly that one joint, under its
URDF name, stamped with the simulation time.

Needs a system ROS 2 under /opt/ros (the distro Isaac's bridge runs on, $ROS_DISTRO, when
set); skipped without one. The probe runs outside Isaac, in a clean environment, as Isaac's
Python cannot load the system rclpy; the ROS variables that decide whom it can talk to are
passed on to it.
"""

from __future__ import annotations

import glob
import json
import os
import runpy
import subprocess
import time
from pathlib import Path

import omni.timeline
import pytest

from _scene import REPO, load_gripper, step

PROBE = Path(__file__).with_name("_ros_probe.py")
BUILDER = REPO / "grippers" / "ros2" / "build_gripper_graph.py"

# Commanded position and the tolerance on it, well inside the travel: radians on the 2F knuckle (0 - 0.8), metres on the
# C10's finger (0 - 0.0192).
TARGET_2F, TOLERANCE_2F = 0.5, 0.05
TARGET_C10, TOLERANCE_C10 = 0.010, 0.001
SECONDS = 12.0  # wall time the probe publishes; the app is stepped meanwhile

# (gripper, Physics variant) -> (model of the builder, the ROS joint name, mimic joints the plugin also sends,
# commanded position, tolerance)
_85 = ("2f_85", "robotiq_85_left_knuckle_joint",
       ["robotiq_85_right_knuckle_joint", "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"], TARGET_2F, TOLERANCE_2F)
_140 = ("2f_140", "finger_joint",
        ["left_inner_knuckle_joint", "left_inner_finger_joint", "right_outer_knuckle_joint",
         "right_inner_knuckle_joint", "right_inner_finger_joint"], TARGET_2F, TOLERANCE_2F)
_C10 = ("hand_e_c10", "hande_c10_finger_1_joint", ["hande_c10_finger_2_joint", "hande_c10_finger_3_joint"],
        TARGET_C10, TOLERANCE_C10)
CASES = {
    ("Robotiq_2F_85", "Physx_compliant"): _85,
    ("Robotiq_2F_85", "Physx_parallel_grip"): _85,
    ("Robotiq_2F_85", "Newton_compliant"): _85,
    ("Robotiq_2F_85", "Newton_parallel_grip"): _85,
    ("Robotiq_2F_140", "Physx_compliant"): _140,
    ("Robotiq_2F_140", "Physx_parallel_grip"): _140,
    ("Robotiq_2F_140", "Newton_compliant"): _140,
    ("Robotiq_2F_140", "Newton_parallel_grip"): _140,
    ("Robotiq_Hand_E_C10", "PhysX"): _C10,
    ("Robotiq_Hand_E_C10", "Newton"): _C10,
}


# What decides which nodes the probe can reach: dropped by the clean environment otherwise,
# and Isaac on another domain, or on another RMW, then sees nothing with no hint why.
_ROS_ENV = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE", "RMW_IMPLEMENTATION",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI")


def _ros_setup() -> str | None:
    """The system ROS of the distro Isaac's bridge runs on, else the newest installed."""
    distro = os.environ.get("ROS_DISTRO")
    if distro and Path(f"/opt/ros/{distro}/setup.bash").is_file():
        return f"/opt/ros/{distro}/setup.bash"
    setups = sorted(glob.glob("/opt/ros/*/setup.bash"))
    return setups[-1] if setups else None


@pytest.mark.skipif(_ros_setup() is None, reason="no system ROS 2 under /opt/ros")
@pytest.mark.parametrize(("gripper", "physics"), list(CASES), ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_ros2(simulation_app, view, tmp_path, gripper: str, physics: str) -> None:
    model, ros_joint, mimics, target, tolerance = CASES[(gripper, physics)]
    scene = load_gripper(simulation_app, gripper, physics, view)
    step(simulation_app, 30)

    # The builder is a script for Isaac's Script Editor: run it as one.
    namespace = runpy.run_path(str(BUILDER))
    try:
        namespace["build_graph"](model=model)
    except namespace["Ros2BridgeUnavailable"] as error:
        # Without it Isaac segfaults on the first ROS2 node, which kills the whole session.
        pytest.skip(str(error))
    step(simulation_app, 10)

    # Output to files, not pipes: nothing reads a pipe while the app is stepped, and a full
    # one (rclpy warnings) would block the probe for good.
    env = {"HOME": str(Path.home()), "PATH": "/usr/bin:/bin"}
    env.update({k: os.environ[k] for k in _ROS_ENV if k in os.environ})
    log_dir = Path(tmp_path)
    with open(log_dir / "probe.out", "w") as out_file, open(log_dir / "probe.err", "w") as err_file:
        probe = subprocess.Popen(
            ["bash", "-c",
             f"source {_ros_setup()} && exec python3 {PROBE} --joint {ros_joint} --target {target} "
             f"--mimic {' '.join(mimics)} --seconds {SECONDS}"],
            env=env, stdout=out_file, stderr=err_file, text=True,
        )
        deadline = time.monotonic() + SECONDS + 10.0
        while probe.poll() is None and time.monotonic() < deadline:
            step(simulation_app, 5)
        if probe.poll() is None:
            probe.kill()
            probe.wait()
    omni.timeline.get_timeline_interface().stop()
    out = (log_dir / "probe.out").read_text()
    err = (log_dir / "probe.err").read_text()
    assert probe.returncode == 0, f"probe failed or timed out ({probe.returncode}):\n{err[-4000:]}"

    state = json.loads(out.strip().splitlines()[-1])
    print(f"{gripper} {physics}: {state}")
    assert state["messages"] > 0, "nothing published on the state topic"
    assert state["name"] == [ros_joint], f"state topic reports {state['name']}, expected only {[ros_joint]}"
    assert abs(state["position"][0] - target) < tolerance, f"{ros_joint}={state['position'][0]}, commanded {target}"
    assert len(state["velocity"]) == 1, "no velocity published"
    assert state["stamp"] > 0.0, "state not stamped with the simulation time"
