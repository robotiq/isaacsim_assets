"""The 2F fingers stay parallel through a close / open cycle.

On the parallel-grip variants (PhysX mimic joints, Newton mimic / weld) the
inner fingers, which carry the pads, must keep their orientation while the
gripper closes and opens again: the four-bar keeps them parallel to each
other and to the base. The orientation of each inner finger is sampled every
few frames during the whole cycle and compared with its orientation when
open; the finger joint must also come back to open.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim

from _scene import load_gripper, step

GRIPPERS = ["Robotiq_2F_85", "Robotiq_2F_140"]
PHYSICS = ["Physx_parallel_grip", "Newton_parallel_grip"]
PADS = ["left_inner_finger", "right_inner_finger"]

SETTLE_FRAMES = 30
MOVE_FRAMES = 120  # per direction, 2 s at 60 Hz
SAMPLE_EVERY = 10
CLOSE_TARGET = 0.8  # rad, ~46 deg
PARALLEL_TOL = math.radians(2.0)  # max pad rotation away from its open orientation
REOPEN_TOL = math.radians(1.0)


def _angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Rotation angle (rad) between two wxyz unit quaternions."""
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


@pytest.mark.parametrize("physics", PHYSICS)
@pytest.mark.parametrize("gripper", GRIPPERS)
def test_fingers_stay_parallel(simulation_app, view, gripper: str, physics: str) -> None:
    scene = load_gripper(simulation_app, gripper, physics, view)
    robot = scene.robot
    pads = RigidPrim([f"{scene.root}/{p}" for p in PADS])
    step(simulation_app, SETTLE_FRAMES)

    finger = scene.dof("finger_joint")
    q_open = pads.get_world_poses()[1].numpy()  # (2, 4) wxyz
    worst = {p: (0.0, "") for p in PADS}  # pad -> (max deviation, phase)
    phase = "closing"

    def sample() -> None:
        q = pads.get_world_poses()[1].numpy()
        for i, p in enumerate(PADS):
            dev = _angle_between(q_open[i], q[i])
            if dev > worst[p][0]:
                worst[p] = (dev, phase)

    robot.set_dof_position_targets([CLOSE_TARGET], dof_indices=[finger])
    step(simulation_app, MOVE_FRAMES, SAMPLE_EVERY, sample)
    closed = float(robot.get_dof_positions().numpy()[0][finger])

    phase = "opening"
    robot.set_dof_position_targets([0.0], dof_indices=[finger])
    step(simulation_app, MOVE_FRAMES, SAMPLE_EVERY, sample)
    reopened = float(robot.get_dof_positions().numpy()[0][finger])
    omni.timeline.get_timeline_interface().stop()

    print(
        f"{gripper} {physics}: closed {math.degrees(closed):.2f} deg, reopened {math.degrees(reopened):.2f} deg, "
        + ", ".join(f"{p} max {math.degrees(d):.2f} deg ({w})" for p, (d, w) in worst.items())
    )
    assert closed > CLOSE_TARGET * 0.9, f"did not close: finger_joint={math.degrees(closed):.2f} deg"
    assert abs(reopened) < REOPEN_TOL, f"did not reopen: finger_joint={math.degrees(reopened):.2f} deg"
    for p, (dev, when) in worst.items():
        assert dev < PARALLEL_TOL, f"{p} rotated {math.degrees(dev):.2f} deg from its open orientation while {when}"
