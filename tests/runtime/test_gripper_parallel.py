"""The 2F fingers stay parallel through a close / open cycle.

Without an object the inner fingers, which carry the pads, must keep their
orientation while the gripper closes and opens again: on the parallel-grip
variants the coupling (PhysX mimic joints, Newton mimic / weld) enforces it,
on the compliant variants the finger springs hold the pads parallel until
they meet something. The orientation of each inner finger is sampled every
few frames during the whole cycle and compared with its orientation when
open; the finger joint must also come back to open.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim

from _scene import FRAME_DT, load_gripper, move_to, step

GRIPPERS = ["Robotiq_2F_85", "Robotiq_2F_140"]
PHYSICS = ["Physx_parallel_grip", "Physx_compliant", "Newton_parallel_grip", "Newton_compliant"]
PADS = ["left_inner_finger", "right_inner_finger"]

SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz after each ramp (close / open at the datasheet finger speed)
SAMPLE_EVERY = 5
CLOSE_TARGET = 0.8  # rad, ~46 deg
SETTLED_TOL = math.radians(2.0)  # pad rotation away from its open orientation, at rest (closed / reopened)
# While moving: the rigid parallel-grip coupling holds the pads; the compliant
# springs deflect under the fingers' inertia and recover once at rest.
MOVING_TOL = {"parallel_grip": math.radians(2.0), "compliant": math.radians(3.0)}
REOPEN_TOL = math.radians(1.0)

# Known: the 2F-85 Newton_compliant pads swing ~11 deg while closing (settled
# pose is fine). Expected to fail until the Newton spring / inertia tuning changes.
KNOWN_SWING = {("Robotiq_2F_85", "Newton_compliant")}  # left pad: ~11 deg moving, ~2.6 deg settled


def _case_id(gripper: str, physics: str):
    marks = [pytest.mark.xfail(strict=True, reason="pads swing while closing")] if (gripper, physics) in KNOWN_SWING else []
    return pytest.param(gripper, physics, id=f"{gripper}-{physics}", marks=marks)


def _angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Rotation angle (rad) between two wxyz unit quaternions."""
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


@pytest.mark.parametrize(("gripper", "physics"), [_case_id(g, p) for p in PHYSICS for g in GRIPPERS])
def test_fingers_stay_parallel(simulation_app, view, gripper: str, physics: str) -> None:
    scene = load_gripper(simulation_app, gripper, physics, view)
    robot = scene.robot
    pads = RigidPrim([f"{scene.root}/{p}" for p in PADS])
    step(simulation_app, SETTLE_FRAMES)

    finger = scene.dof("finger_joint")
    q_open = pads.get_world_poses()[1].numpy()  # (2, 4) wxyz
    worst = {p: (0.0, "") for p in PADS}  # pad -> (max deviation, phase)
    settled = {}  # phase -> per-pad deviation at the end of that phase
    phase = "closing"

    def deviations() -> list[float]:
        q = pads.get_world_poses()[1].numpy()
        return [_angle_between(q_open[i], q[i]) for i in range(len(PADS))]

    def sample() -> None:
        for p, dev in zip(PADS, deviations()):
            if dev > worst[p][0]:
                worst[p] = (dev, phase)

    # Close, then open, at the datasheet finger speed (ramped targets).
    ramp = move_to(simulation_app, scene, finger, CLOSE_TARGET, settle_frames=HOLD_FRAMES, every=SAMPLE_EVERY, on_sample=sample)
    closed = float(robot.get_dof_positions().numpy()[0][finger])
    settled["closed"] = deviations()

    phase = "opening"
    move_to(simulation_app, scene, finger, 0.0, settle_frames=HOLD_FRAMES, every=SAMPLE_EVERY, on_sample=sample)
    reopened = float(robot.get_dof_positions().numpy()[0][finger])
    settled["reopened"] = deviations()
    omni.timeline.get_timeline_interface().stop()

    print(
        f"{gripper} {physics}: closed {math.degrees(closed):.2f} deg (ramp {ramp * FRAME_DT:.2f} s), "
        f"reopened {math.degrees(reopened):.2f} deg; "
        + ", ".join(f"{p} max {math.degrees(d):.2f} deg ({w})" for p, (d, w) in worst.items())
        + "; settled "
        + ", ".join(f"{k} L/R {math.degrees(v[0]):.2f}/{math.degrees(v[1]):.2f} deg" for k, v in settled.items())
    )
    assert closed > CLOSE_TARGET * 0.9, f"did not close: finger_joint={math.degrees(closed):.2f} deg"
    assert abs(reopened) < REOPEN_TOL, f"did not reopen: finger_joint={math.degrees(reopened):.2f} deg"
    for when, devs in settled.items():
        for p, dev in zip(PADS, devs):
            assert dev < SETTLED_TOL, f"{p} is {math.degrees(dev):.2f} deg off its open orientation when {when}"
    moving_tol = MOVING_TOL["compliant" if physics.endswith("compliant") else "parallel_grip"]
    for p, (dev, when) in worst.items():
        assert dev < moving_tol, f"{p} rotated {math.degrees(dev):.2f} deg from its open orientation while {when}"
