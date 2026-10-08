"""The gripper closes on command.

For each gripper and physics variant (PhysX and Newton): load it, check it is
open by default, drive the driven joint to a target and check it gets there
and the coupled follower joint tracks it (mimic joint on PhysX, mimic / joint
equality on Newton).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import omni.timeline
import pytest

from _scene import load_gripper, step

SETTLE_FRAMES = 30
CLOSE_FRAMES = 120  # 2 s at 60 Hz


@dataclass(frozen=True)
class Spec:
    driven: str  # DOF that receives the position target
    follower: str  # DOF coupled to it (same sign, same magnitude when closed)
    target: float  # close target, in the DOF's unit (rad or m)
    open_tol: float
    close_tol: float
    unit: str  # for messages: "deg" or "mm"


def _fmt(x: float, unit: str) -> str:
    return f"{math.degrees(x):.2f} deg" if unit == "deg" else f"{x * 1e3:.2f} mm"


_REVOLUTE_2F = Spec(
    driven="finger_joint",
    follower="right_outer_knuckle_joint",
    target=0.8,  # ~46 deg; finger_joint upper limit is 47 deg
    open_tol=math.radians(1.0),
    close_tol=math.radians(2.0),
    unit="deg",
)
# Two sliding fingers, 0 (open) to 25 mm each; right_finger_joint mimics the left.
_PRISMATIC_HAND_E = Spec(
    driven="left_finger_joint",
    follower="right_finger_joint",
    target=0.020,
    open_tol=0.5e-3,
    close_tol=1.0e-3,
    unit="mm",
)

# (gripper, Physics variant) -> spec; the engine follows the variant name.
CASES = {
    ("Robotiq_2F_85", "Physx_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_2F_140", "Physx_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_Hand_E", "PhysX"): _PRISMATIC_HAND_E,
    ("Robotiq_2F_85", "Newton_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_2F_140", "Newton_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_Hand_E", "Newton"): _PRISMATIC_HAND_E,
}


@pytest.mark.parametrize(("gripper", "physics"), list(CASES), ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_closes(simulation_app, view, gripper: str, physics: str) -> None:
    spec = CASES[(gripper, physics)]
    scene = load_gripper(simulation_app, gripper, physics, view)
    robot = scene.robot
    step(simulation_app, SETTLE_FRAMES)

    driven = scene.dof(spec.driven)
    follower = scene.dof(spec.follower)

    q = robot.get_dof_positions().numpy()[0]
    assert np.all(np.isfinite(q)), f"non-finite DOF positions after settle: {q}"
    assert abs(q[driven]) < spec.open_tol, f"not open by default: {spec.driven}={_fmt(q[driven], spec.unit)}"

    robot.set_dof_position_targets([spec.target], dof_indices=[driven])
    step(simulation_app, CLOSE_FRAMES)

    q = robot.get_dof_positions().numpy()[0]
    omni.timeline.get_timeline_interface().stop()

    assert np.all(np.isfinite(q)), f"non-finite DOF positions after close: {q}"
    assert abs(q[driven] - spec.target) < spec.close_tol, (
        f"{spec.driven}={_fmt(q[driven], spec.unit)}, target {_fmt(spec.target, spec.unit)}"
    )
    assert abs(q[follower] - q[driven]) < spec.close_tol, (
        f"{spec.follower}={_fmt(q[follower], spec.unit)} does not follow {spec.driven}={_fmt(q[driven], spec.unit)}"
    )
