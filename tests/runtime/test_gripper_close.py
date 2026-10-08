"""The gripper closes on command.

For each gripper and physics variant (PhysX and Newton): reference the asset
into a fresh stage, select the engine, play, check it is open by default,
drive the driven joint to a target and check it gets there and the coupled
follower joint tracks it (mimic joint on PhysX, mimic / joint equality on
Newton).
"""

from __future__ import annotations

import math
import runpy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdPhysics

REPO = Path(__file__).resolve().parents[2]

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
    tuning: str | None = None  # script to run once the sim plays (Newton 2F variants)


def _fmt(x: float, unit: str) -> str:
    return f"{math.degrees(x):.2f} deg" if unit == "deg" else f"{x * 1e3:.2f} mm"


_REVOLUTE_2F = dict(
    driven="finger_joint",
    follower="right_outer_knuckle_joint",
    target=0.8,  # ~46 deg; finger_joint upper limit is 47 deg
    open_tol=math.radians(1.0),
    close_tol=math.radians(2.0),
    unit="deg",
)
# Two sliding fingers, 0 (open) to 25 mm each; right_finger_joint mimics the left.
_PRISMATIC_HAND_E = dict(
    driven="left_finger_joint",
    follower="right_finger_joint",
    target=0.020,
    open_tol=0.5e-3,
    close_tol=1.0e-3,
    unit="mm",
)

# (gripper, Physics variant) -> spec. A variant starting with "Newton" runs on
# the Newton engine, anything else on PhysX. The Newton 2F variants need the
# gripper's runtime tuning after each play (see grippers/<gripper>/newton/).
CASES = {
    ("Robotiq_2F_85", "Physx_parallel_grip"): Spec(**_REVOLUTE_2F),
    ("Robotiq_2F_140", "Physx_parallel_grip"): Spec(**_REVOLUTE_2F),
    ("Robotiq_Hand_E", "PhysX"): Spec(**_PRISMATIC_HAND_E),
    ("Robotiq_2F_85", "Newton_parallel_grip"): Spec(**_REVOLUTE_2F, tuning="newton/apply_gripper_tuning.py"),
    ("Robotiq_2F_140", "Newton_parallel_grip"): Spec(**_REVOLUTE_2F, tuning="newton/apply_gripper_tuning.py"),
    ("Robotiq_Hand_E", "Newton"): Spec(**_PRISMATIC_HAND_E),
}


def _engine(physics: str) -> str:
    return "newton" if physics.startswith("Newton") else "physx"


@pytest.mark.parametrize(("gripper", "physics"), list(CASES), ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_closes(simulation_app, view, gripper: str, physics: str) -> None:
    spec = CASES[(gripper, physics)]
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    stage = stage_utils.create_new_stage()
    UsdPhysics.Scene.Define(stage, "/physicsScene")
    stage_utils.add_reference_to_stage(
        usd_path=str(REPO / "grippers" / gripper / f"{gripper}.usda"),
        path="/World/Gripper",
        variants=[("Physics", physics)],
    )
    view()  # --gui only: saved camera + lights for this case
    simulation_app.update()

    # Select the engine once the stage is populated (the switch resets the
    # physics scene), as Isaac's own Newton tests do.
    assert SimulationManager.switch_physics_engine(_engine(physics)), f"could not switch to {_engine(physics)}"
    simulation_app.update()

    # The articulation root sits on the inner <gripper> prim, not the asset root.
    robot = Articulation(f"/World/Gripper/{gripper}")
    timeline.play()
    simulation_app.update()
    if spec.tuning:
        runpy.run_path(str(REPO / "grippers" / gripper / spec.tuning))  # idempotent, needs a built model
    for _ in range(SETTLE_FRAMES):
        simulation_app.update()

    names = robot.dof_names
    driven = names.index(spec.driven)
    follower = names.index(spec.follower)

    q = robot.get_dof_positions().numpy()[0]
    assert np.all(np.isfinite(q)), f"non-finite DOF positions after settle: {q}"
    assert abs(q[driven]) < spec.open_tol, f"not open by default: {spec.driven}={_fmt(q[driven], spec.unit)}"

    robot.set_dof_position_targets([spec.target], dof_indices=[driven])
    for _ in range(CLOSE_FRAMES):
        simulation_app.update()

    q = robot.get_dof_positions().numpy()[0]
    timeline.stop()

    assert np.all(np.isfinite(q)), f"non-finite DOF positions after close: {q}"
    assert abs(q[driven] - spec.target) < spec.close_tol, (
        f"{spec.driven}={_fmt(q[driven], spec.unit)}, target {_fmt(spec.target, spec.unit)}"
    )
    assert abs(q[follower] - q[driven]) < spec.close_tol, (
        f"{spec.follower}={_fmt(q[follower], spec.unit)} does not follow {spec.driven}={_fmt(q[driven], spec.unit)}"
    )
