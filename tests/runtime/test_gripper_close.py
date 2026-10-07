"""The gripper closes on command.

For each gripper (default PhysX variant): reference the asset into a fresh
stage, play, check it is open by default, drive ``finger_joint`` to a target
and check the finger reaches it and the mimic-coupled right knuckle follows.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import stage as stage_utils
from pxr import UsdPhysics

REPO = Path(__file__).resolve().parents[2]
GRIPPERS = ["Robotiq_2F_85", "Robotiq_2F_140"]
PHYSICS = "Physx_parallel_grip"

TARGET_RAD = 0.8  # ~46 deg; finger_joint upper limit is 47 deg on both grippers
SETTLE_FRAMES = 30
CLOSE_FRAMES = 120  # 2 s at 60 Hz
OPEN_TOL_DEG = 1.0
CLOSE_TOL_DEG = 2.0


def _deg(x: float) -> float:
    return math.degrees(float(x))


@pytest.mark.parametrize("gripper", GRIPPERS)
def test_gripper_closes(simulation_app, gripper: str) -> None:
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    stage = stage_utils.create_new_stage()
    UsdPhysics.Scene.Define(stage, "/physicsScene")
    stage_utils.add_reference_to_stage(
        usd_path=str(REPO / "grippers" / gripper / f"{gripper}.usda"),
        path="/World/Gripper",
        variants=[("Physics", PHYSICS)],
    )
    simulation_app.update()

    # The articulation root sits on the inner <gripper> prim, not the asset root.
    robot = Articulation(f"/World/Gripper/{gripper}")
    timeline.play()
    for _ in range(SETTLE_FRAMES):
        simulation_app.update()

    names = robot.dof_names
    finger = names.index("finger_joint")
    right = names.index("right_outer_knuckle_joint")

    q = robot.get_dof_positions().numpy()[0]
    assert np.all(np.isfinite(q)), f"non-finite DOF positions after settle: {q}"
    assert abs(_deg(q[finger])) < OPEN_TOL_DEG, f"not open by default: finger_joint={_deg(q[finger]):.2f} deg"

    robot.set_dof_position_targets([TARGET_RAD], dof_indices=[finger])
    for _ in range(CLOSE_FRAMES):
        simulation_app.update()

    q = robot.get_dof_positions().numpy()[0]
    timeline.stop()

    assert np.all(np.isfinite(q)), f"non-finite DOF positions after close: {q}"
    err = _deg(q[finger]) - _deg(TARGET_RAD)
    assert abs(err) < CLOSE_TOL_DEG, f"finger_joint={_deg(q[finger]):.2f} deg, target {_deg(TARGET_RAD):.2f} deg"
    mimic_err = _deg(q[right]) - _deg(q[finger])
    assert abs(mimic_err) < CLOSE_TOL_DEG, (
        f"right_outer_knuckle_joint={_deg(q[right]):.2f} deg does not follow finger_joint={_deg(q[finger]):.2f} deg"
    )
