"""The 2F grippers close on a cylinder fixed in space.

For both 2F grippers on all four Physics variants: a cylinder that nothing can move (a
static collider, no holder, axis along the gripper's X for every gripper, placed and
sized by hand per gripper in fixed_cylinders/, see _cylinder.py) sits between the open pads. The driven joint is
ramped at the datasheet finger speed toward its full-close target, past the cylinder,
so the cylinder stops the fingers. PhysX and Newton must behave the same, so they
share one baseline per grip (parallel_grip, compliant). During the hold that
follows, check against it:

1. each finger's knuckle joint (finger_joint on the left, right_outer_knuckle_joint
   on the right) stalls within the grip's joint tolerance of the baseline, at rest;
2. each fingertip's rotation from its open orientation is within the grip's pad
   tolerance of the baseline (about zero on parallel grip; the compliant pads tilt).

The baselines live in the gripper's preset: "recorded" holds what each Physics
variant measured, "expected" the shared baseline per grip, the mean of its PhysX and
Newton recordings. Record them with ``--record-baseline`` (run both engines); moving
the cylinder with ``--save-cube`` drops them.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim
from pxr import PhysxSchema, UsdPhysics

from _cylinder import add_fixed_cylinder, load_preset, write_preset
from _scene import engine_for, load_gripper, move_to, step

PADS = ("left_fingertip", "right_fingertip")
JOINTS = ("finger_joint", "right_outer_knuckle_joint")  # left, right
DRIVEN = "finger_joint"
TARGET = 0.8  # rad, full close: beyond the cylinder
SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz after the close ramp
SAMPLE_FRAMES = 20  # the last frames of the hold are sampled

# (joint, pad) tolerance per grip, around the shared PhysX / Newton baseline. The
# compliant variants differ more between engines: on the 2F-140 Newton closes ~3 deg
# further and its pads tilt ~2 deg more.
TOL = {
    "parallel_grip": (math.radians(1.0), math.radians(1.0)),
    "compliant": (math.radians(2.0), math.radians(1.5)),
}
# The 2F-85 compliant PhysX and Newton recordings are far apart, so each is far from their
# shared mean: a looser tolerance for that gripper only.
GRIPPER_TOL = {
    ("Robotiq_2F_85", "compliant"): (math.radians(3.0), math.radians(5.5)),
    # The 2F-140 compliant PhysX and Newton recordings are 3.9 deg apart (the pads of the two engines
    # close differently on the cylinder), so each sits ~1.95 deg from their shared mean, plus the PhysX
    # left/right knuckle asymmetry: a wider joint tolerance for that gripper only.
    ("Robotiq_2F_140", "compliant"): (math.radians(2.5), math.radians(1.5)),
}
PHYSX_HZ = 1000  # as in test_gripper_grasp.py: 60 Hz mis-resolves a stiff squeeze
REST_DRIFT = math.radians(0.1)  # each joint moves less than this over the sampled frames
# The 2F-85 Newton_compliant finger is still ringing down slowly after it hits the cylinder (a lightly
# damped oscillation, a little of it left in the sampled frames), so it gets a looser rest limit.
CASE_REST_DRIFT = {("Robotiq_2F_85", "Newton_compliant"): math.radians(0.25)}

CASES = [
    (g, p)
    for g in ("Robotiq_2F_85", "Robotiq_2F_140")
    for p in ("Physx_parallel_grip", "Physx_compliant", "Newton_parallel_grip", "Newton_compliant")
]


def grip_of(physics: str) -> str:
    return physics.split("_", 1)[1]  # "Physx_parallel_grip" -> "parallel_grip"


def _angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


@pytest.mark.parametrize(("gripper", "physics"), CASES, ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_closes_on_fixed_cylinder(request, simulation_app, view, gripper: str, physics: str) -> None:
    def setup(stage):
        add_fixed_cylinder(stage, gripper)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, gripper, physics, view, setup=setup)
    robot = scene.robot
    pads = RigidPrim([f"{scene.root}/{p}" for p in PADS])
    step(simulation_app, SETTLE_FRAMES)
    dofs = [scene.dof(j) for j in JOINTS]
    q_open = pads.get_world_poses()[1].numpy().copy()

    # Close toward full close, past the cylinder, then hold and sample.
    move_to(simulation_app, scene, scene.dof(DRIVEN), TARGET, settle_frames=HOLD_FRAMES - SAMPLE_FRAMES)
    joint_q = []
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        # .numpy() can alias the live tensor buffers, which stop() resets: copy.
        joint_q.append(robot.get_dof_positions().numpy()[0][dofs].copy())
    q_pads = pads.get_world_poses()[1].numpy().copy()
    omni.timeline.get_timeline_interface().stop()

    joint_q = np.array(joint_q)
    q = joint_q[-1]
    drift = joint_q.max(axis=0) - joint_q.min(axis=0)
    measured = {f"{j}_deg": round(math.degrees(float(v)), 3) for j, v in zip(JOINTS, q)}
    measured |= {f"{p}_rotation_deg": round(math.degrees(_angle_between(q_open[i], q_pads[i])), 3)
                 for i, p in enumerate(PADS)}
    print(
        f"{gripper} {physics}: "
        + ", ".join(f"{j}={measured[f'{j}_deg']:.2f} deg (drift {math.degrees(d):.3f})" for j, d in zip(JOINTS, drift))
        + f"; pad rotation L/R {measured[f'{PADS[0]}_rotation_deg']:.2f}/{measured[f'{PADS[1]}_rotation_deg']:.2f} deg"
    )

    grip = grip_of(physics)
    joint_tol, pad_tol = GRIPPER_TOL.get((gripper, grip), TOL[grip])
    rest_drift = CASE_REST_DRIFT.get((gripper, physics), REST_DRIFT)
    preset = load_preset(gripper)
    if request.config.getoption("--record-baseline"):
        recorded = preset.setdefault("recorded", {})
        recorded[physics] = measured
        runs = [v for k, v in recorded.items() if grip_of(k) == grip]
        preset.setdefault("expected", {})[grip] = {
            k: round(sum(r[k] for r in runs) / len(runs), 3) for k in measured
        }
        print(f"baseline recorded ({grip}, mean of {len(runs)}): {write_preset(gripper, preset)}")
        return
    expected = preset.get("expected", {}).get(grip)
    assert expected, f"no {grip} baseline in fixed_cylinders/{gripper}.json: run with --record-baseline"
    # 1. joint positions, at rest
    for j, d in zip(JOINTS, drift):
        got, want = measured[f"{j}_deg"], expected[f"{j}_deg"]
        assert abs(got - want) <= math.degrees(joint_tol), f"{j}={got:.2f} deg, baseline {want:.2f} deg"
        assert d < rest_drift, f"{j} still moving: {math.degrees(d):.3f} deg over the sampled frames"
    # 2. fingertip orientation
    for p in PADS:
        got, want = measured[f"{p}_rotation_deg"], expected[f"{p}_rotation_deg"]
        assert abs(got - want) <= math.degrees(pad_tol), (
            f"{p} rotated {got:.2f} deg from its open orientation, baseline {want:.2f} deg"
        )
