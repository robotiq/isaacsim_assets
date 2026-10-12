"""The 2F grippers close on a cube fixed in space.

For both 2F grippers, on PhysX and Newton, in two tests: the parallel-grip variants
and the compliant variants. A cube that nothing can move (a static collider, no
holder, placed by hand per gripper in fixed_cubes/, see _cube.py; the compliant test
has its own placement, fixed_cubes/<gripper>_compliant.json) sits between the open
pads. The driven joint is ramped at the datasheet finger speed
toward its full-close target, past the cube, so the cube stops the fingers. During
the hold that follows, check:

1. each finger's knuckle joint (finger_joint on the left, right_outer_knuckle_joint
   on the right) stalls at its recorded baseline, within JOINT_TOL, at rest;
2. the pads stay parallel to their open orientation.

The baselines live in the case's preset, under "expected" per Physics variant.
Record them with ``--record-baseline`` (it writes the measured positions instead of
checking them); moving or resizing the cube with ``--save-cube`` drops them.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim
from pxr import PhysxSchema, UsdPhysics

from _cube import add_fixed_cube, load_preset, write_preset
from _scene import engine_for, load_gripper, move_to, step

PADS = ("left_fingertip", "right_fingertip")
JOINTS = ("finger_joint", "right_outer_knuckle_joint")  # left, right
DRIVEN = "finger_joint"
TARGET = 0.8  # rad, full close: beyond the cube
SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz after the close ramp
SAMPLE_FRAMES = 20  # the last frames of the hold are sampled

JOINT_TOL = math.radians(1.0)
REST_DRIFT = math.radians(0.1)  # each joint moves less than this over the sampled frames
PARALLEL_TOL = math.radians(2.0)
PHYSX_HZ = 1000  # as in test_gripper_grasp.py: 60 Hz mis-resolves a stiff squeeze

CASES = [
    ("Robotiq_2F_85", "Physx_parallel_grip"),
    ("Robotiq_2F_85", "Newton_parallel_grip"),
    ("Robotiq_2F_140", "Physx_parallel_grip"),
    ("Robotiq_2F_140", "Newton_parallel_grip"),
]
COMPLIANT_CASES = [
    ("Robotiq_2F_85", "Physx_compliant"),
    ("Robotiq_2F_85", "Newton_compliant"),
    ("Robotiq_2F_140", "Physx_compliant"),
    ("Robotiq_2F_140", "Newton_compliant"),
]


def preset_name(gripper: str, physics: str) -> str:
    """The fixed_cubes/ preset of a case: the compliant variants have their own."""
    return f"{gripper}_compliant" if physics.endswith("compliant") else gripper


def _angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


@pytest.mark.parametrize(("gripper", "physics"), CASES, ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_closes_on_fixed_cube(request, simulation_app, view, gripper: str, physics: str) -> None:
    _close_on_fixed_cube(request, simulation_app, view, gripper, physics)


@pytest.mark.parametrize(("gripper", "physics"), COMPLIANT_CASES, ids=[f"{g}-{p}" for g, p in COMPLIANT_CASES])
def test_compliant_gripper_closes_on_fixed_cube(request, simulation_app, view, gripper: str, physics: str) -> None:
    _close_on_fixed_cube(request, simulation_app, view, gripper, physics)


def _close_on_fixed_cube(request, simulation_app, view, gripper: str, physics: str) -> None:
    name = preset_name(gripper, physics)

    def setup(stage):
        add_fixed_cube(stage, name)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, gripper, physics, view, setup=setup)
    robot = scene.robot
    pads = RigidPrim([f"{scene.root}/{p}" for p in PADS])
    step(simulation_app, SETTLE_FRAMES)
    dofs = [scene.dof(j) for j in JOINTS]
    q_open = pads.get_world_poses()[1].numpy().copy()

    # Close toward full close, past the cube, then hold and sample.
    move_to(simulation_app, scene, scene.dof(DRIVEN), TARGET, settle_frames=HOLD_FRAMES - SAMPLE_FRAMES)
    joint_q, pad_dev = [], [0.0, 0.0]
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        # .numpy() can alias the live tensor buffers, which stop() resets: copy.
        joint_q.append(robot.get_dof_positions().numpy()[0][dofs].copy())
        q = pads.get_world_poses()[1].numpy()
        pad_dev = [max(pad_dev[i], _angle_between(q_open[i], q[i])) for i in range(2)]
    omni.timeline.get_timeline_interface().stop()

    joint_q = np.array(joint_q)
    q = joint_q[-1]
    drift = joint_q.max(axis=0) - joint_q.min(axis=0)
    measured = {j: round(math.degrees(float(v)), 3) for j, v in zip(JOINTS, q)}
    print(
        f"{gripper} {physics}: "
        + ", ".join(f"{j}={measured[j]:.2f} deg (drift {math.degrees(d):.3f})" for j, d in zip(JOINTS, drift))
        + f"; pad rotation L/R {math.degrees(pad_dev[0]):.2f}/{math.degrees(pad_dev[1]):.2f} deg"
    )

    preset = load_preset(name, fixed=True)
    if request.config.getoption("--record-baseline"):
        preset.setdefault("expected", {})[physics] = {f"{j}_deg": measured[j] for j in JOINTS}
        print(f"baseline recorded: {write_preset(name, preset, fixed=True)}")
    else:
        expected = preset.get("expected", {}).get(physics)
        assert expected, f"no baseline for {physics} in fixed_cubes/{name}.json: run with --record-baseline"
        # 1. joint positions, at rest
        for j, v, d in zip(JOINTS, q, drift):
            want = math.radians(expected[f"{j}_deg"])
            assert abs(v - want) <= JOINT_TOL, (
                f"{j}={math.degrees(v):.2f} deg, baseline {expected[f'{j}_deg']:.2f} deg"
            )
            assert d < REST_DRIFT, f"{j} still moving: {math.degrees(d):.3f} deg over the sampled frames"
    # 2. pads parallel
    for p, dev in zip(PADS, pad_dev):
        assert dev < PARALLEL_TOL, f"{p} rotated {math.degrees(dev):.2f} deg from its open orientation on the cube"
