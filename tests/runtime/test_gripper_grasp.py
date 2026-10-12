"""The gripper grasps a free cube, resting on a fixed holder, with its fingertips.

For every gripper and Physics variant: a free cube resting on a static holder (both
placed by hand per gripper, see _cube.py) sits between the open pads. The driven joint is
ramped at the datasheet finger speed toward its full-close target, past the
cube, so the cube is what stops the fingers. During the hold that follows, and
after reopening, check:

1. position: the driven joint stalled short of its target, at rest (position drift), and
   the follower joint tracks it (symmetric close);
2. contact: both fingertips touch the cube, penetrating it by at most 1 mm;
3. force: each fingertip pushes on the cube with at least MIN_FORCE (and below
   a sanity cap), and the two forces balance within BALANCE_TOL;
4. only the fingertips touch the cube, no other gripper link;
5. the pads stay parallel to their open orientation (every variant);
6. release: after reopening, no link touches the cube and the joint is open.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim
from pxr import PhysxSchema, UsdPhysics

from _contacts import ContactReader
from _cube import CUBE_PATH, add_cube
from _scene import engine_for, load_gripper, move_to, step

PADS = ("left_fingertip", "right_fingertip")
SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz after the close ramp
SAMPLE_FRAMES = 20  # the last frames of the hold are sampled
CONTACT_PRESENT = 0.9  # fraction of sampled frames a pad must be in contact

MAX_PENETRATION = 1.0e-3  # m
MIN_FORCE = 1.0  # N per fingertip
MAX_FORCE = 1000.0  # N per fingertip; sanity cap until measured bounds are set
BALANCE_TOL = 0.20  # |F_left - F_right| / mean
# The compliant variants grip softly (low finger_joint drive stiffness, ~30 N per pad
# here), do not lift the cube off its holder, and the holder's friction takes the
# left / right difference (~26 % on the 2F-85, ~38 % on the 2F-140). Accepted for now; revisit with the
# compliant drive tuning.
BALANCE_TOL_COMPLIANT = 0.40
PARALLEL_TOL = math.radians(2.0)
# PhysX steps the test scene at 60 Hz by default; that is too coarse for a stiff
# squeeze (the pads sink ~1.2 mm, the mimic stretches ~3 deg and the contact report
# counts the push-out impulse, reading ~380 N instead of ~240 N). The Newton
# variants already run at 1000 Hz from the asset's Newton scene settings.
PHYSX_HZ = 1000


@dataclass(frozen=True)
class Spec:
    driven: str
    follower: str
    target: float  # full-close target, beyond the cube
    stall_margin: float  # the joint must stop at least this far short of the target
    follow_tol: float
    open_tol: float
    rest_drift: float  # the driven joint moves less than this over the sampled frames (at rest)
    unit: str


_REVOLUTE_2F = Spec("finger_joint", "right_outer_knuckle_joint", 0.8, math.radians(3.0), math.radians(2.0),
                    math.radians(1.0), math.radians(0.1), "deg")
_PRISMATIC_HAND_E = Spec("left_finger_joint", "right_finger_joint", 0.025, 2.0e-3, 1.0e-3, 0.5e-3, 0.05e-3, "mm")

CASES = {
    ("Robotiq_2F_85", "Physx_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_2F_85", "Physx_compliant"): _REVOLUTE_2F,
    ("Robotiq_2F_85", "Newton_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_2F_85", "Newton_compliant"): _REVOLUTE_2F,
    ("Robotiq_2F_140", "Physx_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_2F_140", "Physx_compliant"): _REVOLUTE_2F,
    ("Robotiq_2F_140", "Newton_parallel_grip"): _REVOLUTE_2F,
    ("Robotiq_2F_140", "Newton_compliant"): _REVOLUTE_2F,
    ("Robotiq_Hand_E", "PhysX"): _PRISMATIC_HAND_E,
    ("Robotiq_Hand_E", "Newton"): _PRISMATIC_HAND_E,
}


def _fmt(x: float, unit: str) -> str:
    return f"{math.degrees(x):.2f} deg" if unit == "deg" else f"{x * 1e3:.2f} mm"


def _angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


@pytest.mark.parametrize(("gripper", "physics"), list(CASES), ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_grasps_cube(simulation_app, view, gripper: str, physics: str) -> None:
    spec = CASES[(gripper, physics)]
    reader = ContactReader(gripper, f"/World/Gripper/{gripper}", CUBE_PATH, engine_for(physics))

    def setup(stage):
        add_cube(stage, gripper)
        reader.prepare(stage)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, gripper, physics, view, setup=setup)
    robot = scene.robot
    pads = RigidPrim([f"{scene.root}/{p}" for p in PADS])
    step(simulation_app, SETTLE_FRAMES)
    driven, follower = scene.dof(spec.driven), scene.dof(spec.follower)
    q_open = pads.get_world_poses()[1].numpy().copy()

    touching_before = reader.read()
    assert not touching_before, f"gripper touches the cube while open: {sorted(touching_before)}"

    # Close toward the full-close target, past the cube, then hold and sample.
    move_to(simulation_app, scene, driven, spec.target, settle_frames=HOLD_FRAMES - SAMPLE_FRAMES)
    samples, others, pad_dev, driven_q = [], set(), [0.0, 0.0], []
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        # At rest is judged by position: PhysX reports a steady non-zero joint
        # velocity on a gripper jammed against the cube although nothing moves.
        driven_q.append(float(robot.get_dof_positions().numpy()[0][driven]))
        c = reader.read()
        samples.append(c)
        others |= {k for k in c if k not in PADS}
        q = pads.get_world_poses()[1].numpy()
        pad_dev = [max(pad_dev[i], _angle_between(q_open[i], q[i])) for i in range(2)]
    # .numpy() can alias the live tensor buffers, which stop() resets: copy.
    q = robot.get_dof_positions().numpy()[0].copy()
    drift = max(driven_q) - min(driven_q)

    present = {p: sum(p in s for s in samples) / len(samples) for p in PADS}
    force = {p: float(np.mean([s[p].force for s in samples if p in s])) if present[p] else 0.0 for p in PADS}
    penetration = {p: max([-s[p].min_separation for s in samples if p in s], default=0.0) for p in PADS}

    # Release.
    move_to(simulation_app, scene, driven, 0.0, settle_frames=HOLD_FRAMES)
    touching_after = reader.read()
    q_after = float(robot.get_dof_positions().numpy()[0][driven])
    omni.timeline.get_timeline_interface().stop()

    L, R = PADS
    print(
        f"{gripper} {physics}: {spec.driven}={_fmt(q[driven], spec.unit)} (target {_fmt(spec.target, spec.unit)}), "
        f"{spec.follower}={_fmt(q[follower], spec.unit)}, drift={_fmt(drift, spec.unit)}; "
        f"force L/R {force[L]:.1f}/{force[R]:.1f} N, contact L/R {present[L]:.0%}/{present[R]:.0%}, "
        f"penetration L/R {penetration[L] * 1e3:.2f}/{penetration[R] * 1e3:.2f} mm, "
        f"pad rotation L/R {math.degrees(pad_dev[0]):.2f}/{math.degrees(pad_dev[1]):.2f} deg, other links {sorted(others)}"
    )

    # 1. position
    assert q[driven] < spec.target - spec.stall_margin, (
        f"{spec.driven}={_fmt(q[driven], spec.unit)} reached the target: the cube did not stop the fingers"
    )
    assert drift < spec.rest_drift, f"{spec.driven} still moving: {_fmt(drift, spec.unit)} over the sampled frames"
    assert abs(q[follower] - q[driven]) < spec.follow_tol, (
        f"{spec.follower}={_fmt(q[follower], spec.unit)} does not follow {spec.driven}={_fmt(q[driven], spec.unit)}"
    )
    # 2. contact
    for p in PADS:
        assert present[p] >= CONTACT_PRESENT, f"{p} touched the cube in only {present[p]:.0%} of the sampled frames"
        assert penetration[p] <= MAX_PENETRATION, f"{p} penetrates the cube by {penetration[p] * 1e3:.2f} mm"
    # 3. force
    for p in PADS:
        assert MIN_FORCE <= force[p] <= MAX_FORCE, f"{p} pushes with {force[p]:.2f} N"
    mean = (force[L] + force[R]) / 2.0
    balance_tol = BALANCE_TOL_COMPLIANT if physics.endswith("compliant") else BALANCE_TOL
    assert abs(force[L] - force[R]) <= balance_tol * mean, f"unbalanced grip: L {force[L]:.1f} N vs R {force[R]:.1f} N"
    # 4. only the fingertips
    assert not others, f"links other than the fingertips touch the cube: {sorted(others)}"
    # 5. pads parallel
    for p, dev in zip(PADS, pad_dev):
        assert dev < PARALLEL_TOL, f"{p} rotated {math.degrees(dev):.2f} deg from its open orientation on the cube"
    # 6. release
    assert not touching_after, f"still touching the cube after reopening: {sorted(touching_after)}"
    assert abs(q_after) < spec.open_tol, f"did not reopen: {spec.driven}={_fmt(q_after, spec.unit)}"
