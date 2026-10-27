"""The Hand-E C10 grasps a free puck on its axis with its three fingertips.

The C10 closes three fingers, 120 degrees apart, toward the gripper axis, so the two-pad
cube test does not apply. A free puck (a short puck) lies on the axis, on the top of
the housing, between the open pads. The stepped fingertip's pad is only the lowest 9 mm of
the tip (the tip steps back above it), so the puck is 8 mm tall: a taller puck is held
at its bottom edge only and tilts. The driven joint is ramped at the datasheet finger speed toward its
full-close target, past the puck, so the puck is what stops the fingers. During the
hold that follows, and after reopening, check:

1. position: the driven joint stalled short of its target, at rest (position drift), and
   the two followers track it (symmetric close);
2. contact: all three fingertips touch the puck, penetrating it by at most 1 mm;
3. force: each fingertip pushes on the puck with at least MIN_FORCE (and below a sanity
   cap) and carries at least MIN_SHARE of the mean pad force. The normal forces are NOT
   required to be equal: with three pads and friction the grasp is statically indeterminate,
   so the pads' friction can carry part of the difference (the two-pad cube test balances
   them because there the friction cannot). What shows the grasp is sound is that the puck
   ends up centred on the axis and at rest;
4. only the fingertips and the housing (it carries the puck) touch the puck;
5. release: after reopening, no fingertip touches the puck and the joint is open.
"""

from __future__ import annotations

import numpy as np
import pytest
from isaacsim.core.experimental.prims import RigidPrim
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from _contacts import ContactReader
from _scene import engine_for, load_gripper, move_to, step

GRIPPER = "Robotiq_Hand_E_C10"
PADS = ("fingertip_1", "fingertip_2", "fingertip_3")
CARRIER = ("base_link",)  # the housing's top face carries the puck: allowed to touch it
DRIVEN, FOLLOWERS = "finger_1_joint", ("finger_2_joint", "finger_3_joint")
TARGET = 0.0192  # full close (the joint's upper limit), beyond the puck
STALL_MARGIN = 2.0e-3  # the joint must stop at least this far short of the target
FOLLOW_TOL = 1.0e-3
OPEN_TOL = 0.5e-3
REST_DRIFT = 0.05e-3

PUCK_PATH = "/World/Puck"
RADIUS = 0.012  # m: 24 mm across, between the open pads (40 mm) and well above the closed ones (2 mm)
HEIGHT = 0.008  # m, inside the pads' 9 mm (z 114 to 123 mm)
MASS = 0.020  # kg
HOUSING_TOP = 0.1152  # m, z of the top face of the base's front plate: the puck stands on it

SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz after the close ramp
SAMPLE_FRAMES = 20  # the last frames of the hold are sampled
CONTACT_PRESENT = 0.9  # fraction of sampled frames a pad must be in contact

MAX_PENETRATION = 1.0e-3  # m
MIN_FORCE = 1.0  # N per fingertip
MAX_FORCE = 1000.0  # N per fingertip; sanity cap until measured bounds are set
MIN_SHARE = 0.4  # every pad carries at least this fraction of the mean pad force
CENTRE_TOL = 1.0e-3  # m, the puck's axis ends up this close to the gripper axis
PUCK_REST = 0.05e-3  # m, the puck moves less than this over the sampled frames
PHYSX_HZ = 1000  # PhysX steps the test scene at 60 Hz by default: too coarse for a stiff squeeze (see test_gripper_grasp.py)

CASES = ["PhysX", "Newton"]


def add_puck(stage) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, PUCK_PATH)
    cyl.CreateAxisAttr("Z")
    cyl.CreateRadiusAttr(RADIUS)
    cyl.CreateHeightAttr(HEIGHT)
    cyl.GetDisplayColorAttr().Set([Gf.Vec3f(0.25, 0.65, 0.35)])
    # standing on the housing, 0.5 mm clear so the settle is a drop, not an overlap
    UsdGeom.Xformable(cyl.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, HOUSING_TOP + HEIGHT / 2 + 0.0005))
    UsdPhysics.RigidBodyAPI.Apply(cyl.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    UsdPhysics.MassAPI.Apply(cyl.GetPrim()).CreateMassAttr(MASS)


@pytest.mark.parametrize("physics", CASES)
def test_c10_grasps_puck(simulation_app, view, physics: str) -> None:
    reader = ContactReader(GRIPPER, f"/World/Gripper/{GRIPPER}", PUCK_PATH, engine_for(physics))

    def setup(stage):
        add_puck(stage)
        reader.prepare(stage)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, GRIPPER, physics, view, setup=setup)
    robot = scene.robot
    step(simulation_app, SETTLE_FRAMES)
    puck = RigidPrim(PUCK_PATH)
    driven = scene.dof(DRIVEN)
    followers = [scene.dof(n) for n in FOLLOWERS]

    open_contacts = {k for k in reader.read() if k in PADS}
    assert not open_contacts, f"a fingertip touches the puck while open: {sorted(open_contacts)}"

    # Close toward the full-close target, past the puck, then hold and sample.
    move_to(simulation_app, scene, driven, TARGET, settle_frames=HOLD_FRAMES - SAMPLE_FRAMES)
    samples, others, driven_q, puck_xy = [], set(), [], []
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        # At rest is judged by position (PhysX reports a steady non-zero velocity on a jammed gripper).
        driven_q.append(float(robot.get_dof_positions().numpy()[0][driven]))
        puck_xy.append(puck.get_world_poses()[0].numpy()[0][:2].copy())
        c = reader.read()
        samples.append(c)
        # Newton also lists links within 1 mm of the puck (its soft contacts let the pads sink ~1 mm, which brings the
        # finger stubs that close): only a link that actually pushes counts.
        others |= {k for k, v in c.items() if k not in PADS and k not in CARRIER and v.force > 0.0}
    q = robot.get_dof_positions().numpy()[0].copy()  # .numpy() can alias live buffers, which stop() resets
    drift = max(driven_q) - min(driven_q)

    present = {p: sum(p in s for s in samples) / len(samples) for p in PADS}
    force = {p: float(np.mean([s[p].force for s in samples if p in s])) if present[p] else 0.0 for p in PADS}
    penetration = {p: max([-s[p].min_separation for s in samples if p in s], default=0.0) for p in PADS}

    # Release.
    move_to(simulation_app, scene, driven, 0.0, settle_frames=HOLD_FRAMES)
    q_open = robot.get_dof_positions().numpy()[0].copy()
    after = {k for k in reader.read() if k in PADS}

    mean_force = float(np.mean(list(force.values())))
    centre = float(np.linalg.norm(puck_xy[-1]))
    puck_drift = float(np.linalg.norm(np.ptp(np.array(puck_xy), axis=0)))
    print(
        f"{GRIPPER} {physics}: {DRIVEN}={q[driven] * 1e3:.2f} mm (target {TARGET * 1e3:.2f}), "
        f"followers={[round(float(q[f]) * 1e3, 2) for f in followers]} mm, drift {drift * 1e3:.3f} mm; "
        f"force per pad {[round(f, 1) for f in force.values()]} N, puck {centre * 1e3:.2f} mm off the axis (moved {puck_drift * 1e3:.3f} mm), "
        f"penetration {[round(p * 1e3, 2) for p in penetration.values()]} mm; reopened to {q_open[driven] * 1e3:.2f} mm"
    )

    assert q[driven] < TARGET - STALL_MARGIN, f"did not stall on the puck: {q[driven] * 1e3:.2f} mm, target {TARGET * 1e3:.2f}"
    assert drift < REST_DRIFT, f"driven joint not at rest: moved {drift * 1e3:.3f} mm over the sampled frames"
    for name, dof in zip(FOLLOWERS, followers):
        assert abs(q[dof] - q[driven]) < FOLLOW_TOL, f"{name}={q[dof] * 1e3:.2f} mm does not follow {DRIVEN}={q[driven] * 1e3:.2f} mm"
    for p in PADS:
        assert present[p] >= CONTACT_PRESENT, f"{p} touches the puck in only {present[p]:.0%} of the sampled frames"
        assert penetration[p] <= MAX_PENETRATION, f"{p} penetrates the puck by {penetration[p] * 1e3:.2f} mm"
        assert MIN_FORCE <= force[p] <= MAX_FORCE, (
            f"{p} pushes with {force[p]:.1f} N, expected {MIN_FORCE} to {MAX_FORCE} N; per sampled frame "
            f"{[[round(s[q].force, 1) if q in s else None for s in samples] for q in PADS]}, present {present}"
        )
    for p in PADS:
        assert force[p] >= MIN_SHARE * mean_force, f"{p} carries only {force[p]:.1f} N of a mean {mean_force:.1f} N: {force}"
    assert centre <= CENTRE_TOL, f"the puck is {centre * 1e3:.2f} mm off the gripper axis"
    assert puck_drift < PUCK_REST, f"the puck is not at rest: moved {puck_drift * 1e3:.3f} mm over the sampled frames"
    assert not others, f"links other than the fingertips and the housing touch the puck: {sorted(others)}"
    assert not after, f"a fingertip still touches the puck after reopening: {sorted(after)}"
    assert abs(q_open[driven]) < OPEN_TOL, f"not open after release: {q_open[driven] * 1e3:.2f} mm"
