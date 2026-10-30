"""The Hand-E C10 grasps a free puck on its axis with its three fingertips.

The C10 closes three fingers, 120 degrees apart, toward the gripper axis, so the two-pad
cube test does not apply. A free puck lies on the axis, on the top of
the housing, between the open pads. The stepped fingertip's pad is only the lowest 9 mm of
the tip (the tip steps back above it), so the puck is 8 mm tall: a taller puck is held
at its bottom edge only and tilts. The driven joint is ramped at the datasheet finger speed toward its
full-close target, past the puck, so the puck is what stops the fingers. During the
hold that follows, and after reopening, check:

1. position: the driven joint stalled short of its target, at rest (position drift), and
   the two followers track it (symmetric close);
2. contact: all three fingertips touch the puck, penetrating it by at most the tip's TIP_LIMITS (about 1 mm);
3. force: at least two fingertips (all three when none is wedged, see TIP_LIMITS) push on the
   puck with at least MIN_FORCE (and below a sanity cap), and when all three push each carries
   at least MIN_SHARE of the mean pad force. The normal forces are NOT
   required to be equal: with three pads and friction the grasp is statically indeterminate,
   so the pads' friction can carry part of the difference (the two-pad cube test balances
   them because there the friction cannot). What shows the grasp is sound is that the puck
   ends up centred on the axis and at rest. The mean pad force is a third of the drive's push
   (its stiffness x the stroke left, up to the 525 N cap) within FORCE_TOL: the cap, 175 N per
   pad, on a 24 mm puck, and below it on a 10 mm pin, where the stroke left is short;
4. only the fingertips and the housing (it carries the puck) touch the puck;
5. release: after reopening, no fingertip touches the puck and the joint is open.
"""

from __future__ import annotations

import numpy as np
import omni.usd
import pytest
from isaacsim.core.experimental.prims import RigidPrim
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

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
# Puck radii (m): 24 mm across, between the open pads (40 mm) and above the 12.5 mm from which the drive reaches its force cap;
# and 10 mm across, a pin, below it, where the force follows the stroke left.
RADII = (0.012, 0.005)
HEIGHT = 0.008  # m, inside the pads' 9 mm (z 114 to 123 mm)
MASS = 0.020  # kg
HOUSING_TOP = 0.1152  # m, z of the top face of the base's front plate: the puck stands on it

SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz after the close ramp
SAMPLE_FRAMES = 20  # the last frames of the hold are sampled
CONTACT_PRESENT = 0.9  # fraction of sampled frames a pad must be in contact

# Per tip: (penetration cap, pads that must push, extra links allowed to touch).
# With three pads and friction the grasp is statically indeterminate, and the puck can wedge between two pads by friction
# while the third stays 0.1 mm clear and pushes with nothing (PhysX, the 10 mm pin or the extender): two pushing pads are
# required, and every pad's share only when all three push. The extender's pad is a 3 mm wide nose, not a 12 mm face: the
# same squeeze presses it deeper into Newton's soft contact (1.44 mm at 158 N on each nose, against about 1.0 mm for the
# stepped tip, which is allowed 1.2 mm), and a nose that sinks 1.4 mm also brings the finger stub behind it into touch.
TIP_LIMITS = {
    "Stepped": (1.2e-3, 2, ()),
    "Extender": (2.0e-3, 2, ("finger_1", "finger_2", "finger_3")),
    "Flipped": (1.2e-3, 2, ()),  # the stepped tip's pad, at the same place
}
# Total gripper mass (kg) per tip: the datasheet's 1.7 kg, plus the manual's +180.22 g for the three extenders (1875.22 g
# against 1695 g) in the Extender variant.
GRIPPER_MASS = {"Stepped": 1.7, "Extender": 1.7 + 0.18022, "Flipped": 1.7}
MASS_TOL = 1.0e-3  # kg
DRIVE_CAP = 525.0  # N: the driven joint's cap, three times the datasheet's 175 N per finger (the three fingers share it)
DRIVE_STIFFNESS = 100000.0  # N/m, as authored in the payloads: the drive pushes with stiffness x the remaining stroke
FORCE_TOL = 0.20  # the mean pad force is within this fraction of the drive's
MIN_FORCE = 1.0  # N per fingertip
MAX_FORCE = 1000.0  # N per fingertip; sanity cap until measured bounds are set
MIN_SHARE = 0.4  # every pad carries at least this fraction of the mean pad force
CENTRE_TOL = 1.0e-3  # m, the puck's axis ends up this close to the gripper axis
PUCK_REST = 0.05e-3  # m, the puck moves less than this over the sampled frames
PHYSX_HZ = 1000  # PhysX steps the test scene at 60 Hz by default: too coarse for a stiff squeeze (see test_gripper_grasp.py)

# (gripper, Physics variant, Fingertip variant): the gripper is a parameter so that the test id names it, which is how the
# saved view presets (tests/runtime/views/<gripper>.json) are found. Both tips present the same 9 mm tall pad (z 114 to 123 mm)
# whose inner face is 20.2 mm from the axis, so the same puck serves both.
CASES = [(GRIPPER, ph, tip, r) for r in RADII for tip in ("Stepped", "Extender", "Flipped") for ph in ("PhysX", "Newton")]


def add_puck(stage, radius: float) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, PUCK_PATH)
    cyl.CreateAxisAttr("Z")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(HEIGHT)
    cyl.GetDisplayColorAttr().Set([Gf.Vec3f(0.25, 0.65, 0.35)])
    # standing on the housing, 0.5 mm clear so the settle is a drop, not an overlap
    UsdGeom.Xformable(cyl.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, HOUSING_TOP + HEIGHT / 2 + 0.0005))
    UsdPhysics.RigidBodyAPI.Apply(cyl.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    UsdPhysics.MassAPI.Apply(cyl.GetPrim()).CreateMassAttr(MASS)


@pytest.mark.parametrize(
    ("gripper", "physics", "tip", "radius"), CASES, ids=[f"{g}-{p}-{t}-{round(2e3 * r)}mm" for g, p, t, r in CASES]
)
def test_c10_grasps_puck(simulation_app, view, gripper: str, physics: str, tip: str, radius: float) -> None:
    reader = ContactReader(GRIPPER, f"/World/Gripper/{GRIPPER}", PUCK_PATH, engine_for(physics))

    def setup(stage):
        add_puck(stage, radius)
        reader.prepare(stage)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, GRIPPER, physics, view, setup=setup, variants=[("Fingertip", tip)])
    robot = scene.robot
    stage = omni.usd.get_context().get_stage()
    mass = sum(
        p.GetAttribute("physics:mass").Get() or 0.0
        for p in Usd.PrimRange(stage.GetPrimAtPath(f"/World/Gripper/{GRIPPER}"), Usd.TraverseInstanceProxies())
        if p.HasAPI(UsdPhysics.MassAPI)
    )
    assert abs(mass - GRIPPER_MASS[tip]) < MASS_TOL, f"{tip}: the bodies weigh {mass:.4f} kg, expected {GRIPPER_MASS[tip]:.4f} kg"
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
        f"{GRIPPER} {physics} {tip} {2e3 * radius:g} mm: {DRIVEN}={q[driven] * 1e3:.2f} mm (target {TARGET * 1e3:.2f}), "
        f"followers={[round(float(q[f]) * 1e3, 2) for f in followers]} mm, drift {drift * 1e3:.3f} mm; "
        f"force per pad {[round(f, 1) for f in force.values()]} N, puck {centre * 1e3:.2f} mm off the axis (moved {puck_drift * 1e3:.3f} mm), "
        f"penetration {[round(p * 1e3, 2) for p in penetration.values()]} mm; reopened to {q_open[driven] * 1e3:.2f} mm"
    )

    assert q[driven] < TARGET - STALL_MARGIN, f"did not stall on the puck: {q[driven] * 1e3:.2f} mm, target {TARGET * 1e3:.2f}"
    assert drift < REST_DRIFT, f"driven joint not at rest: moved {drift * 1e3:.3f} mm over the sampled frames"
    for name, dof in zip(FOLLOWERS, followers):
        assert abs(q[dof] - q[driven]) < FOLLOW_TOL, f"{name}={q[dof] * 1e3:.2f} mm does not follow {DRIVEN}={q[driven] * 1e3:.2f} mm"
    max_penetration, pushing_pads, extra_touching = TIP_LIMITS[tip]
    for p in PADS:
        assert present[p] >= CONTACT_PRESENT, f"{p} touches the puck in only {present[p]:.0%} of the sampled frames"
        assert penetration[p] <= max_penetration, f"{p} penetrates the puck by {penetration[p] * 1e3:.2f} mm"
        assert force[p] <= MAX_FORCE, f"{p} pushes with {force[p]:.1f} N, over {MAX_FORCE} N: {force}"
    pushing = [p for p in PADS if force[p] >= MIN_FORCE]
    assert len(pushing) >= pushing_pads, f"only {pushing} push with at least {MIN_FORCE} N, expected {pushing_pads} pads: {force}"
    if len(pushing) == len(PADS):
        for p in PADS:
            assert force[p] >= MIN_SHARE * mean_force, f"{p} carries only {force[p]:.1f} N of a mean {mean_force:.1f} N: {force}"
    drive_force = min(DRIVE_CAP, DRIVE_STIFFNESS * (TARGET - q[driven]))  # the driven joint's push, shared by three pads
    assert abs(mean_force - drive_force / 3) <= FORCE_TOL * drive_force / 3, (
        f"the pads push with a mean of {mean_force:.1f} N, expected a third of the drive's {drive_force:.0f} N "
        f"(stiffness x the {(TARGET - q[driven]) * 1e3:.2f} mm left, capped at {DRIVE_CAP:.0f} N) +/- {FORCE_TOL:.0%}: {force}"
    )
    assert centre <= CENTRE_TOL, f"the puck is {centre * 1e3:.2f} mm off the gripper axis"
    assert puck_drift < PUCK_REST, f"the puck is not at rest: moved {puck_drift * 1e3:.3f} mm over the sampled frames"
    others -= set(extra_touching)
    assert not others, f"links other than the fingertips and the housing touch the puck: {sorted(others)}"
    assert not after, f"a fingertip still touches the puck after reopening: {sorted(after)}"
    assert abs(q_open[driven]) < OPEN_TOL, f"not open after release: {q_open[driven] * 1e3:.2f} mm"
