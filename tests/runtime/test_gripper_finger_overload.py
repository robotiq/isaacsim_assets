"""An outside load on an open 2F finger must not break the four-bar.

For both 2F grippers on Physx_compliant: the gripper is held open (finger_joint
target 0). A force is applied straight on the left inner finger body, along its
local Y, for PUSH_FRAMES frames, then released: +Y is the closing direction (#128:
without limits on the passive joints (inner knuckle, inner finger knuckle) the
linkage folds past its stops and tears and the gripper stays broken), -Y the opening
direction (#127: the linkage flips into its second, crossed configuration). Check:

While pushed:

1. the inner knuckle stays within MAX_KNUCKLE of open (its limit, ~52 deg).

After the release:

2. the gripper is back in its original configuration: finger_joint, both inner
   knuckles and both inner finger knuckles within RESTORE_TOL of where they
   started, at rest.
"""

from __future__ import annotations

import math

import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim

from _scene import load_gripper, step

SETTLE_FRAMES = 30
PUSH_FRAMES = 60  # 1 s at 60 Hz
RELEASE_FRAMES = 120
SAMPLE_FRAMES = 20  # the last frames after the release are checked for rest

JOINTS = (
    "finger_joint",
    "right_outer_knuckle_joint",
    "left_inner_knuckle_joint",
    "right_inner_knuckle_joint",
    "left_inner_finger_knuckle_joint",
    "right_inner_finger_knuckle_joint",
)
KNUCKLES = ("left_inner_knuckle_joint", "right_inner_knuckle_joint")

MAX_KNUCKLE = math.radians(60.0)  # the inner knuckle limits are ~52 deg
RESTORE_TOL = math.radians(1.0)
REST_DRIFT = math.radians(0.1)

# (direction name, sign of the push along the finger's local Y, forces in N)
PUSHES = (("close", 1.0, (100.0, 200.0)), ("open", -1.0, (30.0, 100.0, 300.0)))
# Still broken with the CAD limits of #165 (#127): at 100 N the 2F-85 is pushed to 131 deg and
# ends 306 deg off, and the 2F-140 blows up at every -Y force (inner knuckle angles in the
# millions of degrees). Strict, so a fix turns them into failures to un-mark.
KNOWN_BROKEN = {
    ("Robotiq_2F_85", "open", 100.0),
    ("Robotiq_2F_140", "open", 30.0),
    ("Robotiq_2F_140", "open", 100.0),
    ("Robotiq_2F_140", "open", 300.0),
}
CASES = [
    pytest.param(
        g, d, sign, f,
        id=f"{g}-{d}-{int(f)}N",
        marks=pytest.mark.xfail(strict=True, reason="#127: still breaks with the #165 limits")
        if (g, d, f) in KNOWN_BROKEN else (),
    )
    for g in ("Robotiq_2F_85", "Robotiq_2F_140")
    for d, sign, forces in PUSHES
    for f in forces
]


@pytest.mark.parametrize(("gripper", "direction", "sign", "force"), CASES)
def test_gripper_finger_overload(
    simulation_app, view, gripper: str, direction: str, sign: float, force: float
) -> None:
    scene = load_gripper(simulation_app, gripper, "Physx_compliant", view)
    robot = scene.robot
    robot.set_dof_position_targets([[0.0]], dof_indices=[scene.dof("finger_joint")])
    step(simulation_app, SETTLE_FRAMES)
    dofs = [scene.dof(j) for j in JOINTS]
    knuckles = [JOINTS.index(j) for j in KNUCKLES]
    finger = RigidPrim(f"{scene.root}/left_inner_finger")

    def joints():
        return robot.get_dof_positions().numpy()[0][dofs].copy()

    q_open = joints()
    peak = 0.0  # largest |inner knuckle| while pushed
    for _ in range(PUSH_FRAMES):
        finger.apply_forces([[0.0, sign * force, 0.0]], local_frame=True)
        simulation_app.update()
        peak = max(peak, max(abs(float(joints()[i])) for i in knuckles))
    step(simulation_app, RELEASE_FRAMES - SAMPLE_FRAMES)
    samples = []
    for _ in range(SAMPLE_FRAMES):
        simulation_app.update()
        samples.append(joints())
    omni.timeline.get_timeline_interface().stop()

    q = samples[-1]
    spread = [max(s[i] for s in samples) - min(s[i] for s in samples) for i in range(len(JOINTS))]
    print(
        f"{gripper} {direction} {force:.0f} N: inner knuckle peak {math.degrees(peak):.1f} deg while pushed; after release "
        + ", ".join(f"{j}={math.degrees(float(v - o)):+.2f}" for j, v, o in zip(JOINTS, q, q_open))
        + " deg from open"
    )

    # 1. the linkage does not fold over
    assert peak <= MAX_KNUCKLE, f"inner knuckle reached {math.degrees(peak):.1f} deg while pushed: past its stop"
    # 2. back to the original configuration, at rest
    for j, v, o, sp in zip(JOINTS, q, q_open, spread):
        assert abs(v - o) <= RESTORE_TOL, f"{j} {math.degrees(float(v - o)):+.1f} deg from open after the release"
        assert sp < REST_DRIFT, f"{j} still moving after the release: {math.degrees(sp):.3f} deg"
