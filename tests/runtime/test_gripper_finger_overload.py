"""An outside load on an open 2F finger must not fold the four-bar past its stops.

For both 2F grippers on Physx_compliant: the gripper is held open (finger_joint
target 0). A force is applied straight on the left inner finger body, along its
local +Y (the closing direction), for PUSH_FRAMES frames, then released. The
passive joints (inner knuckle, inner finger knuckle) used to be limited to +-180
deg, so the load folded the linkage over and it stayed broken (#128). Check:

While pushed:

1. the inner knuckle stays within MAX_KNUCKLE of open (it is limited to ~52 deg;
   folded over it reached 180 deg).

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

MAX_KNUCKLE = math.radians(60.0)  # limits are ~52 deg; the broken linkage reached 180
RESTORE_TOL = math.radians(1.0)
REST_DRIFT = math.radians(0.1)

CASES = [(g, f) for g in ("Robotiq_2F_85", "Robotiq_2F_140") for f in (100.0, 200.0)]


@pytest.mark.parametrize(("gripper", "force"), CASES, ids=[f"{g}-{int(f)}N" for g, f in CASES])
def test_gripper_finger_overload(simulation_app, view, gripper: str, force: float) -> None:
    scene = load_gripper(simulation_app, gripper, "Physx_compliant", view)
    robot = scene.robot
    step(simulation_app, SETTLE_FRAMES)
    dofs = [scene.dof(j) for j in JOINTS]
    knuckles = [JOINTS.index(j) for j in KNUCKLES]
    finger = RigidPrim(f"{scene.root}/left_inner_finger")

    def joints():
        return robot.get_dof_positions().numpy()[0][dofs].copy()

    q_open = joints()
    peak = 0.0  # largest |inner knuckle| while pushed
    for _ in range(PUSH_FRAMES):
        finger.apply_forces([[0.0, force, 0.0]], local_frame=True)
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
        f"{gripper} {force:.0f} N: inner knuckle peak {math.degrees(peak):.1f} deg while pushed; after release "
        + ", ".join(f"{j}={math.degrees(float(v - o)):+.2f}" for j, v, o in zip(JOINTS, q, q_open))
        + " deg from open"
    )

    # 1. the linkage does not fold over
    assert peak <= MAX_KNUCKLE, f"inner knuckle reached {math.degrees(peak):.1f} deg while pushed: folded past its stop"
    # 2. back to the original configuration, at rest
    for j, v, o, sp in zip(JOINTS, q, q_open, spread):
        assert abs(v - o) <= RESTORE_TOL, f"{j} {math.degrees(float(v - o)):+.1f} deg from open after the release"
        assert sp < REST_DRIFT, f"{j} still moving after the release: {math.degrees(sp):.3f} deg"
