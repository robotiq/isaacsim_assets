"""The 2F finger joint speed is capped at the same value on every variant.

The finger joint velocity cap (86 deg/s, the datasheet 150 mm/s) is authored once per gripper
(PhysX, ``physxJoint:maxJointVelocity``) and shared by the parallel-grip and
compliant variants. Command a full close in one step so the drive saturates,
sample the finger joint velocity every frame, and check the peak sits at the
cap: not above it, and close to it (so a missing or raised cap is caught too).
Run on both 2F grippers, both grip types and both engines, so the speeds
cannot drift apart between them. The Newton cases are expected to fail
(xfail): Newton's MuJoCo solver ignores joint velocity limits, so the finger
runs at what its drive allows (~255 deg/s) instead of the cap.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest

from _scene import engine_for, load_gripper, step

GRIPPERS = ["Robotiq_2F_85", "Robotiq_2F_140"]
PHYSICS = ["Physx_parallel_grip", "Physx_compliant", "Newton_parallel_grip", "Newton_compliant"]
NEWTON_NO_CAP = pytest.mark.xfail(
    reason="SolverMuJoCo ignores joint_velocity_limit (physxJoint:maxJointVelocity), so Newton is uncapped",
    strict=True,
)

MAX_SPEED = math.radians(86)  # rad/s, the cap authored in the physx_common layers (datasheet 150 mm/s, guide 4.5)
UPPER_TOL = 1.05  # peak may exceed the cap by at most 5 %
LOWER_TOL = 0.90  # and must reach at least 90 % of it

SETTLE_FRAMES = 30
MOVE_FRAMES = 60  # 1 s at 60 Hz; the full close takes ~0.55 s at the cap
CLOSE_TARGET = 0.8  # rad, ~46 deg


@pytest.mark.parametrize(
    "physics",
    [pytest.param(p, marks=NEWTON_NO_CAP) if engine_for(p) == "newton" else p for p in PHYSICS],
)
@pytest.mark.parametrize("gripper", GRIPPERS)
def test_finger_speed_is_capped(simulation_app, view, gripper: str, physics: str) -> None:
    scene = load_gripper(simulation_app, gripper, physics, view)
    robot = scene.robot
    step(simulation_app, SETTLE_FRAMES)

    finger = scene.dof("finger_joint")
    peak = 0.0

    def sample() -> None:
        nonlocal peak
        peak = max(peak, abs(float(robot.get_dof_velocities().numpy()[0][finger])))

    robot.set_dof_position_targets([CLOSE_TARGET], dof_indices=[finger])
    step(simulation_app, MOVE_FRAMES, 1, sample)
    closed = float(robot.get_dof_positions().numpy()[0][finger])
    omni.timeline.get_timeline_interface().stop()

    print(
        f"{gripper} {physics}: peak {math.degrees(peak):.1f} deg/s "
        f"(cap {math.degrees(MAX_SPEED):.2f}), closed to {math.degrees(closed):.1f} deg"
    )
    assert closed > CLOSE_TARGET * 0.9, f"did not close: finger_joint={math.degrees(closed):.2f} deg"
    assert np.isfinite(peak)
    assert peak <= MAX_SPEED * UPPER_TOL, (
        f"finger joint reached {math.degrees(peak):.1f} deg/s, above the {math.degrees(MAX_SPEED):.2f} deg/s cap"
    )
    assert peak >= MAX_SPEED * LOWER_TOL, (
        f"finger joint peaked at {math.degrees(peak):.1f} deg/s, below the {math.degrees(MAX_SPEED):.2f} deg/s cap: "
        "the cap is not what limits the motion"
    )
