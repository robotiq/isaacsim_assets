"""An upside-down gripper grasps a cube on a small fixed plate, then lifts it without slipping.

For every gripper and Physics variant: the gripper hangs fingers-down from a vertical
prismatic joint (_lift.py, base at the height that puts the cube between the pads, as in
the grasp test: same ``cubes/<gripper>.json`` size and gripper-relative position). The
cube, a free body, rests on the preset's small static plate. The gripper closes at
the datasheet finger speed toward full close, so the cube stops the fingers, and holds.
Then the slider lifts the gripper (and the cube) LIFT_DISTANCE up: constant acceleration
LIFT_ACCEL up to LIFT_SPEED, cruise, the same deceleration down to a stop, then a hold.
Check:

1. grasp: both fingertips touch the cube before the lift;
2. lift: the gripper follows the commanded motion within TRACK_TOL (so it really
   accelerates at LIFT_ACCEL), peaks near LIFT_SPEED, and travels LIFT_DISTANCE
   (never more, the joint limit is just beyond);
3. no slip: from the end of the grasp hold through the lift, the stop and the final
   hold, the cube never moves more than SLIP_TOL (translation) / SLIP_ROT_TOL
   (rotation) in the gripper's base frame, and it rose with the gripper;
4. both fingertips keep touching the cube, and no other link, during the lift.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import RigidPrim
from pxr import PhysxSchema, UsdPhysics

from _contacts import ContactReader
from _cube import CUBE_PATH, add_hanging_cube, load_preset
from _lift import LIFT_JOINT, duration, mount_lift, pose_change, profile, relative_pose
from _scene import FRAME_DT, STROKE100_SUFFIX, engine_for, load_gripper, move_to, step

PADS = ("left_fingertip", "right_fingertip")
SETTLE_FRAMES = 30
GRASP_HOLD_FRAMES = 10  # frames after the close ramp, before the lift
FINAL_HOLD_FRAMES = 60  # frames after the lift stops

LIFT_DISTANCE = 0.30  # m
LIFT_ACCEL = 5.0  # m/s^2
LIFT_SPEED = 0.5  # m/s

TRACK_TOL = 5.0e-3  # m, gripper vs its commanded lift (the position drive lags the command)
SPEED_TOL = 0.10  # peak speed within this fraction of LIFT_SPEED
DISTANCE_TOL = 1.0e-3  # m
SLIP_TOL = 2.0e-3  # m, cube vs gripper base frame
SLIP_ROT_TOL = math.radians(1.0)
CONTACT_PRESENT = 0.9  # fraction of lift frames a pad must be in contact
CUBE_POSITION_ITERATIONS = 64  # PhysX solver position iterations of the cube body (above the default)
CUBE_VELOCITY_ITERATIONS = 4  # PhysX solver velocity iterations of the cube body (above the default)
PHYSX_HZ = 1000  # as in test_gripper_grasp.py: the default rate mis-resolves the squeeze

# Driven joint and its full-close target, beyond the cube (as in test_gripper_grasp.py).
TARGET = {"Robotiq_2F_85": 0.8, "Robotiq_2F_140": 0.8, "Robotiq_Hand_E": 0.025}
# The Stroke100 variant closes 0.050 m per finger: the cube stops them at 40 mm, so the target must reach past that.
TARGET_STROKE100 = 0.050
DRIVEN = {"Robotiq_2F_85": "finger_joint", "Robotiq_2F_140": "finger_joint", "Robotiq_Hand_E": "left_finger_joint"}

CASES = [
    ("Robotiq_2F_85", "Physx_parallel_grip"),
    ("Robotiq_2F_85", "Physx_compliant"),
    ("Robotiq_2F_85", "Newton_parallel_grip"),
    ("Robotiq_2F_85", "Newton_compliant"),
    ("Robotiq_2F_140", "Physx_parallel_grip"),
    ("Robotiq_2F_140", "Physx_compliant"),
    ("Robotiq_2F_140", "Newton_parallel_grip"),
    ("Robotiq_2F_140", "Newton_compliant"),
    ("Robotiq_Hand_E", "PhysX"),
    ("Robotiq_Hand_E", "Newton"),
    ("Robotiq_Hand_E", "PhysX_Stroke100"),
    ("Robotiq_Hand_E", "Newton_Stroke100"),
]


def _pose(prim: RigidPrim) -> tuple[np.ndarray, np.ndarray]:
    # .numpy() can alias the live tensor buffers, which stop() resets: copy.
    pos, quat = prim.get_world_poses()
    return pos.numpy()[0].copy(), quat.numpy()[0].copy()


@pytest.mark.parametrize(("gripper", "physics"), CASES, ids=[f"{g}-{p}" for g, p in CASES])
def test_gripper_lifts_cube(simulation_app, view, gripper: str, physics: str) -> None:
    root = f"/World/Gripper/{gripper}"
    reader = ContactReader(gripper, root, CUBE_PATH, engine_for(physics))
    preset = load_preset(gripper)
    # The plate's top is the world's z = 0, so the cube's bottom sits there too.
    base_height = float(preset["position"][2]) + float(preset["size"]) / 2.0

    def setup(stage):
        add_hanging_cube(stage, gripper, base_height)
        cube_body = PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(CUBE_PATH))
        cube_body.CreateSolverPositionIterationCountAttr().Set(CUBE_POSITION_ITERATIONS)
        cube_body.CreateSolverVelocityIterationCountAttr().Set(CUBE_VELOCITY_ITERATIONS)
        mount_lift(stage, root, base_height, LIFT_DISTANCE)
        reader.prepare(stage)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, gripper, physics, view, setup=setup)
    robot = scene.robot
    assert LIFT_JOINT in robot.dof_names, f"the lift joint did not load: {robot.dof_names}"
    lift, driven = scene.dof(LIFT_JOINT), scene.dof(DRIVEN[gripper])
    base, cube = RigidPrim(f"{root}/base_link"), RigidPrim(CUBE_PATH)
    step(simulation_app, SETTLE_FRAMES)
    assert not reader.read(), f"gripper touches the cube while open: {sorted(reader.read())}"

    # Grasp: close toward full close, past the cube, and hold.
    target = TARGET_STROKE100 if physics.endswith(STROKE100_SUFFIX) else TARGET[gripper]
    move_to(simulation_app, scene, driven, target, settle_frames=GRASP_HOLD_FRAMES)
    grasped = reader.read()
    q_lift0 = float(robot.get_dof_positions().numpy()[0][lift])
    cube_z0 = _pose(cube)[0][2]
    ref = relative_pose(*_pose(base), *_pose(cube))  # the cube in the gripper's base frame

    # Lift, stop, hold; sample every frame.
    lift_frames = math.ceil(duration(LIFT_DISTANCE, LIFT_SPEED, LIFT_ACCEL) / FRAME_DT)
    rise, command, speed, slip, slip_rot, present, others = [], [], [], [], [], [], set()
    force = {p: [] for p in PADS}

    def sample(commanded: float, lifting: bool) -> None:
        q = float(robot.get_dof_positions().numpy()[0][lift]) - q_lift0
        rise.append(q)
        command.append(commanded)
        speed.append((q - rise[-2]) / FRAME_DT if len(rise) > 1 else 0.0)
        dt, dr = pose_change(ref, relative_pose(*_pose(base), *_pose(cube)))
        slip.append(dt)
        slip_rot.append(dr)
        contacts = reader.read()
        if lifting:
            present.append({p: p in contacts for p in PADS})
            others.update(k for k in contacts if k not in PADS)
            for p in PADS:
                force[p].append(contacts[p].force if p in contacts else 0.0)

    for i in range(1, lift_frames + 1):
        s, v = profile(i * FRAME_DT, LIFT_DISTANCE, LIFT_SPEED, LIFT_ACCEL)
        robot.set_dof_position_targets([[q_lift0 + s]], dof_indices=[lift])
        robot.set_dof_velocity_targets([[v]], dof_indices=[lift])
        simulation_app.update()
        sample(s, True)
    for _ in range(FINAL_HOLD_FRAMES):
        simulation_app.update()
        sample(LIFT_DISTANCE, False)
    cube_rise = _pose(cube)[0][2] - cube_z0
    omni.timeline.get_timeline_interface().stop()

    track = max(abs(r - c) for r, c in zip(rise, command))
    final = rise[-1]
    top = max(rise)
    peak_speed = max(speed)
    mean_force = {p: float(np.mean(force[p])) for p in PADS}
    in_contact = {p: sum(f[p] for f in present) / len(present) for p in PADS}
    print(
        f"{gripper} {physics}: lifted {final * 1e3:.2f} mm (peak {top * 1e3:.2f}), peak speed "
        f"{peak_speed:.3f} m/s, tracking error {track * 1e3:.2f} mm; cube rose {cube_rise * 1e3:.2f} mm; "
        f"slip max {max(slip) * 1e3:.3f} mm / {math.degrees(max(slip_rot)):.3f} deg "
        f"(at frame {int(np.argmax(slip)) + 1} of {len(slip)}); grip force L/R "
        f"{mean_force[PADS[0]]:.1f}/{mean_force[PADS[1]]:.1f} N, contact L/R "
        f"{in_contact[PADS[0]]:.0%}/{in_contact[PADS[1]]:.0%}, other links {sorted(others)}"
    )

    # 1. grasp
    assert all(p in grasped for p in PADS), f"fingertips touching the cube before the lift: {sorted(grasped)}"
    # 2. lift
    assert track <= TRACK_TOL, f"gripper off the commanded lift by {track * 1e3:.2f} mm"
    assert abs(peak_speed - LIFT_SPEED) <= SPEED_TOL * LIFT_SPEED, (
        f"peak lift speed {peak_speed:.3f} m/s, commanded {LIFT_SPEED} m/s"
    )
    assert top <= LIFT_DISTANCE + DISTANCE_TOL, f"lifted {top * 1e3:.2f} mm, more than {LIFT_DISTANCE * 1e3:.0f} mm"
    assert abs(final - LIFT_DISTANCE) <= DISTANCE_TOL, (
        f"lifted {final * 1e3:.2f} mm, expected {LIFT_DISTANCE * 1e3:.0f} mm"
    )
    # 3. no slip
    assert max(slip) <= SLIP_TOL, (
        f"cube slipped {max(slip) * 1e3:.3f} mm in the gripper (frame {int(np.argmax(slip)) + 1})"
    )
    assert max(slip_rot) <= SLIP_ROT_TOL, f"cube rotated {math.degrees(max(slip_rot)):.2f} deg in the gripper"
    assert cube_rise >= final - SLIP_TOL - TRACK_TOL, (
        f"cube rose {cube_rise * 1e3:.2f} mm while the gripper rose {final * 1e3:.2f} mm"
    )
    # 4. contact during the lift
    for p in PADS:
        assert in_contact[p] >= CONTACT_PRESENT, f"{p} touched the cube in only {in_contact[p]:.0%} of the lift frames"
    assert not others, f"links other than the fingertips touch the cube: {sorted(others)}"
