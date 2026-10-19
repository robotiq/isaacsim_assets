"""A flat plate driven along the gripper's Z pushes on the open 2F fingers.

For both 2F grippers on all four Physics variants: the gripper is held open. A plate on
a prismatic joint (placed, shaped and given its travel by hand per gripper in
pushers/, see _pusher.py) starts beyond the fingertips and is driven toward the base
at PUSH_SPEED by its displacement, pressing with at most 50 N (_pusher.MAX_FORCE),
holds, then is driven back up to its start at the same speed and holds again. Check:

0. the plate follows its ramp (within TRACK_TOL) while it touches nothing, going
   down and coming back up;

During the low hold:

1. nothing is still moving (knuckle joints and plate);
2. parallel grip, the fingers stay rigid: they stop the plate short of its end (by
   more than PUSHER_TOL), each pad rotates less than RIGID_PAD_TOL from open and both
   knuckle joints stay within RIGID_JOINT_TOL of open;
3. compliant, the fingers bend: the plate reaches its end (within PUSHER_TOL), each
   pad rotates more than MIN_BEND, and the pad rotations and knuckle joints match the
   baseline PhysX and Newton share (the mean of their recordings) within COMPLIANT_TOL.

After the plate went back up:

4. the plate is back at its start (within PUSHER_TOL) and the gripper is back in its
   original configuration: both knuckle joints within RESTORE_TOL of open, each pad
   within RESTORE_TOL of its open orientation, at rest.

Record the compliant baselines with ``--record-baseline`` (run both engines); moving
the plate or its end marker with ``--save-cube`` drops them.
"""

from __future__ import annotations

import math

import numpy as np
import omni.timeline
import pytest
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from pxr import PhysxSchema, UsdPhysics

from _contacts import ContactReader
from _pusher import PUSHER_PATH, RIG_PATH, add_pusher, end_position, load_preset, write_preset
from _scene import FRAME_DT, engine_for, load_gripper, step

PADS = ("left_fingertip", "right_fingertip")
JOINTS = ("finger_joint", "right_outer_knuckle_joint")  # left, right
SETTLE_FRAMES = 30
HOLD_FRAMES = 60  # 1 s at 60 Hz, back at the start
LOW_HOLD_FRAMES = 35  # ~0.6 s at the low position
SAMPLE_FRAMES = 20  # the last frames of the hold are sampled
PUSH_SPEED = 0.05  # m/s

PUSHER_TOL = 1.0e-3  # m
TRACK_TOL = 1.0e-3  # m, plate vs its commanded ramp while it touches nothing
NO_CONTACT = 0.5  # N, below this total contact force the plate touches nothing
RESTORE_TOL = math.radians(0.5)
REST_DRIFT = math.radians(0.1)  # knuckle joints, over the sampled frames
PUSHER_REST_DRIFT = 0.1e-3  # m
RIGID_PAD_TOL = math.radians(1.0)
RIGID_JOINT_TOL = math.radians(1.0)
MIN_BEND = math.radians(2.0)
COMPLIANT_TOL = (math.radians(2.0), math.radians(1.5))  # (joint, pad) around the shared baseline
PHYSX_HZ = 1000  # as in test_gripper_grasp.py: 60 Hz mis-resolves stiff contacts

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
def test_gripper_pusher(request, simulation_app, view, gripper: str, physics: str) -> None:
    grip = grip_of(physics)
    preset = {}
    reader = ContactReader(gripper, f"/World/Gripper/{gripper}", PUSHER_PATH, engine_for(physics))

    def setup(stage):
        preset.update(add_pusher(stage, gripper))
        reader.prepare(stage)
        if engine_for(physics) == "physx":
            scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
            PhysxSchema.PhysxSceneAPI.Apply(scene_prim).CreateTimeStepsPerSecondAttr().Set(PHYSX_HZ)

    scene = load_gripper(simulation_app, gripper, physics, view, setup=setup)
    robot = scene.robot
    pusher = Articulation(RIG_PATH)
    plate = RigidPrim(PUSHER_PATH)
    pads = RigidPrim([f"{scene.root}/{p}" for p in PADS])
    step(simulation_app, SETTLE_FRAMES)
    # Isaac Sim 6.1+ lists loop-closure joints as extra DOFs; drive only the slide.
    slide = [i for i, name in enumerate(pusher.dof_names) if "loop_closure" not in name]
    assert len(slide) == 1, f"the plate's prismatic joint did not load: {pusher.dof_names}"
    dofs = [scene.dof(j) for j in JOINTS]
    q_open = robot.get_dof_positions().numpy()[0][dofs].copy()
    pads_open = pads.get_world_poses()[1].numpy().copy()

    start_z = float(plate.get_world_poses()[0].numpy()[0][2])
    d = float(preset["displacement"])
    frames = max(1, math.ceil(d / (PUSH_SPEED * FRAME_DT)))
    track_err = [0.0, 0.0]  # max |plate - command| while touching nothing: down, up

    def ramp(a: float, b: float, leg: int) -> None:
        """Ramp the plate's target from offset a to offset b over ``frames`` frames."""
        for i in range(1, frames + 1):
            target = a + (b - a) * i / frames
            pusher.set_dof_position_targets([[target]], dof_indices=slide)
            simulation_app.update()
            if sum(c.force for c in reader.read().values()) < NO_CONTACT:
                z = float(plate.get_world_poses()[0].numpy()[0][2])
                track_err[leg] = max(track_err[leg], abs(z - (start_z + target)))

    def sample(hold: int) -> tuple[np.ndarray, list[float], list[float], np.ndarray]:
        """Hold ``hold`` frames, sampling the last ones: (knuckle joints, plate z, contact force, pad quats)."""
        step(simulation_app, hold - SAMPLE_FRAMES)
        joint_q, plate_z, push = [], [], []
        for _ in range(SAMPLE_FRAMES):
            simulation_app.update()
            push.append(sum(c.force for c in reader.read().values()))
            # .numpy() can alias the live tensor buffers, which stop() resets: copy.
            joint_q.append(robot.get_dof_positions().numpy()[0][dofs].copy())
            plate_z.append(float(plate.get_world_poses()[0].numpy()[0][2]))
        return np.array(joint_q), plate_z, push, pads.get_world_poses()[1].numpy().copy()

    # Push down, hold low; then back up, hold at the start.
    ramp(0.0, -d, 0)
    joint_q, plate_z, push, pads_q = sample(LOW_HOLD_FRAMES)
    ramp(-d, 0.0, 1)
    back_q, back_z, _, back_pads = sample(HOLD_FRAMES)
    omni.timeline.get_timeline_interface().stop()

    q = joint_q[-1]
    drift = joint_q.max(axis=0) - joint_q.min(axis=0)
    end_z = end_position(preset)[2]
    plate_err = plate_z[-1] - end_z
    rot = [_angle_between(pads_open[i], pads_q[i]) for i in range(2)]
    measured = {f"{j}_deg": round(math.degrees(float(v)), 3) for j, v in zip(JOINTS, q)}
    measured |= {f"{p}_rotation_deg": round(math.degrees(r), 3) for p, r in zip(PADS, rot)}
    print(
        f"{gripper} {physics}: plate {plate_err * 1e3:+.2f} mm from its end (drift "
        f"{(max(plate_z) - min(plate_z)) * 1e3:.3f} mm), contact force on the gripper {np.mean(push):.1f} N; "
        + ", ".join(f"{j}={math.degrees(v):.2f} deg (open {math.degrees(o):.2f}, drift {math.degrees(dr):.3f})"
                    for j, v, o, dr in zip(JOINTS, q, q_open, drift))
        + f"; pad rotation L/R {math.degrees(rot[0]):.2f}/{math.degrees(rot[1]):.2f} deg"
    )
    back_rot = [_angle_between(pads_open[i], back_pads[i]) for i in range(2)]
    back_err = back_z[-1] - start_z
    back_drift = back_q.max(axis=0) - back_q.min(axis=0)
    print(
        f"  ramp tracking down/up {track_err[0] * 1e3:.3f}/{track_err[1] * 1e3:.3f} mm; back up: plate "
        f"{back_err * 1e3:+.2f} mm from its start, "
        + ", ".join(f"{j} {math.degrees(v - o):+.2f} deg from open" for j, v, o in zip(JOINTS, back_q[-1], q_open))
        + f", pad rotation L/R {math.degrees(back_rot[0]):.2f}/{math.degrees(back_rot[1]):.2f} deg"
    )

    if request.config.getoption("--record-baseline") and grip == "compliant":
        stored = load_preset(gripper)
        recorded = stored.setdefault("recorded", {})
        recorded[physics] = measured
        runs = [v for k, v in recorded.items() if grip_of(k) == grip]
        stored.setdefault("expected", {})[grip] = {
            k: round(sum(r[k] for r in runs) / len(runs), 3) for k in measured
        }
        print(f"baseline recorded ({grip}, mean of {len(runs)}): {write_preset(gripper, stored)}")
        return

    # 0. the plate follows its ramp
    for leg, err in zip(("down", "up"), track_err):
        assert err <= TRACK_TOL, f"plate off its ramp by {err * 1e3:.2f} mm going {leg}"
    # 1. at rest
    assert max(plate_z) - min(plate_z) < PUSHER_REST_DRIFT, "the plate is still moving"
    for j, dr in zip(JOINTS, drift):
        assert dr < REST_DRIFT, f"{j} still moving: {math.degrees(dr):.3f} deg over the sampled frames"
    if grip == "parallel_grip":
        # 2. rigid: the fingers stop the plate and do not give way
        assert plate_err > PUSHER_TOL, (
            f"plate reached {plate_err * 1e3:+.2f} mm from its end: the fingers did not stop it"
        )
        for p, r in zip(PADS, rot):
            assert r < RIGID_PAD_TOL, f"{p} rotated {math.degrees(r):.2f} deg: the parallel-grip finger bent"
        for j, v, o in zip(JOINTS, q, q_open):
            assert abs(v - o) < RIGID_JOINT_TOL, (
                f"{j} moved {math.degrees(v - o):+.2f} deg from open: the parallel-grip finger gave way"
            )
    else:
        # 3. bends, as the shared baseline says
        assert abs(plate_err) <= PUSHER_TOL, f"plate stopped {plate_err * 1e3:+.2f} mm from its end position"
        for p, r in zip(PADS, rot):
            assert r > MIN_BEND, f"{p} rotated only {math.degrees(r):.2f} deg: the compliant finger did not bend"
        expected = load_preset(gripper).get("expected", {}).get(grip)
        assert expected, f"no {grip} baseline in pushers/{gripper}.json: run with --record-baseline"
        joint_tol, pad_tol = COMPLIANT_TOL
        for k, tol in [(f"{j}_deg", joint_tol) for j in JOINTS] + [(f"{p}_rotation_deg", pad_tol) for p in PADS]:
            assert abs(measured[k] - expected[k]) <= math.degrees(tol), (
                f"{k} = {measured[k]:.2f}, shared baseline {expected[k]:.2f}"
            )
    # 4. back to the original configuration once the plate went back up
    assert abs(back_err) <= PUSHER_TOL, f"plate back {back_err * 1e3:+.2f} mm from its start"
    assert max(back_z) - min(back_z) < PUSHER_REST_DRIFT, "the plate is still moving after going back up"
    for j, v, o, dr in zip(JOINTS, back_q[-1], q_open, back_drift):
        assert abs(v - o) <= RESTORE_TOL, f"{j} {math.degrees(v - o):+.2f} deg from open after the plate went back up"
        assert dr < REST_DRIFT, f"{j} still moving after the plate went back up: {math.degrees(dr):.3f} deg"
    for p, r in zip(PADS, back_rot):
        assert r <= RESTORE_TOL, f"{p} still rotated {math.degrees(r):.2f} deg after the plate went back up"
