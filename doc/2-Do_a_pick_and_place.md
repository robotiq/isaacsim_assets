# Pick an place

## Requirements

The following instructions suppose that you setup the Isaac Sim scene as
described in the `Setup scene` documentation.

## Pick and place from a Python script

This script picks a cube and puts it down somewhere else. It runs inside
Isaac Sim, from the **Script Editor**, and only uses what ships with
Isaac Sim 6.1:

- **Where to go.** Each waypoint is a position of the gripper (above the
  cube, on the cube, above the drop point...), always pointing down. The
  Robot Poser's inverse kinematics solver (`isaacsim.robot.poser`) turns it
  into joint angles.
- **How to get there.** The script ramps the arm's drive targets from one
  waypoint to the next, like a `MoveJ` on a real robot, then sets
  `finger_joint` to open or close the gripper.

It assumes the UR5e scene of §1.1 with the 2F-85 attached (§1.3), and the
cube of *Add a cube to manipulate* at `/World/Cube`.

The script finds the cube where it is, so you can move it before you run the
script. With the timeline **stopped**, open **Window → Script Editor**, paste
the script and click **Run**. It presses **Play** itself; the whole cycle
takes about 15 seconds.

```python
import asyncio

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.robot.poser import RobotPoser, Transform
from pxr import UsdGeom, UsdPhysics

ROBOT = "/World/ur5e"
GRIPPER = f"{ROBOT}/wrist_3_link/Robotiq_2F_85/Robotiq_2F_85"
CUBE = "/World/Cube"
PLACE = (0.45, -0.25)   # x, y where the cube is put down, in metres
TCP = 0.135             # wrist flange to the middle of the finger pads, in metres
HOVER = 0.15            # height above the grasp point for the approach, in metres
GRIP_OPEN, GRIP_CLOSED = 0.0, 45.0   # finger_joint drive target, in degrees

stage = omni.usd.get_context().get_stage()
arm_joints = [stage.GetPrimAtPath(f"{ROBOT}/joints/{name}") for name in (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
finger = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{GRIPPER}/Joints/finger_joint"), "angular")

# Inverse kinematics from the robot base to the wrist flange.
poser = RobotPoser(stage, stage.GetPrimAtPath(ROBOT),
                   stage.GetPrimAtPath(f"{ROBOT}/base_link"),
                   stage.GetPrimAtPath(f"{ROBOT}/wrist_3_link"))
home = np.radians([0, -90, 90, -90, -90, 0])
down = poser.chain.compute_fk(home)[0].q   # flange orientation with the gripper pointing down


def ik(x, y, z_tcp, seed):
    """Joint angles (radians) that put the finger pads at (x, y, z_tcp), gripper pointing down."""
    result = poser.solve_ik(Transform([x, y, z_tcp + TCP], down), seed=seed)
    if not result.success:
        raise RuntimeError(f"No IK solution for ({x:.3f}, {y:.3f}, {z_tcp:.3f})")
    return np.array([result.joints[j.prim_path] for j in poser.joints])


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


async def move_to(q_from, q_to, n=90):
    """Joint-space move (like MoveJ): ramp the arm drive targets over n frames."""
    for i in range(1, n + 1):
        q = q_from + (q_to - q_from) * i / n
        for joint, angle in zip(arm_joints, np.degrees(q)):
            UsdPhysics.DriveAPI.Get(joint, "angular").GetTargetPositionAttr().Set(float(angle))
        await frames(1)
    await frames(20)   # let the arm settle on the waypoint
    return q_to


async def grip(angle):
    finger.GetTargetPositionAttr().Set(angle)
    await frames(60)


async def pick_and_place():
    # Where is the cube? Use its centre and half its height.
    box = UsdGeom.BBoxCache(0, ["default"]).ComputeWorldBound(stage.GetPrimAtPath(CUBE)).ComputeAlignedRange()
    cx, cy, cz = box.GetMidpoint()
    px, py = PLACE

    omni.timeline.get_timeline_interface().play()
    await frames(30)
    q = home
    await grip(GRIP_OPEN)
    q = await move_to(q, ik(cx, cy, cz + HOVER, q))         # above the cube
    q = await move_to(q, ik(cx, cy, cz, q), n=60)           # down to the cube
    await grip(GRIP_CLOSED)                                 # grasp
    q = await move_to(q, ik(cx, cy, cz + HOVER, q), n=60)   # lift
    q = await move_to(q, ik(px, py, cz + HOVER, q))         # above the place point
    q = await move_to(q, ik(px, py, cz + 0.005, q), n=60)   # down, just above the table
    await grip(GRIP_OPEN)                                   # release
    q = await move_to(q, ik(px, py, cz + HOVER, q), n=60)   # retreat
    await move_to(q, home)                                  # back home
    print("Pick and place done")


asyncio.ensure_future(pick_and_place())
```

![Pick and place](../static/img/UR5e%20pick%20and%20place.png)

When it ends, the Console prints `Pick and place done` and the cube sits at
`PLACE`. Change `PLACE` to put it elsewhere, and `GRIP_CLOSED` (up to `47`)
to squeeze harder. The script works on the gripper's default
`Physx_parallel_grip` variant.

Press **Stop** to put the cube back where it started.

