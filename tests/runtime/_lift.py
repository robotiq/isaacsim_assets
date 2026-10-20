"""Upside-down gripper on a vertical prismatic joint, for test_gripper_lift.py.

``mount_lift`` flips the gripper (rotated 180 deg about X, fingers pointing down, base at
``base_height``) and replaces its world-pinned ``root_joint`` with the same thing plus a
slider: the joint-less gripper is attached to a ``LiftBase`` body, fixed to the world,
through a prismatic joint ``lift_joint`` along world +Z, driven by position + velocity
targets. All three prims are children of the articulation root, where ``root_joint`` was,
so the gripper stays one articulation (the extra DOF is ``lift_joint``).

``profile`` is the lift's motion law: constant acceleration up to a peak speed, cruise,
then the same deceleration, over a total distance.
"""

from __future__ import annotations

import math

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

GRIPPER_PATH = "/World/Gripper"
LIFT_JOINT = "lift_joint"

BASE_MASS = 0.05  # kg
DRIVE_STIFFNESS = 1.0e5  # N/m: stiff enough that the gripper's weight sags it negligibly
DRIVE_DAMPING = 2.0e3  # N s/m, on the velocity target
MAX_FORCE = 500.0  # N, well above what the load needs at the lift acceleration
LIMIT_MARGIN = 0.01  # m, joint limits beyond the commanded travel

_FLIP = Gf.Quatf(0.0, 1.0, 0.0, 0.0)  # 180 deg about X


def mount_lift(stage: Usd.Stage, root: str, base_height: float, travel: float) -> None:
    """Flip the gripper at ``GRIPPER_PATH`` and hang it from a vertical slider; ``root``
    is its articulation root prim. Joint limits are [-margin, travel + margin]."""
    gripper = UsdGeom.Xformable(stage.GetPrimAtPath(GRIPPER_PATH))
    gripper.ClearXformOpOrder()
    gripper.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, base_height))
    gripper.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(_FLIP)

    stage.GetPrimAtPath(f"{root}/root_joint").SetActive(False)

    base_path = f"{root}/LiftBase"
    base = UsdGeom.Xform.Define(stage, base_path).GetPrim()  # identity: coincides with base_link
    UsdPhysics.RigidBodyAPI.Apply(base)
    UsdPhysics.MassAPI.Apply(base).CreateMassAttr().Set(BASE_MASS)

    anchor = UsdPhysics.FixedJoint.Define(stage, f"{root}/lift_anchor")
    anchor.CreateBody1Rel().SetTargets([base_path])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, base_height))  # world frame
    anchor.CreateLocalRot0Attr().Set(_FLIP)
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    # Joint frames turned by the flip on both bodies: the axis is world +Z.
    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/{LIFT_JOINT}")
    joint.CreateBody0Rel().SetTargets([base_path])  # Newton needs the parent as body0
    joint.CreateBody1Rel().SetTargets([f"{root}/base_link"])
    joint.CreateAxisAttr().Set(UsdPhysics.Tokens.z)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot0Attr().Set(_FLIP)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(_FLIP)
    joint.CreateLowerLimitAttr().Set(-LIMIT_MARGIN)
    joint.CreateUpperLimitAttr().Set(travel + LIMIT_MARGIN)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.linear)
    drive.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
    drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
    drive.CreateDampingAttr().Set(DRIVE_DAMPING)
    drive.CreateMaxForceAttr().Set(MAX_FORCE)
    drive.CreateTargetPositionAttr().Set(0.0)


def duration(distance: float, speed: float, accel: float) -> float:
    ramp = speed / accel
    return 2.0 * ramp + max(0.0, distance - speed * ramp) / speed


def profile(t: float, distance: float, speed: float, accel: float) -> tuple[float, float]:
    """(position, velocity) at time ``t`` of a lift of ``distance`` that accelerates at
    ``accel`` up to ``speed``, cruises, decelerates at ``accel`` and stops."""
    ramp = speed / accel
    ramp_d = 0.5 * accel * ramp**2
    cruise = max(0.0, distance - 2.0 * ramp_d) / speed
    if t <= 0.0:
        return 0.0, 0.0
    if t <= ramp:
        return 0.5 * accel * t * t, accel * t
    if t <= ramp + cruise:
        return ramp_d + speed * (t - ramp), speed
    tau = t - ramp - cruise
    if tau < ramp:
        return ramp_d + speed * cruise + speed * tau - 0.5 * accel * tau * tau, speed - accel * tau
    return distance, 0.0


# -- poses (quaternions are w, x, y, z) ----------------------------------------------
def _rotation(quat) -> Gf.Rotation:
    return Gf.Rotation(Gf.Quatd(*(float(x) for x in quat)))


def relative_pose(frame_pos, frame_quat, pos, quat) -> tuple[np.ndarray, np.ndarray]:
    """Pose of (``pos``, ``quat``) seen from the frame (``frame_pos``, ``frame_quat``)."""
    inv = _rotation(frame_quat).GetInverse()
    p = inv.TransformDir(Gf.Vec3d(*(float(a - b) for a, b in zip(pos, frame_pos))))
    q = (_rotation(quat) * inv).GetQuat()  # Gf applies the left rotation first
    return np.array(p), np.array([q.GetReal(), *q.GetImaginary()])


def pose_change(ref: tuple[np.ndarray, np.ndarray], cur: tuple[np.ndarray, np.ndarray]) -> tuple[float, float]:
    """(translation m, rotation rad) between two relative poses."""
    degrees = (_rotation(ref[1]).GetInverse() * _rotation(cur[1])).GetAngle() % 360.0
    return float(np.linalg.norm(cur[0] - ref[0])), math.radians(min(degrees, 360.0 - degrees))
