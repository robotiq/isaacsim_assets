"""Flat plate driven along the gripper's Z for test_gripper_pusher.py: one preset per gripper, placed by hand.

A preset is ``pushers/<gripper folder>.json``, in the world frame of the test stage
(gripper at /World/Gripper, identity transform, fingers pointing along +Z)::

    {"size": [sx, sy, sz],                     # plate, m
     "start": [x, y, z],                       # plate centre before the push
     "displacement": d,                        # m, travelled toward the gripper base (-Z)
     "recorded": {<Physics variant>: {...}},   # what each variant measured
     "expected": {<grip>: {...}}}              # shared baseline, see the test

``add_pusher`` builds the plate as its own articulation (Newton's importer drops
non-fixed body-to-world joints outside an articulation, and a PhysX articulation only
attaches to the world with a fixed joint): a base body fixed to the world, and the
plate, a rigid box mesh with its size baked into the points (on Newton a moving body is
drawn without its xformOp:scale: a scaled unit box became a 1 m box around the camera
and the viewport went black), on a prismatic joint base -> plate along Z with a stiff position drive capped at
MAX_FORCE, so once the fingers stop the plate it presses on them with that force; the drive target is the plate's offset
from ``start`` (0 at start, -displacement at the end). A see-through marker without
physics shows the end position, see-through (a UsdPreviewSurface with opacity).

``save_pusher`` reads them back after someone shaped (the scale gizmo adds a scale op
on top of the baked points) / moved the plate and moved the end marker along Z in the
Isaac UI (``--save-cube``, see conftest.py); moving either
drops the baselines.
"""

from __future__ import annotations

import json
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

PUSHERS_DIR = Path(__file__).resolve().parent / "pushers"
RIG_PATH = "/World/PusherRig"
BASE_PATH = f"{RIG_PATH}/Base"
PUSHER_PATH = f"{RIG_PATH}/Pusher"
ANCHOR_PATH = f"{RIG_PATH}/Anchor"
JOINT_PATH = f"{RIG_PATH}/Slide"
END_PATH = "/World/PusherEnd"

MASS = 0.05  # kg: its weight (0.5 N) is negligible next to MAX_FORCE
DRIVE_STIFFNESS = 1.0e5  # N/m
DRIVE_DAMPING = 150.0  # N s/m, about critical for MASS
MAX_FORCE = 50.0  # N, the push once the fingers stop the plate
MARKER_OPACITY = 0.25


def preset_path(gripper: str) -> Path:
    return PUSHERS_DIR / f"{gripper}.json"


def load_preset(gripper: str) -> dict:
    p = preset_path(gripper)
    if not p.is_file():
        raise FileNotFoundError(f"no pusher preset {p}: place one with --gui --hold --save-cube -k {gripper}")
    return json.loads(p.read_text())


def write_preset(gripper: str, data: dict) -> Path:
    path = preset_path(gripper)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path


def _box_mesh(stage: Usd.Stage, path: str, size) -> UsdGeom.Mesh:
    """Box mesh of ``size`` (x, y, z), baked into the points."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    hx, hy, hz = (0.5 * float(v) for v in size)
    pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
           (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.GetFaceVertexCountsAttr().Set([4] * 6)
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
    mesh.GetExtentAttr().Set([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    return mesh


def end_position(preset: dict) -> list[float]:
    x, y, z = (float(v) for v in preset["start"])
    return [x, y, z - float(preset["displacement"])]


def add_pusher(stage: Usd.Stage, gripper: str) -> dict:
    preset = load_preset(gripper)
    size, start = [float(v) for v in preset["size"]], [float(v) for v in preset["start"]]

    rig = UsdGeom.Xform.Define(stage, RIG_PATH).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(rig)

    plate = _box_mesh(stage, PUSHER_PATH, size)
    UsdGeom.XformCommonAPI(plate.GetPrim()).SetTranslate(Gf.Vec3d(*start))
    prim = plate.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(MASS)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)
    plate.GetDisplayColorAttr().Set([Gf.Vec3f(0.75, 0.6, 0.2)])

    # Base body at the start position, fixed to the world (no collider).
    base = UsdGeom.Xform.Define(stage, BASE_PATH).GetPrim()
    UsdGeom.XformCommonAPI(base).SetTranslate(Gf.Vec3d(*start))
    UsdPhysics.RigidBodyAPI.Apply(base)
    UsdPhysics.MassAPI.Apply(base).CreateMassAttr().Set(MASS)
    anchor = UsdPhysics.FixedJoint.Define(stage, ANCHOR_PATH)
    anchor.CreateBody1Rel().SetTargets([BASE_PATH])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*start))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    # Prismatic joint base -> plate along Z (Newton needs the parent as body0).
    joint = UsdPhysics.PrismaticJoint.Define(stage, JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([BASE_PATH])
    joint.CreateBody1Rel().SetTargets([PUSHER_PATH])
    joint.CreateAxisAttr().Set(UsdPhysics.Tokens.z)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.linear)
    drive.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
    drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
    drive.CreateDampingAttr().Set(DRIVE_DAMPING)
    drive.CreateMaxForceAttr().Set(MAX_FORCE)
    drive.CreateTargetPositionAttr().Set(0.0)

    marker = _box_mesh(stage, END_PATH, size)
    UsdGeom.XformCommonAPI(marker.GetPrim()).SetTranslate(Gf.Vec3d(*end_position(preset)))
    marker.GetDisplayColorAttr().Set([Gf.Vec3f(0.75, 0.6, 0.2)])
    marker.GetDisplayOpacityAttr().Set([MARKER_OPACITY])
    material = UsdShade.Material.Define(stage, f"{END_PATH}_Looks/Glass")
    shader = UsdShade.Shader.Define(stage, f"{END_PATH}_Looks/Glass/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.75, 0.6, 0.2))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(MARKER_OPACITY)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(marker.GetPrim()).Bind(material)
    return preset


def save_pusher(stage: Usd.Stage, gripper: str) -> Path:
    """Write the plate's size and start position and the end marker's offset along Z
    to the gripper's preset. Rotations, and the marker's X / Y, are not kept."""
    plate, marker = stage.GetPrimAtPath(PUSHER_PATH), stage.GetPrimAtPath(END_PATH)
    if not plate or not marker:
        raise RuntimeError(f"no {PUSHER_PATH} / {END_PATH} on the stage")
    t = Gf.Transform(UsdGeom.Xformable(plate).GetLocalTransformation())
    m = Gf.Transform(UsdGeom.Xformable(marker).GetLocalTransformation())
    if abs(t.GetRotation().GetAngle()) > 1e-4:
        print("pusher preset: plate rotation ignored")
    start = [round(float(v), 6) for v in t.GetTranslation()]
    lo, hi = UsdGeom.Mesh(plate).GetExtentAttr().Get()  # the baked size, times any gizmo scale
    data = {
        "size": [round(float(b - a) * float(s), 6) for a, b, s in zip(lo, hi, t.GetScale())],
        "start": start,
        "displacement": round(start[2] - float(m.GetTranslation()[2]), 6),
    }
    old = load_preset(gripper) if preset_path(gripper).is_file() else {}
    if "expected" in old or "recorded" in old:
        if all(data[k] == old.get(k) for k in ("size", "start", "displacement")):
            data.update({k: old[k] for k in ("recorded", "expected") if k in old})
        else:
            print("pusher preset: plate moved: recorded baselines dropped, record them again with --record-baseline")
    return write_preset(gripper, data)
