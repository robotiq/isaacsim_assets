"""Cylinder fixed in space for test_gripper_fixed_cylinder.py: one preset per gripper, placed by hand.

A preset is ``fixed_cylinders/<gripper folder>.json``, in the world frame of the test
stage (gripper at /World/Gripper, identity transform)::

    {"size": [sx, sy, sz],                    # diameter along X / Y, height along Z (local), m
     "position": [x, y, z],                   # centre
     "expected": {<Physics variant>: {...}}}  # baselines, see the test

``add_fixed_cylinder`` builds a static collider (no rigid body, no holder): a cylinder
mesh of unit diameter and height, along its local Z, scaled by ``xformOp:scale``
(Newton applies scale to mesh colliders, not to primitive ones), then turned by the
fixed ``ORIENTATION`` (same for every gripper, never saved): its axis along the
gripper's X. Moved and stretched in the Isaac UI, it is read back by
``save_cylinder`` (``--save-cube``, see conftest.py); moving it drops the baselines.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

CYLINDERS_DIR = Path(__file__).resolve().parent / "fixed_cylinders"
CYLINDER_PATH = "/World/Cylinder"
ORIENTATION = (0.7071068, 0.0, 0.7071068, 0.0)  # (w, x, y, z): 90 deg about Y, axis along X
SEGMENTS = 24  # 48 hull vertices, under PhysX's 64-vertex convex hull limit


def preset_path(gripper: str) -> Path:
    return CYLINDERS_DIR / f"{gripper}.json"


def load_preset(gripper: str) -> dict:
    p = preset_path(gripper)
    if not p.is_file():
        raise FileNotFoundError(f"no cylinder preset {p}: place one with --gui --hold --save-cube -k {gripper}")
    return json.loads(p.read_text())


def write_preset(gripper: str, data: dict) -> Path:
    path = preset_path(gripper)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path


def _unit_cylinder_mesh(stage: Usd.Stage, path: str) -> UsdGeom.Mesh:
    mesh = UsdGeom.Mesh.Define(stage, path)
    ring = [(0.5 * math.cos(2 * math.pi * i / SEGMENTS), 0.5 * math.sin(2 * math.pi * i / SEGMENTS))
            for i in range(SEGMENTS)]
    pts = [(x, y, -0.5) for x, y in ring] + [(x, y, 0.5) for x, y in ring]
    n = SEGMENTS
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]  # bottom (facing -Z), top
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.GetFaceVertexCountsAttr().Set([len(f) for f in faces])
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
    mesh.GetExtentAttr().Set([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    return mesh


def add_fixed_cylinder(stage: Usd.Stage, gripper: str) -> UsdGeom.Mesh:
    preset = load_preset(gripper)
    mesh = _unit_cylinder_mesh(stage, CYLINDER_PATH)
    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*preset["position"]))
    xf.AddOrientOp().Set(Gf.Quatf(*ORIENTATION))
    xf.AddScaleOp().Set(Gf.Vec3f(*preset["size"]))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())  # collider only: static
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
    mesh.GetDisplayColorAttr().Set([Gf.Vec3f(0.25, 0.65, 0.35)])
    return mesh


def save_cylinder(stage: Usd.Stage, gripper: str) -> Path:
    """Write the cylinder's size and position to the gripper's preset, whatever
    xformOps the Isaac gizmos authored. Its orientation is fixed: a rotation made in
    the UI is not kept."""
    prim = stage.GetPrimAtPath(CYLINDER_PATH)
    if not prim:
        raise RuntimeError(f"no {CYLINDER_PATH} on the stage")
    t = Gf.Transform(UsdGeom.Xformable(prim).GetLocalTransformation())
    q = t.GetRotation().GetQuat()
    if abs(sum(a * b for a, b in zip((q.GetReal(), *q.GetImaginary()), ORIENTATION))) < 1.0 - 1e-6:
        print("cylinder preset: rotation ignored (the cylinder's orientation is fixed, see _cylinder.ORIENTATION)")
    data = {
        "size": [round(float(v), 6) for v in t.GetScale()],
        "position": [round(float(v), 6) for v in t.GetTranslation()],
    }
    old = load_preset(gripper) if preset_path(gripper).is_file() else {}
    if "expected" in old:
        if all(data[k] == old.get(k) for k in ("size", "position")):
            data["expected"] = old["expected"]
        else:
            print("cylinder preset: cylinder moved: recorded baselines dropped, record them again with --record-baseline")
    return write_preset(gripper, data)
