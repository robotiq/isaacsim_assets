"""Cube to grasp, resting free on a fixed holder: one preset per gripper, placed by hand.

A preset is ``cubes/<gripper folder>.json``, in the world frame of the test stage
(gripper at /World/Gripper, identity transform)::

    {"size": <cube edge, m>, "position": [x, y, z],          # cube centre
     "mass": <kg, optional>,
     "holder": {"size": [sx, sy, sz], "position": [x, y]}}     # optional

``add_cube`` builds a free dynamic cube (no joint) resting on a static holder:
a box collider with no rigid body, so it does not move, sized smaller than the
cube so the fingers reach the cube's sides. The holder's height is never stored:
its top is always placed exactly at the cube's bottom, so the two can't start
overlapping (PhysX would kick the cube apart at play). Its size and its X / Y
position come from the preset; without a "holder" entry it is a thin plate centred
under the cube.

``save_cube`` reads both back from the stage after someone moved / scaled them in
the Isaac UI (``--save-cube``, see conftest.py).

The cube's edge length is baked into ``UsdGeom.Cube.size`` with an identity scale
(Newton ignores ``xformOp:scale`` on primitive colliders). The holder is a box
*mesh* of unit size scaled by ``xformOp:scale``: Newton does apply scale to mesh
colliders, so the holder can be sized with the scale gizmo on both engines.
"""

from __future__ import annotations

import json
from pathlib import Path

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, Vt

CUBES_DIR = Path(__file__).resolve().parent / "cubes"
CUBE_PATH = "/World/Cube"
HOLDER_PATH = "/World/CubeHolder"

DEFAULT_MASS = 1.0  # kg: much lighter makes PhysX mis-solve the squeeze (lopsided, links catch it)
HOLDER_FRACTION = 0.5  # default holder footprint, as a fraction of the cube edge
HOLDER_THICKNESS = 0.005  # m, default


def preset_path(gripper: str) -> Path:
    return CUBES_DIR / f"{gripper}.json"


def load_preset(gripper: str) -> dict:
    p = preset_path(gripper)
    if not p.is_file():
        raise FileNotFoundError(f"no cube preset {p}: place one with --gui --hold --save-cube -k {gripper}")
    return json.loads(p.read_text())


def holder_layout(preset: dict) -> tuple[list[float], list[float]]:
    """(size, centre) of the holder: size and X / Y from the preset (or the
    default plate), Z so that its top touches the cube's bottom."""
    s = float(preset["size"])
    x, y, z = (float(v) for v in preset["position"])
    holder = preset.get("holder") or {}
    size = [float(v) for v in holder.get("size", [HOLDER_FRACTION * s, HOLDER_FRACTION * s, HOLDER_THICKNESS])]
    hx, hy = (float(v) for v in holder.get("position", [x, y])[:2])
    return size, [hx, hy, z - s / 2.0 - size[2] / 2.0]


def _unit_box_mesh(stage: Usd.Stage, path: str) -> UsdGeom.Mesh:
    mesh = UsdGeom.Mesh.Define(stage, path)
    h = 0.5
    pts = [(-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h), (-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.GetFaceVertexCountsAttr().Set([4] * 6)
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
    mesh.GetExtentAttr().Set([Gf.Vec3f(-h, -h, -h), Gf.Vec3f(h, h, h)])
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    return mesh


def add_cube(stage: Usd.Stage, gripper: str) -> UsdGeom.Cube:
    preset = load_preset(gripper)
    cube = UsdGeom.Cube.Define(stage, CUBE_PATH)
    cube.GetSizeAttr().Set(float(preset["size"]))
    h = float(preset["size"]) / 2.0
    cube.GetExtentAttr().Set([Gf.Vec3f(-h, -h, -h), Gf.Vec3f(h, h, h)])
    UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(*preset["position"]))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(preset.get("mass", DEFAULT_MASS)))
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)  # keep reporting contacts
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.85, 0.35, 0.1)])

    hsize, hpos = holder_layout(preset)
    mesh = _unit_box_mesh(stage, HOLDER_PATH)
    xf = UsdGeom.XformCommonAPI(mesh.GetPrim())
    xf.SetTranslate(Gf.Vec3d(*hpos))
    xf.SetScale(Gf.Vec3f(*hsize))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())  # collider only: static
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
    mesh.GetDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.3, 0.35)])
    return cube


def save_cube(stage: Usd.Stage, gripper: str) -> Path:
    """Write the cube (edge, position) and the holder (size, position) to the
    gripper's preset. A non-uniform cube scale is averaged; rotations are not kept."""
    prim = stage.GetPrimAtPath(CUBE_PATH)
    if not prim:
        raise RuntimeError(f"no {CUBE_PATH} on the stage")
    notes = []
    t, r, s, _pivot, _order = UsdGeom.XformCommonAPI(prim).GetXformVectors(Usd.TimeCode.Default())
    scale = [float(v) for v in s]
    size = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) * sum(scale) / 3.0
    if max(scale) - min(scale) > 1e-6:
        notes.append(f"non-uniform cube scale {scale} averaged")
    if any(abs(float(v)) > 1e-6 for v in r):
        notes.append(f"cube rotation {list(r)} ignored")
    data = {"size": round(size, 6), "position": [round(float(v), 6) for v in t]}
    old = load_preset(gripper) if preset_path(gripper).is_file() else {}
    if "mass" in old:
        data["mass"] = old["mass"]
    hprim = stage.GetPrimAtPath(HOLDER_PATH)
    if hprim:
        ht, hr, hs, _p, _o = UsdGeom.XformCommonAPI(hprim).GetXformVectors(Usd.TimeCode.Default())
        if any(abs(float(v)) > 1e-6 for v in hr):
            notes.append(f"holder rotation {list(hr)} ignored")
        # Height is not kept: add_cube always puts the holder's top at the cube's bottom.
        data["holder"] = {"size": [round(float(v), 6) for v in hs], "position": [round(float(v), 6) for v in ht][:2]}
    path = preset_path(gripper)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    if notes:
        print("cube preset:", "; ".join(notes))
    return path
