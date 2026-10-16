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

``add_fixed_cube`` builds the fixed-cube test's box instead (preset
``fixed_cubes/<gripper folder>.json``, ``<gripper folder>_compliant.json`` for the
compliant cases: "size" [sx, sy, sz] and "position", its
centre): a static collider with no rigid body and no holder, so nothing can move it.
Like the holder it is a unit box mesh scaled by ``xformOp:scale``, so it can be
stretched per axis with the scale gizmo on both engines. That preset also keeps the
test's recorded joint baselines under "expected" (see test_gripper_fixed_cube.py);
moving or resizing the box drops them.

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

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

CUBES_DIR = Path(__file__).resolve().parent / "cubes"
FIXED_CUBES_DIR = Path(__file__).resolve().parent / "fixed_cubes"
CUBE_PATH = "/World/Cube"
HOLDER_PATH = "/World/CubeHolder"

DEFAULT_MASS = 1.0  # kg: much lighter makes PhysX mis-solve the squeeze (lopsided, links catch it)
HOLDER_FRACTION = 0.5  # default holder footprint, as a fraction of the cube edge
HOLDER_THICKNESS = 0.005  # m, default


def preset_path(gripper: str, fixed: bool = False) -> Path:
    return (FIXED_CUBES_DIR if fixed else CUBES_DIR) / f"{gripper}.json"


def load_preset(gripper: str, fixed: bool = False) -> dict:
    p = preset_path(gripper, fixed)
    if not p.is_file():
        raise FileNotFoundError(f"no cube preset {p}: place one with --gui --hold --save-cube -k {gripper}")
    return json.loads(p.read_text())


def write_preset(gripper: str, data: dict, fixed: bool = False) -> Path:
    path = preset_path(gripper, fixed)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path


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


def _define_cube(stage: Usd.Stage, preset: dict) -> UsdGeom.Cube:
    cube = UsdGeom.Cube.Define(stage, CUBE_PATH)
    cube.GetSizeAttr().Set(float(preset["size"]))
    h = float(preset["size"]) / 2.0
    cube.GetExtentAttr().Set([Gf.Vec3f(-h, -h, -h), Gf.Vec3f(h, h, h)])
    UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(*preset["position"]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_fixed_cube(stage: Usd.Stage, gripper: str) -> UsdGeom.Mesh:
    """The fixed-cube test's box: a collider only (static), no holder."""
    preset = load_preset(gripper, fixed=True)
    size = preset["size"]
    size = [float(v) for v in size] if isinstance(size, list) else [float(size)] * 3
    mesh = _unit_box_mesh(stage, CUBE_PATH)
    xf = UsdGeom.XformCommonAPI(mesh.GetPrim())
    xf.SetTranslate(Gf.Vec3d(*preset["position"]))
    xf.SetScale(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
    mesh.GetDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.45, 0.8)])
    return mesh


def _local_trs(prim: Usd.Prim) -> tuple[list[float], list[float], bool]:
    """(translation, per-axis scale, rotated?) of ``prim``'s local transform,
    whatever xformOps the Isaac gizmos authored."""
    t = Gf.Transform(UsdGeom.Xformable(prim).GetLocalTransformation())
    rotated = abs(t.GetRotation().GetAngle()) > 1e-4
    return [float(v) for v in t.GetTranslation()], [float(v) for v in t.GetScale()], rotated


def _add_holder(stage: Usd.Stage, size: list[float], centre: list[float]) -> UsdGeom.Mesh:
    """The static plate the cube rests on: a collider only (no rigid body)."""
    mesh = _unit_box_mesh(stage, HOLDER_PATH)
    xf = UsdGeom.XformCommonAPI(mesh.GetPrim())
    xf.SetTranslate(Gf.Vec3d(*centre))
    xf.SetScale(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)
    mesh.GetDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.3, 0.35)])
    return mesh


def _add_dynamic_cube(stage: Usd.Stage, preset: dict) -> UsdGeom.Cube:
    cube = _define_cube(stage, preset)
    prim = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(preset.get("mass", DEFAULT_MASS)))
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)  # keep reporting contacts
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.85, 0.35, 0.1)])
    return cube


def set_holder_friction(stage: Usd.Stage, friction: float) -> None:
    """PhysX only: give the holder a physics material with static = dynamic friction
    ``friction`` and combine mode "min" (it beats the cube's default "average"), so the
    cube-holder contact has that coefficient as long as the cube's own is higher.
    MuJoCo takes the larger coefficient of the two, so this does not lower it on Newton."""
    material = UsdShade.Material.Define(stage, "/World/Looks/HolderFriction")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(friction)
    api.CreateDynamicFrictionAttr().Set(friction)
    api.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr().Set("min")
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(HOLDER_PATH)).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")


def add_cube(stage: Usd.Stage, gripper: str) -> UsdGeom.Cube:
    preset = load_preset(gripper)
    cube = _add_dynamic_cube(stage, preset)
    hsize, hpos = holder_layout(preset)
    _add_holder(stage, hsize, hpos)
    return cube


def add_hanging_cube(stage: Usd.Stage, gripper: str, base_height: float) -> None:
    """The preset's cube and holder for a gripper hung upside down (rotated 180 deg
    about X, fingers pointing down) with its base at world height ``base_height``:
    the preset's gripper-frame layout mirrored through that flip, so the cube sits
    between the pads exactly as in the upright tests, resting on the holder, which
    now lies below it."""
    preset = load_preset(gripper)
    x, y, z = (float(v) for v in preset["position"])
    flipped = dict(preset, position=[x, -y, base_height - z])
    _add_dynamic_cube(stage, flipped)
    hsize, (hx, hy, _hz) = holder_layout(preset)
    s = float(preset["size"])
    _add_holder(stage, hsize, [hx, -hy, flipped["position"][2] - s / 2.0 - hsize[2] / 2.0])


def save_cube(stage: Usd.Stage, gripper: str, fixed: bool = False) -> Path:
    """Write the cube (edge, position) and the holder (size, position) to the
    gripper's preset; with ``fixed``, the fixed-cube test's box (per-axis size,
    position). A non-uniform grasp-cube scale is averaged; rotations are not kept."""
    prim = stage.GetPrimAtPath(CUBE_PATH)
    if not prim:
        raise RuntimeError(f"no {CUBE_PATH} on the stage")
    notes = []
    if fixed:
        t, scale, rotated = _local_trs(prim)
        if rotated:
            notes.append("box rotation ignored")
        if prim.IsA(UsdGeom.Cube):  # an older preset's primitive cube
            scale = [v * float(UsdGeom.Cube(prim).GetSizeAttr().Get()) for v in scale]
        data = {"size": [round(v, 6) for v in scale], "position": [round(v, 6) for v in t]}
        old = load_preset(gripper, fixed) if preset_path(gripper, fixed).is_file() else {}
        if "expected" in old:
            if (data["size"], data["position"]) == (old.get("size"), old.get("position")):
                data["expected"] = old["expected"]
            else:
                notes.append("box moved: recorded joint baselines dropped, record them again with --record-baseline")
        path = write_preset(gripper, data, fixed)
        if notes:
            print("cube preset:", "; ".join(notes))
        return path
    t, r, s, _pivot, _order = UsdGeom.XformCommonAPI(prim).GetXformVectors(Usd.TimeCode.Default())
    scale = [float(v) for v in s]
    size = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) * sum(scale) / 3.0
    if max(scale) - min(scale) > 1e-6:
        notes.append(f"non-uniform cube scale {scale} averaged")
    if any(abs(float(v)) > 1e-6 for v in r):
        notes.append(f"cube rotation {list(r)} ignored")
    data = {"size": round(size, 6), "position": [round(float(v), 6) for v in t]}
    old = load_preset(gripper, fixed) if preset_path(gripper, fixed).is_file() else {}
    if "mass" in old:
        data["mass"] = old["mass"]
    hprim = stage.GetPrimAtPath(HOLDER_PATH)
    if hprim:
        ht, hr, hs, _p, _o = UsdGeom.XformCommonAPI(hprim).GetXformVectors(Usd.TimeCode.Default())
        if any(abs(float(v)) > 1e-6 for v in hr):
            notes.append(f"holder rotation {list(hr)} ignored")
        # Height is not kept: add_cube always puts the holder's top at the cube's bottom.
        data["holder"] = {"size": [round(float(v), 6) for v in hs], "position": [round(float(v), 6) for v in ht][:2]}
    path = write_preset(gripper, data, fixed)
    if notes:
        print("cube preset:", "; ".join(notes))
    return path
