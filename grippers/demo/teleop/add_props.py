#!/usr/bin/env python3
"""Author the manipulation props and soften the lighting in the teleop scene.

Run OUTSIDE Isaac, like author_scene.py:

    USDLIB=$(dirname $(find ~/isaacsim/extscache -maxdepth 2 -type d \
             -path "*omni.usd.libs*/pxr" | head -1))
    PYTHONPATH="$USDLIB:$PYTHONPATH" LD_LIBRARY_PATH="$USDLIB/bin:$LD_LIBRARY_PATH" \
      ~/isaacsim/python.sh grippers/demo/teleop/add_props.py

Idempotent: re-running overwrites the props it owns and leaves everything else
alone, so it can be re-run after tweaking a size.

WHAT IT ADDS
------------
* A short thick-walled tube, inner diameter sized so the existing 50 mm
  cylinder drops into it with a few mm of clearance -- an insertion task.
* Four more cubes in different colours, for stacking.

LIGHTING
--------
The scene lit with a single DistantLight at inputs:angle = 1 degree. That is
very nearly a point source, so every shadow has a hard edge and props sitting
in one are genuinely hard to line up against. Widening the angle softens the
penumbra, and a dome light fills the shadows so nothing goes black.
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "ur5robot_with_2F-85.usda")

CYL_RADIUS = 0.025           # the cylinder already in the scene
# Per side. 4 mm was a fiddle rather than a task -- and a convexDecomposition
# bore is not perfectly round, so the usable gap is smaller than the nominal one.
CLEARANCE = 0.009
WALL = 0.010
TUBE_H = 0.060
TUBE_IN = CYL_RADIUS + CLEARANCE
TUBE_OUT = TUBE_IN + WALL
# Pulled in towards the base: it is the thing you carry something TO, so it
# wants to be inside the workspace rather than out where the arm is stretched.
TUBE_AT = (0.430, -0.205, 0.0)
SEGS = 48

# Upright, so they can be grabbed from the side and lowered straight into the
# tube. The cylinder already in the scene lies on its side; leaving it that way
# gives both starting positions.
CYL_H = 0.100
CYLINDERS = [                # (name, x, y, colour)
    # Set back behind the cubes -- +y is away from the default viewpoint, which
    # looks along +y -- so they start clear of everything else rather than
    # crowding the cubes and the tube.
    ("CylinderRed",   0.200,  0.090, (0.80, 0.22, 0.20)),
    ("CylinderGreen", 0.330,  0.120, (0.24, 0.64, 0.30)),
    ("CylinderBlue",  0.420,  0.070, (0.22, 0.42, 0.82)),
]

CUBES = [                    # (name, size, x, y, colour)
    ("CubeRed",    0.024, 0.300, -0.235, (0.78, 0.16, 0.16)),
    ("CubeGreen",  0.024, 0.360, -0.290, (0.22, 0.62, 0.28)),
    ("CubeBlue",   0.024, 0.240, -0.310, (0.20, 0.40, 0.80)),
    ("CubeYellow", 0.024, 0.430, -0.060, (0.88, 0.72, 0.18)),
]


def _tube_mesh(inner, outer, height, segs):
    """Points and quad faces for a hollow cylinder, open-ended walls plus rims."""
    pts, counts, idx = [], [], []
    for r in (outer, inner):                       # 0..segs-1 outer, then inner
        for k in range(segs):
            a = 2.0 * math.pi * k / segs
            pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), 0.0))
    base = len(pts)
    for r in (outer, inner):
        for k in range(segs):
            a = 2.0 * math.pi * k / segs
            pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), height))

    def quad(a, b, c, d):
        counts.append(4)
        idx.extend([a, b, c, d])

    o0, i0 = 0, segs                      # bottom rings
    o1, i1 = base, base + segs            # top rings
    for k in range(segs):
        n = (k + 1) % segs
        quad(o0 + k, o0 + n, o1 + n, o1 + k)      # outer wall
        quad(i0 + n, i0 + k, i1 + k, i1 + n)      # inner wall (reversed)
        quad(o1 + k, o1 + n, i1 + n, i1 + k)      # top rim
        quad(i0 + k, i0 + n, o0 + n, o0 + k)      # bottom rim
    return pts, counts, idx


def _apply_api(prim, name):
    """Add an applied API schema by name, for schemas USD does not know offline.

    PhysxSchema ships with the omni.physx extension and is not importable from
    a plain USD interpreter, but its attributes are ordinary typed attributes
    and the schema is just a token in apiSchemas.
    """
    from pxr import Sdf

    op = prim.GetMetadata("apiSchemas") or Sdf.TokenListOp()
    items = list(op.GetAddedOrExplicitItems())
    if name not in items:
        items.append(name)
        new = Sdf.TokenListOp()
        new.prependedItems = items
        prim.SetMetadata("apiSchemas", new)


def _hollow_collider(prim):
    """Make the tube's bore actually empty in the collision model.

    A mesh collider defaults to a CONVEX HULL, which for a tube is a solid
    cylinder -- visually a hole, physically a plug, and nothing can be inserted.

    Setting physics:approximation alone is not enough: the attribute belongs to
    PhysicsMeshCollisionAPI, and without that schema applied PhysX never reads
    it. That is what was wrong the first time -- the attribute was present and
    correct, and ignored.

    The decomposition parameters matter too. At default resolution the hulls
    bridge a 68 mm bore and close it again, so the hull budget and voxel
    resolution are raised until the hole survives.
    """
    mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mesh_api.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexDecomposition)

    _apply_api(prim, "PhysxConvexDecompositionCollisionAPI")
    for name, tv, value in (
            ("maxConvexHulls", Sdf.ValueTypeNames.Int, 64),
            ("voxelResolution", Sdf.ValueTypeNames.Int, 500000),
            ("errorPercentage", Sdf.ValueTypeNames.Float, 0.5),
            ("minThickness", Sdf.ValueTypeNames.Float, 0.002),
            ("shrinkWrap", Sdf.ValueTypeNames.Bool, True)):
        prim.CreateAttribute("physxConvexDecompositionCollision:" + name,
                             tv).Set(value)


def _rigid(prim, mass, colour):
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*colour)])


def main():
    stage = Usd.Stage.Open(SCENE)
    if stage is None:
        sys.exit("could not open %s" % SCENE)

    # ---- tube -------------------------------------------------------------
    path = "/World/Tube"
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts, counts, idx = _tube_mesh(TUBE_IN, TUBE_OUT, TUBE_H, SEGS)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.AddTranslateOp().Set(Gf.Vec3d(*TUBE_AT))
    prim = mesh.GetPrim()
    _rigid(prim, 0.25, (0.55, 0.55, 0.60))
    _hollow_collider(prim)

    # ---- cubes ------------------------------------------------------------
    for name, size, x, y, colour in CUBES:
        p = "/World/%s" % name
        if stage.GetPrimAtPath(p):
            stage.RemovePrim(p)
        cube = UsdGeom.Cube.Define(stage, p)
        cube.CreateSizeAttr(size)
        cube.AddTranslateOp().Set(Gf.Vec3d(x, y, size / 2.0 + 0.0005))
        _rigid(cube.GetPrim(), 0.06, colour)

    # ---- cylinders --------------------------------------------------------
    for name, x, y, colour in CYLINDERS:
        p = "/World/%s" % name
        if stage.GetPrimAtPath(p):
            stage.RemovePrim(p)
        cyl = UsdGeom.Cylinder.Define(stage, p)
        cyl.CreateRadiusAttr(CYL_RADIUS)
        cyl.CreateHeightAttr(CYL_H)
        cyl.CreateAxisAttr("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(x, y, CYL_H / 2.0 + 0.0005))
        _rigid(cyl.GetPrim(), 0.10, colour)

    # ---- lighting ---------------------------------------------------------
    sun = stage.GetPrimAtPath("/World/Environment/defaultLight")
    notes = []
    if sun and sun.IsValid():
        # 1 degree is very nearly a point source: hard-edged shadows that make
        # a prop sitting in one hard to line up against. The sun subtends about
        # half a degree; 12 is not physical, it is legible.
        sun.GetAttribute("inputs:angle").Set(12.0)
        sun.GetAttribute("inputs:intensity").Set(1800.0)
        notes.append("softened defaultLight (angle 1 -> 12, intensity 3000 -> 1800)")
    dome_path = "/World/Environment/FillDome"
    if stage.GetPrimAtPath(dome_path):
        stage.RemovePrim(dome_path)
    dome = UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateIntensityAttr(320.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.86, 0.95))
    # Fill only: it must not cast its own shadows or we are back where we started.
    dome.GetPrim().CreateAttribute("inputs:shadow:enable",
                                   Sdf.ValueTypeNames.Bool).Set(False)
    notes.append("added FillDome (intensity 320, shadows off)")

    stage.GetRootLayer().Save()
    print("tube: inner d %.1f mm, outer d %.1f mm, wall %.1f mm, height %.0f mm"
          % (TUBE_IN * 2000, TUBE_OUT * 2000, WALL * 1000, TUBE_H * 1000))
    print("      cylinder is %.1f mm across -> %.1f mm clearance per side"
          % (CYL_RADIUS * 2000, CLEARANCE * 1000))
    print("cubes: %s" % ", ".join(c[0] for c in CUBES))
    print("cylinders: %s (upright, r %.3f h %.3f)"
          % (", ".join(c[0] for c in CYLINDERS), CYL_RADIUS, CYL_H))
    for n in notes:
        print(n)


if __name__ == "__main__":
    main()
