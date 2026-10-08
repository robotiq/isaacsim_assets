#!/usr/bin/env python3
"""Bake the collision pieces of the Hand-E fingertips.

A convex hull of an L-shaped / stepped fingertip fills its notches (the Std tip's hull has
~2x the volume of the tip), and Newton's MuJoCo solver collides every mesh shape as ONE convex
hull whatever the USD asks for -- so a "convexDecomposition" approximation does not help there.
Both engines therefore collide with the SAME pre-split convex pieces, authored here:

    parts/Robotiq_Hand_E_fingertip_<key>_<side>_collision.usd   (/World/hull_0 .. hull_N)

Each piece is a closed convex Mesh (<= 64 vertices, so a solver never has to re-simplify it) with
purpose=guide so it is never rendered. The layer generator (gen_hande_layers.py) references the
pieces under each tip and authors a collider on every one of them.

The pieces come from CoACD run on the visual mesh of the tip with a per-tip hull-count cap (the tips have
screw holes and chamfers; without a cap CoACD returns up to ~100 pieces for a plate). Fixed seed,
so re-running reproduces the same pieces.

Usage (needs `usd-core`, `numpy`, `scipy`, `coacd`; import order matters: pxr before coacd):
    pip install usd-core numpy scipy coacd
    python3 devel_helpers/gen_hande_fingertip_colliders.py
"""
import glob
import os

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

import numpy as np  # noqa: E402  (after pxr: coacd's bundled libs crash a later pxr import)
import coacd  # noqa: E402
from scipy.spatial import ConvexHull  # noqa: E402

PARTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "grippers", "Robotiq_Hand_E", "parts")
THRESHOLD = 0.05        # CoACD concavity threshold
# Pieces per tip (CoACD hull-count cap). Chosen from a sweep of the extra volume the pieces add over the
# real tip (lower = tighter): Std 4 -> +19% (8 -> +15%), Flat_Overmolded 4 -> +16% (8 -> +14%),
# Support 1 -> +37% (more pieces fit a thin plate WORSE: the splitter returns overlapping pieces),
# Extender 3 -> +39% (8 -> +41%), Bin_Picking 6 -> +28% (8 -> +25%). More pieces cost generated layer lines.
MAX_HULLS = {"std": 4, "flat_overmolded": 4, "support": 1, "extender": 3, "binpick": 6}
MAX_HULL_VERTS = 64     # per piece


def _triangles(mesh):
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
    idx = np.array(mesh.GetFaceVertexIndicesAttr().Get())
    tris, o = [], 0
    for c in counts:
        for j in range(1, c - 1):
            tris.append((idx[o], idx[o + j], idx[o + j + 1]))
        o += c
    return np.array(tris)


def _hull_mesh(points):
    """Outward-oriented triangle mesh of the convex hull of `points`."""
    h = ConvexHull(points)
    used = np.unique(h.simplices)
    remap = {int(v): i for i, v in enumerate(used)}
    tris = []
    for simplex, eq in zip(h.simplices, h.equations):
        a, b, c = points[simplex]
        if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
            simplex = simplex[[0, 2, 1]]
        tris.append([remap[int(v)] for v in simplex])
    return points[used], np.array(tris)


def decompose(src_path, max_hulls):
    """Convex pieces (list of Nx3 vertex arrays) of the tip part at `src_path`."""
    src = Usd.Stage.Open(src_path)
    mesh = UsdGeom.Mesh(src.GetPrimAtPath("/World/mesh_0"))
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    parts = coacd.run_coacd(coacd.Mesh(pts, _triangles(mesh)), threshold=THRESHOLD,
                            max_convex_hull=max_hulls, max_ch_vertex=MAX_HULL_VERTS, seed=0)
    return [np.asarray(v, dtype=np.float64) for v, _ in parts]


def write(src_path, pieces):
    out_path = src_path.replace(".usd", "_collision.usd")
    if os.path.exists(out_path):
        os.remove(out_path)
    st = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(st, "Z")
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    total = 0.0
    for i, verts in enumerate(pieces):
        v, t = _hull_mesh(verts)
        total += ConvexHull(v).volume
        m = UsdGeom.Mesh.Define(st, f"/World/hull_{i}")
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(t), 3, dtype=np.int32)))
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(t.astype(np.int32).reshape(-1)))
        m.GetSubdivisionSchemeAttr().Set("none")
        m.GetExtentAttr().Set([Gf.Vec3f(*v.min(0)), Gf.Vec3f(*v.max(0))])
        # Flat per-face normals (VG.027 wants normals on every mesh).
        fn = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
        fn /= np.linalg.norm(fn, axis=1, keepdims=True)
        m.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(np.repeat(fn, 3, axis=0).astype(np.float32)))
        m.SetNormalsInterpolation("faceVarying")
        m.GetPurposeAttr().Set("guide")
    st.GetRootLayer().Save()
    return len(pieces), total


def _rotate_z_180(pieces):
    """The right tip is the left one turned 180 deg about Z: (x, y, z) -> (-x, -y, z)."""
    return [p * np.array([-1.0, -1.0, 1.0]) for p in pieces]


if __name__ == "__main__":
    for key, hulls in MAX_HULLS.items():
        left = os.path.join(PARTS, f"Robotiq_Hand_E_fingertip_{key}_left.usd")
        right = os.path.join(PARTS, f"Robotiq_Hand_E_fingertip_{key}_right.usd")
        lp = decompose(left, hulls)
        # Every tip's right part is the left rotated 180 deg about Z -- except Bin_Picking, which has
        # its own mirrored STEP for the right finger -- so give both sides the same pieces.
        rp = decompose(right, hulls) if key == "binpick" else _rotate_z_180(lp)
        for path, pieces in ((left, lp), (right, rp)):
            n, vol = write(path, pieces)
            print(f"{os.path.basename(path)}: {n} pieces, {vol * 1e6:.2f} cm3")
