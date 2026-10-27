#!/usr/bin/env python3
"""Bake the collision pieces of the Hand-E C10 fingertips.

A convex hull of the stepped fingertip fills its step, and Newton's MuJoCo solver collides every mesh shape as ONE
convex hull whatever the USD asks for. Both engines therefore collide with the SAME pre-split convex pieces,
authored here (same approach as gen_hande_fingertip_colliders.py for the Hand-E):

    parts/Robotiq_Hand_E_C10_fingertip_<key>_<1|2|3>_collision.usd   (/World/hull_0 .. hull_N)

Each piece is a closed convex Mesh (<= 64 vertices, so a solver never has to re-simplify it) with purpose=guide so it
is never rendered. The pieces come from CoACD run on the visual mesh of finger 1's tip with a per-tip hull-count cap
and a fixed seed; fingers 2 and 3 get the same pieces turned by 120 and 240 degrees about Z, so the three tips are
identical.

Usage (needs `usd-core`, `numpy`, `scipy`, `coacd`; import order matters: pxr before coacd):
    pip install usd-core numpy scipy coacd
    python3 devel_helpers/gen_hande_c10_fingertip_colliders.py
"""
import math
import os

from pxr import Gf, Usd, UsdGeom, Vt

import numpy as np  # noqa: E402  (after pxr: coacd's bundled libs crash a later pxr import)
import coacd  # noqa: E402
from scipy.spatial import ConvexHull  # noqa: E402

PARTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "grippers", "Robotiq_Hand_E_C10", "parts")
N = "Robotiq_Hand_E_C10"
THRESHOLD = 0.05  # CoACD concavity threshold
# Pieces per tip (CoACD hull-count cap). From a sweep of the volume the pieces add over the real tip (11.4 cm3): 4 -> +54%,
# 6 -> +54%, 8 -> +48%, 12 -> +42%. The stepped tip has fine ridges that any hull fills, so more pieces help little.
MAX_HULLS = {"stepped": 8}
MAX_HULL_VERTS = 64  # per piece


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


def _rotate_z(pieces, degrees):
    t = math.radians(degrees)
    r = np.array([[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]])
    return [p @ r.T for p in pieces]


if __name__ == "__main__":
    for key, hulls in MAX_HULLS.items():
        first = decompose(os.path.join(PARTS, f"{N}_fingertip_{key}_1.usd"), hulls)
        for finger in (1, 2, 3):
            pieces = _rotate_z(first, 120 * (finger - 1))
            n, vol = write(os.path.join(PARTS, f"{N}_fingertip_{key}_{finger}.usd"), pieces)
            print(f"{N}_fingertip_{key}_{finger}_collision.usd: {n} pieces, {vol * 1e6:.2f} cm3")
