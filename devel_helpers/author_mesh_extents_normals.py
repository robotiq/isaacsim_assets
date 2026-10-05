#!/usr/bin/env python3
"""Author `extent` and `normals` on the Mesh prims of the given USD layers.

The 2F-140 part meshes come from a CAD import that wrote only points/topology, so
SimReady Robot-Gripper fails VG.002 (boundable extent) and VG.027 (mesh normals).
This fills both in place (idempotent; existing authored normals are kept):

  * extent  = exact local bounding box of `points`;
  * normals = faceVarying, area-weighted over the adjacent faces whose normal is
    within --crease-angle degrees of the corner's own face (so curved CAD
    surfaces shade smoothly while hard edges stay sharp).

Usage (needs `usd-core` + `numpy`):
    python3 devel_helpers/author_mesh_extents_normals.py grippers/Robotiq_2F_140/parts/*.usd
"""
import argparse

import numpy as np
from pxr import Sdf, Usd, UsdGeom, Vt


def _face_data(points, counts, indices):
    """Per-face area-weighted normal (Newell), first-corner offsets."""
    offs = np.concatenate(([0], np.cumsum(counts)))
    face_of_corner = np.repeat(np.arange(len(counts)), counts)
    p = points[indices]
    nxt = np.empty_like(p)
    nxt[:-1] = p[1:]
    nxt[offs[1:] - 1] = p[offs[:-1]]  # close each polygon
    cross = np.cross(p, nxt)
    fn = np.zeros((len(counts), 3))
    np.add.at(fn, face_of_corner, cross)  # |fn| = 2*area, direction = face normal
    return fn, face_of_corner


def compute_normals(points, counts, indices, crease_deg, left_handed=False):
    fn, face_of_corner = _face_data(points, counts, indices)
    if left_handed:
        fn = -fn
    area = np.linalg.norm(fn, axis=1)
    unit = fn / np.maximum(area, 1e-30)[:, None]
    cos_t = np.cos(np.radians(crease_deg))

    # Group corners by vertex; smooth each corner over its vertex's compatible faces.
    order = np.argsort(indices, kind="stable")
    sorted_v = indices[order]
    bounds = np.flatnonzero(np.diff(sorted_v)) + 1
    out = np.zeros((len(indices), 3))
    for grp in np.split(order, bounds):
        faces = face_of_corner[grp]
        u = unit[faces]
        w = (u @ u.T >= cos_t) * area[faces][None, :]  # [corner, other face]
        n = w @ u
        out[grp] = n
    nrm = np.linalg.norm(out, axis=1)
    bad = nrm < 1e-20
    out[bad] = unit[face_of_corner[bad]]
    nrm[bad] = 1.0
    return out / np.where(bad, 1.0, nrm)[:, None]


def process(path, crease_deg):
    layer = Sdf.Layer.FindOrOpen(path)
    stage = Usd.Stage.Open(layer)
    n_done = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
        idx = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        mesh.GetExtentAttr().Set(Vt.Vec3fArray([tuple(map(float, lo)), tuple(map(float, hi))]))
        if not mesh.GetNormalsAttr().HasAuthoredValue():
            left = mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded
            nrm = compute_normals(pts, counts, idx, crease_deg, left)
            mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(nrm.astype(np.float32)))
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        n_done += 1
    layer.Save()
    print(f"{path}: {n_done} mesh(es)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("layers", nargs="+")
    ap.add_argument("--crease-angle", type=float, default=40.0)
    args = ap.parse_args()
    for f in args.layers:
        process(f, args.crease_angle)
