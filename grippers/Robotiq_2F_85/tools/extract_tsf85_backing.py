#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Robotiq.
"""Extract the collider of the rigid part of the TSF-85 for ``SensorSim=Precise`` into ``tools/tsf85_backing_hull.json``.

The ``Tactile_TSF85`` fingertip collides with the convex hull of the real sensor's CAD mesh (``parts/TSF_85_case.usd``:
housing, screws and the rigid tactile pad). With ``Precise`` the lab's deformable pad takes the place of the pad, so the
rigid part is that same hull cut at the plane where the deformable's back face starts (y = +-41.45 mm in the
inner_finger frame): the housing and whatever of the pad lies behind that plane. The cut hull is one small convex mesh
per finger (points and triangles, metres, in the inner_finger frame). Needs Isaac Sim's ``pxr`` and scipy:

    python3 tools/extract_tsf85_backing.py
"""

import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom
from scipy.spatial import ConvexHull, HalfspaceIntersection

ASSET = Path(__file__).resolve().parents[1] / "Robotiq_2F_85.usda"
OUT = Path(__file__).resolve().parent / "tsf85_backing_hull.json"
CUT = {"right": 0.04145, "left": -0.04154}  # m, |y| of the deformable's back face in the inner_finger frame
ROOT = "/Robotiq_2F_85/Robotiq_2F_85"


def case_points(stage, side):
    """The rigid case mesh of ``<side>_fingertip`` in the inner_finger frame (the links sit at the identity)."""
    mesh = stage.GetPrimAtPath(f"{ROOT}/{side}_fingertip/visuals/tip_tsf85/TSF_85_case")
    to_world = UsdGeom.XformCache().GetLocalToWorldTransform(mesh)
    pts = np.array(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=np.float64)
    return np.array([to_world.Transform(Gf.Vec3d(*p)) for p in pts])


def cut_hull(points, side):
    """The convex hull of ``points`` on the housing side of the deformable's back face (``CUT``)."""
    hull = ConvexHull(points)
    sign = 1.0 if side == "right" else -1.0
    # keep sign * y >= CUT: -sign * y + |CUT| <= 0, as a (normal, offset) half-space  a.x + b <= 0
    plane = np.array([[0.0, -sign, 0.0, abs(CUT[side])]])
    behind = points[hull.vertices][sign * points[hull.vertices][:, 1] >= abs(CUT[side])]
    inside = behind.mean(axis=0)
    cut = ConvexHull(HalfspaceIntersection(np.vstack([hull.equations, plane]), inside).intersections)
    verts = cut.points[cut.vertices]
    index = {v: i for i, v in enumerate(cut.vertices)}
    centre = verts.mean(axis=0)
    faces = []
    for tri in cut.simplices:
        a, b, c = (cut.points[v] for v in tri)
        outward = np.dot(np.cross(b - a, c - a), (a + b + c) / 3 - centre) > 0
        faces.append([index[tri[0]], index[tri[1] if outward else tri[2]], index[tri[2] if outward else tri[1]]])
    return verts, faces


def main():
    stage = Usd.Stage.Open(str(ASSET))
    root = stage.GetDefaultPrim()
    root.GetVariantSet("Physics").SetVariantSelection("Physx_parallel_grip")
    root.GetVariantSet("Fingertip").SetVariantSelection("Tactile_TSF85")
    out = {}
    for side in ("left", "right"):
        verts, faces = cut_hull(case_points(stage, side), side)
        out[side] = {"points": np.round(verts, 7).tolist(), "faces": faces}
        print(side, len(verts), "vertices,", len(faces), "triangles; y", verts[:, 1].min(), verts[:, 1].max())
    OUT.write_text(json.dumps(out, indent=1) + "\n")
    print("Wrote", OUT)


if __name__ == "__main__":
    main()
