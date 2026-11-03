"""Signed-distance grids of the shape sorter pieces, for the shape-aware tactile model.

    python make_piece_sdfs.py [--assets assets/shape_sorter] [--pitch 0.001] [--margin 0.015]

For every ``piece_*.obj`` (Z up, origin at the centre of mass: the frame of the USD asset) writes ``sdf/<piece>.npz``:
``sdf`` [z, y, x] in metres (0 on the surface, positive outside, negative inside; float16), ``origin`` (the position of voxel
[0, 0, 0]), ``pitch``. The grid extends ``margin`` beyond the bounding box, the value outside is the distance to the box edge,
which is all the tactile model needs (it only looks within a few millimetres of the surface)."""

import argparse
from pathlib import Path

import numpy as np
import trimesh
from trimesh.proximity import signed_distance

ap = argparse.ArgumentParser()
ap.add_argument("--assets", default=str(Path(__file__).resolve().parents[1] / "assets" / "shape_sorter"))
ap.add_argument("--pitch", type=float, default=0.001)
ap.add_argument("--margin", type=float, default=0.015)
args = ap.parse_args()

out = Path(args.assets) / "sdf"
out.mkdir(exist_ok=True)
for obj in sorted(Path(args.assets).glob("piece_*.obj")):
    mesh = trimesh.load(obj, process=False)
    lo, hi = mesh.bounds[0] - args.margin, mesh.bounds[1] + args.margin
    axes = [np.arange(lo[i], hi[i] + args.pitch, args.pitch) for i in range(3)]
    X, Y, Z = np.meshgrid(*axes, indexing="ij")
    pts = np.stack([X, Y, Z], -1).reshape(-1, 3)
    sd = np.concatenate([-signed_distance(mesh, pts[i : i + 20000]) for i in range(0, len(pts), 20000)])  # positive outside
    grid = sd.reshape(X.shape).transpose(2, 1, 0).astype(np.float16)  # [z, y, x]
    np.savez_compressed(out / f"{obj.stem}.npz", sdf=grid, origin=np.array([a[0] for a in axes]), pitch=args.pitch)
    print(f"{obj.stem}: grid {grid.shape[::-1]} (x, y, z), {grid.nbytes / 1e6:.1f} MB raw, min {float(grid.min()) * 1000:.1f} mm, max {float(grid.max()) * 1000:.1f} mm")
