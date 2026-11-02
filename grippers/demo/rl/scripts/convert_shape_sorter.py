"""Convert the loose pieces of the shape sorter OBJ into one USD asset per piece type.

    python convert_shape_sorter.py <shape_sorter.obj> <out_dir>

The OBJ holds five arrangements of the toy (``shapes_poseNN`` = the loose pieces, ``house_poseNN`` = the sorter, not used).
``shapes_pose01`` has exactly one piece of each of the 6 types, so it is the source of the geometry. Per type this writes

  piece_<k>_<name>.usd   Z-up metres, origin at the centre of mass, resting on its most stable face; collision =
                         convex decomposition (a convex hull would fill the star's and the cross's notches, which are what
                         the pads feel), wood density, friction, the diffuse atlas of the OBJ
  piece_<k>_<name>.obj   the same clean mesh (for an SDF / depth based tactile model)
  poses.json             per type: size, volume, mass, and the stable resting poses (quaternion w x y z, probability, height of
                         the centre of mass above the floor)
  textures/shape_sorter_diffuse.png

The output of this script is committed in grippers/demo/rl/assets/shape_sorter (see the README there)."""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

DENSITY = 550.0  # kg/m3: painted solid wood (beech / birch ply), the pieces are wooden
FRICTION = 0.5
ARRANGEMENT = "shapes_pose01"  # one piece of each type
MIN_POSE_PROBABILITY = 0.02


def read_obj(path: Path, object_name: str):
    """Positions, texture coordinates, normals and the faces of one OBJ object: faces are lists of (v, vt, vn) indices."""
    v, vt, vn, faces, cur = [], [], [], [], None
    for line in path.read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            v.append([float(x) for x in p[1:4]])
        elif p[0] == "vt":
            vt.append([float(x) for x in p[1:3]])
        elif p[0] == "vn":
            vn.append([float(x) for x in p[1:4]])
        elif p[0] == "o":
            cur = p[1]
        elif p[0] == "f" and cur is not None and object_name in cur:
            faces.append([tuple((int(i) - 1) if i else -1 for i in (t.split("/") + ["", ""])[:3]) for t in p[1:]])
    return np.array(v), np.array(vt), np.array(vn), faces


def pieces(v, faces):
    """Split the faces of one OBJ object into connected pieces (lists of face indices)."""
    idx = np.unique([c[0] for f in faces for c in f])
    remap = {int(a): i for i, a in enumerate(idx)}
    rows, cols = [], []
    for f in faces:
        for a, b in zip(f, f[1:] + f[:1]):
            rows.append(remap[a[0]])
            cols.append(remap[b[0]])
    _, lab = connected_components(coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(idx), len(idx))), directed=False)
    out = {}
    for fi, f in enumerate(faces):
        out.setdefault(int(lab[remap[f[0][0]]]), []).append(fi)
    return [out[k] for k in sorted(out)]


def to_z_up(a: np.ndarray) -> np.ndarray:
    """OBJ is Y up, -Z forward; the sim is Z up: (x, y, z) -> (x, -z, y)."""
    return np.stack([a[..., 0], -a[..., 2], a[..., 1]], axis=-1)


def quat_from_matrix(R: np.ndarray) -> list[float]:
    q = trimesh.transformations.quaternion_from_matrix(np.pad(R, ((0, 1), (0, 1)), constant_values=0) + np.diag([0, 0, 0, 1]))
    return [float(x) for x in q]  # (w, x, y, z)


# the six pieces of ``shapes_pose01`` in the order ``pieces`` returns them; each is a 4 cm thick extrusion of this outline
# (checked on the mid-height section of every piece, see the README of the demo)
NAMES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("out")
    args = ap.parse_args()
    obj, out = Path(args.obj), Path(args.out)
    (out / "textures").mkdir(parents=True, exist_ok=True)
    shutil.copy(obj.parent / "maps" / "shape_sorter_diffuse.png", out / "textures" / "shape_sorter_diffuse.png")

    v, vt, vn, faces = read_obj(obj, ARRANGEMENT)
    summary = {}
    for k, fidx in enumerate(pieces(v, faces)):
        pf = [faces[i] for i in fidx]
        vids = sorted({c[0] for f in pf for c in f})
        local = {a: i for i, a in enumerate(vids)}
        pts = to_z_up(v[vids])
        tris = np.array([[local[f[0][0]], local[f[i][0]], local[f[i + 1][0]]] for f in pf for i in range(1, len(f) - 1)])
        mesh = trimesh.Trimesh(pts, tris, process=False)
        com = mesh.center_mass if mesh.is_watertight else mesh.convex_hull.center_mass
        volume = abs(mesh.volume) if mesh.is_watertight else mesh.convex_hull.volume
        # the most stable face goes down; poses are computed on the convex hull (the notches never touch the floor)
        poses, probs = mesh.convex_hull.compute_stable_poses(center_mass=com, sigma=0.0, n_samples=1, threshold=0.0)
        order = np.argsort(-probs)
        poses, probs = poses[order], probs[order]
        T0 = poses[0]  # maps the piece (as authored) to its most stable resting orientation, xy not centred
        R0 = T0[:3, :3]
        pts_rest = (pts - com) @ R0.T  # origin at the centre of mass, resting orientation
        # the xy of the centre of mass is the origin; the floor is at z = -h
        h = -pts_rest[:, 2].min()
        # per-face-corner data for USD
        normals = (to_z_up(vn[[c[2] for f in pf for c in f]]) @ R0.T) if len(vn) and pf[0][0][2] >= 0 else None
        st = vt[[c[1] for f in pf for c in f]] if len(vt) and pf[0][0][1] >= 0 else None
        counts = [len(f) for f in pf]
        indices = [local[c[0]] for f in pf for c in f]

        clean = trimesh.Trimesh(pts_rest, tris, process=False)
        name = NAMES[k]
        stem = f"piece_{k}_{name}"
        clean.export(out / f"{stem}.obj")

        stage = Usd.Stage.CreateNew(str(out / f"{stem}.usd"))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, "/Piece")
        stage.SetDefaultPrim(root.GetPrim())
        geom = UsdGeom.Mesh.Define(stage, "/Piece/Geom")
        geom.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts_rest.astype(np.float32)))
        geom.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        geom.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
        geom.CreateSubdivisionSchemeAttr("none")
        if normals is not None:
            geom.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
            geom.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if st is not None:
            uv = UsdGeom.PrimvarsAPI(geom).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
            uv.Set(Vt.Vec2fArray.FromNumpy(st.astype(np.float32)))
        prim = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexDecomposition")
        prim.AddAppliedSchema("PhysxConvexDecompositionCollisionAPI")
        prim.CreateAttribute("physxConvexDecompositionCollision:maxConvexHulls", Sdf.ValueTypeNames.Int).Set(16)
        prim.CreateAttribute("physxConvexDecompositionCollision:hullVertexLimit", Sdf.ValueTypeNames.Int).Set(64)
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateDensityAttr(DENSITY)

        # visual material: the diffuse atlas
        mat = UsdShade.Material.Define(stage, "/Piece/Looks/Wood")
        shader = UsdShade.Shader.Define(stage, "/Piece/Looks/Wood/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        reader = UsdShade.Shader.Define(stage, "/Piece/Looks/Wood/st")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, "/Piece/Looks/Wood/diffuse")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./textures/shape_sorter_diffuse.png")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        # physics material: friction
        pmat = UsdShade.Material.Define(stage, "/Piece/PhysicsMaterial")
        api = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
        api.CreateStaticFrictionAttr(FRICTION)
        api.CreateDynamicFrictionAttr(FRICTION)
        api.CreateRestitutionAttr(0.05)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(pmat, UsdShade.Tokens.weakerThanDescendants, "physics")
        stage.GetRootLayer().Save()

        # size: the yaw does not matter, so the footprint is the minimum-area rectangle of the outline, not the axis-aligned box
        xy = pts_rest[:, :2]
        best = None
        for ang in np.linspace(0.0, np.pi / 2, 181):
            c, sn = np.cos(ang), np.sin(ang)
            e = (xy @ np.array([[c, -sn], [sn, c]])).max(0) - (xy @ np.array([[c, -sn], [sn, c]])).min(0)
            if best is None or e[0] * e[1] < best[0]:
                best = (e[0] * e[1], e)
        thickness = pts_rest[:, 2].max() - pts_rest[:, 2].min()
        ext = np.append(best[1], thickness)
        keep = probs >= MIN_POSE_PROBABILITY
        resting = []
        for T, p in zip(poses[keep], probs[keep]):
            R = T[:3, :3] @ R0.T  # relative to the authored-then-rested frame: pose of the piece as spawned
            rest_pts = pts_rest @ R.T
            resting.append({"quat_wxyz": quat_from_matrix(R), "probability": float(p), "com_height": float(-rest_pts[:, 2].min())})
        total = sum(r["probability"] for r in resting)
        for r in resting:
            r["probability"] /= total
        summary[stem] = {
            "name": name,
            "oriented_box_cm": sorted([round(float(s) * 100, 2) for s in ext], reverse=True),
            "height_resting_cm": round(float(thickness) * 100, 2),
            "volume_cm3": round(float(volume) * 1e6, 1),
            "mass_g": round(float(volume) * DENSITY * 1000, 1),
            "watertight": bool(mesh.is_watertight),
            "rest_com_height_cm": round(float(h) * 100, 2),
            "stable_poses": resting,
        }
        print(f"{stem:22s} oriented box {np.round(sorted(ext, reverse=True) * np.array(100.0), 1)} cm, {thickness * 100:.1f} cm tall resting  volume {volume * 1e6:6.1f} cm3  mass {volume * DENSITY * 1000:5.1f} g  "
              f"poses {len(resting)}  watertight {mesh.is_watertight}")
    (out / "poses.json").write_text(json.dumps(summary, indent=2))
    print("wrote", out)


if __name__ == "__main__":
    main()
