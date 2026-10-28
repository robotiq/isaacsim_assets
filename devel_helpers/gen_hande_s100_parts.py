#!/usr/bin/env python3
"""Bake the CAD parts of the Robotiq Hand-E 100 mm stroke ("Stroke100" variant) into USD.

Reads Robotiq's ROBOTIQ_HAND-E-P7L_DEFEATURE.STEP (fingers open; the stroke-100 gripper) and writes, under
grippers/Robotiq_Hand_E/parts/:

    Robotiq_Hand_E_s100_base.usd           housing, rack plate, flange dowels and pins: everything that does not move
    Robotiq_Hand_E_s100_finger_left.usd    left finger slider (the stub the fingertip is screwed to)
    Robotiq_Hand_E_s100_finger_right.usd   right finger slider

The fingertips are NOT baked here: Stroke100 reuses the stock Hand-E fingertip parts, shifted 25 mm along Y.
(The tips in the P7L open model are the Flat_Overmolded ones: shells 4 + 5 = 9445 mm3 against 9440 mm3.)

The STEP's open model is one assembly shape of 33 SHELLS (OCP: 0 solids), not named parts. The shells are
told apart by face count and checked against the geometry below; in the file's order they are:
    base:   0 (167 faces), 1 (52), 2 (162, the 125 mm rack plate), 6-10 (dowels), 14-19 (pins), 28 (1251, housing)
    right finger (Isaac y > 0): 3 (146 faces)      left finger (Isaac y < 0): 25 (146 faces)
    fingertip and its screws (skipped): 4, 5, 11-13 / 20-24
    degenerate 1-2 face surfaces of zero area (skipped): 26, 27, 29-32
Open versus closed, the moving shells translate 50.0 mm along the CAD Z axis each: 100 mm total.

Every part is baked into the same frame as the stroke-50 parts, so every link Xform is identity:
asset = (-x, z, y + 86.1) mm -> m, flange = back of the locating ring, fingers close along Y, approach +Z.
Materials: each STEP face colour (linear RGB) -> sRGB -> nearest material of the shared library
(grippers/Robotiq_Hand_E/materials/materials.usda, kept identical across the grippers), restricted to the palette the stroke-50 parts use.

Usage (needs `cadquery-ocp`, `usd-core` and `numpy`; the STEP is not in the repository):
    pip install cadquery-ocp usd-core numpy
    python3 devel_helpers/gen_hande_s100_parts.py /path/to/ROBOTIQ_HAND-E-P7L_DEFEATURE.STEP
"""
import os
import sys

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool
from pxr import Gf, Usd, UsdGeom, UsdShade, Vt

HERE = os.path.dirname(os.path.abspath(__file__))
GRIPPER = os.path.join(HERE, "..", "grippers", "Robotiq_Hand_E")
OUT = os.path.join(GRIPPER, "parts")
MATERIALS = os.path.join(GRIPPER, "materials", "materials.usda")
N = "Robotiq_Hand_E"

# CAD frame -> asset frame: x_a = -x, y_a = z, z_a = y + 86.1 (mm). A proper rotation (determinant +1).
M = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]], float)
FLANGE_Y = 86.1

BASE = (0, 1, 2, 6, 7, 8, 9, 10, 14, 15, 16, 17, 18, 19, 28)
LEFT, RIGHT = 25, 3
# The stroke-50 parts use only these library materials; the same palette keeps the two strokes looking alike.
PALETTE = ("Cod_Gray", "Periwinkle_Gray", "Gallery", "White", "Silver_Rust", "Celeste")


def to_asset(p_mm):
    q = np.asarray(p_mm, float) @ M.T
    q[..., 2] += FLANGE_Y
    return q * 1e-3


def srgb(c):  # linear -> sRGB, the library stores sRGB values
    f = lambda v: 12.92 * v if v <= 0.0031308 else 1.055 * v ** (1 / 2.4) - 0.055
    return tuple(f(v) for v in c)


def tessellate(shape, lin=0.05, ang=0.3):
    """Per CAD face: (vertices (n,3) mm, triangles (m,3), smooth vertex normals (n,3)); outward-facing winding."""
    BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            out.append(None)
            ex.Next()
            continue
        trsf = loc.Transformation()
        v = np.array([(p.X(), p.Y(), p.Z()) for p in (tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1))])
        f = np.array([[n - 1 for n in tri.Triangle(i).Get()] for i in range(1, tri.NbTriangles() + 1)])
        if face.Orientation() == TopAbs_REVERSED:
            f = f[:, [0, 2, 1]]
        fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])  # area-weighted face normals
        vn = np.zeros_like(v)
        for k in range(3):
            np.add.at(vn, f[:, k], fn)
        length = np.linalg.norm(vn, axis=1, keepdims=True)
        out.append((v, f, vn / np.where(length == 0, 1, length)))
        ex.Next()
    return out


# ---------------------------------------------------------------- STEP
def load(path):
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise SystemExit(f"cannot read {path}")
    reader.Transfer(doc)
    return doc


def shells(doc):
    """[(shape, [linear rgb or None per face])] for every shell of the open model, in file order."""
    ctool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    top = TDF_LabelSequence()
    XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).GetFreeShapes(top)
    assert top.Length() == 1
    root = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()).GetShape_s(top.Value(1))
    out = []
    ex = TopExp_Explorer(root, TopAbs_SHELL)
    while ex.More():
        shell, faces = ex.Current(), []
        fx = TopExp_Explorer(shell, TopAbs_FACE)
        while fx.More():
            c = Quantity_Color()
            got = None
            for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
                if ctool.GetColor(fx.Current(), kind, c):
                    got = (c.Red(), c.Green(), c.Blue())
                    break
            faces.append(got)
            fx.Next()
        out.append((shell, faces))
        ex.Next()
    return out



# ---------------------------------------------------------------- materials
def library():
    """{name: sRGB} of the shared materials library."""
    st = Usd.Stage.Open(MATERIALS)
    colors = {}
    for p in st.Traverse():
        if p.IsA(UsdShade.Material):
            for c in p.GetChildren():
                s = UsdShade.Shader(c)
                if s and s.GetInput("diffuseColor"):
                    colors[p.GetName()] = tuple(s.GetInput("diffuseColor").Get())
    return colors


def material_name(rgb, colors):
    c = np.array(srgb(rgb))
    return min(PALETTE, key=lambda n: float(np.linalg.norm(c - np.array(colors[n]))))

# ---------------------------------------------------------------- USD
def write_part(path, group, colors):
    """group: [(shape, face colours)]. One mesh /World/mesh_0 with a GeomSubset per material."""
    pts, norms, counts, idx, face_mat = [], [], [], [], []
    for shape, face_colors in group:
        for fm, tess in zip(face_colors, tessellate(shape)):
            if tess is None:
                continue
            v, f, vn = tess
            base = len(pts)
            pts.extend(map(tuple, to_asset(v)))
            nrm = vn @ M.T  # directions only: rotate, no translation or scale
            norms.extend(map(tuple, nrm[f].reshape(-1, 3)))  # faceVarying: one per triangle corner
            idx.extend((f + base).reshape(-1).tolist())
            counts.extend([3] * len(f))
            face_mat.extend([material_name(fm or (0.5, 0.5, 0.5), colors)] * len(f))
    if os.path.exists(path):
        os.remove(path)
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, "Z")
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    used = sorted(set(face_mat))
    # The materials come from the shared library, sublayered like the Hand-E parts do (one source of truth, kept identical
    # across the grippers); it also puts the library in the composed layer stack, as SimReady ISA.001 expects.
    st.GetRootLayer().subLayerPaths.append("../materials/materials.usda")
    mesh = UsdGeom.Mesh.Define(st, "/World/mesh_0")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in norms]))
    mesh.SetNormalsInterpolation("faceVarying")
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(False)
    arr = np.array(pts)
    mesh.CreateExtentAttr([Gf.Vec3f(*arr.min(0)), Gf.Vec3f(*arr.max(0))])
    fm = np.array(face_mat)
    for m in used:
        sub = UsdGeom.Subset.Define(st, f"/World/mesh_0/{m}")
        sub.CreateElementTypeAttr("face")
        sub.CreateIndicesAttr(Vt.IntArray(np.nonzero(fm == m)[0].tolist()))
        sub.CreateFamilyNameAttr("materialBind")
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(UsdShade.Material(st.GetPrimAtPath(f"/World/Looks/{m}")))
    st.GetRootLayer().Save()
    return len(counts), used



def main(step):
    colors = library()
    allshells = shells(load(step))
    assert len(allshells) == 33
    faces = [len(faces) for _, faces in allshells]
    assert faces[LEFT] == faces[RIGHT] == 146 and faces[2] == 162 and faces[28] == 1251, faces
    jobs = [("base", BASE), ("finger_left", (LEFT,)), ("finger_right", (RIGHT,))]
    for name, idx in jobs:
        n, used = write_part(os.path.join(OUT, f"{N}_s100_{name}.usd"), [allshells[i] for i in idx], colors)
        print(f"{N}_s100_{name}.usd: {len(idx)} shells, {n} triangles, materials {', '.join(used)}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
