#!/usr/bin/env python3
"""Bake the CAD parts of the Robotiq Hand-E C10 into USD.

The C10 has three concentric fingers, 120 degrees apart, that slide radially toward the gripper
axis. Reads Robotiq's ROBOTIQ_HAND-E_C10_DEFEATURE.STEP (the assembly with the stepped fingers, fingers
open) and writes, under grippers/Robotiq_Hand_E_C10/parts/ (one mesh per part, like the Hand-E parts):

    Robotiq_Hand_E_C10_base.usd                      the housing and everything that does not move
    Robotiq_Hand_E_C10_finger_<1|2|3>.usd            finger stub: the slider and its two tip screws
    Robotiq_Hand_E_C10_fingertip_stepped_<1|2|3>.usd the stepped fingertip

Every part is baked into the asset frame, so every link Xform of the asset is identity: the origin is on the
gripper axis at the mounting flange (the back face of the housing), +Z toward the fingertips, finger 1 along
+X, finger 2 at +120 degrees and finger 3 at +240 degrees about Z. Metres.

The meshes are tessellated from the B-rep with smooth normals inside each CAD face and one GeomSubset per
material. Each STEP face colour is converted to sRGB and bound to the nearest material of the shared
materials library (grippers/Robotiq_Hand_E_C10/materials/materials.usda, a copy of the Hand-E's, kept identical
across the grippers), which every part sublayers;
the gray faces of the aluminium finger parts (D-038, D-033) are bound to the library's aluminium instead.

Usage (needs `cadquery-ocp`, `usd-core` and `numpy`; the STEP is not in the repository):
    pip install cadquery-ocp usd-core numpy
    python3 devel_helpers/gen_hande_c10_parts.py /path/to/ROBOTIQ_HAND-E_C10_DEFEATURE.STEP
"""
import math
import os
import sys

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool
from pxr import Gf, Usd, UsdGeom, UsdShade, Vt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "grippers", "Robotiq_Hand_E_C10", "parts")
MATERIALS = os.path.join(HERE, "..", "grippers", "Robotiq_Hand_E_C10", "materials", "materials.usda")
N = "Robotiq_Hand_E_C10"

# CAD frame -> asset frame: x_a = -(z - Z0), y_a = -(x - X0), z_a = y - YF. A proper rotation (determinant +1).
# (X0, Z0) is the gripper axis and YF the mounting flange plane, both in CAD millimetres.
X0, Z0, YF = -32.96, 14.29, -26.1
M = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)

# Parts: STEP part number -> which part it belongs to. Moving solids belong to the finger of their sector.
TIP_PART = "D-033"
FINGER_PARTS = ("D-038", "Y-140410-A")
ALUMINIUM_PARTS = ("D-038", "D-033")  # their gray faces are aluminium; the STEP gives them a plastic-looking gray
ALUMINIUM = "Silver_Rust"  # the library's aluminium (metallic, roughness 0.3)


def to_asset(p_mm):
    q = (np.asarray(p_mm, float) - [X0, 0.0, Z0]) @ M.T
    q[..., 2] -= YF
    return q * 1e-3


def srgb(c):  # linear -> sRGB, the library stores sRGB values
    f = lambda v: 12.92 * v if v <= 0.0031308 else 1.055 * v ** (1 / 2.4) - 0.055
    return tuple(f(v) for v in c)


# ---------------------------------------------------------------- STEP
def load(path):
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise SystemExit(f"cannot read {path}")
    reader.Transfer(doc)
    return doc


def solids(doc):
    """[(part number, transformed shape, [linear rgb or None per face])] for every leaf solid of the assembly."""
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    out = []

    def name_of(label):
        n = TDataStd_Name()
        return label.FindAttribute(TDataStd_Name.GetID_s(), n) and n.Get().ToExtString() or "?"

    def color(shape):
        c = Quantity_Color()
        for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if colors.GetColor(shape, kind, c):
                return (c.Red(), c.Green(), c.Blue())
        return None

    def rec(label, loc):
        if shapes.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            shapes.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                ref = TDF_Label()
                if shapes.GetReferredShape_s(comps.Value(i), ref):
                    rec(ref, loc.Multiplied(shapes.GetLocation_s(comps.Value(i))))
            return
        orig = shapes.GetShape_s(label)
        solid = color(orig)
        faces = []
        ex = TopExp_Explorer(orig, TopAbs_FACE)
        while ex.More():
            faces.append(color(ex.Current()) or solid)
            ex.Next()
        shape = BRepBuilderAPI_Transform(orig, loc.Transformation(), True).Shape() if not loc.IsIdentity() else orig
        out.append((name_of(label).split(" ")[0].replace("_Defeature", ""), shape, faces))

    top = TDF_LabelSequence()
    shapes.GetFreeShapes(top)
    for i in range(1, top.Length() + 1):
        rec(top.Value(i), TopLoc_Location())
    return out


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


# ---------------------------------------------------------------- materials
def library():
    """(stage, {name: sRGB}) of the shared materials library."""
    st = Usd.Stage.Open(MATERIALS)
    colors = {}
    for p in st.Traverse():
        if p.IsA(UsdShade.Material):
            for c in p.GetChildren():
                s = UsdShade.Shader(c)
                if s and s.GetInput("diffuseColor"):
                    colors[p.GetName()] = tuple(s.GetInput("diffuseColor").Get())
    return st, colors


def nearest(rgb_linear, colors):
    c = np.array(srgb(rgb_linear))
    return min(colors, key=lambda n: float(np.linalg.norm(c - np.array(colors[n]))))


def material_name(part, rgb, colors):
    name = nearest(rgb, colors)
    if part in ALUMINIUM_PARTS and name in ("Nobel", "Gravel", "Gray", "Stack", "Pumice"):
        return ALUMINIUM
    return name


# ---------------------------------------------------------------- USD
def write_part(path, group, colors):
    """group: [(part number, shape, face colours)]. One mesh /World/mesh_0 with a GeomSubset per material."""
    pts, norms, counts, idx, face_mat = [], [], [], [], []
    for part, shape, face_colors in group:
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
            face_mat.extend([material_name(part, fm or (0.5, 0.5, 0.5), colors)] * len(f))
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


def sector_of(shape):
    """Which finger (0, 1, 2) a moving solid belongs to: its angle about the axis, in 120 degree steps from +X."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass()
    a = to_asset([c.X(), c.Y(), c.Z()]) * 1e3
    return int(round((math.degrees(math.atan2(a[1], a[0])) % 360) / 120.0)) % 3


def main(step):
    _, colors = library()
    moving = FINGER_PARTS + (TIP_PART,)
    base, fingers, tips = [], {0: [], 1: [], 2: []}, {0: [], 1: [], 2: []}
    for part, shape, faces in solids(load(step)):
        if part.startswith(moving):
            (tips if part.startswith(TIP_PART) else fingers)[sector_of(shape)].append((part, shape, faces))
        else:
            base.append((part, shape, faces))
    os.makedirs(OUT, exist_ok=True)
    jobs = [("base", base)] + [(f"finger_{k + 1}", fingers[k]) for k in range(3)] + [(f"fingertip_stepped_{k + 1}", tips[k]) for k in range(3)]
    for name, group in jobs:
        n, used = write_part(os.path.join(OUT, f"{N}_{name}.usd"), group, colors)
        print(f"{N}_{name}.usd: {len(group)} solids, {n} triangles, materials {', '.join(used)}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
