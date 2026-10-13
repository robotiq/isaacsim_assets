#!/usr/bin/env python3
"""Generate the PhysX surface-deformable "tissue" prop.

Writes grippers/demo/teleop/props/Tissue.usda: a flat 0.28 x 0.30 m sheet (a tissue / cloth) that PhysX
simulates as a surface deformable body, so it drapes, wrinkles and can be pinched by a gripper. The prop is not
referenced by any scene yet.

The structure mirrors what Isaac's own `deformableUtils.create_auto_surface_deformable_hierarchy`
authors (see omni.physx's SurfaceDeformableDemo), but is written straight into USD so the prop
is a plain referenced file with no live authoring step:

    /Tissue                    Xform, PhysxAutoDeformableBodyAPI + OmniPhysicsDeformableBodyAPI +
                               PhysxSurfaceDeformableBodyAPI (the deformable body root)
      /skin                    the visible Mesh; PhysX cooks the simulation mesh FROM it
      /simMesh                 empty sim mesh (OmniPhysicsSurfaceDeformableSimAPI + collision); PhysX
                               fills it at cook time, guide purpose so it is never drawn
      /TissueMaterial          the surface-deformable material (thickness, stretch/shear/bend, friction)
      /TissueLook              the visual material (UsdPreviewSurface)

The schemas are not registered in plain usd-core, so API schemas are authored as explicit `apiSchemas`
metadata and the attributes with explicit types (names / types from Isaac's PhysxSchema and
OmniUsdPhysicsDeformableSchema).

Usage (needs `usd-core`):
    python3 devel_helpers/gen_tissue_prop.py
"""
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "grippers", "demo", "teleop", "props", "Tissue.usda")

SIZE_X = 0.28        # m, sheet size along X
SIZE_Y = 0.30        # m, sheet size along Y
CELL = 0.005         # m, quad size (kept below the collision offsets' limit, see below)
CELLS_X = round(SIZE_X / CELL)   # 56
CELLS_Y = round(SIZE_Y / CELL)   # 60 -> 57 x 61 = 3477 vertices
STEPS_PER_SECOND = 120   # the teleop scene's physxScene:timeStepsPerSecond

# Collision offsets (m). The rest offset is the distance the sheet keeps from other surfaces; the
# contact offset (>= 3 x rest) is where contacts start to be generated -- both matter for a sheet this thin.
# Both MUST stay below the cell size (5 mm): with a 5 mm contact offset and self-collision on, neighbouring
# vertices collided constantly and the sheet exploded (1.7 m wide within seconds).
REST_OFFSET = 0.001
CONTACT_OFFSET = 0.003
# Vertices closer than this ignore each other in self-collision (a few cells), so the sheet can still fold
# over itself but its own neighbours do not fight.
SELF_COLLISION_FILTER_DISTANCE = 0.012
# Cap on the vertex speed (m/s). With the default 5 m/s a fully squeezed pinch followed by a lift made the sheet
# explode (0.95 m wide); at 0.5 m/s it stays intact. Hand motion is well below that.
MAX_LINEAR_VELOCITY = 0.5

# Surface deformable material. Initial values, to be tuned live (the units follow the stage: m, kg).
THICKNESS = 0.001            # m, simulated thickness of the sheet
DENSITY = 100.0              # kg/m^3 -> with 1 mm thickness ~100 g/m^2 (a heavy tissue); ~4 g per sheet
STATIC_FRICTION = 1.0
DYNAMIC_FRICTION = 1.0
# The in-plane stiffness comes from the Young's modulus (0 = no in-plane elasticity: the sheet then stretches
# without limit, whatever the stretch stiffness says). The extra surface stretch stiffness is left off.
YOUNGS_MODULUS = 5.0e5       # Pa, Isaac's default deformable value
POISSONS_RATIO = 0.3
STRETCH_STIFFNESS = 0.0
SHEAR_STIFFNESS = 0.1
BEND_STIFFNESS = 0.001
ELASTICITY_DAMPING = 0.0
BEND_DAMPING = 0.0


def _apis(prim, names):
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(names))


def _attr(prim, name, vtype, value):
    prim.CreateAttribute(name, vtype, custom=False).Set(value)


def grid():
    nx, ny = CELLS_X, CELLS_Y
    pts = [Gf.Vec3f(-SIZE_X / 2 + SIZE_X * i / nx, -SIZE_Y / 2 + SIZE_Y * j / ny, 0.0)
           for j in range(ny + 1) for i in range(nx + 1)]
    idx = []
    for j in range(ny):
        for i in range(nx):
            a = j * (nx + 1) + i
            b = a + 1
            c = a + (nx + 1)
            d = c + 1
            idx += [a, b, d, a, d, c]          # counter-clockwise seen from +Z
    return Vt.Vec3fArray(pts), Vt.IntArray(idx)


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    st = Usd.Stage.CreateNew(OUT)
    UsdGeom.SetStageUpAxis(st, "Z")
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/Tissue")
    st.SetDefaultPrim(root.GetPrim())
    rp = root.GetPrim()

    # --- visible skin mesh (what PhysX cooks the simulation mesh from) --------------------------
    pts, idx = grid()
    skin = UsdGeom.Mesh.Define(st, "/Tissue/skin")
    skin.CreatePointsAttr(pts)
    skin.CreateFaceVertexCountsAttr(Vt.IntArray([3] * (len(idx) // 3)))
    skin.CreateFaceVertexIndicesAttr(idx)
    skin.CreateSubdivisionSchemeAttr("loop")   # smooth-shaded: with "none" every triangle of the sim mesh shows
    skin.CreateDoubleSidedAttr(True)
    skin.CreateExtentAttr([Gf.Vec3f(-SIZE_X / 2, -SIZE_Y / 2, 0), Gf.Vec3f(SIZE_X / 2, SIZE_Y / 2, 0)])
    skin.CreateDisplayColorAttr([Gf.Vec3f(0.93, 0.92, 0.90)])
    sp = skin.GetPrim()
    _apis(sp, ["OmniPhysicsDeformablePoseAPI:default"])
    _attr(sp, "deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray, Vt.TokenArray(["bindPose"]))
    _attr(sp, "deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray, pts)

    # --- simulation mesh (empty: PhysX auto-cooks it from the skin) -----------------------------
    sim = UsdGeom.Mesh.Define(st, "/Tissue/simMesh")
    sim.CreatePurposeAttr(UsdGeom.Tokens.guide)
    simp = sim.GetPrim()
    _apis(simp, ["OmniPhysicsSurfaceDeformableSimAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI",
                 "OmniPhysicsDeformablePoseAPI:default"])
    _attr(simp, "deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray, Vt.TokenArray(["bindPose"]))
    _attr(simp, "physxCollision:restOffset", Sdf.ValueTypeNames.Float, REST_OFFSET)
    _attr(simp, "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, CONTACT_OFFSET)

    # --- the deformable body root ----------------------------------------------------------------
    _apis(rp, ["PhysxAutoDeformableBodyAPI", "OmniPhysicsDeformableBodyAPI", "PhysxSurfaceDeformableBodyAPI"])
    rp.CreateRelationship("physxDeformableBody:cookingSourceMesh", custom=False).SetTargets([Sdf.Path("/Tissue/skin")])
    _attr(rp, "physxDeformableBody:selfCollision", Sdf.ValueTypeNames.Bool, True)
    _attr(rp, "physxDeformableBody:selfCollisionFilterDistance", Sdf.ValueTypeNames.Float, SELF_COLLISION_FILTER_DISTANCE)
    _attr(rp, "physxDeformableBody:enableSpeculativeCCD", Sdf.ValueTypeNames.Bool, True)
    _attr(rp, "physxDeformableBody:solverPositionIterationCount", Sdf.ValueTypeNames.UInt, 16)
    _attr(rp, "physxDeformableBody:collisionPairUpdateFrequency", Sdf.ValueTypeNames.UInt, 4)
    _attr(rp, "physxDeformableBody:collisionIterationMultiplier", Sdf.ValueTypeNames.UInt, 4)
    _attr(rp, "physxDeformableBody:maxLinearVelocity", Sdf.ValueTypeNames.Float, MAX_LINEAR_VELOCITY)

    # --- the deformable (physics) material ---------------------------------------------------------
    mat = UsdShade.Material.Define(st, "/Tissue/TissueMaterial")
    mp = mat.GetPrim()
    _apis(mp, ["OmniPhysicsDeformableMaterialAPI", "OmniPhysicsSurfaceDeformableMaterialAPI",
               "PhysxSurfaceDeformableMaterialAPI"])
    for name, value in (
        ("omniphysics:density", DENSITY),
        ("omniphysics:youngsModulus", YOUNGS_MODULUS),
        ("omniphysics:poissonsRatio", POISSONS_RATIO),
        ("omniphysics:staticFriction", STATIC_FRICTION),
        ("omniphysics:dynamicFriction", DYNAMIC_FRICTION),
        ("omniphysics:surfaceThickness", THICKNESS),
        ("omniphysics:surfaceStretchStiffness", STRETCH_STIFFNESS),
        ("omniphysics:surfaceShearStiffness", SHEAR_STIFFNESS),
        ("omniphysics:surfaceBendStiffness", BEND_STIFFNESS),
        ("physxDeformableMaterial:elasticityDamping", ELASTICITY_DAMPING),
        ("physxDeformableMaterial:bendDamping", BEND_DAMPING),
    ):
        _attr(mp, name, Sdf.ValueTypeNames.Float, value)
    UsdShade.MaterialBindingAPI.Apply(rp).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # --- the visual material -----------------------------------------------------------------------
    look = UsdShade.Material.Define(st, "/Tissue/TissueLook")
    sh = UsdShade.Shader.Define(st, "/Tissue/TissueLook/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.93, 0.92, 0.90))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    look.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(sp).Bind(look)

    st.GetRootLayer().Save()
    print("wrote", os.path.relpath(OUT), "-", len(pts), "vertices,", len(idx) // 3, "triangles")


if __name__ == "__main__":
    main()
