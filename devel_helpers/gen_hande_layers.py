#!/usr/bin/env python3
"""Generate the repetitive USD layers of the Robotiq Hand-E asset.

USD has no loops, so everything that repeats per fingertip / per side / per link is
written by this script from ONE list of fingertips and ONE set of constants. Adding a
fingertip means: bake its `parts/Robotiq_Hand_E_fingertip_<key>_{left,right}.usd`, add a
row to TIPS below, and re-run -- the variant payload, the variant entry in the root layer
and the PhysX collider are all derived from that row, so a tip cannot end up without a
collider.

Generated (all under grippers/Robotiq_Hand_E/):
  Robotiq_Hand_E.usda                       root asset (SimReady metadata is COMPUTED here)
  configuration/Robotiq_Hand_E_robot.usda   Isaac robot schema
  configuration/Robotiq_Hand_E_config_physics_physx.usda
  payloads/{base,geometries,instances,materials}.usda/.usd
  payloads/Robotiq_Hand_E_fingertip_<key>.usda          one per non-default tip
  payloads/Robotiq_Hand_E_fingertip_mount_inside.usda
  payloads/Robotiq_Hand_E_{body_mass,fingertip,kinematics,physx_common,physx}_physics.usda
  payloads/Robotiq_Hand_E_{newton_common,newton}_physics.usda

Not generated: `parts/` (baked from the CAD) and `materials/`.

Usage (needs the `pxr` USD python bindings, e.g. `usd-core`):
    python3 devel_helpers/gen_hande_layers.py
"""
import os

from pxr import Usd, UsdGeom, UsdPhysics

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "grippers", "Robotiq_Hand_E")
N = "Robotiq_Hand_E"
R = f"/{N}/{N}"

# ---------------------------------------------------------------- single source of truth
# (key, variant name, description). The first row is the default tip: its geometry is
# authored in base.usda; every other row gets a payload that swaps it in.
TIPS = [
    ("std", "Std", "standard Hand-E fingertip"),
    ("flat_overmolded", "Flat_Overmolded", "flat rubber overmolded fingertip"),
    ("support", "Support", "finger support / adapter plate (the plate the rubber and V-groove tips mount on)"),
    ("extender", "Extender", "finger extender"),
    ("binpick", "Bin_Picking", "bin-picking fingertip"),
]
# Body links that carry a PhysX collider on their /Meshes/<name>/mesh_0.
BODY_LINKS = ["base_link", "left_finger", "right_finger"]

# ---- Max payload / external force vs fingertip Z offset (Hand-E spec sheet, Fig. 6-11) ----
# The recommended maximum force F applied at the fingertip (middle of the inner pad surface) falls
# with the Z offset of that point (measured from the housing face), and depends on how the finger is
# mounted: "blue" = 2 x M3 directly on the rack, "red" = on a fingertip holder (2 x M3), "yellow" =
# 3 x M3 directly on the rack. Breakpoints (Z offset mm, N) were READ OFF the published figure
# (+-5 mm / +-5 N); the 100 N plateau, which is where every shipped tip sits, is exact.
FORCE_CAP = 100.0  # N, plateau of the spec-sheet graph
CURVES = {
    "blue": [(70, 100.0), (88, 69.0), (107, 46.0), (137, 22.0), (167, 8.0), (187, 0.0)],
    "red": [(70, 100.0), (88, 69.0), (104, 0.0)],
    "yellow": [(115, 100.0), (157, 57.0), (207, 45.0), (300, 20.0), (400, 0.0)],
}
# How each fingertip is mounted -> which curve limits it (confirmed by Robotiq).
TIP_MOUNT = {"std": "blue", "flat_overmolded": "blue", "support": "red", "extender": "red", "binpick": "yellow"}
HOUSING_FACE_Z = 0.0861  # m, housing face = CAD y = 0 (flange + 86.1 mm): the origin of the Z offset


def tip_z_offset(key):
    """Z offset (mm) of the fingertip's end from the housing face, from the baked CAD part."""
    st = Usd.Stage.Open(os.path.join(ROOT, "parts", f"{N}_fingertip_{key}_left.usd"))
    pts = UsdGeom.Mesh(st.GetPrimAtPath("/World/mesh_0")).GetPointsAttr().Get()
    return (max(p[2] for p in pts) - HOUSING_FACE_Z) * 1000.0


# ---- Finger drive ----
# Only left_finger_joint is driven; right_finger_joint follows it through a rigid coupling (PhysX mimic /
# Newton joint equality), so the left drive force is SHARED between the two fingers: each pad presses
# on the object with half of it (measured live: a 130 N drive cap gave ~65 N per pad). The spec-sheet
# force is a force at the pad, so the drive cap is twice the graph value.
DRIVE_TO_PAD = 2.0
# The drive is a position spring toward the closed target: force = stiffness x remaining stroke, clipped
# at the cap. With the old 5000 N/m an object leaving ~10 mm of stroke only got ~50 N and the cap was
# never reached. 50000 N/m saturates the cap for any object wider than a few mm, so a fully closed
# command always applies the cap force. Damping 200 keeps it overdamped for a ~40 g finger + tip.
DRIVE_STIFFNESS = 50000.0  # N/m
DRIVE_DAMPING = 200.0      # N.s/m


def tip_drive_force(key):
    """Drive force cap (N) that gives the tip's spec-sheet force at each pad."""
    return tip_max_force(key) * DRIVE_TO_PAD


def tip_max_force(key):
    """Max force (N) the spec-sheet curve of the tip's mounting allows at its Z offset."""
    z, pts = tip_z_offset(key), CURVES[TIP_MOUNT[key]]
    if z <= pts[0][0]:
        return min(FORCE_CAP, pts[0][1])
    for (z0, f0), (z1, f1) in zip(pts, pts[1:]):
        if z <= z1:
            return round(f0 + (f1 - f0) * (z - z0) / (z1 - z0), 1)
    return 0.0


INSIDE_OFFSET = 0.009  # m, Inside screw-hole row vs Outside (rows at 31 mm / 22 mm from the centre)
STROKE = 0.025  # m, travel of each finger
DENSITY = 2800.0  # kg/m^3 (aluminium); reproduces the published 0.864 kg body / 0.038 kg finger+tip
# Mass/CoM/box-inertia estimates from the baked CAD meshes (volume x DENSITY, bbox centre).
BASE = dict(mass=0.8633, com=(0, 0, 0.0472), dims=(0.075, 0.075, 0.1041))
STUB = dict(mass=0.01784, com=(-0.0141, -0.0139, 0.0925), dims=(0.0138, 0.0452, 0.0164))  # LEFT finger stub
TIP = dict(mass=0.0201, com=(-0.0041, -0.0271, 0.1229), dims=(0.0292, 0.0174, 0.0465))  # LEFT Std tip, shared by ALL tips
QCODE = "Q1340324"  # Wikidata Q-code SimReady SR.003 requires; same gripper concept as the 2F-85 / 2F-140
GRIP_FRAME_Z = 0.13  # m, provisional TCP between the pads


def mirror(v):
    """Right-side value of a left-side CoM: the right finger is the left turned 180 deg about Z."""
    return (-v[0], -v[1], v[2])


def box_inertia(m, d):
    return (m / 12 * (d[1] ** 2 + d[2] ** 2), m / 12 * (d[0] ** 2 + d[2] ** 2), m / 12 * (d[0] ** 2 + d[1] ** 2))


def fmt3(t):
    return "(%.9g, %.9g, %.9g)" % tuple(t)


def w(path, txt):
    with open(os.path.join(ROOT, path), "w") as f:
        f.write(txt)


HDR = "#usda 1.0\n"
IDQ = "(1, 0, 0, 0)"

# ---------------------------------------------------------------- base.usda & layout slots
def _link(name, mesh, extra=""):
    return f'''        def Xform "{name}" (
            kind = "group"
        )
        {{
            def Xform "visuals" (
                instanceable = true
                kind = "subcomponent"
                prepend references = </Meshes/{mesh}>
            )
            {{
            }}
{extra}        }}

'''


def gen_base():
    grip = f'''
            # Tool center point (grasp frame) on the gripper centerline, between the
            # fingertip pads. Refined against the pads during validation.
            def Xform "grip_frame"
            {{
                double3 xformOp:translate = (0, 0, {GRIP_FRAME_Z})
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }}
'''
    default = TIPS[0][0]
    tipgrp = lambda side: f'''    def Xform "fingertip_{side}" (
        kind = "group"
    )
    {{
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Xform "tip_{default}" (
            prepend references = @../parts/{N}_fingertip_{default}_{side}.usd@
        )
        {{
        }}
    }}
'''
    w("payloads/base.usda", HDR + f'''(
    # Kinematic body tree + geometry for the Robotiq Hand-E. Same layout as the 2F-140:
    # the visible body links live under /{N}/{N}, and the geometry is referenced from the
    # invisible /Meshes scope (one part per link). The per-link CAD parts are baked into
    # the final Isaac frame (approach +Z, fingers close along Y, base flange at z=0), so
    # every link Xform is identity.
    customLayerData = {{
        dictionary omni_layer = {{
            string authoring_layer = "./base.usda"
        }}
    }}
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./geometries.usd@,
        @./instances.usda@
    ]
    upAxis = "Z"
)

def Xform "{N}" (
    kind = "assembly"
)
{{
    def Xform "{N}" (
        kind = "assembly"
    )
    {{
        # DISP.001: every renderable GPrim must resolve a display color.
        color3f[] primvars:displayColor = [(0.1, 0.1, 0.1)]
        quatd xformOp:orient = {IDQ}
        double3 xformOp:scale = (1, 1, 1)
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

''' + _link("base_link", "base_link", grip) + _link("left_finger", "left_finger") + _link("right_finger", "right_finger")
      + _link("left_fingertip", "fingertip_left") + _link("right_fingertip", "fingertip_right").rstrip("\n") + f'''
    }}
}}

def Scope "Meshes"
{{
    token visibility = "invisible"

    def Xform "base_link" (
        prepend references = @../parts/{N}_base.usd@
    )
    {{
    }}

    def Xform "left_finger" (
        prepend references = @../parts/{N}_finger_left.usd@
    )
    {{
    }}

    def Xform "right_finger" (
        prepend references = @../parts/{N}_finger_right.usd@
    )
    {{
    }}

    # Fingertip geometry groups. The active tip child is chosen by the "Fingertip"
    # variantSet ("tip_{default}" is the default, authored here; the other variants
    # deactivate it and add their own tip child). The group's translate is the
    # "FingertipMount" variantSet's hook: Outside (default, 0) / Inside (tip moved one hole
    # row towards the gripper centerline).
{tipgrp("left")}
{tipgrp("right")}}}
''')
    w("payloads/geometries.usd", HDR + '''# Canonical SimReady "geometries" layer (ISA.001 layout slot). The render/collision
# geometry is authored as the invisible `/Meshes` scope inside ./base.usda; this layer
# is intentionally empty so it does not alter that authoring.
(
    metersPerUnit = 1
    upAxis = "Z"
)
''')
    w("payloads/instances.usda", HDR + '''# Canonical SimReady "instances" layer (ISA.001 layout slot). Instancing is authored
# inline in ./base.usda (each link's `visuals` child is instanceable); this layer is
# intentionally empty of overrides.
(
    metersPerUnit = 1
    upAxis = "Z"
)
''')
    w("payloads/materials.usda", HDR + '''# Canonical SimReady "materials" payload layer: sublayers the shared material library.
(
    defaultPrim = "World"
    metersPerUnit = 1
    subLayers = [
        @../materials/materials.usd@
    ]
    upAxis = "Z"
)
''')


# ---------------------------------------------------------------- variant payloads
def gen_tip_payloads():
    default = TIPS[0][0]
    for key, var, desc in TIPS[1:]:
        t = ""
        for side in ("left", "right"):
            t += f'''    over "fingertip_{side}"
    {{
        over "tip_{default}" (
            active = false
        )
        {{
        }}

        def Xform "tip_{key}" (
            instanceable = false
            kind = "subcomponent"
            prepend references = @../parts/{N}_fingertip_{key}_{side}.usd@
        )
        {{
        }}
    }}

'''
        w(f"payloads/{N}_fingertip_{key}.usda", HDR + f'''(
    # "{var}" fingertip variant: deactivate the default tip and reference the {desc}
    # parts (baked into the Isaac frame at the OUTSIDE hole row). Its collider is authored
    # (inert) in {N}_physx_common_physics.usda, so it composes only under a PhysX Physics
    # variant. Tip mass is the shared value from fingertip_physics.
    # GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./base.usda@
    ]
    upAxis = "Z"
)

over "Meshes"
{{
{t.rstrip()}
}}

# Drive force cap for this tip: twice the spec-sheet force at the pad (the two fingers share the
# left joint's drive force), see CURVES / DRIVE_TO_PAD in the generator.
over "{N}"
{{
    over "{N}"
    {{
        over "Joints"
        {{
            over "left_finger_joint"
            {{
                float drive:linear:physics:maxForce = {tip_drive_force(key):g}
            }}
        }}
    }}
}}
''')
    mm = INSIDE_OFFSET * 1000
    w(f"payloads/{N}_fingertip_mount_inside.usda", HDR + f'''(
    # "Inside" FingertipMount variant: the fingertips are screwed to the inner hole row of
    # the finger instead of the outer one -- {mm:g} mm towards the gripper centerline. The left
    # fingertip moves +Y, the right one -Y; geometry and tip collider both follow (the
    # collider is part of the instanced fingertip geometry). The tip rigid body's centre of
    # mass is not shifted ({mm:g} mm on a ~{TIP["mass"] * 1000:.0f} g body).
    # GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./base.usda@
    ]
    upAxis = "Z"
)

over "Meshes"
{{
    over "fingertip_left"
    {{
        double3 xformOp:translate = (0, {INSIDE_OFFSET:g}, 0)
    }}

    over "fingertip_right"
    {{
        double3 xformOp:translate = (0, {-INSIDE_OFFSET:g}, 0)
    }}
}}
''')


# ---------------------------------------------------------------- physics layers
def gen_body_mass():
    def link(name, d, com, note=""):
        return f'''        {note}over "{name}" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
        )
        {{
            point3f physics:centerOfMass = {fmt3(com)}
            float3 physics:diagonalInertia = {fmt3(box_inertia(d["mass"], d["dims"]))}
            float physics:mass = {d["mass"]}
            quatf physics:principalAxes = {IDQ}
        }}
'''
    note = "# Mirrors left_finger: same mass/inertia, CoM x/y negated.\n        "
    right_finger = link("right_finger", STUB, mirror(STUB["com"]), note).rstrip()
    w(f"payloads/{N}_body_mass_physics.usda", HDR + f'''(
    # Shared rigid-body mass for the three Hand-E body links (base + two sliding finger
    # stubs). Engine-neutral (UsdPhysics only), sublayered by the PhysX variant.
    #
    # Every body link carries its collider in the shared invisible /Meshes scope, so PhysX
    # cannot auto-compute mass -- it is authored explicitly. Values are derived from the
    # baked CAD meshes: mass = {DENSITY:.0f} kg/m^3 (aluminium) x mesh volume (this reproduces
    # the published 0.864 kg body / 0.038 kg finger+tip figures), centre of mass = bounding-box
    # centre, inertia = solid-box model on the mesh bounding box.
    # GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    upAxis = "Z"
)

over "{N}"
{{
    over "{N}"
    {{
{link("base_link", BASE, BASE["com"])}
{link("left_finger", STUB, STUB["com"])}
{right_finger}
    }}
}}
''')


def gen_fingertip_physics():
    def tip(side, com, note=""):
        return f'''        {note}over "{side}_fingertip" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
            kind = "group"
        )
        {{
            point3f physics:centerOfMass = {fmt3(com)}
            float3 physics:diagonalInertia = {fmt3(box_inertia(TIP["mass"], TIP["dims"]))}
            float physics:mass = {TIP["mass"]}
            quatf physics:principalAxes = {IDQ}

            def PhysicsFixedJoint "FixedJoint"
            {{
                rel physics:body0 = <{R}/{side}_finger>
                rel physics:body1 = <{R}/{side}_fingertip>
                float physics:breakForce = 3.4028235e38
                float physics:breakTorque = 3.4028235e38
                point3f physics:localPos0 = (0, 0, 0)
                point3f physics:localPos1 = (0, 0, 0)
                quatf physics:localRot0 = {IDQ}
                quatf physics:localRot1 = {IDQ}
            }}
        }}
'''
    w(f"payloads/{N}_fingertip_physics.usda", HDR + f'''(
    # Engine-neutral fingertip physics for the Hand-E (UsdPhysics only): the screwed-on
    # fingertip rigid bodies and their PhysicsFixedJoints (welded to the finger stub, rigid
    # like a screw). Sublayered by every Physics variant.
    #
    # Mass/CoM/inertia are shared by ALL Fingertip variants (estimates from the Std tip:
    # {DENSITY:.0f} kg/m^3 x CAD volume = {TIP["mass"] * 1000:.0f} g, CoM = bounding-box centre, box inertia). The
    # other tips differ by a few grams (Support ~6 g ... Flat_Overmolded ~26 g); refine per
    # tip if grasp dynamics ever need it. The right side mirrors the left (CoM x/y negated).
    # This layer only composes under a Physics variant, never under Physics = "None".
    # GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./base.usda@
    ]
    upAxis = "Z"
)

over "{N}"
{{
    over "{N}"
    {{
{tip("left", TIP["com"])}
{tip("right", mirror(TIP["com"]))}    }}
}}
''')


def gen_kinematics():
    def joint(name, rot):
        side = name.split("_")[0]
        return f'''            def PhysicsPrismaticJoint "{name}" (
                prepend apiSchemas = ["PhysicsJointStateAPI:linear"]
            )
            {{
                uniform token physics:axis = "Y"
                rel physics:body0 = <{R}/base_link>
                rel physics:body1 = <{R}/{side}_finger>
                point3f physics:localPos0 = (0, 0, 0)
                point3f physics:localPos1 = (0, 0, 0)
                quatf physics:localRot0 = {rot}
                quatf physics:localRot1 = {rot}
                float physics:lowerLimit = 0
                float physics:upperLimit = {STROKE:g}
                float state:linear:physics:position = 0
                float state:linear:physics:velocity = 0
            }}
'''
    w(f"payloads/{N}_kinematics_physics.usda", HDR + f'''(
    defaultPrim = "{N}"
    doc = """Shared, engine-NEUTRAL kinematic tree for the Robotiq Hand-E.

Two prismatic joints (base_link -> left_finger / right_finger), each with a {STROKE * 1000:g} mm
stroke ({2 * STROKE * 1000:g} mm total opening). Link frames are identity/world, so the joint frames
are the world origin. The left joint axis is +Y (closing = +position). The right joint
frame is turned 180 deg about Z so its +Y axis points -Y: closing is also +position on
both joints and the mimic coupling is a plain gearing.

Neutrality rule (SimReady RV.011): only UsdPhysics + PhysicsJointStateAPI here -- no
Physx*/Mjc*/Newton* schemas, no drives/mimics/armature.
GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
"""
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{N}"
{{
    def Xform "{N}"
    {{
        def Scope "Joints"
        {{
{joint("left_finger_joint", IDQ)}
{joint("right_finger_joint", "(0, 0, 0, 1)").rstrip()}
        }}
    }}
}}
''')


COLLIDER_APIS = '["PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]'


def _collider(name, depth=1):
    """`over` block putting the convex-hull collider on <name>/mesh_0 (name may be a nested path)."""
    ind = "    "
    parts = name.split("/")
    out = ""
    for i, p in enumerate(parts):
        out += f'{ind * (depth + i)}over "{p}"\n{ind * (depth + i)}{{\n'
    d = depth + len(parts)
    out += f'''{ind * d}over "mesh_0" (
{ind * d}    prepend apiSchemas = {COLLIDER_APIS}
{ind * d})
{ind * d}{{
{ind * d}    uniform token physics:approximation = "convexHull"
{ind * d}    bool physics:collisionEnabled = 1
{ind * d}}}
'''
    for i in reversed(range(len(parts))):
        out += f"{ind * (depth + i)}}}\n"
    return out


def gen_physx_common():
    colliders = "".join(_collider(l) + "\n" for l in BODY_LINKS)
    colliders += "    # Tip colliders for every Fingertip variant (derived from TIPS; authored here, not in the\n"
    colliders += '    # variant payloads, so no tip collider ever composes under Physics = "None").\n'
    for side in ("left", "right"):
        colliders += f'    over "fingertip_{side}"\n    {{\n'
        colliders += "\n".join(_collider(f"tip_{k}", depth=2) for k, _, _ in TIPS)
        colliders += "    }\n\n"
    w(f"payloads/{N}_physx_common_physics.usda", HDR + f'''(
    # PhysX-only physics for the Hand-E: the articulation root + root_joint (fixed base), the
    # collision-group plumbing, and every PhysX convex-hull collider -- the body-link
    # colliders and the tip collider of each Fingertip variant. Everything here carries
    # Physx* schemas. Only the tip collider of the selected "Fingertip" variant exists in the
    # composed stage; the other tip overs are inert. None of this composes under
    # Physics = "None".
    # GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./base.usda@
    ]
    upAxis = "Z"
)

def Xform "{N}" (
    kind = "assembly"
)
{{
    over Xform "{N}" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
        kind = "assembly"
    )
    {{
        bool physxArticulation:enabledSelfCollisions = 0

        # Root/mount joint pinning the gripper base to the world (fixed base).
        def PhysicsFixedJoint "root_joint"
        {{
            rel physics:body1 = <{R}/base_link>
            point3f physics:localPos0 = (0, 0, 0)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot0 = {IDQ}
            quatf physics:localRot1 = {IDQ}
        }}
    }}
}}

# Collision-group plumbing: keep the shared /Meshes colliders from colliding with the
# articulation-body copies (same pattern as the 2F-85 / 2F-140).
over "Meshes"
{{
    def PhysicsCollisionGroup "CollisionGroup"
    {{
        rel collection:colliders:includes = </Meshes>
        rel physics:filteredGroups
    }}

    def PhysicsCollisionGroup "CollisionGroup_01"
    {{
        rel collection:colliders:includes = </{N}>
        rel physics:filteredGroups = </Meshes/CollisionGroup>
    }}

{colliders.rstrip()}
}}
''')


def gen_physx():
    w(f"payloads/{N}_physx_physics.usda", HDR + f'''(
    # PhysX variant of the Hand-E: left_finger_joint is the single driven DOF (linear position
    # drive); right_finger_joint mimics it (rigid, gearing -1 against the mirrored joint
    # frame -> both fingers close symmetrically). Shared neutral joint frames/limits live in
    # ./{N}_kinematics_physics.usda (sublayered below).
    #
    # Drive: the force cap is twice the spec-sheet payload-vs-Z-offset force of the default tip
    # ({tip_max_force(TIPS[0][0]):g} N at each pad -> {tip_drive_force(TIPS[0][0]):g} N on the driven joint, because the
    # rigidly coupled right finger shares it; each Fingertip variant overrides the cap for its own tip),
    # 0.15 m/s max speed (20-150 mm/s published). Stiffness {DRIVE_STIFFNESS:g} N/m saturates the cap for
    # any object, damping {DRIVE_DAMPING:g}; both are checked live on PhysX and still to be tuned on Newton.
    # GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @./{N}_physx_common_physics.usda@,
        @./{N}_kinematics_physics.usda@,
        @./{N}_body_mass_physics.usda@,
        @./{N}_fingertip_physics.usda@
    ]
)

def Xform "{N}" (
    kind = "assembly"
)
{{
    over Xform "{N}" (
        kind = "assembly"
    )
    {{
        int physxArticulation:solverPositionIterationCount = 64

        over "Joints"
        {{
            over "left_finger_joint" (
                prepend apiSchemas = ["PhysxJointAPI", "PhysicsDriveAPI:linear"]
            )
            {{
                float drive:linear:physics:damping = {DRIVE_DAMPING:g}
                float drive:linear:physics:maxForce = {tip_drive_force(TIPS[0][0]):g}
                float drive:linear:physics:stiffness = {DRIVE_STIFFNESS:g}
                float drive:linear:physics:targetPosition = 0
                float drive:linear:physics:targetVelocity = 0
                float physxJoint:armature = 0.001
                float physxJoint:maxJointVelocity = 0.15
            }}

            over "right_finger_joint" (
                prepend apiSchemas = ["PhysxJointAPI", "PhysxMimicJointAPI:rotX"]
            )
            {{
                float physxJoint:armature = 0.001
                float physxJoint:maxJointVelocity = 10000
                float physxMimicJoint:rotX:dampingRatio = 0
                float physxMimicJoint:rotX:gearing = -1
                float physxMimicJoint:rotX:naturalFrequency = 0
                rel physxMimicJoint:rotX:referenceJoint = <{R}/Joints/left_finger_joint>
            }}
        }}
    }}
}}
''')


# ---------------------------------------------------------------- Newton (MuJoCo-Warp)
NEWTON_COLLIDER_APIS = '["PhysicsCollisionAPI", "NewtonCollisionAPI", "PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]'
GRIP_MATERIAL = f"{R}/Physics/GripMaterial"
SOLIMP = "[0.95, 0.99, 0.001, 0.5, 2]"


def _newton_over(path, body, depth=1):
    """`over` chain `path` ending in an `over "mesh_0"` whose collider block is `body`."""
    ind = "    "
    parts = path.split("/")
    out = ""
    for i, p in enumerate(parts):
        out += f'{ind * (depth + i)}over "{p}"\n{ind * (depth + i)}{{\n'
    d = depth + len(parts)
    out += body(ind * d)
    for i in reversed(range(len(parts))):
        out += f"{ind * (depth + i)}}}\n"
    return out


def _tip_blocks(body):
    """Per-side `over "fingertip_<side>"` wrapper holding one tip `over` each (wrapper written once)."""
    out = ""
    for s in ("left", "right"):
        out += f'    over "fingertip_{s}"\n    {{\n'
        out += "\n".join(_newton_over(f"tip_{k}", body, depth=2) for k, _, _ in TIPS)
        out += "    }\n\n"
    return out


def gen_newton_common():
    link = lambda i: f'''{i}over "mesh_0" (
{i}    prepend apiSchemas = {NEWTON_COLLIDER_APIS}
{i})
{i}{{
{i}    uniform token physics:approximation = "convexHull"
{i}    bool physics:collisionEnabled = 1
{i}    float newton:contactGap = 0
{i}}}
'''
    tip = lambda i: f'''{i}over "mesh_0" (
{i}    prepend apiSchemas = {NEWTON_COLLIDER_APIS[:-1]}, "MaterialBindingAPI"]
{i})
{i}{{
{i}    uniform token physics:approximation = "convexDecomposition"
{i}    bool physics:collisionEnabled = 1
{i}    rel material:binding:physics = <{GRIP_MATERIAL}>
{i}    float newton:contactGap = 0
{i}}}
'''
    cols = "\n".join(_newton_over(l, link) for l in BODY_LINKS) + "\n"
    cols += "    # Tip colliders for every Fingertip variant (derived from TIPS); only the one the\n"
    cols += "    # Fingertip variant activates composes. Fingertips carry the grip-pad friction material.\n"
    cols += _tip_blocks(tip)
    w(f"payloads/{N}_newton_common_physics.usda", HDR + f'''(
    defaultPrim = "{N}"
    doc = """Shared Newton-pure physics for the Robotiq Hand-E: the grip-pad friction material
and the per-mesh convex-hull colliders (+ pad material binding on every fingertip).
Sublayered by the Newton variant payload, which adds the Mjc* solver tuning on top (this
layer is Newton-pure: only UsdPhysics + Newton* schemas, no Physx*/Mjc*, RV.011).

Link mass / CoM / inertia come from the shared ./{N}_body_mass_physics.usda and the
fingertip bodies + welds from ./{N}_fingertip_physics.usda (both engine-neutral, shared with
the PhysX variant), not re-authored here.
GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand.
"""
    metersPerUnit = 1
    upAxis = "Z"
)

over "{N}"
{{
    over "{N}"
    {{
        # Fingertip pad friction. The Newton variant adds MjcMaterialAPI.
        def Scope "Physics"
        {{
            def Material "GripMaterial" (
                prepend apiSchemas = ["PhysicsMaterialAPI", "NewtonMaterialAPI"]
            )
            {{
                float physics:dynamicFriction = 0.7
                float physics:staticFriction = 0.7
                float newton:rollingFriction = 0.0001
                float newton:torsionalFriction = 0.005
            }}
        }}
    }}
}}

# Convex-hull colliders cooked from the shared visual CAD meshes (the baked mesh_0 in /Meshes).
over "Meshes"
{{
{cols.rstrip()}
}}
''')


def gen_newton():
    link_mjc = lambda i: f'''{i}over "mesh_0" (
{i}    prepend apiSchemas = ["MjcCollisionAPI"]
{i})
{i}{{
{i}    uniform int mjc:group = 2
{i}}}
'''
    base_mjc = lambda i: f'''{i}over "mesh_0" (
{i}    prepend apiSchemas = ["MjcCollisionAPI"]
{i})
{i}{{
{i}    uniform int mjc:condim = 3
{i}    uniform double mjc:gap = 0
{i}    uniform int mjc:group = 2
{i}    uniform double mjc:margin = 0
{i}    uniform int mjc:priority = 1
{i}    uniform double[] mjc:solimp = {SOLIMP}
{i}    uniform double mjc:solmix = 1
{i}    uniform double[] mjc:solref = [0.004, 2]
{i}}}
'''
    tip_mjc = lambda i: f'''{i}over "mesh_0" (
{i}    prepend apiSchemas = ["MjcCollisionAPI"]
{i})
{i}{{
{i}    uniform int mjc:group = 3
{i}    uniform int mjc:priority = 1
{i}    uniform double[] mjc:solimp = {SOLIMP}
{i}    uniform double[] mjc:solref = [0.004, 2]
{i}}}
'''
    cols = _newton_over("base_link", base_mjc) + "\n"
    cols += "\n".join(_newton_over(l, link_mjc) for l in BODY_LINKS[1:]) + "\n"
    cols += _tip_blocks(tip_mjc)
    w(f"payloads/{N}_newton_physics.usda", HDR + f'''(
    defaultPrim = "{N}"
    doc = """Newton (MuJoCo-Warp) variant of the Robotiq Hand-E.

The Hand-E has no linkage: two sliding fingers on prismatic joints. left_finger_joint is the
single driven DOF (linear drive); right_finger_joint follows it through a stiff MuJoCo joint
equality (MjcEqualityJointAPI: right = left, both joints close with +position -- see the
kinematics layer). Same fixed-base root_joint + articulation root as the
PhysX variant.

Layers: the Newton-pure colliders / pad material (./{N}_newton_common_physics.usda), the
shared link masses (./{N}_body_mass_physics.usda), the engine-neutral fingertip bodies +
welds (./{N}_fingertip_physics.usda) and the neutral joint frames/limits
(./{N}_kinematics_physics.usda). This layer adds the Mjc* solver tuning for both. Drive gains
and MuJoCo tuning are initial values that need live Newton tuning.

RV.011: Newton + Mjc schemas only -- no Physx* runtime data here.
GENERATED by devel_helpers/gen_hande_layers.py -- do not edit by hand."""
    metersPerUnit = 1
    subLayers = [
        @./{N}_newton_common_physics.usda@,
        @./{N}_body_mass_physics.usda@,
        @./{N}_fingertip_physics.usda@,
        @./{N}_kinematics_physics.usda@,
        @./base.usda@
    ]
    upAxis = "Z"
)

over "{N}"
{{
    over "{N}" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI", "NewtonArticulationRootAPI"]
    )
    {{
        # Same as the PhysX variant's physxArticulation:enabledSelfCollisions = 0: no
        # collisions between the gripper's own bodies (the L-shaped fingertips of the two
        # fingers would otherwise block full closure).
        bool newton:selfCollisionEnabled = 0

        # World-pinned root joint (fixed base). Same name/path as the PhysX variant, so a
        # mounted-gripper override `over "root_joint" (active = false)` applies to both.
        def PhysicsFixedJoint "root_joint"
        {{
            rel physics:body1 = <{R}/base_link>
            point3f physics:localPos0 = (0, 0, 0)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot0 = {IDQ}
            quatf physics:localRot1 = {IDQ}
        }}

        over "Joints"
        {{
            over "left_finger_joint" (
                prepend apiSchemas = ["MjcJointAPI", "PhysicsDriveAPI:linear"]
            )
            {{
                float drive:linear:physics:damping = {DRIVE_DAMPING:g}
                float drive:linear:physics:maxForce = {tip_drive_force(TIPS[0][0]):g}
                float drive:linear:physics:stiffness = {DRIVE_STIFFNESS:g}
                float drive:linear:physics:targetPosition = 0
                uniform double mjc:armature = 0.001
                uniform double mjc:damping = 1
                uniform double[] mjc:solimplimit = {SOLIMP}
                uniform double[] mjc:solreflimit = [0.02, 2]
            }}

            # Rigid coupling: a MuJoCo joint equality (right = left) authored with a stiff
            # solref/solimp. NewtonMimicAPI would give the same constraint but with MuJoCo's
            # default softness (not authorable), which a loaded grasp overpowers: the right
            # finger gets pushed back by the object while the left one keeps closing.
            over "right_finger_joint" (
                prepend apiSchemas = ["MjcJointAPI", "MjcEqualityJointAPI"]
            )
            {{
                uniform double mjc:armature = 0.001
                uniform double mjc:coef0 = 0
                uniform double mjc:coef1 = 1
                uniform double mjc:damping = 1
                uniform double[] mjc:solimp = [0.99, 0.999, 0.0001, 0.5, 2]
                uniform double[] mjc:solimplimit = {SOLIMP}
                uniform double[] mjc:solref = [0.002, 1]
                uniform double[] mjc:solreflimit = [0.02, 2]
                rel mjc:target = <{R}/Joints/left_finger_joint>
            }}
        }}

        # MuJoCo pad-material tuning on top of the shared Newton GripMaterial.
        over "Physics"
        {{
            over "GripMaterial" (
                prepend apiSchemas = ["MjcMaterialAPI"]
            )
            {{
            }}
        }}
    }}
}}

# MuJoCo per-mesh solver tuning on top of the shared Newton convex-hull colliders:
# base_link: priority=1 + soft solref; fingers: group only; every fingertip type: the grip surface.
over "Meshes"
{{
{cols.rstrip()}
}}
''')


# ---------------------------------------------------------------- configuration + root
def gen_configuration():
    links = ["base_link", "left_finger", "right_finger", "left_fingertip", "right_fingertip"]
    site = '''            string isaac:nameOverride (
                displayName = "Name Override"
                doc = "Name override for prim lookup in base name search"
            )

            over "grip_frame" (
                prepend apiSchemas = ["IsaacSiteAPI"]
            )
            {
                string isaac:Description = "Tool center point (grasp frame) centered between the fingertips." (
                    displayName = "Reference Description"
                )
                uniform token isaac:forwardAxis = "Z" (
                    displayName = "Forward Axis"
                )
            }
'''
    lk = lambda n, extra="": f'''        over "{n}" (
            prepend apiSchemas = ["IsaacLinkAPI"]
        )
        {{
{extra}        }}

'''
    fj = lambda n: f'''            over "{n}" (
                prepend apiSchemas = ["IsaacJointAPI"]
            )
            {{
            }}
'''
    w(f"configuration/{N}_robot.usda", HDR + f'''(
    customLayerData = {{
        dictionary omni_layer = {{
            string authoring_layer = "../{N}.usda"
        }}
    }}
    defaultPrim = "World"
    endTimeCode = 0
    metersPerUnit = 1
    startTimeCode = -1
    upAxis = "Z"
)

over "{N}" (
    prepend apiSchemas = ["IsaacRobotAPI"]
)
{{
    string[] isaac:changelog (
        displayName = "Changelog"
    )
    string isaac:description = "Robotiq Hand-E adaptive parallel-jaw gripper (2 sliding fingers, {2 * STROKE * 1000:g} mm stroke)." (
        displayName = "Description"
    )
    token isaac:license (
        displayName = "License"
    )
    string isaac:namespace (
        displayName = "Namespace"
        doc = "Namespace of the prim in Isaac Sim"
    )
    rel isaac:physics:robotJoints (
        displayName = "Robot Joints"
    )
    rel isaac:physics:robotLinks (
        displayName = "Robot Links"
    )
    prepend rel isaac:physics:robotLinks = [
''' + "".join(f"        <{R}/{l}>,\n" for l in links) + f'''    ]
    token isaac:robotType = "End Effector" (
        displayName = "Robot Type"
    )
    string isaac:source = "Robotiq" (
        displayName = "Source"
    )
    string isaac:version = "1.0.0" (
        displayName = "Version"
    )

    # robotJoints is variant-specific (authored per Physics variant); fingertip welds are
    # not listed, as on the 2F-85 / 2F-140. robotLinks stays shared: the bodies are the same
    # across variants.
    variantSet "Physics" = {{
        "PhysX" {{
            prepend rel isaac:physics:robotJoints = [
                <{R}/root_joint>,
                <{R}/Joints/left_finger_joint>,
                <{R}/Joints/right_finger_joint>,
            ]
        }}
        "Newton" {{
            prepend rel isaac:physics:robotJoints = [
                <{R}/root_joint>,
                <{R}/Joints/left_finger_joint>,
                <{R}/Joints/right_finger_joint>,
            ]
        }}
    }}

    over "{N}"
    {{
        string isaac:namespace (
            displayName = "Namespace"
            doc = "Namespace of the prim in Isaac Sim"
        )

''' + lk("base_link", site) + "".join(lk(l) for l in links[1:]) + f'''        over "Joints"
        {{
{fj("left_finger_joint")}
{fj("right_finger_joint")}        }}
    }}
}}
''')
    # Integration-path layer: just the PhysX variant stack + the base tree (no editor
    # Render prims / lights -- those do not belong in a gripper asset).
    w(f"configuration/{N}_config_physics_physx.usda", HDR + f'''(
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @../payloads/{N}_physx_physics.usda@,
        @../payloads/base.usda@
    ]
    upAxis = "Z"
)
''')


def gen_root(meta):
    variants = "".join(
        f'''        "{var}" (
            prepend payload = @./payloads/{N}_fingertip_{key}.usda@
        ) {{

        }}
''' for key, var, _ in TIPS[1:])
    w(f"{N}.usda", HDR + f'''(
    customLayerData = {{
        # SR.001 requires these identity fields authored DIRECTLY in the root layer's
        # customLayerData (not only nested in SimReady_Metadata). They mirror the matching
        # SimReady_Metadata entries below.
        string asset_name = "{N}"
        string asset_type = "gripper"
        string source_file = "ROBOTIQ_HAND-E_NO_FINGERTIPS_20190924.step"
        string usd_date_generated = "2026-10-07"
        # SimReady provenance metadata (SimReady Foundations SR.003 / FET_033_STANDARD).
        # rigid_body_count / mass / asset_extents are COMPUTED from the default composed stage
        # by devel_helpers/gen_hande_layers.py -- re-run it instead of editing them by hand.
        dictionary SimReady_Metadata = {{
            string asset_name = "{N}"
            string asset_type = "gripper"
            string category = "robot_gripper"
            string author = "Robotiq"
            string asset_license = "CC-BY-4.0"
            string source_file = "ROBOTIQ_HAND-E_NO_FINGERTIPS_20190924.step"
            string usd_date_generated = "2026-10-07"
            string qcode = "{QCODE}"
            int rigid_body_count = {meta["bodies"]}
            float3 asset_extents = ({meta["ext"][0]:.4f}, {meta["ext"][1]:.4f}, {meta["ext"][2]:.4f})
            float mass = {meta["mass"]:.4f}
        }}
    }}
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./configuration/{N}_robot.usda@
    ]
    upAxis = "Z"
)

def Xform "{N}" (
    kind = "component"
    prepend references = @./payloads/base.usda@
    variants = {{
        string FingertipMount = "Outside"
        string Fingertip = "{TIPS[0][1]}"
        string Physics = "PhysX"
    }}
    # FingertipMount is listed first: it must be stronger than Fingertip, whose payloads
    # re-sublayer base.usda (translate = 0) and would otherwise cancel the Inside offset.
    prepend variantSets = ["FingertipMount", "Fingertip", "Physics"]
)
{{
    string isaac:namespace (
        displayName = "Namespace"
        doc = "Namespace of the prim in Isaac Sim"
    )

    over "{N}"
    {{
        string isaac:namespace (
            displayName = "Namespace"
            doc = "Namespace of the prim in Isaac Sim"
        )
    }}

    variantSet "Physics" = {{
        "None" (
            customData = {{
                string[] variantPrimPaths = ["{N}/Joints"]
            }}
        ) {{
            over "{N}"
            {{
                over "Joints" (
                    active = false
                )
                {{
                }}
            }}

        }}
        "PhysX" (
            customData = {{
                string[] variantPrimPaths = ["."]
            }}
            prepend payload = @./payloads/{N}_physx_physics.usda@
        ) {{

        }}
        "Newton" (
            customData = {{
                string[] variantPrimPaths = ["."]
            }}
            prepend payload = @./payloads/{N}_newton_physics.usda@
        ) {{

        }}
    }}
    # Fingertip screw-hole row: "Outside" (default, the authored position) or "Inside" (every
    # fingertip moved {INSIDE_OFFSET * 1000:g} mm towards the gripper centerline).
    variantSet "FingertipMount" = {{
        "Outside" {{

        }}
        "Inside" (
            prepend payload = @./payloads/{N}_fingertip_mount_inside.usda@
        ) {{

        }}
    }}
    variantSet "Fingertip" = {{
        "{TIPS[0][1]}" {{

        }}
{variants}    }}
}}
''')


def compute_metadata():
    st = Usd.Stage.Open(os.path.join(ROOT, f"{N}.usda"))
    bodies, mass = 0, 0.0
    for p in st.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies += 1
            mass += p.GetAttribute("physics:mass").Get() or 0.0
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(st.GetDefaultPrim()).ComputeAlignedRange()
    return dict(bodies=bodies, mass=mass, ext=tuple(r.GetSize()))


if __name__ == "__main__":
    gen_base()
    gen_tip_payloads()
    gen_body_mass()
    gen_fingertip_physics()
    gen_kinematics()
    gen_physx_common()
    gen_physx()
    gen_newton_common()
    gen_newton()
    gen_configuration()
    gen_root(dict(bodies=0, mass=0.0, ext=(0, 0, 0)))  # first pass so the stage composes
    meta = compute_metadata()
    gen_root(meta)
    print("SimReady metadata:", meta)
