#!/usr/bin/env python3
"""Generate the repetitive USD layers of the Robotiq Hand-E C10 asset.

The C10 has three concentric fingers, 120 degrees apart, that slide radially toward the gripper axis: three
prismatic joints, one driven (finger_1_joint) and two that follow it. USD has no loops, so everything that
repeats per finger is written by this script from ONE list of fingers and ONE set of constants (same approach
as gen_hande_layers.py for the two-finger Hand-E).

Generated (all under grippers/Robotiq_Hand_E_C10/):
  Robotiq_Hand_E_C10.usda                       root asset (SimReady metadata is COMPUTED here)
  configuration/Robotiq_Hand_E_C10_robot.usda   Isaac robot schema
  configuration/Robotiq_Hand_E_C10_config_physics_physx.usda
  payloads/{base,geometries,instances,materials}.usda
  payloads/Robotiq_Hand_E_C10_{body_mass,fingertip,kinematics,physx_common,physx}_physics.usda
  payloads/Robotiq_Hand_E_C10_{newton_common,newton}_physics.usda

Not generated: `parts/` (baked from the CAD by gen_hande_c10_parts.py and gen_hande_c10_fingertip_colliders.py) and
`materials/` (a copy of the shared library).

Usage (needs the `pxr` USD python bindings, e.g. `usd-core`, and `numpy`):
    python3 devel_helpers/gen_hande_c10_layers.py
"""
import math
import os

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "grippers", "Robotiq_Hand_E_C10")
N = "Robotiq_Hand_E_C10"
R = f"/{N}/{N}"

# ---------------------------------------------------------------- single source of truth
FINGERS = (1, 2, 3)  # finger k sits at 120 * (k - 1) degrees about Z from +X
# (key, variant name, description). The first row is the default tip: its geometry is authored in base.usda; every
# other row gets a payload that swaps it in. The long and short spacers are not here yet: they still have to be placed.
TIPS = [
    ("stepped", "Stepped", "stepped fingertip"),
    ("extender", "Extender", "finger extender (FINGER_EXTENDER_C10, D-041); the stepped tip stays, mounted on it"),
]
BODY_LINKS = ["base_link"] + [f"finger_{k}" for k in FINGERS]  # links that carry a collider on their /Meshes/<name>/mesh_0

# ---- Datasheet (Hand-E C10) ----
STROKE = 0.0192  # m, travel of each finger, from the CAD's open and closed assemblies (51.3 -> 32.1 mm from the axis)
FORCE_PER_FINGER = 175.0  # N, datasheet maximum grip force per finger
FINGER_SPEED = 0.0535  # m/s per finger: half the datasheet's 107 mm/s closing speed, read as the diameter's
GRIPPER_MASS = 1.7  # kg, datasheet weight
MAX_PAYLOAD = 10.0  # kg, datasheet payload
MAX_OPENING = 0.0404  # m, pad-to-pad diameter at full open of the default tip: its inner face is 20.2 mm from the axis
PAD_WIDTH = 0.015  # m, fingertip pad width (tangential), from the baked tip
GRIP_FRAME_Z = 0.1343  # m, provisional TCP: the middle of the fingertips' length
QCODE = "Q1340324"  # Wikidata Q-code SimReady SR.003 requires; same gripper concept as the Hand-E / 2F-85 / 2F-140

# ---- Finger drive ----
# Only finger_1_joint is driven; the other two follow it through a rigid coupling (PhysX mimic / Newton joint equality),
# so the driven joint's force is SHARED by the three fingers: each pad presses with a third of it. The cap is therefore
# three times the force at one pad.
DRIVE_TO_PAD = 3.0
# The drive force is stiffness x the remaining stroke up to the cap, and a finger stopped by an object of diameter d is left
# with an error of d/2 - 1 mm (a closed command is 19.2 mm of travel, the open pads are 40.4 mm apart). With the stiffness
# below the cap (525 N) is reached from 5.25 mm of error, that is for objects wider than 12.5 mm; a narrower one gets
# stiffness x (d/2 - 1 mm) in total (a 10 mm pin: up to 400 N, 133 N per pad). The Hand-E's 50000 N/m reached the cap only above
# 23 mm. Raising the stiffness further (262500 N/m, the cap from 2 mm, was tried) made both engines unstable: PhysX lost
# most of the pad force and Newton diverged. Damping keeps the Hand-E's damping ratio: 200 N.s/m at 50000 N/m, scaled by
# the square root of the stiffness ratio.
DRIVE_STIFFNESS = 100000.0  # N/m
DRIVE_DAMPING = 283.0  # N.s/m

DENSITY_ALU = 2700.0  # kg/m^3, the finger slider and tip (aluminium)
DENSITY_STEEL = 7850.0  # kg/m^3, the two screws that hold each tip


def tip_opening(key):
    """Pad-to-pad diameter (m) at full open: twice the distance from the axis of the tip's nearest point (finger 1's part)."""
    p, _ = part_mesh(f"fingertip_{key}_1")
    return round(2 * float(np.hypot(p[:, 0], p[:, 1]).min()), 4)


def drive_force():
    return FORCE_PER_FINGER * DRIVE_TO_PAD


def angle(k):
    return 120.0 * (k - 1)


def q_z(deg):
    """Quaternion (w, x, y, z) of a turn of `deg` about Z."""
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def joint_frame(k):
    """Joint frame of finger k: its +X axis points toward the gripper axis, so +position closes every finger."""
    return q_z(angle(k) + 180.0)


# ---------------------------------------------------------------- mass properties from the baked parts
def part_mesh(name):
    st = Usd.Stage.Open(os.path.join(ROOT, "parts", f"{N}_{name}.usd"))
    m = UsdGeom.Mesh(st.GetPrimAtPath("/World/mesh_0"))
    pts = np.array(m.GetPointsAttr().Get(), dtype=float)
    idx = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    return pts, idx


def mesh_props(name):
    """Volume (m^3), centre of mass (m) and the inertia tensor about it per unit density (m^5), from the closed mesh."""
    p, f = part_mesh(name)
    a, b, c = p[f[:, 0]], p[f[:, 1]], p[f[:, 2]]
    cross = np.cross(b, c)
    v6 = np.einsum("ij,ij->i", a, cross)  # 6 x the signed volume of each tetrahedron against the origin
    vol = v6.sum() / 6.0
    com = ((a + b + c) * v6[:, None]).sum(0) / (24.0 * vol)
    # second moments of each tetrahedron (origin, a, b, c): the standard polyhedron formulas
    s = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            xi = np.stack([a[:, i], b[:, i], c[:, i]])
            xj = np.stack([a[:, j], b[:, j], c[:, j]])
            s[i, j] = (v6 / 120.0 * ((xi.sum(0) * xj.sum(0)) + (xi * xj).sum(0))).sum()
    inertia_origin = np.trace(s) * np.eye(3) - s
    d = com
    inertia = inertia_origin - vol * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return vol, com, inertia


def body(name, density=None, mass=None):
    """mass (kg), CoM, principal axes and diagonal inertia of a part at the given density (or scaled to `mass`)."""
    vol, com, inertia = mesh_props(name)
    m = mass if mass is not None else density * vol
    rho = m / vol
    w, vecs = np.linalg.eigh(inertia * rho)
    if np.linalg.det(vecs) < 0:
        vecs[:, 2] *= -1
    return dict(mass=m, com=tuple(com), diag=tuple(w), axes=quat_of(vecs), volume=vol)


def quat_of(r):
    """(w, x, y, z) of a rotation matrix."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((0.25 * s), (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
        qv = [0.0, 0.0, 0.0]
        qv[i] = 0.25 * s
        qv[j] = (r[j, i] + r[i, j]) / s
        qv[k] = (r[k, i] + r[i, k]) / s
        q = ((r[k, j] - r[j, k]) / s, *qv)
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


# The Extender variant keeps the default tip and mounts it on the extender: the tip moves by this translation (m, finger 1's
# frame: +X radially outward, +Z toward the fingertips; no rotation), the user's hand placement of the tip on the extender in
# Isaac Sim. The manual's TCP height with extenders is 8 mm further out (173.75 mm against 165.75 mm).
TIP_ON_EXTENDER = (0.036454796838537404, 0.0, 0.00801)


def _rotated_z(v, degrees):
    t = math.radians(degrees)
    return (v[0] * math.cos(t) - v[1] * math.sin(t), v[0] * math.sin(t) + v[1] * math.cos(t), v[2])


def masses():
    """Per-link rigid-body properties. The finger slider is the aluminium slider plus two steel screws; the base takes the
    rest of the datasheet weight (the defeatured CAD has no motor inside, so its mass is spread over the visible solids)."""
    stub_vol, _, _ = mesh_props("finger_1")
    screws = 2 * 0.3e-6  # m^3: two Y-140410-A, 0.3 cm^3 each, from the CAD
    stub_mass = (stub_vol - screws) * DENSITY_ALU + screws * DENSITY_STEEL
    tip_mass = mesh_props("fingertip_stepped_1")[0] * DENSITY_ALU
    out = {"finger": {k: body(f"finger_{k}", mass=stub_mass) for k in FINGERS},
           "tip": {k: body(f"fingertip_stepped_{k}", mass=tip_mass) for k in FINGERS}}
    out["base"] = body("base", mass=GRIPPER_MASS - 3 * (stub_mass + tip_mass))
    return out


EXTENDER_MASS = 0.18022 / 3  # kg: the manual's +180.22 g with the three finger extenders (1875.22 g against 1695 g), with their screws


def tip_on_extender(k):
    """The Extender variant's fingertip body k: the stepped tip, moved onto the extender, plus the extender. The tip keeps its
    mass; the extender takes the manual's mass spread over its CAD volume (the CAD has no screws; the manual's figure has)."""
    tip = MASS["tip"][k]
    shift = np.array(_rotated_z(TIP_ON_EXTENDER, 120 * (k - 1)))
    ext = body(f"fingertip_extender_{k}", mass=EXTENDER_MASS)
    parts = [(tip["mass"], np.array(tip["com"]) + shift, tip), (ext["mass"], np.array(ext["com"]), ext)]
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    inertia = np.zeros((3, 3))
    for mass, c, d in parts:
        r = np.array(d["axes"])  # (w, x, y, z) of the principal frame: rebuild the tensor about the part's own CoM
        rot = _rot_of(r)
        own = rot @ np.diag(d["diag"]) @ rot.T
        off = c - com
        inertia += own + mass * (off @ off * np.eye(3) - np.outer(off, off))
    w, vecs = np.linalg.eigh(inertia)
    if np.linalg.det(vecs) < 0:
        vecs[:, 2] *= -1
    return dict(mass=m, com=tuple(com), diag=tuple(w), axes=quat_of(vecs))


def _rot_of(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


MASS = masses()


def fmt3(t):
    return "(%.9g, %.9g, %.9g)" % tuple(t)


def fmt4(t):
    return "(%.9g, %.9g, %.9g, %.9g)" % tuple(t)


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
            # Tool center point (grasp frame) on the gripper axis, in the middle of the fingertips' length.
            # Provisional: the pad faces are not marked in the CAD.
            def Xform "grip_frame"
            {{
                double3 xformOp:translate = (0, 0, {GRIP_FRAME_Z})
                uniform token[] xformOpOrder = ["xformOp:translate"]

                # FET028 gripper site. forward is the approach axis (Z); grip_line is the width of finger 1's pad
                # (along Y), so the closure, forward x grip_line, is toward the axis along -X. The C10's other two
                # fingers close at 120 and 240 degrees; the site describes the first.
                custom token simready:attactment:socketType = "Gripper"
                float gripper_maxOpening = {MAX_OPENING:g}
                float gripper_maxPayload = {MAX_PAYLOAD:g}

                def BasisCurves "gripper_forward_axis"
                {{
                    point3f[] points = [(0, 0, 0), (0, 0, 0.05)]
                    float3[] extent = [(0, 0, 0), (0, 0, 0.05)]
                    int[] curveVertexCounts = [2]
                    uniform token type = "linear"
                    uniform token purpose = "guide"
                }}

                def BasisCurves "gripper_grip_line"
                {{
                    point3f[] points = [(0, {-PAD_WIDTH / 2:g}, 0), (0, {PAD_WIDTH / 2:g}, 0)]
                    float3[] extent = [(0, {-PAD_WIDTH / 2:g}, 0), (0, {PAD_WIDTH / 2:g}, 0)]
                    int[] curveVertexCounts = [2]
                    uniform token type = "linear"
                    uniform token purpose = "guide"
                }}
            }}
'''
    default = TIPS[0][0]
    tipgrp = lambda k: f'''    def Xform "fingertip_{k}" (
        kind = "group"
    )
    {{
        def Xform "tip_{default}" (
            prepend references = @../parts/{N}_fingertip_{default}_{k}.usd@
        )
        {{
            # Collision pieces (convex, purpose=guide): the tip's colliders; see gen_hande_c10_fingertip_colliders.py.
            def Xform "collision" (
                prepend references = @../parts/{N}_fingertip_{default}_{k}_collision.usd@
            )
            {{
            }}
        }}
    }}
'''
    links = _link("base_link", "base_link", grip)
    for k in FINGERS:
        links += _link(f"finger_{k}", f"finger_{k}")
    links += "".join(_link(f"fingertip_{k}", f"fingertip_{k}") for k in FINGERS).rstrip("\n")
    meshes = '''    def Xform "base_link" (
        prepend references = @../parts/{N}_base.usd@
    )
    {{
    }}
'''.format(N=N)
    for k in FINGERS:
        meshes += f'''
    def Xform "finger_{k}" (
        prepend references = @../parts/{N}_finger_{k}.usd@
    )
    {{
    }}
'''
    w("payloads/base.usda", HDR + f'''(
    # Kinematic body tree + geometry for the Robotiq Hand-E C10. Same layout as the Hand-E: the visible body links live
    # under /{N}/{N}, and the geometry is referenced from the invisible /Meshes scope (one part per link). The per-link
    # CAD parts are baked into the final Isaac frame (approach +Z, gripper axis on Z, base flange at z=0, finger 1 along
    # +X), so every link Xform is identity.
    customLayerData = {{
        dictionary omni_layer = {{
            string authoring_layer = "./base.usda"
        }}
    }}
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./geometries.usda@,
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

''' + links + f'''
    }}
}}

def Scope "Meshes"
{{
    token visibility = "invisible"

{meshes}
    # Fingertip geometry groups. The active tip child is chosen by the "Fingertip" variantSet ("tip_{default}" is the
    # default, authored here).
''' + "\n".join(tipgrp(k) for k in FINGERS).rstrip("\n") + "\n}\n")
    w("payloads/geometries.usda", HDR + '''# Canonical SimReady "geometries" layer (ISA.001 layout slot). The render/collision
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
        @../materials/materials.usda@
    ]
    upAxis = "Z"
)
''')


# ---------------------------------------------------------------- variant payloads
def gen_tip_payloads():
    default = TIPS[0][0]
    for key, var, desc in TIPS[1:]:
        t = ""
        for k in FINGERS:
            t += f'''    over "fingertip_{k}"
    {{
        over "tip_{default}"
        {{
            double3 xformOp:translate = ({", ".join(f"{c:.9g}" for c in _rotated_z(TIP_ON_EXTENDER, 120 * (k - 1)))})
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}

        def Xform "tip_{key}" (
            instanceable = false
            kind = "subcomponent"
            prepend references = @../parts/{N}_fingertip_{key}_{k}.usd@
        )
        {{
            def Xform "collision" (
                prepend references = @../parts/{N}_fingertip_{key}_{k}_collision.usd@
            )
            {{
            }}
        }}
    }}

'''
        w(f"payloads/{N}_fingertip_{key}.usda", HDR + f'''(
    # "{var}" fingertip variant: reference the {desc}
    # (parts baked into the asset frame). The default tip is moved by TIP_ON_EXTENDER in the generator. Its collider is authored (inert) in {N}_physx_common_physics.usda and
    # {N}_newton_common_physics.usda, so it composes only under a Physics variant. The mass properties of the fingertip
    # bodies in this variant (the tip plus the extender) are authored per Fingertip variant in {N}_fingertip_physics.usda.
    # GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
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

over "{N}"
{{
    over "{N}"
    {{
        over "base_link"
        {{
            over "grip_frame"
            {{
                float gripper_maxOpening = {tip_opening(key):g}
            }}
        }}
    }}
}}
''')


# ---------------------------------------------------------------- physics layers
def _mass_block(name, d, kind=None, extra="", mass=True, prim_apis=True):
    """A rigid body's block. mass=False leaves the mass attributes out (they are authored per Fingertip variant);
    prim_apis=False leaves the schemas and kind out (the block only carries the mass attributes)."""
    k = f'\n            kind = "{kind}"' if kind else ""
    head = f'''(
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]{k}
        )''' if prim_apis else ""
    attrs = f'''            point3f physics:centerOfMass = {fmt3(d["com"])}
            float3 physics:diagonalInertia = {fmt3(d["diag"])}
            float physics:mass = {d["mass"]:.9g}
            quatf physics:principalAxes = {fmt4(d["axes"])}
''' if mass else ""
    return f'''        over "{name}"{" " + head if head else ""}
        {{
{attrs}{extra}        }}
'''


def gen_body_mass():
    blocks = [_mass_block("base_link", MASS["base"])] + [_mass_block(f"finger_{k}", MASS["finger"][k]) for k in FINGERS]
    stub = MASS["finger"][1]
    w(f"payloads/{N}_body_mass_physics.usda", HDR + f'''(
    # Shared rigid-body mass for the four Hand-E C10 body links (base + three sliding finger stubs). Engine-neutral
    # (UsdPhysics only), sublayered by the PhysX and Newton variants.
    #
    # Every body link carries its collider in the shared invisible /Meshes scope, so PhysX cannot auto-compute mass:
    # it is authored explicitly, from the baked CAD meshes. A finger stub (the slider and its two tip screws) is
    # {stub["mass"] * 1000:.1f} g (aluminium {DENSITY_ALU:.0f} kg/m^3, steel screws {DENSITY_STEEL:.0f} kg/m^3); the base takes the rest of the
    # datasheet weight ({GRIPPER_MASS} kg). The defeatured CAD has no motor inside, so the base mass is spread over the visible solids and
    # its real centre of mass is probably deeper. Centre of mass and the principal moments come from the closed meshes.
    # GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    upAxis = "Z"
)

over "{N}"
{{
    over "{N}"
    {{
''' + "\n".join(blocks) + '''    }
}
''')


def gen_fingertip_physics():
    def tip(k):
        d = MASS["tip"][k]
        joint = f'''
            def PhysicsFixedJoint "FixedJoint"
            {{
                rel physics:body0 = <{R}/finger_{k}>
                rel physics:body1 = <{R}/fingertip_{k}>
                float physics:breakForce = 3.4028235e38
                float physics:breakTorque = 3.4028235e38
                point3f physics:localPos0 = (0, 0, 0)
                point3f physics:localPos1 = (0, 0, 0)
                quatf physics:localRot0 = {IDQ}
                quatf physics:localRot1 = {IDQ}
            }}
'''
        return _mass_block(f"fingertip_{k}", d, kind="group", extra=joint, mass=False)

    def tip_mass(variant):
        return "".join(
            _mass_block(f"fingertip_{k}", MASS["tip"][k] if variant == TIPS[0][1] else tip_on_extender(k), kind=None, prim_apis=False)
            for k in FINGERS)

    variants = "".join(f'''        "{v}" {{
            over "{N}"
            {{
{tip_mass(v)}            }}
        }}
''' for v in (TIPS[0][1], TIPS[1][1]))
    w(f"payloads/{N}_fingertip_physics.usda", HDR + f'''(
    # Engine-neutral fingertip physics for the Hand-E C10 (UsdPhysics only): the screwed-on fingertip rigid bodies and
    # their PhysicsFixedJoints (welded to the finger stub, rigid like a screw). Sublayered by every Physics variant.
    #
    # Mass is {MASS["tip"][1]["mass"] * 1000:.1f} g per tip (aluminium {DENSITY_ALU:.0f} kg/m^3 x CAD volume); centre of mass and moments from the baked mesh.
    # With the Extender variant the body is the tip moved onto the extender plus the extender ({EXTENDER_MASS * 1000:.1f} g: the manual's +180.2 g for
    # the three extenders with their screws): {tip_on_extender(1)["mass"] * 1000:.1f} g.
    # This layer only composes under a Physics variant, never under Physics = "None".
    # GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
    defaultPrim = "{N}"
    metersPerUnit = 1
    subLayers = [
        @./base.usda@
    ]
    upAxis = "Z"
)

over "{N}" (
    prepend variantSets = ["Fingertip"]
)
{{
    # The mass properties depend on the Fingertip variant (the Extender variant adds the extenders to each tip body), and a
    # local opinion would beat a variant one, so the two sets of values are both authored in variants.
    variantSet "Fingertip" = {{
{variants}    }}

    over "{N}"
    {{
''' + "\n".join(tip(k) for k in FINGERS) + '''    }
}
''')


def gen_kinematics():
    def joint(k):
        q = fmt4(joint_frame(k))
        return f'''            def PhysicsPrismaticJoint "finger_{k}_joint" (
                prepend apiSchemas = ["PhysicsJointStateAPI:linear"]
            )
            {{
                uniform token physics:axis = "X"
                rel physics:body0 = <{R}/base_link>
                rel physics:body1 = <{R}/finger_{k}>
                point3f physics:localPos0 = (0, 0, 0)
                point3f physics:localPos1 = (0, 0, 0)
                quatf physics:localRot0 = {q}
                quatf physics:localRot1 = {q}
                float physics:lowerLimit = 0
                float physics:upperLimit = {STROKE:g}
                float state:linear:physics:position = 0
                float state:linear:physics:velocity = 0
            }}
'''
    w(f"payloads/{N}_kinematics_physics.usda", HDR + f'''(
    defaultPrim = "{N}"
    doc = """Shared, engine-NEUTRAL kinematic tree for the Robotiq Hand-E C10.

Three prismatic joints (base_link -> finger_1 / finger_2 / finger_3), each with a {STROKE * 1000:g} mm stroke. The fingers
sit {angle(2):g} degrees apart about the gripper axis (Z), finger 1 along +X. Link frames are identity/world, so a joint's
frame is its own turn about Z: its +X axis points toward the gripper axis, so closing is +position on all three joints
and the mimic coupling is a plain gearing.

Neutrality rule (SimReady RV.011): only UsdPhysics + PhysicsJointStateAPI here -- no
Physx*/Mjc*/Newton* schemas, no drives/mimics/armature.
GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
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
''' + "\n".join(joint(k) for k in FINGERS).rstrip() + '''
        }
    }
}
''')


COLLIDER_APIS = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]
GRIP_FRICTION = 1.0  # fingertip pad friction (static = dynamic), PhysX and Newton; PhysX combine mode "max", see gen_physx_common


def _usd_list(names):
    return "[" + ", ".join(f'"{n}"' for n in names) + "]"


def tip_hulls(key, k):
    st = Usd.Stage.Open(os.path.join(ROOT, "parts", f"{N}_fingertip_{key}_{k}_collision.usd"))
    return len(st.GetPrimAtPath("/World").GetChildren())


def _px_block(i, name, material=None):
    apis = COLLIDER_APIS + (["MaterialBindingAPI"] if material else [])
    binding = f"\n{i}    rel material:binding:physics = <{material}>" if material else ""
    return f'''{i}over "{name}" (
{i}    prepend apiSchemas = {_usd_list(apis)}
{i})
{i}{{
{i}    uniform token physics:approximation = "convexHull"
{i}    bool physics:collisionEnabled = 1{binding}
{i}}}
'''


def _collider(name, depth=1):
    ind = "    "
    out = f'{ind * depth}over "{name}"\n{ind * depth}{{\n'
    out += _px_block(ind * (depth + 1), "mesh_0")
    return out + f"{ind * depth}}}\n"


def _tip_hull_blocks(block, depth=1):
    """For every finger / tip: `over fingertip_<k>/tip_<key>/collision` with one `block(indent, "hull_<i>")` per baked piece."""
    ind = "    "
    out = ""
    for k in FINGERS:
        out += f'{ind * depth}over "fingertip_{k}"\n{ind * depth}{{\n'
        tips = []
        for key, _, _ in TIPS:
            d = depth + 1
            blk = f'{ind * d}over "tip_{key}"\n{ind * d}{{\n{ind * (d + 1)}over "collision"\n{ind * (d + 1)}{{\n'
            blk += "\n".join(block(ind * (d + 2), f"hull_{h}") for h in range(tip_hulls(key, k)))
            blk += f"{ind * (d + 1)}}}\n{ind * d}}}\n"
            tips.append(blk)
        out += "\n".join(tips) + f"{ind * depth}}}\n\n"
    return out


GRIP_MATERIAL = f"{R}/Physics/GripMaterial"


def gen_physx_common():
    colliders = "".join(_collider(l) + "\n" for l in BODY_LINKS)
    colliders += "    # Tip colliders for every Fingertip variant (derived from TIPS; authored here, not in the\n"
    colliders += '    # variant payloads, so no tip collider ever composes under Physics = "None").\n'
    colliders += _tip_hull_blocks(lambda i, name: _px_block(i, name, GRIP_MATERIAL))  # the pads carry the friction material
    w(f"payloads/{N}_physx_common_physics.usda", HDR + f'''(
    # PhysX-only physics for the Hand-E C10: the articulation root + root_joint (fixed base), the collision-group
    # plumbing, and every PhysX convex-hull collider -- the body-link colliders and the tip collider of each
    # Fingertip variant. Everything here carries Physx* schemas. None of this composes under Physics = "None".
    # GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
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

        # Fingertip pad friction (PhysX): static = dynamic {GRIP_FRICTION:g}; combine mode "max" so it holds
        # against any object whose own material is softer (the default "average" would drag it down).
        # Bound to every tip collider below. The Newton variants author their own pad material.
        def Scope "Physics"
        {{
            def Material "GripMaterial" (
                prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
            )
            {{
                float physics:dynamicFriction = {GRIP_FRICTION:g}
                float physics:staticFriction = {GRIP_FRICTION:g}
                float physics:restitution = 0
                token physxMaterial:frictionCombineMode = "max"
            }}
        }}
    }}
}}

# Collision-group plumbing: keep the shared /Meshes colliders from colliding with the
# articulation-body copies (same pattern as the 2F-85 / 2F-140 / Hand-E).
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
    followers = "".join(f'''
            over "finger_{k}_joint" (
                prepend apiSchemas = ["PhysxJointAPI", "PhysxMimicJointAPI:rotX"]
            )
            {{
                float physxJoint:armature = 0.001
                float physxJoint:maxJointVelocity = 10000
                float physxMimicJoint:rotX:dampingRatio = 0
                float physxMimicJoint:rotX:gearing = -1
                float physxMimicJoint:rotX:naturalFrequency = 0
                rel physxMimicJoint:rotX:referenceJoint = <{R}/Joints/finger_1_joint>
            }}
''' for k in FINGERS[1:])
    w(f"payloads/{N}_physx_physics.usda", HDR + f'''(
    # PhysX variant of the Hand-E C10: finger_1_joint is the single driven DOF (linear position drive); finger_2_joint and
    # finger_3_joint mimic it (rigid, gearing -1 as on the Hand-E: every joint frame points inward, so all three close
    # with +position). Shared neutral joint frames/limits live in ./{N}_kinematics_physics.usda (sublayered below).
    #
    # Drive: maxJointVelocity {FINGER_SPEED:g} m/s per finger (half the datasheet's closing speed of 14 to 107 mm/s, read as the
    # diameter's). maxForce {drive_force():g} N = {DRIVE_TO_PAD:g} x the datasheet's {FORCE_PER_FINGER:g} N per finger, because the three
    # fingers share the driven joint's force. Stiffness {DRIVE_STIFFNESS:g} N/m reaches the cap from {drive_force() / DRIVE_STIFFNESS * 1000:g} mm of remaining stroke (objects wider than 12.5 mm; a narrower one gets less), damping {DRIVE_DAMPING:g}.
    # GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
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
            over "finger_1_joint" (
                prepend apiSchemas = ["PhysxJointAPI", "PhysicsDriveAPI:linear"]
            )
            {{
                float drive:linear:physics:damping = {DRIVE_DAMPING:g}
                float drive:linear:physics:maxForce = {drive_force():g}
                float drive:linear:physics:stiffness = {DRIVE_STIFFNESS:g}
                float drive:linear:physics:targetPosition = 0
                float drive:linear:physics:targetVelocity = 0
                float physxJoint:armature = 0.001
                float physxJoint:maxJointVelocity = {FINGER_SPEED:g}
            }}
{followers}        }}
    }}
}}
''')


# ---------------------------------------------------------------- Newton (MuJoCo-Warp)
NEWTON_COLLIDER_APIS = '["PhysicsCollisionAPI", "NewtonCollisionAPI", "PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"]'
SOLIMP = "[0.95, 0.99, 0.001, 0.5, 2]"
TIP_SOLREF = "[0.002, 2]"  # Newton fingertip contact solref [timeconst, dampratio]: stiffer than the body links'


def _newton_over(path, body, depth=1):
    ind = "    "
    out = f'{ind * depth}over "{path}"\n{ind * depth}{{\n'
    out += body(ind * (depth + 1), "mesh_0")
    return out + f"{ind * depth}}}\n"


def gen_newton_common():
    link = lambda i, name: f'''{i}over "{name}" (
{i}    prepend apiSchemas = {NEWTON_COLLIDER_APIS}
{i})
{i}{{
{i}    uniform token physics:approximation = "convexHull"
{i}    bool physics:collisionEnabled = 1
{i}    float newton:contactGap = 0
{i}}}
'''
    tip = lambda i, name: f'''{i}over "{name}" (
{i}    prepend apiSchemas = {NEWTON_COLLIDER_APIS[:-1]}, "MaterialBindingAPI"]
{i})
{i}{{
{i}    uniform token physics:approximation = "convexHull"
{i}    bool physics:collisionEnabled = 1
{i}    rel material:binding:physics = <{GRIP_MATERIAL}>
{i}    float newton:contactGap = 0
{i}}}
'''
    cols = "\n".join(_newton_over(l, link) for l in BODY_LINKS) + "\n"
    cols += "    # Tip colliders for every Fingertip variant (derived from TIPS); only the one the\n"
    cols += "    # Fingertip variant activates composes. Fingertips carry the grip-pad friction material.\n"
    cols += _tip_hull_blocks(tip)
    w(f"payloads/{N}_newton_common_physics.usda", HDR + f'''(
    defaultPrim = "{N}"
    doc = """Shared Newton-pure physics for the Robotiq Hand-E C10: the grip-pad friction material and the per-mesh
convex-hull colliders (+ pad material binding on every fingertip). Sublayered by the Newton variant payload, which adds the
Mjc* solver tuning on top (this layer is Newton-pure: only UsdPhysics + Newton* schemas, no Physx*/Mjc*, RV.011).

Link mass / CoM / inertia come from the shared ./{N}_body_mass_physics.usda and the fingertip bodies + welds from
./{N}_fingertip_physics.usda (both engine-neutral, shared with the PhysX variant), not re-authored here.
GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand.
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
                float physics:dynamicFriction = {GRIP_FRICTION:g}
                float physics:staticFriction = {GRIP_FRICTION:g}
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
    link_mjc = lambda i, name: f'''{i}over "{name}" (
{i}    prepend apiSchemas = ["MjcCollisionAPI"]
{i})
{i}{{
{i}    uniform int mjc:group = 2
{i}}}
'''
    base_mjc = lambda i, name: f'''{i}over "{name}" (
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
    tip_mjc = lambda i, name: f'''{i}over "{name}" (
{i}    prepend apiSchemas = ["MjcCollisionAPI"]
{i})
{i}{{
{i}    uniform int mjc:group = 3
{i}    uniform int mjc:priority = 1
{i}    uniform double[] mjc:solimp = {SOLIMP}
{i}    uniform double[] mjc:solref = {TIP_SOLREF}
{i}}}
'''
    cols = _newton_over("base_link", base_mjc) + "\n"
    cols += "\n".join(_newton_over(l, link_mjc) for l in BODY_LINKS[1:]) + "\n"
    cols += _tip_hull_blocks(tip_mjc)
    followers = "".join(f'''
            # Rigid coupling: a MuJoCo joint equality (finger_{k} = finger_1) authored with a stiff solref/solimp.
            # NewtonMimicAPI would give the same constraint but with MuJoCo's default softness (not authorable),
            # which a loaded grasp overpowers.
            over "finger_{k}_joint" (
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
                rel mjc:target = <{R}/Joints/finger_1_joint>
            }}
''' for k in FINGERS[1:])
    w(f"payloads/{N}_newton_physics.usda", HDR + f'''(
    defaultPrim = "{N}"
    doc = """Newton (MuJoCo-Warp) variant of the Robotiq Hand-E C10.

Three sliding fingers on prismatic joints, no linkage. finger_1_joint is the single driven DOF (linear drive); finger_2_joint
and finger_3_joint follow it through stiff MuJoCo joint equalities (MjcEqualityJointAPI: finger_k = finger_1; every joint
closes with +position, see the kinematics layer). Same fixed-base root_joint + articulation root as the PhysX variant.

Layers: the Newton-pure colliders / pad material (./{N}_newton_common_physics.usda), the shared link masses
(./{N}_body_mass_physics.usda), the engine-neutral fingertip bodies + welds (./{N}_fingertip_physics.usda) and the neutral joint
frames/limits (./{N}_kinematics_physics.usda). This layer adds the Mjc* solver tuning for both. Drive gains and MuJoCo tuning are
copied from the Hand-E and still need live Newton validation on the C10.

RV.011: Newton + Mjc schemas only -- no Physx* runtime data here.
GENERATED by devel_helpers/gen_hande_c10_layers.py -- do not edit by hand."""
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
        # Same as the PhysX variant's physxArticulation:enabledSelfCollisions = 0: no collisions between the
        # gripper's own bodies (the stepped fingertips would otherwise block full closure).
        bool newton:selfCollisionEnabled = 0

        # World-pinned root joint (fixed base). Same name/path as the PhysX variant, so a mounted-gripper override
        # `over "root_joint" (active = false)` applies to both.
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
            over "finger_1_joint" (
                prepend apiSchemas = ["MjcJointAPI", "PhysicsDriveAPI:linear"]
            )
            {{
                float drive:linear:physics:damping = {DRIVE_DAMPING:g}
                float drive:linear:physics:maxForce = {drive_force():g}
                float drive:linear:physics:stiffness = {DRIVE_STIFFNESS:g}
                float drive:linear:physics:targetPosition = 0
                uniform double mjc:armature = 0.001
                uniform double mjc:damping = 1
                uniform double[] mjc:solimplimit = {SOLIMP}
                uniform double[] mjc:solreflimit = [0.02, 2]
            }}
{followers}        }}

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
    links = ["base_link"] + [f"finger_{k}" for k in FINGERS] + [f"fingertip_{k}" for k in FINGERS]
    joints = [f"{R}/Joints/finger_{k}_joint" for k in FINGERS]
    site = '''            string isaac:nameOverride (
                displayName = "Name Override"
                doc = "Name override for prim lookup in base name search"
            )

            over "grip_frame" (
                prepend apiSchemas = ["IsaacSiteAPI"]
            )
            {
                string isaac:Description = "Tool center point (grasp frame) on the gripper axis, between the three fingertips." (
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
    robot_joints = "".join(f"                <{j}>,\n" for j in [f"{R}/root_joint"] + joints)
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
    string isaac:description = "Robotiq Hand-E C10 adaptive gripper (3 concentric sliding fingers, {2 * STROKE * 1000:.1f} mm of travel in diameter)." (
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

    # robotJoints is variant-specific (authored per Physics variant); fingertip welds are not listed, as on the
    # 2F-85 / 2F-140 / Hand-E. robotLinks stays shared: the bodies are the same across variants.
    variantSet "Physics" = {{
        "PhysX" {{
            prepend rel isaac:physics:robotJoints = [
{robot_joints}            ]
        }}
        "Newton" {{
            prepend rel isaac:physics:robotJoints = [
{robot_joints}            ]
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
''' + "\n".join(fj(f"finger_{k}_joint") for k in FINGERS) + '''        }
    }
}
''')
    # Integration-path layer: just the PhysX variant stack + the base tree (no editor Render prims / lights).
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
        # SR.001 requires these identity fields authored DIRECTLY in the root layer's customLayerData (not only nested
        # in SimReady_Metadata). They mirror the matching SimReady_Metadata entries below.
        string asset_name = "{N}"
        string asset_type = "gripper"
        string source_file = "ROBOTIQ_HAND-E_C10_DEFEATURE.STEP"
        string usd_date_generated = "2026-10-27"
        # SimReady provenance metadata (SimReady Foundations SR.003 / FET_033_STANDARD).
        # rigid_body_count / mass / asset_extents are COMPUTED from the default composed stage by
        # devel_helpers/gen_hande_c10_layers.py -- re-run it instead of editing them by hand.
        dictionary SimReady_Metadata = {{
            string asset_name = "{N}"
            string asset_type = "gripper"
            string category = "robot_gripper"
            string author = "Robotiq"
            string asset_license = "CC-BY-4.0"
            string source_file = "ROBOTIQ_HAND-E_C10_DEFEATURE.STEP"
            string usd_date_generated = "2026-10-27"
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
        string Fingertip = "{TIPS[0][1]}"
        string Physics = "PhysX"
    }}
    prepend variantSets = ["Fingertip", "Physics"]
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
    # The default tip is authored in base.usda; every other tip swaps it for its own parts (see gen_tip_payloads). The long
    # and short spacers still have to be placed on the finger, then each gets a row in TIPS of gen_hande_c10_layers.py.
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
    for sub in ("payloads", "configuration"):
        os.makedirs(os.path.join(ROOT, sub), exist_ok=True)
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
    print("masses (g): base %.1f, finger stub %.1f, tip %.1f" % (MASS["base"]["mass"] * 1e3, MASS["finger"][1]["mass"] * 1e3, MASS["tip"][1]["mass"] * 1e3))
