#!/usr/bin/env python3
"""Generate the physics payload layers of the Robotiq 2F-85 and 2F-140 assets.

The two 2F grippers are the same linkage (a four-bar finger driven by finger_joint,
mirrored left/right) with different dimensions, masses, meshes and fingertips. Their
physics layers used to be hand-maintained copies of each other and drifted apart; this
script writes both from ONE template and a per-gripper table (GRIPPERS below), so a
structural change is made once and lands in both.

Generated, for G in Robotiq_2F_85, Robotiq_2F_140 (all under grippers/G/payloads/):
  engine-neutral (UsdPhysics only, shared by every Physics variant)
    G_kinematics_physics.usda      joint frames/limits, articulation root, root_joint
    G_body_mass_physics.usda       mass/CoM/inertia of the nine body links
    G_fingertip_physics.usda       fingertip bodies, default-pad mass, welds
    G_collision_physics.usda       which /Meshes meshes collide (convex hulls)
    G_parallel_weld_physics.usda   outer_finger weld of both *_parallel_grip variants
  PhysX
    G_physx_common_physics.usda    shared by both PhysX variants
    G_parallel_grip_physics.usda   Physics = "Physx_parallel_grip" payload
    G_compliant_physics.usda       Physics = "Physx_compliant" payload
  Newton (MuJoCo-Warp)
    G_newton_common_physics.usda   Newton-pure, shared by both Newton variants
    G_newton_fourbar_physics.usda  Newton + Mjc four-bar, shared by both Newton variants
    G_newton_parallel_grip_physics.usda  Physics = "Newton_parallel_grip" payload
    G_newton_compliant_physics.usda      Physics = "Newton_compliant" payload

Not generated: the root layer, configuration/, base.usda and the fingertip variant
payloads (geometry), parts/ and materials/.

Constraints the template keeps (see grippers/GRIPPER_SIMULATION_GUIDE.md):
  * Every layer that authors physics ends in _physics.usda (SimReady RC.005).
  * A Physics variant composes only its own engine's schemas (SimReady RV.011); Mjc*
    data stays out of newton_common so a pure-Newton selection remains possible.
    Isaac's Newton import reads Physx* attributes as a fallback, so no Physx* opinion
    may reach a layer a Newton variant composes.
  * Every joint has its parent as physics:body0 (Newton rejects reversed joints), so
    the two engines cut the four-bar loop in different places: PhysX excludes
    inner_finger_joint from the articulation, Newton suppresses
    inner_finger_knuckle_joint and closes the loop with a spherical equality there.
  * Everything stays inside the Physics variant payloads (nothing composes under
    Physics = "None").

Usage (plain Python, no USD bindings needed):
    python3 devel_helpers/gen_2f_layers.py
then check what changed in the COMPOSED variants, not in the text: compare a per-prim
physics signature (applied schemas from the composed apiSchemas list-op, every
physics/physx/mjc/newton/drive attribute and relationship) of every Physics x Fingertip
selection before and after, and run tests/runtime.
"""
import os
import textwrap

GRIPPERS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "grippers")
SIDES = ("left", "right")
FLT_MAX = "3.4028235e38"
SOLIMP = "[0.95, 0.99, 0.001, 0.5, 2]"

# ------------------------------------------------------------------ per-gripper data
# Values are kept as the text they are authored with (USD float32 text), so a
# regeneration does not change a single bit of them.
#
# JOINTS: (name, body0, body1, localPos (both frames), localRot (both frames), lower, upper)
#   in the PhysX body order; every link frame is the gripper base frame (identity).
# BODY_MASS / FINGERTIP: link -> (mass, centerOfMass, diagonalInertia, principalAxes)
# COLLIDERS: (path under /Meshes, role); role picks the Newton/MuJoCo contact tuning:
#   "base" and "inner_knuckle" get the full Mjc contact set, "link" only its group,
#   "tip" the pad set + the GripMaterial binding.
GRIPPERS = {
    "Robotiq_2F_85": dict(
        label="2F-85",
        joints=[
            ("finger_joint", "base_link", "left_outer_knuckle", "(0, -0.0306, 0.05466)", "(0.5, 0.5, -0.5, -0.5)", "0", "47"),
            ("right_outer_knuckle_joint", "base_link", "right_outer_knuckle", "(0, 0.0306, 0.05466)", "(0.5, 0.5, 0.5, 0.5)", "0", "47"),
            ("left_outer_finger_joint", "left_outer_knuckle", "left_outer_finger", "(0, -0.06213, 0.0509)", "(0.5, 0.5, 0.5, 0.5)", "0", "45"),
            ("right_outer_finger_joint", "right_outer_knuckle", "right_outer_finger", "(0, 0.06213, 0.0509)", "(0.5, 0.5, 0.5, 0.5)", "-45", "0"),
            ("left_inner_knuckle_joint", "base_link", "left_inner_knuckle", "(0, -0.0127, 0.06118)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("right_inner_knuckle_joint", "base_link", "right_inner_knuckle", "(0, 0.0127, 0.06118)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("left_inner_finger_joint", "left_outer_finger", "left_inner_finger", "(0, -0.06776, 0.09809)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("right_inner_finger_joint", "right_outer_finger", "right_inner_finger", "(0, 0.06776, 0.09809)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("left_inner_finger_knuckle_joint", "left_inner_finger", "left_inner_knuckle", "(0, -0.04986, 0.1046)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("right_inner_finger_knuckle_joint", "right_inner_finger", "right_inner_knuckle", "(0, 0.04986, 0.1046)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
        ],
        mass_note="""Mass + CoM come from the gripper's CAD. base_link is a fixed base, so its
inertia is inert; the outer knuckles get a small tensor on the order of the sibling
knuckles. inner_finger is the distal body (finger4step) only -- the pad is the separate
fingertip rigid body: its mass is the CAD part ratio (11.29 g distal / 22.10 g pad)
applied to the 0.039254718 kg per-side total, so it is not the raw CAD part mass; its
inertia is the part's own tensor about its own CoM.""",
        body_mass={
            "base_link": ("0.77744", "(0, 0, 0.035508)", "(0.001, 0.001, 0.001)", "(1, 0, 0, 0)"),
            "left_outer_knuckle": ("0.02", "(0, -0.048356, 0.056017)", "(0.00001, 0.000008, 0.000004)", "(1, 0, 0, 0)"),
            "right_outer_knuckle": ("0.02", "(0, 0.048356, 0.056017)", "(0.00001, 0.000008, 0.000004)", "(1, 0, 0, 0)"),
            "left_outer_finger": ("0.039179202", "(0.000005531254, -0.06595047, 0.06895309)", "(0.0000132201985, 0.0000042931024, 0.000010880907)", "(0.76604444, -0.64278764, 0, 0)"),
            "right_outer_finger": ("0.039179202", "(-0.000005531254, 0.06595047, 0.06895309)", "(0.0000132201985, 0.0000042931024, 0.000010880907)", "(0.76604444, 0.64278764, 0, 0)"),
            "left_inner_finger": ("0.01327301", "(-0.00000139, -0.05940770, 0.10376717)", "(8.956112865e-07, 1.209160565e-06, 1.440650775e-06)", "(0.1989801, 0.6786246, -0.6783625, -0.1992485)"),
            "right_inner_finger": ("0.01327301", "(0.00000139, 0.05940770, 0.10376717)", "(8.956112865e-07, 1.209160565e-06, 1.440650775e-06)", "(-0.6783625, -0.1992485, 0.1989801, 0.6786246)"),
            "left_inner_knuckle": ("0.027306942", "(-2.005321e-10, -0.03222066, 0.083420366)", "(0.000011838778, 0.0000044877793, 0.00000800488)", "(0.9063078, -0.42261827, 0, 0)"),
            "right_inner_knuckle": ("0.027306942", "(2.005321e-10, 0.03222066, 0.083420366)", "(0.000011838778, 0.0000044877793, 0.00000800488)", "(0.9063078, 0.42261827, 0, 0)"),
        },
        default_tip="Standard",
        tip_note="""Mass/inertia are the Standard pad's; the "Tactile_TSF85" Fingertip variant
overrides them with the sensor-case values in the root layer.""",
        fingertip={
            "left_fingertip": ("0.02598171", "(0, -0.05063447, 0.12616345)", "(0.0000013906384, 0.0000033053761, 0.0000037298817)", "(0.550085, -0.4443045, -0.4443045, 0.550085)"),
            "right_fingertip": ("0.02598171", "(0, 0.05063447, 0.12616345)", "(0.0000013906384, 0.0000033053761, 0.0000037298817)", "(0.550085, 0.4443045, 0.4443045, 0.550085)"),
        },
        # One mesh per link type, shared by both sides (instanced).
        colliders=[
            ("base_link/Defeatured_2F_85_PAD_OPEN_basestep_01/Defeatured_2F_85_PAD_OPEN_basestep", "base"),
            ("inner_finger/Defeatured_2F_85_PAD_OPEN_finger4step_01/Defeatured_2F_85_PAD_OPEN_finger4step", "link"),
            ("inner_knuckle/Defeatured_2F_85_PAD_OPEN_finger3step_01/Defeatured_2F_85_PAD_OPEN_finger3step", "inner_knuckle"),
            ("outer_finger/Defeatured_2F_85_PAD_OPEN_finger2step_01/Defeatured_2F_85_PAD_OPEN_finger2step", "link"),
            ("outer_knuckle/Defeatured_2F_85_PAD_OPEN_Finger1step_01/Defeatured_2F_85_PAD_OPEN_Finger1step", "link"),
            ("fingertip/tip_standard/Defeatured_2F_85_PAD_OPEN_fingertipsstep", "tip"),
            ("fingertip/tip_tsf85/TSF_85_case", "tip"),
        ],
        # PhysX compliant pad spring (N.m/deg) and the same spring for MuJoCo (N.m/rad).
        pad_spring_physx="0.0004",
        pad_spring_mjc="0.0229",
    ),
    "Robotiq_2F_140": dict(
        label="2F-140",
        joints=[
            ("finger_joint", "base_link", "left_outer_knuckle", "(0, -0.030601, 0.054663)", "(0.5, 0.5, -0.5, -0.5)", "0", "45"),
            ("right_outer_knuckle_joint", "base_link", "right_outer_knuckle", "(0, 0.030601, 0.054663)", "(0.5, 0.5, 0.5, 0.5)", "0", "45"),
            ("left_outer_finger_joint", "left_outer_knuckle", "left_outer_finger", "(0, -0.062026, 0.050130)", "(0.5, 0.5, 0.5, 0.5)", "0", "45"),
            ("right_outer_finger_joint", "right_outer_knuckle", "right_outer_finger", "(0, 0.062026, 0.050130)", "(0.5, 0.5, 0.5, 0.5)", "-45", "0"),
            ("left_inner_knuckle_joint", "base_link", "left_inner_knuckle", "(0, -0.012700, 0.061178)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("right_inner_knuckle_joint", "base_link", "right_inner_knuckle", "(0, 0.012700, 0.061178)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("left_inner_finger_joint", "left_outer_finger", "left_inner_finger", "(0, -0.097456, 0.129029)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("right_inner_finger_joint", "right_outer_finger", "right_inner_finger", "(0, 0.097456, 0.129029)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("left_inner_finger_knuckle_joint", "left_inner_finger", "left_inner_knuckle", "(0, -0.079555, 0.135545)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
            ("right_inner_finger_knuckle_joint", "right_inner_finger", "right_inner_knuckle", "(0, 0.079555, 0.135545)", "(0.5, 0.5, 0.5, 0.5)", "-180", "180"),
        ],
        mass_note="""Values are taken from NVIDIA's official Isaac 2F-140 asset
(Robotiq_2F_140_physics_edit.usd), re-expressed in this asset's base-link frame. base_link
has no mass in the official asset; its value comes from the ROS robotiq_description URDF
(it is a fixed base, so its inertia is inert).""",
        body_mass={
            "base_link": ("0.22652", "(0, 0, 0.03145)", "(0.00020005, 0.00017832, 0.00013478)", "(1, 0, 0, 0)"),
            "left_outer_knuckle": ("0.009163", "(0, -0.052894, 0.052941)", "(0.000000213107, 0.000000673439, 0.000000751601)", "(1, 0, 0, 0)"),
            "right_outer_knuckle": ("0.009163", "(0, 0.052894, 0.052941)", "(0.000000213107, 0.000000673439, 0.000000751601)", "(1, 0, 0, 0)"),
            "left_outer_finger": ("0.077064", "(0.000003, -0.076708, 0.084331)", "(0.00000681569, 0.0000535424, 0.0000575694)", "(0.356478, 0.610675, 0.610675, -0.356478)"),
            "right_outer_finger": ("0.077064", "(-0.000003, 0.076708, 0.084331)", "(0.00000681569, 0.0000535424, 0.0000575694)", "(-0.356478, 0.610675, -0.610675, -0.356478)"),
            "left_inner_finger": ("0.049834", "(-0.000001, -0.081480, 0.160890)", "(0.0000030511, 0.0000191144, 0.0000171297)", "(0.076951, 0.702633, 0.07576, 0.703311)"),
            "right_inner_finger": ("0.049834", "(-0.000001, 0.081477, 0.160888)", "(0.0000030511, 0.0000191144, 0.0000171297)", "(-0.075845, 0.702516, -0.076863, 0.703428)"),
            "left_inner_knuckle": ("0.068112", "(0, -0.048258, 0.100770)", "(0.00000731063, 0.0000453843, 0.0000510923)", "(0.640339, -0.30047, 0.295076, 0.64235)"),
            "right_inner_knuckle": ("0.068112", "(0, 0.048258, 0.100770)", "(0.00000731063, 0.0000453843, 0.0000510923)", "(0.64235, 0.295076, 0.30047, -0.640339)"),
        },
        default_tip="Flat_Overmolded",
        tip_note="""Mass/inertia are the Flat-Overmolded pad's (CoM = the baked pad geometry centre;
mass + inertia are box-model estimates, refine against CAD if needed). The Silicone /
V-Groove / Tactile_TSF140 Fingertip variants override them with their own values.""",
        fingertip={
            "left_fingertip": ("0.04", "(0, -0.0799, 0.1768)", "(0.0000153, 0.0000036, 0.0000165)", "(1, 0, 0, 0)"),
            "right_fingertip": ("0.04", "(0, 0.0793, 0.1772)", "(0.0000153, 0.0000036, 0.0000165)", "(1, 0, 0, 0)"),
        },
        # One baked mesh per link and side.
        colliders=[("base_link/mesh_0", "base")]
        + [(f"{s}_{link}/mesh_0", "inner_knuckle" if link == "inner_knuckle" else "link")
           for link in ("outer_knuckle", "outer_finger", "inner_finger", "inner_knuckle") for s in SIDES]
        + [(f"fingertip_{s}/tip_{t}/mesh_0", "tip") for s in SIDES for t in ("flat", "silicone", "vgroove", "tsf140")],
        pad_spring_physx="0.00068",
        pad_spring_mjc="0.03896",
    ),
}


# ------------------------------------------------------------------ text helpers
def comment(text, indent, width=92):
    """``text`` as # comment lines at ``indent`` spaces. Prose paragraphs are re-wrapped to
    ``width``; a paragraph with an indented or bulleted line is kept as written."""
    pad = " " * indent
    lines = []
    for para in text.strip("\n").split("\n\n"):
        rows = para.split("\n")
        if any(r[:1] in (" ", "*", "-") for r in rows):
            lines += rows
        else:
            lines += textwrap.wrap(" ".join(rows), width - indent - 2, break_on_hyphens=False)
        lines.append("")
    return "".join(f"{pad}# {line}".rstrip() + "\n" for line in lines[:-1])


def header(g, about, sublayers=()):
    """Layer header: a comment block, defaultPrim and optional sublayers."""
    subs = ""
    if sublayers:
        subs = "    subLayers = [\n" + ",\n".join(f"        @./{s}@" for s in sublayers) + "\n    ]\n"
    return f"#usda 1.0\n(\n{comment(about, 4)}    defaultPrim = \"{g}\"\n    metersPerUnit = 1\n{subs}    upAxis = \"Z\"\n)\n"


def block(indent, head, body="", meta=()):
    """``head`` prim with optional metadata lines and a body (already indented +4)."""
    pad = " " * indent
    out = pad + head
    if meta:
        out += " (\n" + "".join(f"{pad}    {m}\n" for m in meta) + pad + ")"
    return out + f"\n{pad}{{\n{body}{pad}}}\n"


def join(*blocks):
    return "\n".join(b for b in blocks if b)


def attrs(indent, *lines):
    pad = " " * indent
    return "".join(f"{pad}{line}\n" for line in lines)


def api(*schemas):
    return "prepend apiSchemas = [" + ", ".join(f'"{s}"' for s in schemas) + "]"


def mass_lines(indent, values):
    m, com, inertia, axes = values
    return attrs(indent, f"point3f physics:centerOfMass = {com}", f"float3 physics:diagonalInertia = {inertia}",
                 f"float physics:mass = {m}", f"quatf physics:principalAxes = {axes}")


def frames(indent, pos0="(0, 0, 0)", rot0="(1, 0, 0, 0)", pos1=None, rot1=None):
    return attrs(indent, f"point3f physics:localPos0 = {pos0}", f"point3f physics:localPos1 = {pos1 or pos0}",
                 f"quatf physics:localRot0 = {rot0}", f"quatf physics:localRot1 = {rot1 or rot0}")


def weld(indent, g, body0, body1):
    return block(indent, 'def PhysicsFixedJoint "FixedJoint"',
                 attrs(indent + 4, f"rel physics:body0 = </{g}/{g}/{body0}>", f"rel physics:body1 = </{g}/{g}/{body1}>",
                       f"float physics:breakForce = {FLT_MAX}", f"float physics:breakTorque = {FLT_MAX}")
                 + frames(indent + 4))


def gripper_prims(g, body, specifier="over", outer_meta=(), inner_meta=()):
    """/G/G wrapper around ``body`` (indented 8)."""
    inner = block(4, f'{specifier} "{g}"' if specifier == "over" else f'{specifier} "{g}"', body, inner_meta)
    return block(0, f'{specifier} "{g}"', inner, outer_meta)


def meshes(colliders, fn):
    """over "Meshes" nesting for every collider; ``fn(path, role, indent)`` -> (meta, body) or None."""
    tree = {}
    for path, role in colliders:
        node = tree
        for part in path.split("/"):
            node = node.setdefault(part, {})
        node[None] = role

    def emit(node, prefix, indent):
        parts = []
        for name, child in node.items():
            if name is None:
                continue
            path = f"{prefix}/{name}" if prefix else name
            role = child.get(None)
            inner = emit(child, path, indent + 4)
            if role is not None:
                spec = fn(path, role, indent)
                if spec is None:
                    continue
                meta, body = spec
                parts.append(block(indent, f'over "{name}"', body + inner, meta))
            elif inner:
                parts.append(block(indent, f'over "{name}"', inner))
        return join(*parts)

    return emit(tree, "", 4)


def write(g, name, text):
    path = os.path.join(GRIPPERS_DIR, g, "payloads", f"{g}_{name}_physics.usda")
    with open(path, "w") as f:
        f.write(text)


# ------------------------------------------------------------------ engine-neutral layers
def gen_kinematics(g, d):
    joints = []
    for name, b0, b1, pos, rot, lo, hi in d["joints"]:
        joints.append(block(12, f'def PhysicsRevoluteJoint "{name}"',
                            attrs(16, 'uniform token physics:axis = "Z"',
                                  f"rel physics:body0 = </{g}/{g}/{b0}>", f"rel physics:body1 = </{g}/{g}/{b1}>",
                                  f"point3f physics:localPos0 = {pos}", f"point3f physics:localPos1 = {pos}",
                                  f"quatf physics:localRot0 = {rot}", f"quatf physics:localRot1 = {rot}",
                                  f"float physics:lowerLimit = {lo}", f"float physics:upperLimit = {hi}",
                                  "float state:angular:physics:position = 0",
                                  "float state:angular:physics:velocity = 0"),
                            [api("PhysicsJointStateAPI:angular")]))
    root_joint = comment("""World-pinned root joint (RC.009 fixed-base root), shared by every Physics variant:
it makes this a fixed-base articulation, as expected for an End Effector interface
asset. When mounting on an arm, retarget or deactivate it (the teleop demo's mounted
gripper authors `over "root_joint" (active = false)` inside each of its Gripper
variants). The Isaac robot schema (IsaacRobotAPI + per-variant robotJoints + shared
robotLinks) is on the DEFAULT prim, authored in ../configuration/""" + g + "_robot.usda.", 8) \
        + block(8, 'def PhysicsFixedJoint "root_joint"',
                attrs(12, f"rel physics:body1 = </{g}/{g}/base_link>") + frames(12))
    body = block(8, 'def Scope "Joints"', join(*joints)) + "\n" + root_joint
    write(g, "kinematics", header(g, f"""Shared, engine-NEUTRAL kinematic tree for the Robotiq {d['label']}.

Single source of truth for the revolute joint set -- frames (axis / localPos / localRot),
limits, body relations and PhysicsJointStateAPI -- plus the articulation root and the
fixed-base root_joint. Sublayered by every Physics variant payload; each variant then
OVERLAYS only its engine-specific physics (drives, mimics, Mjc tuning), suppresses the
joints its topology does not use (active=false), and adds its variant-only joints
(the parallel weld, Newton's loop closures).

Neutrality rule (SimReady RV.011): only UsdPhysics + PhysicsJointStateAPI here -- no
Physx*/Mjc*/Newton* schemas, no drives/mimics/actuators, no armature, and no
physics:excludeFromArticulation (where to cut the four-bar loop is per engine). The union
is the full 10-revolute compliant set, in the PhysX body order (parent = body0). PhysX
cuts the loop at inner_finger_joint (excluded from the articulation); Newton keeps
inner_finger_joint and suppresses inner_finger_knuckle_joint, closing the loop there with
a spherical equality instead. Every link frame is the gripper base frame (identity).
Generated by devel_helpers/gen_2f_layers.py.""")
          + "\n" + block(0, f'def Xform "{g}"', block(4, f'def Xform "{g}"', body, [api("PhysicsArticulationRootAPI")])))


def gen_body_mass(g, d):
    links = [block(8, f'over "{n}"', mass_lines(12, v), [api("PhysicsRigidBodyAPI", "PhysicsMassAPI")])
             for n, v in d["body_mass"].items()]
    write(g, "body_mass", header(g, f"""Single source of truth for the rigid-body mass of the nine {d['label']} body links.

Every body link carries its collider in the shared invisible /Meshes scope (not as a
rigid-body child), so the mass is authored explicitly rather than left to the engine.
Sublayered by every Physics variant payload; the Newton solver's deliberately different
isotropic diagonal inertia is overridden on top of it in
./{g}_newton_common_physics.usda.

{d['mass_note']}
Generated by devel_helpers/gen_2f_layers.py.""")
          + "\n" + gripper_prims(g, join(*links)))


def gen_fingertip(g, d):
    tips = []
    for s in SIDES:
        tip = f"{s}_fingertip"
        tips.append(block(8, f'over "{tip}"', mass_lines(12, d["fingertip"][tip]) + "\n" + weld(12, g, f"{s}_inner_finger", tip),
                          [api("PhysicsRigidBodyAPI", "PhysicsMassAPI")]))
    write(g, "fingertip", header(g, f"""Engine-neutral fingertip physics for the {d['label']} (UsdPhysics only): the screwed-on
fingertip rigid bodies, their PhysicsFixedJoints (welded to inner_finger, rigid like a
screw; identity frames, as every link frame is the base frame) and the default
"{d['default_tip']}" pad mass/inertia. Sublayered by every Physics variant, so it never
composes under Physics = "None".

The default pad's mass is authored here (not in the "Fingertip" variantSet) so it is
reachable on the configuration/ integration path, which sublayers a variant payload +
base.usda but never {g}.usda. {d['tip_note']}
The tip colliders are declared in ./{g}_collision_physics.usda.
Generated by devel_helpers/gen_2f_layers.py.""", ["base.usda"])
          + "\n" + gripper_prims(g, join(*tips)))


def gen_collision(g, d):
    def fn(path, role, indent):
        return ([api("PhysicsCollisionAPI", "PhysicsMeshCollisionAPI")],
                attrs(indent + 4, 'uniform token physics:approximation = "convexHull"', "bool physics:collisionEnabled = 1"))
    write(g, "collision", header(g, f"""Engine-neutral collider declarations for the {d['label']}: which shared /Meshes CAD
meshes collide, as UsdPhysics convex hulls (PhysicsCollisionAPI + PhysicsMeshCollisionAPI,
physics:approximation, physics:collisionEnabled). Sublayered by every Physics variant
payload, so it never composes under Physics = "None". The engine layers add only their
own collision schemas and parameters on top: ./{g}_physx_common_physics.usda (Physx*) and
./{g}_newton_common_physics.usda (Newton*, + the pad material binding); Mjc* tuning is
in ./{g}_newton_fourbar_physics.usda.

Only the tip collider of the selected "Fingertip" variant exists in the composed stage;
the overs for the other tips are inert. They are authored here rather than in the
fingertip payloads so a tip collider never composes under Physics = "None" (which would
leave a world-static actor with no rigid body).
Generated by devel_helpers/gen_2f_layers.py.""")
          + "\n" + block(0, 'over "Meshes"', meshes(d["colliders"], fn)))


def gen_parallel_weld(g, d):
    welds = [block(8, f'over "{s}_outer_finger"', weld(12, g, f"{s}_outer_knuckle", f"{s}_outer_finger")) for s in SIDES]
    write(g, "parallel_weld", header(g, f"""Engine-neutral parallel-grip weld for the {d['label']} (UsdPhysics only): outer_finger is
rigidly welded to outer_knuckle (a PhysicsFixedJoint, unbreakable like the fingertip
welds), which removes the four-bar's distal DOF. Sublayered by both *_parallel_grip Physics
variants -- ./{g}_parallel_grip_physics.usda and ./{g}_newton_parallel_grip_physics.usda --
and by nothing else, so it composes only inside those variants. Each engine's payload
keeps its own way of making the rest of the linkage parallel (PhysX: mimic joints on an
open tree; Newton: the four-bar loop closure). All link frames are identity/world, so
the weld frames are identity. The prim path matches the robotJoints entries in
../configuration/{g}_robot.usda.
Generated by devel_helpers/gen_2f_layers.py.""")
          + "\n" + gripper_prims(g, join(*welds)))


# ------------------------------------------------------------------ PhysX
def physx_assembly(g, body, inner_api=()):
    meta = ([api(*inner_api)] if inner_api else []) + ['kind = "assembly"']
    return block(0, f'def Xform "{g}"', block(4, f'over Xform "{g}"', body, meta), ['kind = "assembly"'])


def gen_physx_common(g, d):
    ref = f"</{g}/{g}/Joints/finger_joint>"
    joints = block(8, 'over "Joints"', join(
        block(12, 'over "finger_joint"',
              attrs(16, "float drive:angular:physics:targetPosition = 0", "float drive:angular:physics:targetVelocity = 0",
                    "float physxJoint:maxJointVelocity = 86"),
              [api("PhysxJointAPI", "PhysicsDriveAPI:angular")]),
        comment("The right finger follows finger_joint through a PhysX mimic (gearing -1, mirror).\n"
                "Its stiffness (naturalFrequency / dampingRatio) is per variant.", 12)
        + block(12, 'over "right_outer_knuckle_joint"',
                attrs(16, "float physxJoint:maxJointVelocity = 10000", "float physxMimicJoint:rotZ:gearing = -1",
                      f"rel physxMimicJoint:rotZ:referenceJoint = {ref}"),
                [api("PhysxJointAPI", "PhysxMimicJointAPI:rotZ")])))
    body = attrs(8, "bool physxArticulation:enabledSelfCollisions = 0") + "\n" \
        + comment("""PhysX joint content identical in both PhysX grip variants, so they cannot drift apart:
the finger_joint drive API, zero targets and velocity cap (deg/s), and the
right_outer_knuckle mimic wiring. Drive gains, armature and the mimic stiffness stay in
each variant payload.""", 8) + joints

    def fn(path, role, indent):
        return [api("PhysxCollisionAPI", "PhysxConvexHullCollisionAPI")], ""
    groups = join(
        block(4, 'def PhysicsCollisionGroup "CollisionGroup"',
              attrs(8, "rel collection:colliders:includes = </Meshes>", "rel physics:filteredGroups")),
        block(4, 'def PhysicsCollisionGroup "CollisionGroup_01"',
              attrs(8, f"rel collection:colliders:includes = </{g}>", "rel physics:filteredGroups = </Meshes/CollisionGroup>")))
    meshes_body = meshes(d["colliders"], fn) + "\n" + comment(
        "The /Meshes collision groups keep the shared /Meshes colliders from colliding with the\n"
        "articulation-body copies (each link's instanceable visuals reference /Meshes).", 4) + groups
    write(g, "physx_common", header(g, f"""PhysX-only physics shared by the {d['label']} Physx_parallel_grip and Physx_compliant
variants: the PhysX articulation API, the PhysX joint content both grip formulations
share, the /Meshes collision-group plumbing, and the PhysX collision schemas on every
collider (which meshes collide, as UsdPhysics convex hulls, is the engine-neutral
./{g}_collision_physics.usda). Everything here carries Physx* schemas, so the Newton
variants never sublayer it (RV.011; Isaac's Newton import would read Physx* attributes as
a fallback). Nothing here composes under Physics = "None".
Generated by devel_helpers/gen_2f_layers.py.""", ["base.usda"])
          + "\n" + physx_assembly(g, body, ["PhysxArticulationAPI"]) + "\n" + block(0, 'over "Meshes"', meshes_body))


def physx_sublayers(g, extra=()):
    return [f"{g}_physx_common_physics.usda", f"{g}_collision_physics.usda", *extra,
            f"{g}_body_mass_physics.usda", f"{g}_fingertip_physics.usda", f"{g}_kinematics_physics.usda"]


def mimic(name, axis, gearing, g, extra=()):
    return block(12, f'over "{name}"',
                 attrs(16, *extra, "float physxJoint:maxJointVelocity = 10000",
                       f"float physxMimicJoint:{axis}:dampingRatio = 0", f"float physxMimicJoint:{axis}:gearing = {gearing}",
                       f"float physxMimicJoint:{axis}:naturalFrequency = 0",
                       f"rel physxMimicJoint:{axis}:referenceJoint = </{g}/{g}/Joints/finger_joint>"),
                 ['delete apiSchemas = ["PhysicsDriveAPI:angular"]', api("PhysxJointAPI", f"PhysxMimicJointAPI:{axis}")])


def gen_parallel_grip(g, d):
    joints = join(
        block(12, 'over "finger_joint"',
              attrs(16, "float drive:angular:physics:damping = 0.0002", "float drive:angular:physics:maxForce = 26",
                    "float drive:angular:physics:stiffness = 3", "float physxJoint:armature = 0.0001")),
        block(12, 'over "right_outer_knuckle_joint"',
              attrs(16, "float physxJoint:armature = 0.0001", "float physxMimicJoint:rotZ:dampingRatio = 0",
                    "float physxMimicJoint:rotZ:naturalFrequency = 0"),
              ['delete apiSchemas = ["PhysicsDriveAPI:angular"]']),
        mimic("right_inner_finger_joint", "rotX", "1", g),
        mimic("right_inner_finger_knuckle_joint", "rotX", "-1", g),
        mimic("left_inner_finger_knuckle_joint", "rotX", "1", g),
        mimic("left_inner_finger_joint", "rotX", "-1", g),
        comment("Compliant four-bar joints the open-tree parallel grip does not use.", 12)
        + "".join(f'            over "{j}" (active = false) {{}}\n'
                  for j in ("left_outer_finger_joint", "right_outer_finger_joint", "left_inner_knuckle_joint", "right_inner_knuckle_joint")))
    body = attrs(8, "int physxArticulation:solverPositionIterationCount = 64") + "\n" + comment("""PhysX engine physics on the shared neutral joints: the finger_joint driver gains +
armature, and the mimic coupling that algebraically ties the passive joints to it (the
finger_joint drive API and the right_outer_knuckle mimic wiring are in physx_common).
PhysX articulations are trees, so the open-tree parallel grip does NOT use the compliant
four-bar's outer_finger / inner_knuckle revolutes -> suppressed (active=false); the
outer_knuckle<->outer_finger link is the rigid weld of ./""" + g + "_parallel_weld_physics.usda.", 8) \
        + block(8, 'over "Joints"', joints)
    write(g, "parallel_grip", header(g, f"""Physics = "Physx_parallel_grip" payload of the {d['label']}: an open-tree articulation
whose passive joints follow finger_joint through PhysX mimics, so the pads stay parallel.
Generated by devel_helpers/gen_2f_layers.py.""", physx_sublayers(g, [f"{g}_parallel_weld_physics.usda"]))
          + "\n" + physx_assembly(g, body))


def gen_compliant(g, d):
    def drive(name, *extra):
        lines = ["float drive:angular:physics:damping = 0", "float drive:angular:physics:stiffness = 0"]
        return block(12, f'over "{name}"', attrs(16, *lines, *extra), [api("PhysicsDriveAPI:angular")])

    def pad(name, target):
        return block(12, f'over "{name}"',
                     attrs(16, "float drive:angular:physics:damping = 0.00001", "float drive:angular:physics:maxForce = 0.5",
                           f"float drive:angular:physics:stiffness = {d['pad_spring_physx']}",
                           f"float drive:angular:physics:targetPosition = {target}",
                           "uniform bool physics:excludeFromArticulation = 1"),
                     [api("PhysicsDriveAPI:angular")])
    joints = join(
        block(12, 'over "finger_joint"',
              attrs(16, "float drive:angular:physics:damping = 0.0002", "float drive:angular:physics:maxForce = 16.5",
                    "float drive:angular:physics:stiffness = 0.17")),
        block(12, 'over "right_outer_knuckle_joint"',
              attrs(16, "float physxMimicJoint:rotZ:dampingRatio = 0.01", "float physxMimicJoint:rotZ:naturalFrequency = 5000"),
              ['delete apiSchemas = ["PhysicsDriveAPI:angular"]']),
        *[x for s, sign in (("right", "-"), ("left", "")) for x in (
            drive(f"{s}_outer_finger_joint"),
            pad(f"{s}_inner_finger_joint", f"{sign}92"),
            drive(f"{s}_inner_knuckle_joint", "uniform bool physics:excludeFromArticulation = 0"),
            drive(f"{s}_inner_finger_knuckle_joint"))])
    body = comment("""PhysX engine physics on the shared neutral joints. All 10 revolutes are used: the
closed four-bar loop is cut at inner_finger_joint (excludeFromArticulation), which
carries the weak pad spring (target +-92 deg) that holds the pad parallel until an object
pushes it; the right_outer_knuckle mimic stiffness and the finger_joint gains are this
variant's (the drive API and mimic wiring are in physx_common); the other passive joints
carry zero-gain drives so the loop is free to articulate.""", 8) + block(8, 'over "Joints"', joints)
    write(g, "compliant", header(g, f"""Physics = "Physx_compliant" payload of the {d['label']}: the closed four-bar, so the
fingers wrap around an object (encompassing grip) once the pads touch it.
Generated by devel_helpers/gen_2f_layers.py.""", physx_sublayers(g))
          + "\n" + physx_assembly(g, body))


# ------------------------------------------------------------------ Newton
def gen_newton_common(g, d):
    inertia = attrs(12, "float3 physics:diagonalInertia = (0.001, 0.001, 0.001)")
    axes = attrs(12, "quatf physics:principalAxes = (1, 0, 0, 0)")
    links = [block(8, f'over "{s}_outer_knuckle"', inertia) for s in SIDES] + [
        block(8, f'over "{s}_{n}"', inertia + axes) for s in SIDES for n in ("outer_finger", "inner_knuckle", "inner_finger")]
    material = block(8, 'def Scope "Physics"', block(12, 'def Material "GripMaterial"', attrs(
        16, "float physics:dynamicFriction = 0.7", "float physics:staticFriction = 0.7",
        "float newton:rollingFriction = 0.0001", "float newton:torsionalFriction = 0.005"),
        [api("PhysicsMaterialAPI", "NewtonMaterialAPI")]))
    body = comment("""Newton's intentional isotropic (0.001)^3 diagonal inertia (identity principal axes,
overridden together for RB.COL.004) on the moving links, over the shared mass layer.
It stabilizes the soft loop-closure equalities; see ../newton/README.md.""", 8) + join(*links) + "\n" \
        + comment("Fingertip pad friction. The fourbar overlay adds MjcMaterialAPI.", 8) + material

    def fn(path, role, indent):
        schemas = ["NewtonCollisionAPI", "NewtonMeshCollisionAPI"] + (["MaterialBindingAPI"] if role == "tip" else [])
        lines = (["rel material:binding:physics = </%s/%s/Physics/GripMaterial>" % (g, g)] if role == "tip" else []) \
            + ["float newton:contactGap = 0"]
        return [api(*schemas)], attrs(indent + 4, *lines)
    write(g, "newton_common", header(g, f"""Newton-pure physics shared by the {d['label']} Newton_compliant and Newton_parallel_grip
variants: the isotropic inertia override, the GripMaterial pad friction, and the Newton
collision schemas on every collider (+ the pad material binding). Mass + CoM come from
./{g}_body_mass_physics.usda and the fingertip bodies from
./{g}_fingertip_physics.usda (both shared with PhysX). Only UsdPhysics + Newton* here, no
Physx*/Mjc* (RV.011); the MuJoCo tuning is in ./{g}_newton_fourbar_physics.usda.
Generated by devel_helpers/gen_2f_layers.py.""")
          + "\n" + gripper_prims(g, body) + "\n" + block(0, 'over "Meshes"', meshes(d["colliders"], fn)))


def gen_newton_fourbar(g, d):
    lim = ("uniform double[] mjc:solimplimit = " + SOLIMP, "uniform double[] mjc:solreflimit = [0.02, 2]")
    pin = {j[0]: j[3] for j in d["joints"]}
    joints = [
        block(12, 'over "finger_joint"', attrs(
            16, "float drive:angular:physics:damping = 2.7576203", "float drive:angular:physics:maxForce = 15",
            "float drive:angular:physics:stiffness = 24.993114", "float drive:angular:physics:targetPosition = 0",
            "uniform double mjc:armature = 0.3", "uniform double mjc:damping = 0.1", *lim),
            [api("MjcJointAPI", "PhysicsDriveAPI:angular")]),
        block(12, 'over "right_outer_knuckle_joint"', attrs(
            16, "uniform double mjc:armature = 0", "uniform double mjc:damping = 0",
            "uniform double[] mjc:solimp = " + SOLIMP, lim[0], "uniform double[] mjc:solref = [0.005, 1]", lim[1],
            f"rel mjc:target = </{g}/{g}/Joints/finger_joint>", f"rel newton:mimicJoint = </{g}/{g}/Joints/finger_joint>"),
            [api("MjcJointAPI", "NewtonMimicAPI")]),
        *[block(12, f'over "{s}_inner_knuckle_joint"', attrs(
            16, "uniform double mjc:armature = 0", "uniform double mjc:damping = 0", lim[1]), [api("MjcJointAPI")]) for s in SIDES],
        comment("PhysX four-bar revolutes Newton replaces with the loop_closure spherical equalities\n"
                "(below) -> suppressed. The tree is then base -> outer_knuckle -> outer_finger ->\n"
                "inner_finger (pad) and base -> inner_knuckle, exactly the PhysX body order.", 12)
        + "".join(f'            over "{s}_inner_finger_knuckle_joint" (active = false) {{}}\n' for s in SIDES),
        comment(f"""The pad joints (outer_finger -> inner_finger), live as on PhysX, carrying the PhysX
pad spring ({d['pad_spring_physx']} N.m/deg = {d['pad_spring_mjc']} N.m/rad) with springref +/-1.5708 rad
(90 deg preload, PhysX sign). It holds the pad against the outer_finger_joint 0 deg limit
(parallel) until the object pushes it off. The passive loop joints run without joint
damping (tuned live).""", 12)
        + join(*[block(12, f'over "{s}_inner_finger_joint"', attrs(
            16, "uniform double mjc:armature = 0", "uniform double mjc:damping = 0", *lim,
            f"uniform double mjc:springref = {ref}", f"uniform double mjc:stiffness = {d['pad_spring_mjc']}"),
            [api("MjcJointAPI")]) for s, ref in (("left", "1.5708"), ("right", "-1.5708"))]),
        comment("Loop closures at the inner_finger <-> inner_knuckle pin (the PhysX\n"
                "inner_finger_knuckle_joint origin).", 12)
        + join(*[block(12, f'def PhysicsSphericalJoint "{s}_loop_closure"', attrs(
            16, "uniform double[] mjc:solimp = " + SOLIMP, "uniform double[] mjc:solref = [0.005, 1]",
            f"rel physics:body0 = </{g}/{g}/{s}_inner_finger>", f"rel physics:body1 = </{g}/{g}/{s}_inner_knuckle>",
            "uniform bool physics:excludeFromArticulation = 1") + frames(16, pin[f"{s}_inner_finger_knuckle_joint"], rot),
            [api("MjcEqualityConnectAPI")]) for s, rot in (("right", "(1, 0, 0, 0)"), ("left", "(0, 0, 0, 1)"))]),
    ]
    groups = comment("""inner_knuckle <-> inner_finger are joined only by the loop_closure equality (no
revolute, so MuJoCo does not exclude them as parent/child) and their hulls overlap at the
knuckle pin. Newton's USD importer ignores physics:filteredPairs on body prims and the
colliders are instance proxies of shared meshes, so the exclusion is done with collision
groups: a shape in a positive group only collides with its own group and with the
default (negative) group, so two distinct groups never touch each other while both still
collide with everything else (objects, the base, the other finger).""", 12) + join(*[
        block(12, f'def PhysicsCollisionGroup "{grp}"', attrs(16, "prepend rel collection:colliders:includes = [")
              + "".join(f"                    </{g}/{g}/{s}_{link}>,\n" for s in SIDES) + attrs(16, "]"))
        for grp, link in (("InnerKnuckles", "inner_knuckle"), ("InnerFingers", "inner_finger"))]) + "\n" \
        + block(12, 'over "GripMaterial"', "", [api("MjcMaterialAPI")])
    body = comment("""Newton/MuJoCo engine physics on the shared neutral joints: the finger_joint driver, the
right_outer_knuckle NewtonMimic, the mjc solver params and the pad springs; Newton closes
the four-bar with the loop_closure spherical equalities. left/right_outer_finger_joint is
NOT touched here -- its treatment is the per-variant delta.""", 8) \
        + block(8, 'over "Joints"', join(*joints)) + "\n" \
        + comment("MuJoCo pad-material tuning on top of the shared Newton GripMaterial, and the Newton\n"
                  "collision groups (the scope is /G/G/Physics, as in newton_common).", 8) \
        + block(8, 'over "Physics"', groups)

    def fn(path, role, indent):
        full = ["uniform int mjc:condim = 3", "uniform double mjc:gap = 0", "uniform int mjc:group = 2",
                "uniform double mjc:margin = 0", "uniform int mjc:priority = 1", "uniform double[] mjc:solimp = " + SOLIMP,
                "uniform double mjc:solmix = 1", "uniform double[] mjc:solref = [0.004, 2]"]
        lines = {"base": full, "inner_knuckle": full, "link": ["uniform int mjc:group = 2"],
                 "tip": ["uniform int mjc:group = 3", "uniform int mjc:priority = 1",
                         "uniform double[] mjc:solimp = " + SOLIMP, "uniform double[] mjc:solref = [0.004, 2]"]}[role]
        return [api("MjcCollisionAPI")], attrs(indent + 4, *lines)
    write(g, "newton_fourbar", header(g, f"""Newton (MuJoCo-Warp) four-bar physics of the {d['label']}, shared by Newton_compliant and
Newton_parallel_grip: the finger_joint driver, the right_outer_knuckle NewtonMimic, the pad
springs on the inner_finger joints (same joint and preload as PhysX), the loop_closure
spherical equalities (MjcEqualityConnectAPI) that close the four-bar at the inner_finger
<-> inner_knuckle pin in place of the inner_finger_knuckle revolutes, the inner_knuckle /
inner_finger collision groups, and the per-material / per-mesh MuJoCo tuning. The Mjc*
data lives here and not in ./{g}_newton_common_physics.usda so that layer stays
Newton-pure (RV.011).

Each Newton variant payload sublayers this (stronger than kinematics) and adds ONLY its
distal-coupler delta:
  * newton_compliant:     left/right_outer_finger_joint as live revolutes (encompassing grip)
  * newton_parallel_grip: left/right_outer_finger_joint suppressed + the outer_finger weld
                          (./{g}_parallel_weld_physics.usda) -> parallel grip
Generated by devel_helpers/gen_2f_layers.py.""")
          + "\n" + gripper_prims(g, body) + "\n"
          + comment("MuJoCo per-mesh solver tuning on top of the shared Newton convex-hull colliders.", 0)
          + block(0, 'over "Meshes"', meshes(d["colliders"], fn)))


def newton_sublayers(g, extra=()):
    return [f"{g}_newton_fourbar_physics.usda", f"{g}_newton_common_physics.usda", f"{g}_collision_physics.usda", *extra,
            f"{g}_body_mass_physics.usda", f"{g}_fingertip_physics.usda", f"{g}_kinematics_physics.usda", "base.usda"]


def gen_newton_compliant(g, d):
    joints = join(*[block(12, f'over "{s}_outer_finger_joint"', attrs(
        16, "uniform double mjc:armature = 0", "uniform double mjc:damping = 0",
        "uniform double[] mjc:solimplimit = " + SOLIMP, "uniform double[] mjc:solreflimit = [0.00707, 2]"),
        [api("MjcJointAPI")]) for s in SIDES])
    body = comment("Compliant: the distal coupler DOF stays live (MuJoCo tuning only).", 8) + block(8, 'over "Joints"', joints)
    write(g, "newton_compliant", header(g, f"""Physics = "Newton_compliant" payload of the {d['label']}: the shared Newton four-bar with its
distal coupler DOF kept live -- the left/right_outer_finger_joint revolutes stay active, so
the linkage closes by wrapping around the object (encompassing grip).
Generated by devel_helpers/gen_2f_layers.py.""", newton_sublayers(g))
          + "\n" + gripper_prims(g, body))


def gen_newton_parallel_grip(g, d):
    body = comment(f"""Parallel: the distal coupler DOF is removed -- outer_finger is welded to outer_knuckle
in ./{g}_parallel_weld_physics.usda, and its revolutes are suppressed.""", 8) + block(
        8, 'over "Joints"', "".join(f'            over "{s}_outer_finger_joint" (active = false) {{}}\n' for s in SIDES))
    write(g, "newton_parallel_grip", header(g, f"""Physics = "Newton_parallel_grip" payload of the {d['label']}: the shared Newton four-bar with
its distal coupler DOF removed (outer_finger welded to outer_knuckle, its revolutes
suppressed), which forces the linkage PARALLEL. outer_finger stays in the articulation (the
inner_finger pad joint hangs off it). The closed loop makes this rigid, unlike an open
tree coupled by independent mimic constraints.
Generated by devel_helpers/gen_2f_layers.py.""", newton_sublayers(g, [f"{g}_parallel_weld_physics.usda"]))
          + "\n" + gripper_prims(g, body))


GENERATORS = [gen_kinematics, gen_body_mass, gen_fingertip, gen_collision, gen_parallel_weld,
              gen_physx_common, gen_parallel_grip, gen_compliant,
              gen_newton_common, gen_newton_fourbar, gen_newton_compliant, gen_newton_parallel_grip]

if __name__ == "__main__":
    for g, d in GRIPPERS.items():
        for gen in GENERATORS:
            gen(g, d)
        print(f"{g}: {len(GENERATORS)} layers written")
