#!/usr/bin/env python3
"""Author a UR5e + Newton 2F-85 scene (pure USD, no running Isaac app).

    USDLIB=$(dirname $(find ~/isaacsim/extscache -maxdepth 2 -type d -path "*omni.usd.libs*/pxr" | head -1))
    PYTHONPATH="$USDLIB:$PYTHONPATH" \
    LD_LIBRARY_PATH="$USDLIB/bin:$LD_LIBRARY_PATH" \
    ~/isaacsim/python.sh grippers/demo/newton_teleop/author_scene.py

Open the result with the NEWTON experience:
    ISAACSIM_LAUNCHER=isaac-sim.newton.sh ./launch_isaac_with_mcp.sh
and after every stop -> play run
grippers/Gripper_2F85_newton/apply_gripper_tuning.py.

FOUR THINGS LEARNED THE HARD WAY
  1. NO FIXED JOINT for the gripper mount. Newton/MuJoCo cannot represent a
     body-to-body FixedJoint and refuses to build the model:
         [Newton] Initialization failed: Cannot merge joint
           /World/ur5e/wrist_3_link/gripper_fixed_joint of type FixedJoint
     In MuJoCo semantics a child body with NO joint to its parent is already
     rigidly welded, so parenting the gripper under wrist_3_link is the mount.
  2. The arm is authored with its Physics variant set to "None" and the
     gripper KEEPS its own PhysicsArticulationRootAPI. These go together: with
     the arm out of physics there is no articulation on the arm side, so the
     gripper has to own its own root. Composing them into a single root -- what
     you would do for a DYNAMIC arm -- leaves the gripper unsimulated here.
  3. endTimeCode MUST be non-zero. The gripper asset ships start=end=0; with no
     time range Kit's timeline cannot advance and physics never steps (it
     reports playing=True while t stays 0.000).
  4. Authoring must happen OUTSIDE a live session. The MCP extension rebuilds a
     Newton simulation view per call and fails during a stage swap.

PhysX-only attributes (physxCollision:*, physxRigidBody:*) from the PhysX
teleop scene are deliberately NOT copied -- Newton ignores them. Only portable
UsdPhysics values (mass, friction, collision) carry over, so grasp feel will
differ from the PhysX scene and needs its own tuning pass.
"""
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

UR5E = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
        "Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd")
GRID = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
        "Assets/Isaac/6.0/Isaac/Environments/Grid/default_environment.usd")
# Gripper asset. Defaults to this repo's own copy, one level up, so no
# configuration is needed; override with NEWTON_GRIPPER_ASSET to point at
# another checkout. Note the geometry .usd meshes are git-LFS tracked and are
# useless ~130-byte pointers until `git lfs pull` has run.
_REPO_GRIPPER = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                 "Gripper_2F85_newton", "Robotiq_2F85_newton.usda"))
GRIPPER = os.environ.get("NEWTON_GRIPPER_ASSET", _REPO_GRIPPER)
OUT = os.environ.get(
    "NEWTON_SCENE_OUT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "ur5e_2F85_newton.usda"))

GRIP_PATH = "/World/ur5e/wrist_3_link/gripper_2f85"
# same world placement the PhysX teleop scene uses, so the object positions
# copied below land in the same spot relative to the arm
ARM_XLATE = Gf.Vec3d(-0.2035122960805893, -0.21649867296218872, 0.0)


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    stage = Usd.Stage.CreateNew(OUT)

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(1_000_000.0)          # note 3
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetFramesPerSecond(60.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    scene_prim = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
    # ---- Newton / MuJoCo solver cost -------------------------------------
    # Measured on this scene (RTX A4000 laptop): the GPU is saturated by the
    # CONSTRAINT SOLVE, not by rendering or collision.
    #   * dropping render 1280x720 -> 320x180 gained only +5% RTF
    #   * the model has just 25 geoms and ZERO arm colliders, so contacts are cheap
    # timeStepsPerSecond is left at 1000 as the safe default. Lower rates
    # diverge to NaN ONLY on an unpatched Newton -- that was a consequence of
    # the limit-gain importer bug, not a real constraint. With
    # apply_newton_2736_patch.py applied, 500 Hz is stable and faster
    # (RTF 0.420 -> 0.486); 250 Hz is untested post-patch. It stays at 1000
    # here because the patch lives in the Isaac install, not in git, so a fresh
    # machine runs unpatched until someone re-applies it.
    # Schemas are added by name so this authors fine outside Isaac (the mujoco
    # schema python module is not importable standalone).
    # custom=False matters: CreateAttribute defaults to custom=True, and a
    # custom attribute is NOT read by the Mjc/Newton schema -- the model still
    # built with iterations=100. These must be authored as schema attributes.
    scene_prim.AddAppliedSchema("NewtonSceneAPI")
    scene_prim.AddAppliedSchema("MjcSceneAPI")
    scene_prim.CreateAttribute("newton:timeStepsPerSecond",
                               Sdf.ValueTypeNames.Int, False).Set(1000)
    # Deliberately no mjc:option:tolerance here: 1e-6 is Newton's own default
    # despite the schema advertising 1e-8, so authoring it changes nothing.
    # mjc:option:iterations is not a usable cost lever either -- Newton does not
    # honour it from USD at all (authored 30, the model still builds with 100).

    # ---- lighting (values copied from the PhysX teleop scene) --------------
    env = UsdGeom.Xform.Define(stage, "/World/Environment")
    light = UsdLux.DistantLight.Define(stage, "/World/Environment/defaultLight")
    light.CreateIntensityAttr(3000.0)
    light.CreateAngleAttr(1.0)
    light.CreateExposureAttr(1.5)
    light.GetPrim().CreateAttribute("inputs:normalize", Sdf.ValueTypeNames.Bool).Set(True)
    # AddOrientOp() defaults to Quatf precision -- passing Quatd raises
    # "Type mismatch ... expected 'GfQuatf', got 'GfQuatd'"
    UsdGeom.Xformable(light.GetPrim()).AddOrientOp().Set(
        Gf.Quatf(0.6532815, Gf.Vec3f(0.27059805, 0.27059805, 0.65328148)))

    grid = stage.DefinePrim("/World/FlatGrid")
    grid.GetReferences().AddReference(GRID)

    # physics ground so dropped objects have something to land on
    ground = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
    ground.CreateAxisAttr("Z")
    ground.CreateWidthAttr(10.0)
    ground.CreateLengthAttr(10.0)
    UsdGeom.Imageable(ground.GetPrim()).CreateVisibilityAttr("invisible")
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # ---- shared physics material (from the PhysX scene) -------------------
    mat = UsdPhysics.MaterialAPI.Apply(
        stage.DefinePrim("/World/PhysicsMaterial", "Material"))
    mat.CreateStaticFrictionAttr(1.0)
    mat.CreateDynamicFrictionAttr(0.9)

    def graspable(prim, mass):
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        UsdPhysics.MaterialAPI(prim)  # binding below
        rel = prim.CreateRelationship("material:binding:physics", False)
        rel.SetTargets([Sdf.Path("/World/PhysicsMaterial")])

    # ---- graspable cube (2 cm, 100 g) -------------------------------------
    # IMPORTANT: set INTRINSIC dimensions, never a scale xformOp. The PhysX
    # scene uses size=1 + scale=0.02, but Isaac's renderer ignores the scale op
    # on these gprims -- USD composition reports 0.02 m while the viewport draws
    # a 1 m cube, which silently engulfs the whole robot and looks like "the
    # robot is missing".
    cube = UsdGeom.Cube.Define(stage, "/World/Cube")
    cube.CreateSizeAttr(0.02)
    UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0.3703595735293894, -0.101588296036721, 0.009946728221864697))
    graspable(cube.GetPrim(), 0.1)

    # ---- graspable cylinder (100 g, lying on its side) --------------------
    cyl = UsdGeom.Cylinder.Define(stage, "/World/Cylinder")
    cyl.CreateAxisAttr("Z")
    cyl.CreateRadiusAttr(0.025)          # intrinsic, not scaled -- see above
    cyl.CreateHeightAttr(0.10)
    yx = UsdGeom.Xformable(cyl.GetPrim())
    yx.AddTranslateOp().Set(Gf.Vec3d(0.2538029387070952, 0.0,
                                     0.024002205408456715))
    yx.AddOrientOp().Set(Gf.Quatf(0.70710678,
                                  Gf.Vec3f(0.70710678, 0.0, 0.0)))
    graspable(cyl.GetPrim(), 0.1)

    # ---- robot + gripper --------------------------------------------------
    arm = UsdGeom.Xform.Define(stage, "/World/ur5e")
    arm.GetPrim().GetReferences().AddReference(UR5E)
    UsdGeom.Xformable(arm.GetPrim()).AddTranslateOp().Set(ARM_XLATE)

    # Physics variant "None" takes the arm out of the solver -- see the header.
    # Authored through Sdf rather than Usd.Prim.GetVariantSets(), because the
    # https UR5e reference cannot be resolved outside Isaac, so the variant set
    # is not discoverable here; a variant SELECTION can still be written blind.
    arm_spec = stage.GetRootLayer().GetPrimAtPath(Sdf.Path("/World/ur5e"))
    arm_spec.variantSelections["Physics"] = "None"
    assert dict(arm_spec.variantSelections) == {"Physics": "None"}, \
        "arm Physics variant selection did not author"

    grip = UsdGeom.Xform.Define(stage, GRIP_PATH)
    grip.GetPrim().GetReferences().AddReference(GRIPPER)
    # identity mount, exactly as the PhysX scene mounts Robotiq_2F_85_edit;
    # NO fixed joint -- see note 1
    UsdGeom.Xformable(grip.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

    # ---- default camera ---------------------------------------------------
    # Oriented so the gamepad matches the view: stick left moves the gripper
    # left on screen, stick up moves it away. This is the ORIGINAL framing
    # rotated 180 deg about Z through the look-at target -- from the other side
    # the on-screen left/right and near/far were mirrored relative to the
    # sticks. NOTE the stick mapping itself is in the TOOL frame, not world, so
    # the match also depends on the wrist orientation; if it feels inverted in
    # some pose, that is the driver's axis mapping, not this camera.
    eye = Gf.Vec3d(-0.25, 0.65, 0.45)
    target = Gf.Vec3d(0.30, -0.05, 0.08)
    view = Gf.Matrix4d()
    view.SetLookAt(eye, target, Gf.Vec3d(0, 0, 1))
    cam = UsdGeom.Camera.Define(stage, "/World/DemoCam")
    cam.CreateFocalLengthAttr(30.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
    cx = UsdGeom.Xformable(cam.GetPrim())
    cx.ClearXformOpOrder()
    cx.AddTransformOp().Set(view.GetInverse())

    stage.GetRootLayer().Save()

    # ---- start pose: identical to the PhysX teleop scene ------------------
    # Copied from its IsaacNamedPose /World/ur5e/NamedPoses/pose_1
    #   isaac:robot:pose:jointValues (DEGREES)
    # USD angular joint state/drive values are in degrees too, so these go in
    # verbatim. Authoring BOTH the state and the drive target means the arm
    # starts here AND holds here instead of sagging under gravity.
    #
    # NOTE: changing this at runtime while the teleop is up does not stick --
    # newton_kinematic_teleop.py rewrites every link transform each tick from
    # its own joint state. Stop the teleop, set the pose, then restart it so it
    # initialises FROM this pose.
    START_POSE_DEG = {
        "shoulder_pan_joint": -0.00006633832,
        "shoulder_lift_joint": -62.5,
        "elbow_joint": 107.499954,
        "wrist_1_joint": -134.49998,
        "wrist_2_joint": -94.50019,
        "wrist_3_joint": -33.984856,
    }
    stage = Usd.Stage.Open(OUT)
    for jname, deg in START_POSE_DEG.items():
        jp = "/World/ur5e/joints/" + jname
        prim = stage.GetPrimAtPath(jp)
        if not (prim and prim.IsValid()):
            print("  WARN: joint not found, skipping:", jp)
            continue
        prim.CreateAttribute("state:angular:physics:position",
                             Sdf.ValueTypeNames.Float).Set(float(deg))
        prim.CreateAttribute("drive:angular:physics:targetPosition",
                             Sdf.ValueTypeNames.Float).Set(float(deg))
    print("start pose authored (deg):",
          {k: round(v, 1) for k, v in START_POSE_DEG.items()})
    stage.GetRootLayer().Save()

    stage = Usd.Stage.Open(OUT)
    print("arm Physics variant: None (kinematic; gripper keeps its own "
          "articulation root)")
    print("no FixedJoint authored (Newton cannot merge one)")
    print("objects: /World/Cube, /World/Cylinder  material: /World/PhysicsMaterial")
    print("lighting: /World/Environment/defaultLight + /World/FlatGrid")
    print("endTimeCode:", stage.GetEndTimeCode())
    print("\nwrote", OUT)
    print("NOTE: the https UR5e reference cannot be resolved outside Isaac, so"
          " articulation-root count must be verified in the app, not here.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
