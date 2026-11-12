#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Robotiq.
"""Headless grasp with ground truth for ``SensorSim=Precise`` in the teleop scene (development tool, not a test).

Opens ``grippers/demo/teleop/ur5robot_with_2F-85.usda`` (the UR5e with the 2F-85 on its wrist), selects the Precise
sensor, frees the pads from the wrist link, puts a kinematic cube between the pads, drives ``finger_joint`` closed and
prints, for each pad, the thinning of the pad (the CNN's input, um), where it is, how it compares with where the cube
really touches (the cube centre in the pad's file frame) and the CNN map. It reproduces in about a minute what the
teleop demo shows, without a gamepad, so that the pad stiffness, the reader and the network's input can be changed and
compared quickly (see tactile/PRECISE_PLAN.md, spikes 13 and 14 for what it found).

    ~/isaacsim-6.2.0/python.sh grippers/tactile/dev/precise_teleop_grasp.py [options]      # (clean env: no ROS PYTHONPATH)

    --shift MM        move the cube MM millimetres toward the fingertip along the pads (default 0)
    --aligned         turn the cube to the gripper's axes (default: world axes, which at the demo's wrist yaw of ~34 deg
                      makes the cube touch the pads along an edge)
    --nocube          no cube: closes the gripper on itself
    --young PA        override the pad's Young's modulus (default: the asset's)
    --density KG_M3   override the pad's density
    --images          also print the two 18 x 12 thinning images
"""

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
SCENE = REPO / "grippers" / "demo" / "teleop" / "ur5robot_with_2F-85.usda"
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--shift", type=float, default=0.0)
ap.add_argument("--aligned", action="store_true")
ap.add_argument("--nocube", action="store_true")
ap.add_argument("--young", type=float)
ap.add_argument("--density", type=float)
ap.add_argument("--images", action="store_true")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import carb.settings  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, str(REPO / "grippers"))
from tactile import make_tactile_sensor, precise  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(str(SCENE))
for _ in range(10):
    app.update()
stage = ctx.get_stage()
# Deformable bodies and GPU dynamics BEFORE the pads appear (open_scene.py does the same)
carb.settings.get_settings().set("/physics/enableDeformableBodies", True)
for scene in (p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)):
    api = PhysxSchema.PhysxSceneAPI.Apply(scene)
    api.CreateEnableGPUDynamicsAttr(True)
    api.CreateBroadphaseTypeAttr("GPU")
stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("Physics").SetVariantSelection("Physx_parallel_grip")
edit = next(p for p in stage.Traverse() if p.GetName() == "Robotiq_2F_85_edit")
edit.GetVariantSets().GetVariantSet("Fingertip").SetVariantSelection("Tactile_TSF85")
edit.GetVariantSets().GetVariantSet("SensorSim").SetVariantSelection("Precise")
for _ in range(5):
    app.update()
root = f"{edit.GetPath()}/Robotiq_2F_85"
precise.mount_pads(stage, root, stage.GetSessionLayer())  # the gripper hangs under a rigid body (the wrist link)
for side in ("left", "right"):
    material = stage.GetPrimAtPath(f"{edit.GetPath()}/{side}_tsf_pad/Looks/Deformable_Material")
    if args.young is not None:
        material.GetAttribute("omniphysics:youngsModulus").Set(args.young)
    if args.density is not None:
        material.GetAttribute("omniphysics:density").Set(args.density)
for _ in range(5):
    app.update()


def world_matrix(path):
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    return np.array([[m[i][j] for j in range(4)] for i in range(4)])  # USD: row vectors


pads = {s: world_matrix(f"{edit.GetPath()}/{s}_tsf_pad") for s in ("left", "right")}  # pad file frame -> world
centres = {s: (np.array([0.0142, -0.0228, 0.0221, 1.0]) @ pads[s])[:3] for s in pads}  # pad centre at rest, world
cube_centre = (centres["left"] + centres["right"]) / 2 + args.shift / 1000.0 * pads["right"][2, :3] / np.linalg.norm(pads["right"][2, :3])
cube = UsdGeom.Cube.Define(stage, "/World/TestCube")
cube.CreateSizeAttr(0.04)
xform = UsdGeom.Xformable(cube)
if args.aligned:  # the cube's axes = the gripper's axes
    rot = pads["right"][:3, :3] / np.linalg.norm(pads["right"][:3, :3], axis=1, keepdims=True)
    matrix = Gf.Matrix4d(1.0)
    for i in range(3):
        matrix.SetRow3(i, Gf.Vec3d(*(float(v) for v in rot[i])))  # (item assignment on a Gf matrix does not write through)
    matrix.SetTranslateOnly(Gf.Vec3d(*cube_centre))
    xform.AddTransformOp().Set(matrix)
else:
    xform.AddTranslateOp().Set(Gf.Vec3d(*cube_centre))
if args.nocube:
    cube.GetPrim().SetActive(False)
else:
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()).CreateKinematicEnabledAttr(True)

sensor = make_tactile_sensor(stage, root, "physx")  # before play; the first read below is the zero
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(60):
    app.update()
sensor.read()
art = Articulation(next(p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)))
finger = list(art.dof_names).index("finger_joint")
for _ in range(240):
    art.set_dof_position_targets(np.array([[0.8]]), dof_indices=[finger])
    app.update()

maps = sensor.read()
poses = sensor._backing_poses()
cube_world = np.array(UsdGeom.Xformable(cube.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
for side, image in sensor.compression_images().items():
    # the cube centre in the pad's file frame, through the backing's present frame (the pad moves with it)
    R, t = poses[side]
    body = precise.body_frame(np.array([cube_world]), R, t)[0]
    rest_file = np.array(stage.GetPrimAtPath(f"{edit.GetPath()}/{side}_tsf_pad/simulation_mesh").GetAttribute("omniphysics:restShapePoints").Get(), dtype=float)
    affine, *_ = np.linalg.lstsq(np.hstack([rest_file, np.ones((len(rest_file), 1))]), sensor.pads[side]["rest_body"], rcond=None)
    centre_file = np.linalg.solve(affine[:3].T, body - affine[3]) * 1000
    positive = np.clip(image, 0, None)
    total = positive.sum()
    active = image > 0.2 * image.max()
    centroid = (np.nan, np.nan) if total <= 0 else ((positive.sum(axis=1) * np.arange(18)).sum() / total, (positive.sum(axis=0) * np.arange(12)).sum() / total)
    cnn = maps[side]
    print(f"[{side}] thinning peak {image.max() * 1e6:.0f} um, {int(active.sum())}/216 cells above 20 % of the peak, background "
          f"{image[~active].mean() * 1e6:+.0f} +- {image[~active].std() * 1e6:.0f} um; centroid row {centroid[0]:.1f} col {centroid[1]:.1f}; "
          f"cube centre in the pad file frame x {centre_file[0]:.1f} z {centre_file[2]:.1f} mm (row {(42.1 - centre_file[2]) / 2.3682:.1f}, "
          f"col {(centre_file[0] - 1.2) / 2.3682:.1f}); CNN map max {cnn.max():.0f} mean {cnn.mean():.0f}")
    if args.images:
        print("\n".join(" ".join(f"{int(v * 1e6):5d}" for v in row) for row in image))
app.close()
