#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Robotiq.
"""Capture the attachment of the deformable TSF-85 pads to their rigid backings into ``tools/tsf85_pad_attachment.usda``.

PhysX cooks an auto attachment (which pad nodes are pinned to the backing, and which pad elements must not collide
with it) when the simulation first plays. ``tools/generate_tactile_zones.py`` embeds the result in the
``SensorSim=Precise`` payload so that the asset does not depend on that cook at load. Re-run this whenever the pad
or backing placement of the generator changes, then regenerate the payloads. Needs Isaac Sim's Python:

    ~/isaacsim/python.sh grippers/Robotiq_2F_85/tools/capture_tsf85_pad_attachment.py
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

REPO = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "tsf85_pad_attachment.usda"

app = SimulationApp({"headless": True})
sys.path.insert(0, str(REPO / "tests" / "runtime"))

import carb.settings  # noqa: E402
from omni.physx.scripts import deformableUtils  # noqa: E402
from pxr import PhysxSchema, Sdf, UsdPhysics  # noqa: E402

from _scene import load_gripper, step  # noqa: E402

GRIPPER = "Robotiq_2F_85"
built = {}


def setup(stage):
    carb.settings.get_settings().set("/physics/enableDeformableBodies", True)
    scene = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    api = PhysxSchema.PhysxSceneAPI.Apply(scene)
    api.CreateEnableGPUDynamicsAttr(True)
    api.CreateBroadphaseTypeAttr("GPU")
    for side in ("left", "right"):
        stage.GetPrimAtPath(f"/World/Gripper/{side}_pad_attachment").SetActive(False)  # the payload's own, if any
        ok = deformableUtils.create_auto_deformable_attachment(
            stage, Sdf.Path(f"/World/Gripper/{side}_pad_attach_gen"), Sdf.Path(f"/World/Gripper/{side}_tsf_pad"),
            Sdf.Path(f"/World/Gripper/{GRIPPER}/{side}_pad_backing"))
        assert ok, side
        stage.GetPrimAtPath(f"/World/Gripper/{side}_pad_attach_gen").GetAttribute(
            "physxAutoDeformableAttachment:deformableVertexOverlapOffset").Set(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0005)
    built["stage"] = stage


load_gripper(app, GRIPPER, "Physx_parallel_grip", None, setup=setup,
             variants=[("Fingertip", "Tactile_TSF85"), ("SensorSim", "Precise")])
step(app, 30)  # PhysX fills the attachment data at the first play
stage = built["stage"]
layer = Sdf.Layer.CreateAnonymous("attachments.usda")
for side in ("left", "right"):
    Sdf.CopySpec(stage.GetRootLayer(), Sdf.Path(f"/World/Gripper/{side}_pad_attach_gen"), layer, Sdf.Path(f"/{side}_pad_attachment"))
text = layer.ExportToString().replace("/World/Gripper/", "/Robotiq_2F_85/")
OUT.write_text(text)
print("Wrote", OUT, len(text), "bytes")
app.close()
