#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Robotiq.
"""Extract the deformable sensing pad of the TSF-85 into ``parts/TSF_85_pad.usd``.

Source: the TSF-85 Isaac Sim model of the Control and Robotics Laboratory (CoRo) of the Ecole de technologie
superieure (Montreal), ``assets/TSF_85.usd`` of github.com/Lab-CORO/TSF-85, used under agreement with Robotiq.
De la Cruz Sanchez & Roberge, Front. Robot. AI 12:1639524 (2025), doi:10.3389/frobt.2025.1639524.

Keeps ``/World/Deformable`` (the 432-node tetrahedral mesh, its material and its APIs) and drops its attachment to
the rigid case, which is authored on the gripper instead. Two changes, both needed for the pad to carry a grip in
Isaac Sim 6.2 (see tactile/PRECISE_PLAN.md): the material density is raised from the schema default (a 3 g pad, which the
position-based solver cannot hold under a gripper's load) to ``PAD_DENSITY``, and the solver position iteration
count is set to ``SOLVER_ITERATIONS``. Needs Isaac Sim's ``pxr``:

    python3 tools/extract_tsf85_pad.py /path/to/TSF_85.usd
"""

import sys
from pathlib import Path

from pxr import Sdf, Usd

PAD_DENSITY = 4.0e4  # kg/m^3
SOLVER_ITERATIONS = 64
OUT = Path(__file__).resolve().parents[1] / "parts" / "TSF_85_pad.usd"


def main(src: str) -> None:
    src_layer = Sdf.Layer.FindOrOpen(src)
    out = Sdf.Layer.CreateNew(str(OUT))
    out.defaultPrim = "Pad"
    out.metersPerUnit = 1.0
    out.upAxis = "Z"
    root = Sdf.CreatePrimInLayer(out, "/Pad")
    root.specifier = Sdf.SpecifierDef
    Sdf.CopySpec(src_layer, "/World/Deformable", out, "/Pad")
    # the attachment to the case (authored on the gripper instead)
    parent = out.GetPrimAtPath("/Pad")
    del parent.nameChildren["attachment"]
    mat = out.GetPrimAtPath("/Pad/Looks/Deformable_Material")
    # not authored upstream (the schema default is used), so create it
    density = mat.attributes.get("omniphysics:density") or Sdf.AttributeSpec(
        mat, "omniphysics:density", Sdf.ValueTypeNames.Float)
    density.default = PAD_DENSITY
    iters = Sdf.AttributeSpec(parent, "physxDeformableBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int)
    iters.default = SOLVER_ITERATIONS
    gravity = Sdf.AttributeSpec(parent, "physxDeformableBody:disableGravity", Sdf.ValueTypeNames.Bool)
    gravity.default = True
    out.Save()
    stage = Usd.Stage.Open(str(OUT))
    for p in stage.Traverse():
        print(p.GetPath(), p.GetTypeName())
    print("Wrote", OUT, OUT.stat().st_size, "bytes")


if __name__ == "__main__":
    main(sys.argv[1])
