#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Robotiq.
"""Generate the TSF-85 tactile payloads (``SensorSim`` variants) of the 2F grippers that carry it.

The TSF-85 sensor is the same on the 2F-85 (``Fingertip=Tactile_TSF85``) and the 2F-140
(``Fingertip=Tactile_TSF140``); only where its pad sits on the inner finger differs (``MOUNTS``).
Writes, under ``grippers/<gripper>/payloads/``:

* ``<gripper>_tactile_zones.usda``: the 7 x 4 marker grid of each ``<side>_inner_finger``
  (``tactile_zone_visuals/zone_r<row>_c<col>``), shared by both methods;
* ``<gripper>_tactile_raycast.usda``: ``SensorSim = "Raycast"``, the compliant pad and the
  raycast parameters;
* ``<gripper>_tactile_effective_response.usda``: ``SensorSim = "EffectiveResponse"``, the
  Eq. 1 parameters and the articulation kept awake.

The markers do not collide; the readers in this folder only use their poses. Stdlib only:

    python3 grippers/tactile/generate_tactile_zones.py                  # every gripper in MOUNTS
    python3 grippers/tactile/generate_tactile_zones.py Robotiq_2F_140   # one
"""

import sys
from dataclasses import dataclass
from pathlib import Path

ROWS, COLS = 7, 4
SENSING_HEIGHT = 0.038  # m, the 7-row span along local Z
SENSING_WIDTH = 0.0235  # m, the 4-column span along local X
GAP = 0.0004  # m, between markers
ZONE_THICKNESS = 0.004  # m, along the contact normal
ZONE_COLOR = (0.025, 0.025, 0.025)


@dataclass(frozen=True)
class Mount:
    """Where the sensing area sits in each ``<side>_inner_finger`` body frame: 7 rows along +Z (row 0 at the
    fingertip end), 4 columns along +X, the contact normal along Y (left finger at -Y, right mirrored)."""

    center_x: float  # m, sensing-area centre along X
    center_z: float  # m, sensing-area centre along Z
    pad_y: float  # m, |Y| of the marker centres: their front face is flush with the pad's contact surface


GRIPPERS = Path(__file__).resolve().parents[1]
MOUNTS = {
    # TSF-85 case X [-0.0135, 0.0135], Z [0.1024, 0.1539], left Y [-0.0627, -0.0395] (the case is the collider).
    "Robotiq_2F_85": Mount(center_x=0.0, center_z=0.1324, pad_y=0.0416),
    # TSF-140 tip: the sensing pad (mesh_1) is X [-0.01196, 0.01162], Z [0.16822, 0.2068], the same 23.6 x 38.6 mm
    # as the TSF-85's. The collider is the tip body (mesh_0) only, whose face is at |Y| = 0.06807: the visual pad
    # stands 1.9 mm proud of it, so objects stop at the body face and the markers sit flush with that.
    "Robotiq_2F_140": Mount(center_x=-0.00017, center_z=0.18751, pad_y=0.06807 + ZONE_THICKNESS / 2),
}

# The TSF-85 soft layer is 3 mm of E = 1.289 MPa, nu = 0.173 (ETS CoRo lab model,
# github.com/Lab-CORO/TSF-85, doi:10.3389/frobt.2025.1639524).
# PAD_STIFFNESS = E / thickness: pressure per depth, N/mm^3.
PAD_STIFFNESS = 1.289 / 3.0
# PhysX's compliant contact is a linear spring per contact point; 20 kN/m per point gives
# ~85 kN/m on a 12 x 16.5 mm box face, which is PAD_STIFFNESS * face area.
PAD_COMPLIANT_STIFFNESS = 20000.0  # N/m per contact point
PAD_COMPLIANT_DAMPING = 120.0  # N*s/m, same damping ratio as the stiffness it scales with

# Eq. 1 of Shang et al., arXiv 2610.10510 (identified for their pad); sigma in mm, (columns, rows).
ER_SIGMA_MM = (1.5, 1.25)
ER_ETA = 0.7
ER_GAMMA = 0.5

RAY_STANDOFF = 0.02  # m, rays start this far behind each taxel
RAY_MAX_DISTANCE = 0.06  # m, ray length


def f(x):
    return repr(round(float(x), 9))


def zone_offsets(mount, sign):
    """28 ``(x, y, z)`` marker positions in the finger frame, row-major, and the marker size along X and Z.
    ``sign``: -1 for the left finger (at -Y), +1 for the right."""
    pad_y = sign * mount.pad_y
    row_z = (SENSING_HEIGHT - GAP * (ROWS - 1)) / ROWS
    col_x = (SENSING_WIDTH - GAP * (COLS - 1)) / COLS
    z_max = mount.center_z + SENSING_HEIGHT * 0.5
    x_min = mount.center_x - SENSING_WIDTH * 0.5
    offsets = []
    for r in range(ROWS):
        for c in range(COLS):
            z = z_max - ((r + 0.5) * row_z + r * GAP)
            x = x_min + ((c + 0.5) * col_x + c * GAP)
            offsets.append((x, pad_y, z))
    return offsets, col_x, row_z


def marker_cubes(offsets, scale):
    sx, sy, sz = scale
    out = []
    for idx, (x, y, z) in enumerate(offsets):
        r, c = divmod(idx, COLS)
        out.append(f"""\
                def Cube "zone_r{r}_c{c}" (
                    kind = "subcomponent"
                )
                {{
                    float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
                    double size = 1
                    color3f[] primvars:displayColor = [{ZONE_COLOR}]
                    double3 xformOp:translate = ({f(x)}, {f(y)}, {f(z)})
                    double3 xformOp:scale = ({f(sx)}, {f(sy)}, {f(sz)})
                    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
                }}
""")
    return "".join(out)


def grid_layer(gripper):
    mount = MOUNTS[gripper]
    left_off, col_x, row_z = zone_offsets(mount, -1)
    right_off, _, _ = zone_offsets(mount, +1)
    scale = (col_x, ZONE_THICKNESS, row_z)
    return f"""\
#usda 1.0
(
    # Taxel grid shared by the SensorSim variants. GENERATED by grippers/tactile/generate_tactile_zones.py.
    defaultPrim = "{gripper}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{gripper}"
{{
    over "{gripper}"
    {{
        over "left_inner_finger"
        {{
            def Xform "tactile_zone_visuals"
            {{
{marker_cubes(left_off, scale)}\
            }}
        }}

        over "right_inner_finger"
        {{
            def Xform "tactile_zone_visuals"
            {{
{marker_cubes(right_off, scale)}\
            }}
        }}
    }}
}}
"""


def method_layer(gripper, title, body, attrs, inner_meta=""):
    """A layer over the grid: ``robotiq:tactile:*`` attributes on each finger's ``tactile_zone_visuals``."""
    lines = "".join(f"                {a}\n" for a in attrs)
    fingers = "".join(f"""\
        over "{side}_inner_finger"
        {{
            over "tactile_zone_visuals"
            {{
{lines}\
            }}
        }}
""" for side in ("left", "right"))
    return f"""\
#usda 1.0
(
    # {title}
    # GENERATED by grippers/tactile/generate_tactile_zones.py. Layered over {gripper}_tactile_zones.usda.
    defaultPrim = "{gripper}"
    metersPerUnit = 1
    upAxis = "Z"
)

over "{gripper}"
{{
    over "{gripper}"{inner_meta}
    {{
{body}{fingers}    }}
}}
"""


def er_attrs():
    return [f"custom float2 robotiq:tactile:sigmaMm = ({f(ER_SIGMA_MM[0])}, {f(ER_SIGMA_MM[1])})",
            f"custom float robotiq:tactile:eta = {f(ER_ETA)}",
            f"custom float robotiq:tactile:gamma = {f(ER_GAMMA)}"]


def raycast_layer(gripper):
    body = f"""\
        # A soft pad: the gripped object sinks in proportion to grip force, which the rays read as depth.
        over "Physics"
        {{
            over "GripMaterial"
            {{
                float physxMaterial:compliantContactStiffness = {f(PAD_COMPLIANT_STIFFNESS)}
                float physxMaterial:compliantContactDamping = {f(PAD_COMPLIANT_DAMPING)}
                bool physxMaterial:compliantContactAccelerationSpring = 0
            }}
        }}

"""
    attrs = ['custom token robotiq:tactile:method = "raycast"',
             f"custom float robotiq:tactile:rayStandoff = {f(RAY_STANDOFF)}",
             f"custom float robotiq:tactile:rayMaxDistance = {f(RAY_MAX_DISTANCE)}",
             f"custom float robotiq:tactile:padStiffness = {f(PAD_STIFFNESS)}"] + er_attrs()
    return method_layer(gripper, "SensorSim=Raycast: depth per taxel from a ray along the pad normal, over a compliant pad.",
                        body, attrs)


def effective_layer(gripper):
    # A stalled gripper comes to rest and PhysX puts the articulation to sleep; sleeping bodies report no
    # contacts, which this method reads.
    meta = ' (\n        prepend apiSchemas = ["PhysxArticulationAPI"]\n    )'
    body = "        float physxArticulation:sleepThreshold = 0\n\n"
    attrs = ['custom token robotiq:tactile:method = "effectiveResponse"'] + er_attrs()
    return method_layer(gripper, "SensorSim=EffectiveResponse: response r of the pad's contact force and patch (Eq. 1).",
                        body, attrs, meta)


def main(grippers):
    unknown = [g for g in grippers if g not in MOUNTS]
    if unknown:  # before writing anything, so a typo does not leave a partial regeneration
        sys.exit(f"no TSF-85 mount for {', '.join(map(repr, unknown))} (known: {', '.join(MOUNTS)})")
    for gripper in grippers:
        payloads = GRIPPERS / gripper / "payloads"
        layers = {f"{gripper}_tactile_zones.usda": grid_layer(gripper),
                  f"{gripper}_tactile_raycast.usda": raycast_layer(gripper),
                  f"{gripper}_tactile_effective_response.usda": effective_layer(gripper)}
        for name, text in layers.items():
            (payloads / name).write_text(text)
        print("Wrote", ", ".join(f"{gripper}/payloads/{n}" for n in layers))


if __name__ == "__main__":
    main(sys.argv[1:] or list(MOUNTS))
