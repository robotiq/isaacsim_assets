# Robotiq Hand-E

Isaac Sim asset for the Robotiq Hand-E: two sliding fingers (25 mm each, 50 mm stroke) on prismatic
joints, or 50 mm each, 100 mm stroke, with the `Stroke100` variant. `Robotiq_Hand_E.usda` is the entry point.

| Variant set | Variants |
| --- | --- |
| `Stroke` | `Stroke50` *(default)*, `Stroke100` — see [Stroke100](#stroke100-100-mm-stroke) |
| `Physics` | `PhysX` *(default)*, `Newton`, `None` |
| `Fingertip` | `Std` *(default)*, `Flat_Overmolded`, `Support`, `Extender`, `Bin_Picking` |
| `FingertipMount` | `Outside` *(default)*, `Inside` — which fingertip screw-hole row is used; `Inside` moves every tip 9 mm towards the centerline |

Only `left_finger_joint` is driven; `right_finger_joint` follows it rigidly (a PhysX mimic joint, or
a stiff MuJoCo joint equality on Newton), and both close with a positive position, 0 (open) to 25 mm (50 mm on `Stroke100`).

Most of the repetitive layers under `payloads/` and the fingertip collision pieces are generated:
see `devel_helpers/gen_hande_layers.py` and `devel_helpers/gen_hande_fingertip_colliders.py`
(the `Stroke100` parts: `devel_helpers/gen_hande_s100_parts.py`).

## Source of the specifications

The finger speed, the grip force and the fingertip force limits come from the Hand-E instruction
manual, "6. Specifications" page:

<https://assets.robotiq.com/website-assets/support_documents/document/online/Hand-E_Aubo_InstructionManual_HTML5_20190501.zip/Hand-E_Aubo_InstructionManual_HTML5/Content/6.%20Specifications.htm>

| Datasheet value | Used for |
| --- | --- |
| Finger speed 20 to 150 mm/s | `physxJoint:maxJointVelocity = 0.15` m/s on the driven joint (the upper end of the range) |
| Grip force 20 – 130 N | the simulated force at each pad: the default tip's 100 N plus a 30 N simulation-only margin (see [Simulation-only extra force](#simulation-only-extra-force)) |
| Forces exerted at the end of the fingertips must not exceed 100 N, and the maximum payload / external force falls with the Z offset of the fingertip (Fig. 6-10 / 6-11, below) | the finger drive force cap, per `Fingertip` variant (plus the simulation-only margin, which exceeds this 100 N limit) |

## Finger force per fingertip

![Maximum payload / external force vs. custom finger Z offset](Hand-E_max_payload_vs_z_offset.png)

*Hand-E: maximum payload / external force vs. custom finger design (Z offset), copied from the page
above. The force is applied at the tip of the finger, in the middle of the inner surface; the Z offset
is measured from the housing face. Blue: finger mounted directly on the rack with 2 M3 screws. Red:
fingertip on a fingertip holder with 2 M3 screws. Yellow: mounted directly on the rack with 3 M3
screws.*

Each `Fingertip` variant takes the curve of how that tip is mounted and its Z offset (measured from the
baked CAD part):

| Fingertip | Mounting (curve) | Z offset | Force at each pad (manual) | Force at each pad (asset) |
| --- | --- | --- | --- | --- |
| `Std` | 2 × M3 on the rack (blue) | 60.1 mm | 100 N | 130 N |
| `Flat_Overmolded` | 2 × M3 on the rack (blue) | 60.3 mm | 100 N | 130 N |
| `Support` | fingertip holder (red) | 20.1 mm | 100 N | 130 N |
| `Extender` | fingertip holder (red) | 20.2 mm | 100 N | 130 N |
| `Bin_Picking` | 3 × M3 on the rack (yellow) | 76.1 mm | 100 N | 130 N |

All of them are on the graph's 100 N plateau; the asset adds the 30 N simulation-only margin to each. The breakpoints of the sloped part of each curve in the
generator were read off the figure (±5 mm / ±5 N); the plateau is exact. If a tip is added whose Z
offset is beyond the plateau, check the value against the figure.

### How the force is modelled

- **The drive cap is twice the force at the pad.** The right finger follows the left one rigidly, so the
  left drive force is shared between the two pads. Measured in the teleop demo: a 130 N cap gave
  65–75 N per pad. The cap on `left_finger_joint` is therefore 260 N for 130 N at each pad (200 N for the
  manual's 100 N, without the simulation-only margin).
- **The drive is stiff** (50000 N/m, damping 200 N·s/m). It is a position spring toward the closed
  target, so its force is stiffness × the remaining stroke up to the cap. With 5000 N/m an object that
  left ~10 mm of stroke only got ~50 N and the cap was never reached; with 50000 N/m a fully closed
  command applies the cap whatever the object's width.
- Same `drive:linear:physics:*` attributes for PhysX and Newton. Checked live in the UR5e teleop demo
  on both: with the manual's 100 N, gripping an object with the trigger fully pressed read about 100 N at
  each pad. With the 130 N used now, the runtime lift test reads about 130 N at each pad on Newton and
  128–133 N on PhysX.

### Simulation-only extra force

`EXTRA_TIP_FORCE_FOR_SIM` in `devel_helpers/gen_hande_layers.py` (30 N) is added to the force of **every**
tip, so each tip is simulated at its manual value plus 30 N: 100 N + 30 N = **130 N at each pad**, a cap of
260 N on `left_finger_joint`. 130 N is the datasheet's maximum grip force (20 – 130 N). It is needed by the
runtime lift test (`tests/runtime/test_gripper_lift.py`: a 1 kg cube lifted at 5 m/s²): with 100 N at each
pad the Newton case slips 5.5 mm (the limit is 2 mm), with 130 N it slips 0.7 mm (PhysX: 0.8 mm and 0.07 mm).

This is above the manual's 100 N limit on the force exerted at the end of the fingertips, so **the asset
does not respect that limit**: do not use the simulated force as a statement of what the real gripper may
apply. To get the manual's values back (cap 200 N), set `EXTRA_TIP_FORCE_FOR_SIM = 0.0` and regenerate the
layers (`python3 devel_helpers/gen_hande_layers.py`, which needs the `pxr` USD bindings); the Newton lift
test then fails on slip.

## Stroke100 (100 mm stroke)

`Stroke = Stroke100` is the same gripper with 100 mm total opening: 50 mm per finger. `Stroke50` is the
default and is the asset described above; nothing about it changes with this variant set.

| | `Stroke50` | `Stroke100` |
| --- | --- | --- |
| Finger travel (`physics:upperLimit` of both joints) | 25 mm | 50 mm |
| Pad-to-pad opening, Std tips (`gripper_maxOpening`) | 50 mm | 100 mm (other tips: their `Stroke50` value + 50 mm) |
| Base | the Hand-E CAD | the P7L CAD: a 125 mm rack plate along the finger axis, mass 0.9253 kg (was 0.8633 kg) |
| Finger body | 45 mm long, 17.8 g | 70 mm long, 28.5 g |
| Fingertips | stock | the same stock tips (all five), 25 mm further out each |
| Drive, speed, forces, fingertip masses | | unchanged: the datasheet is the same, only the stroke changes |

How it is built:

- The base and the two finger bodies are baked from `ROBOTIQ_HAND-E-P7L_DEFEATURE.STEP` (fingers open) by
  `devel_helpers/gen_hande_s100_parts.py`. That STEP is one assembly of 33 shells, not named parts, so the
  script tells them apart by face count and checks them against the geometry (see its docstring). Open versus
  closed, the moving shells travel exactly 50.0 mm each.
- The fingertips of the P7L open model are not the Std tip: their volume (shells 4 + 5, 9445 mm³) matches the
  stock Flat_Overmolded tip (9440 mm³). The stock tips fit the longer fingers shifted 25 mm along Y (to within
  about 1 mm), so `Stroke100` reuses them all, rather than baking new ones, and the `Fingertip` and
  `FingertipMount` variant sets work as before.
- `Stroke` is the first variant set, so its payloads win over those of the other sets. `Stroke100` sits next to
  the stroke-50 meshes (each link's `mesh_0` is deactivated, the new part is `mesh_s100`) and carries its own
  `Physics` and `Fingertip` deltas as nested variant sets, so none of the `Stroke50` layers is touched.
- Mass, centre of mass and box inertia are estimated like the stroke-50 ones: 2800 kg/m³ × baked mesh volume,
  bounding-box centre. The fingertip centres of mass move 25 mm with the tips. The grasp is the stroke-50
  one: the same `drive:linear:physics:*` values and per-tip force caps (the tips are at the same height above
  the housing, so the spec-sheet force curves apply as they are).

What has been checked: the composed stage of every `Stroke50` combination (Physics × Fingertip × FingertipMount,
30 of them) is identical, attribute for attribute, to the one before this variant set existed; on `Stroke100`
the composed pad-to-pad gap measured on the meshes equals `gripper_maxOpening` for all five tips, and the
colliders, masses and joint limits compose on PhysX, Newton and `None`. The runtime tests have `PhysX_Stroke100`
and `Newton_Stroke100` rows in `test_gripper_close.py`, `test_gripper_grasp.py` and `test_gripper_lift.py`
(the existing cube preset is reused: the cube sits on the centreline between the pads at the same height).
