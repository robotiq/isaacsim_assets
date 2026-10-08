# Robotiq Hand-E

Isaac Sim asset for the Robotiq Hand-E: two sliding fingers (25 mm each, 50 mm stroke) on prismatic
joints. `Robotiq_Hand_E.usda` is the entry point.

| Variant set | Variants |
| --- | --- |
| `Physics` | `PhysX` *(default)*, `Newton`, `None` |
| `Fingertip` | `Std` *(default)*, `Flat_Overmolded`, `Support`, `Extender`, `Bin_Picking` |
| `FingertipMount` | `Outside` *(default)*, `Inside` — which fingertip screw-hole row is used; `Inside` moves every tip 9 mm towards the centerline |

Only `left_finger_joint` is driven; `right_finger_joint` follows it rigidly (a PhysX mimic joint, or
a stiff MuJoCo joint equality on Newton), and both close with a positive position, 0 (open) to 25 mm.

Most of the repetitive layers under `payloads/` and the fingertip collision pieces are generated:
see `devel_helpers/gen_hande_layers.py` and `devel_helpers/gen_hande_fingertip_colliders.py`.

## Source of the specifications

The finger speed, the grip force and the fingertip force limits come from the Hand-E instruction
manual, "6. Specifications" page:

<https://assets.robotiq.com/website-assets/support_documents/document/online/Hand-E_Aubo_InstructionManual_HTML5_20190501.zip/Hand-E_Aubo_InstructionManual_HTML5/Content/6.%20Specifications.htm>

| Datasheet value | Used for |
| --- | --- |
| Finger speed 20 to 150 mm/s | `physxJoint:maxJointVelocity = 0.15` m/s on the driven joint (the upper end of the range) |
| Grip force 20 – 130 N | (documentation only) |
| Forces exerted at the end of the fingertips must not exceed 100 N, and the maximum payload / external force falls with the Z offset of the fingertip (Fig. 6-10 / 6-11, below) | the finger drive force cap, per `Fingertip` variant |

## Finger force per fingertip

![Maximum payload / external force vs. custom finger Z offset](docs/Hand-E_max_payload_vs_z_offset.png)

*Hand-E: maximum payload / external force vs. custom finger design (Z offset), copied from the page
above. The force is applied at the tip of the finger, in the middle of the inner surface; the Z offset
is measured from the housing face. Blue: finger mounted directly on the rack with 2 M3 screws. Red:
fingertip on a fingertip holder with 2 M3 screws. Yellow: mounted directly on the rack with 3 M3
screws.*

Each `Fingertip` variant takes the curve of how that tip is mounted and its Z offset (measured from the
baked CAD part):

| Fingertip | Mounting (curve) | Z offset | Force at each pad |
| --- | --- | --- | --- |
| `Std` | 2 × M3 on the rack (blue) | 60.1 mm | 100 N |
| `Flat_Overmolded` | 2 × M3 on the rack (blue) | 60.3 mm | 100 N |
| `Support` | fingertip holder (red) | 20.1 mm | 100 N |
| `Extender` | fingertip holder (red) | 20.2 mm | 100 N |
| `Bin_Picking` | 3 × M3 on the rack (yellow) | 76.1 mm | 100 N |

All of them are on the graph's 100 N plateau. The breakpoints of the sloped part of each curve in the
generator were read off the figure (±5 mm / ±5 N); the plateau is exact. If a tip is added whose Z
offset is beyond the plateau, check the value against the figure.

### How the force is modelled

- **The drive cap is twice the force at the pad.** The right finger follows the left one rigidly, so the
  left drive force is shared between the two pads. Measured in the teleop demo: a 130 N cap gave
  65–75 N per pad. The cap on `left_finger_joint` is therefore 200 N for 100 N at each pad.
- **The drive is stiff** (50000 N/m, damping 200 N·s/m). It is a position spring toward the closed
  target, so its force is stiffness × the remaining stroke up to the cap. With 5000 N/m an object that
  left ~10 mm of stroke only got ~50 N and the cap was never reached; with 50000 N/m a fully closed
  command applies the cap whatever the object's width.
- Same `drive:linear:physics:*` attributes for PhysX and Newton. The Newton values are not tuned yet
  (only the PhysX values were measured).
