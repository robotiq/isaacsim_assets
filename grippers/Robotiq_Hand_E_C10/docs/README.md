# Robotiq Hand-E C10

Isaac Sim asset for the Robotiq Hand-E C10: **three concentric fingers**, 120 degrees apart, that slide
radially toward the gripper axis (19.2 mm each, 38.4 mm of diameter between the pads). `Robotiq_Hand_E_C10.usda`
is the entry point. It is a separate asset from the two-finger [Hand-E](../../Robotiq_Hand_E/docs/README.md): the
joint layout is different, so it cannot be one of its variants.

| Variant set | Variants |
| --- | --- |
| `Physics` | `PhysX` *(default)*, `Newton`, `None` |
| `Fingertip` | `Stepped` *(default)*, `Extender`, `Flipped` |

`Extender` mounts the finger extender (FINGER_EXTENDER_C10, D-041) on the finger and the stepped tip on the extender: the
extender is a 12 mm wide, 9 mm tall bar that runs radially from 20.2 mm to 104.2 mm from the axis, its inner end (a 3 mm
wide nose at z 114 to 123 mm) being the pad for small objects, and the stepped tip sits at its outer end, 36.5 mm further
out and 8.0 mm higher than on the bare finger, which moves the tip's step to a larger diameter. Both placements were
set by hand in Isaac Sim and decoded to pure translations (no rotation). The manual puts the TCP 8 mm further out with
extenders, as here. `Flipped` is the default stepped tip turned 180 degrees about the vertical (Z) axis through the middle of its footprint (x 44.7 mm
on finger 1), so it sits on the same spot of the finger and its outer back, not the stepped pad, faces the gripper axis. The same
mesh and colliders are reused; only the tip's transform and the tip bodies' centre of mass change (same mass, 30.7 g). The
grip pad of this face has not been measured, and the pivot is the footprint's middle, not a CAD mounting point.
The
long and short spacers are not variants yet: they still have to be placed on the finger (see
[What is not done](#what-is-not-done)).

## Frame and joints

The origin is on the gripper axis at the mounting flange (the back face of the housing); `+Z` points toward the
fingertips. Finger 1 sits along `+X`, finger 2 at +120 degrees and finger 3 at +240 degrees about `Z`. Link frames are
identity, as in the Hand-E: every part is baked in the asset frame.

`finger_1_joint` is the driven joint; `finger_2_joint` and `finger_3_joint` follow it rigidly (a PhysX mimic joint,
or a stiff MuJoCo joint equality on Newton). Each joint frame is turned so its `+X` axis points toward the gripper
axis: all three close with a positive position, 0 (open) to 0.0192 m. The fingertips are separate rigid bodies
welded to the finger stubs, as on the Hand-E.

## Source of the specifications

The datasheet values were provided by Robotiq for the Hand-E C10.

| Datasheet value | Used for |
| --- | --- |
| Stroke (diameter) 40 mm | cross-check of the finger travel: the tip's inner face is 20.2 mm from the axis when open (grip-frame `maxOpening` 0.0404 m) |
| Grip force 25 to 175 N per finger | the force at each pad: 175 N, so a driven-joint cap of 525 N (the three fingers share the driven joint's force) |
| Closing speed 14 to 107 mm/s | `physxJoint:maxJointVelocity` 0.0535 m/s: **assumption**, half of 107 mm/s, reading the datasheet figure as the speed at which the diameter closes (each finger moves half of it). If it is a per-finger speed, set `FINGER_SPEED = 0.107` in `devel_helpers/gen_hande_c10_layers.py` and regenerate |
| Gripper weight 1.7 kg | total mass of the rigid bodies, exactly |
| Payload 10 kg | `gripper_maxPayload` on the gripper site |

The finger travel, 19.2 mm, is **measured** on Robotiq's open and closed CAD assemblies (the fingers go from 51.3 mm
to 32.1 mm from the axis); the datasheet's 40 mm of diameter stroke would give 20 mm.

## How it is modelled

- **Masses.** A finger stub (the aluminium slider plus its two steel tip screws) is 16.1 g and a stepped tip 30.7 g, from
  the baked CAD meshes (aluminium 2700 kg/m3, steel 7850 kg/m3). The base takes the rest of the 1.7 kg (1559.7 g). The
  defeatured CAD has no motor inside, so the base mass is spread over the visible solids and its real centre of mass is
  probably deeper. Centre of mass and principal moments come from the closed meshes.
- **Drive.** A position drive on `finger_1_joint`, stiffness 100000 N/m and damping 283 N.s/m (the Hand-E's damping ratio),
  capped at 525 N: three times the datasheet's 175 N per finger, as the three fingers share the driven joint's force. The
  drive pushes with stiffness x the remaining stroke up to the cap, and a finger stopped by an object of diameter d is left
  with d/2 - 1 mm of it (a closed command is 19.2 mm of travel, the open pads are 40.4 mm apart). So a fully closed command
  applies the cap, 175 N per pad, on objects wider than 12.5 mm; on a narrower one the force is less, and depends on the
  diameter: roughly 100 to 135 N per pad on a 10 mm pin, depending on how far the pads sink into it. The Hand-E's 50000 N/m reached the cap only above 23 mm. A stiffer drive
  (262500 N/m, the cap from 6 mm) was tried and made both engines unstable.
- **Colliders.** Each body link has one convex hull. Each stepped tip has 8 convex pieces (CoACD), which add 48% to the
  tip's volume: its fine ridges are filled by any hull. Each extender has its exact convex hull (64 vertices, +21% over its 7.24 cm3 part;
  CoACD pieces were looser, +39 to +42%, and stood 0.46 mm proud of the pad face). The pad itself is the lowest 9 mm of the tip (z 114 to 123 mm); the
  tip steps back above it.
- **Pad friction.** 1.0, static and dynamic, with combine mode `max` on PhysX, as on the Hand-E.
- **Grip frame** (`grip_frame`, the FET028 gripper site): on the axis, at z 0.1343 m, the middle of the tips' length. This
  is provisional, as the pads are not marked in the CAD. `forward` is `Z`; the grip line is the width of finger 1's pad
  (along `Y`), so the closure, forward x grip line, is toward the axis along `-X`. The other two fingers close at 120 and
  240 degrees; the site describes the first.

## What is not done

- The long and short spacers (D-042, D-043) are in Robotiq's CAD set but not baked: how they mount on the finger has to be
  confirmed first.
- The Newton drive and MuJoCo solver values are copied from the Hand-E and have been checked only by the runtime tests below.
- In the `Extender` variant each fingertip body is the stepped tip moved onto the extender plus the extender: 30.7 g + 60.1 g.
  The extender's mass is the manual's +180.22 g for the three extenders with their screws (1875.22 g against 1695 g),
  spread over the CAD volume (the CAD has no screws, so the density comes out at 8.3 g/cm3, above steel's 7.85). The
  centre of mass and the moments are those of the two meshes together. The base mass is unchanged, so the variant weighs
  1.88 kg.
- A per-tip mass and a measured grip force are not available: the force is the datasheet's maximum.

## Generated files and tests

`parts/` is baked from the CAD by `devel_helpers/gen_hande_c10_parts.py` (needs `cadquery-ocp`; the STEP file is not in
the repository) and `devel_helpers/gen_hande_c10_fingertip_colliders.py` (CoACD). The layers under `payloads/` and
`configuration/`, and the root's SimReady metadata, are generated by `devel_helpers/gen_hande_c10_layers.py`: do not edit
them by hand.

Runtime tests (manual, Isaac Sim, see `tests/runtime/README.md`): `test_gripper_close.py` closes the driven finger and
checks that both followers track it, on PhysX and Newton; `test_gripper_c10_grasp.py` closes the three fingers on a puck
standing on the axis and checks the stall, the contact and force of every pad, that the puck ends up centred and at rest,
and the release.
