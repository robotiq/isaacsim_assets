# 2F-85 linkage and joint setup

How the 2F-85 finger linkage is modelled in the PhysX variants, joint by joint.
The [2F gripper guide](README.md) explains the mechanism itself (§4) and how to
tune it; this page shows where each joint sits and what each variant does
with it. Joint values are read from the 2F-85 payloads; the 2F-140 payloads
follow the same structure, with their own values.

## The finger linkage

The 2F-85 finger is a **closed five-bar linkage** with two degrees of freedom:
one driven, and one underactuated compliance DOF that a pin limits (see
[§4.1](README.md#41-how-the-2f-85-mechanism-works-and-how-compliance-works)).
A physics articulation must be a tree, so it cannot hold the closed loop
directly. The two PhysX variants solve this differently (below).

![2F-85 finger linkage with the joints labelled](img/2f85-linkage-labeled.png)

*The 2F-85 finger linkage: the blue and red links close a kinematic loop (red
arrow). `out_knuckle` is the driven joint.*

The short names in the figure map to these joints (left side; the right side
mirrors them):

| Figure | Joint | Connects |
|---|---|---|
| out_knuckle | `finger_joint` | `base_link` → `left_outer_knuckle` |
| out_finger | `left_outer_finger_joint` | `left_outer_knuckle` → `left_outer_finger` |
| in_finger | `left_inner_finger_joint` | `left_outer_finger` → `left_inner_finger` |
| in_finger_knuckle | `left_inner_finger_knuckle_joint` | `left_inner_finger` → `left_inner_knuckle` |
| in_knuckle | `left_inner_knuckle_joint` | `base_link` → `left_inner_knuckle` |

## `Physx_parallel_grip`: mimic joints

The loop is cut to leave a tree, and **mimic joints** make the remaining
joints follow `finger_joint`. The coupling is rigid, so only the parallel grip
is reproduced, but every joint stays in the articulation solver: cheaper and
more stable.

| Joint | Role | Gearing on `finger_joint` |
|---|---|---|
| `finger_joint` | Driven, the only joint with a drive | — |
| `left_inner_finger_joint` | Mimic | −1 |
| `left_inner_finger_knuckle_joint` | Mimic | +1 |
| `left_outer_finger_joint` | Deactivated, replaced by a fixed joint `left_outer_knuckle` → `left_outer_finger` | — |
| `left_inner_knuckle_joint` | Deactivated: this cut keeps the tree | — |

The right side mirrors this with the gearing signs flipped, and
`right_outer_knuckle_joint` mimics `finger_joint` with gearing −1, so one
command moves both fingers. Values from
[`Robotiq_2F_85_parallel_grip_physics.usda`](../../Robotiq_2F_85/payloads/Robotiq_2F_85_parallel_grip_physics.usda).

![2F-85 mimic-joint layout](img/2f85-mimic-joints.png)

*The left-side chain in the parallel-grip variant: the **driven** joint
(out_knuckle), the **fixed** weld (out_finger), and the mimic joints (magenta
arrows) that follow the driven joint.*

## `Physx_compliant`: loop closure

Nothing is welded or cut: all five joints are kept as revolutes, so the
fingertip pad can rotate under load like the real hardware. One joint per side,
`left_inner_finger_joint` (in_finger), is excluded from the articulation
(`physics:excludeFromArticulation = 1`) and the solver enforces it as a
loop-closure constraint. That joint also carries the underactuation spring
(see [§4.2](README.md#42-the-redundancy--driving-spring)): a drive with
stiffness 0.0004 and target 92°.

| Joint | Role |
|---|---|
| `finger_joint` | Driven |
| `left_outer_finger_joint` | Free revolute, limited to 0…45° (the pin) |
| `left_inner_finger_knuckle_joint` | Free revolute |
| `left_inner_knuckle_joint` | Free revolute |
| `left_inner_finger_joint` | **Loop closure**, outside the articulation; carries the spring drive |

`right_outer_knuckle_joint` mimics `finger_joint` with gearing −1 so both sides
close together; the compliant DOFs of each side can still move independently,
depending on contact. Command only `finger_joint`. Values from
[`Robotiq_2F_85_compliant_physics.usda`](../../Robotiq_2F_85/payloads/Robotiq_2F_85_compliant_physics.usda)
and the shared limits in
[`Robotiq_2F_85_kinematics_physics.usda`](../../Robotiq_2F_85/payloads/Robotiq_2F_85_kinematics_physics.usda).

### The outer-finger limit reproduces the pin

The limit on `*_outer_finger_joint` stands for the mechanical pin that caps
the minimal length of the virtual link. If it is wrong, the fingertip pivots
outward instead of staying parallel. The stock NVIDIA compliant asset had this
problem; ours limits `left_outer_finger_joint` to 0…45° and
`right_outer_finger_joint` to −45…0°.

![Compliant variant with the fingertip pivoted outward](img/loop-fingertip-pivot.png)

*Compliant variant with a wrong outer-finger limit: the fingertip pivots
outward instead of holding parallel.*

**When to pick which:** mimic for a fast, robust parallel pinch; loop when the
grip's realism matters (compliance, the encompassing grip), preferably under
Newton (see [§5](README.md#5-using-isaac-with-newton-and-why-its-better-for-the-compliant-variant)).

## The underactuation spring

Reference figures for the torsion spring that the compliant variant models with
the drive on `*_inner_finger_joint`
([§4.2](README.md#42-the-redundancy--driving-spring)).

![Free and pre-constrained spring positions](img/2f85-link-angles.png)

*Spring positions on the finger: free position, pre-constrained position, and
the reference used to measure the angle.*

![2F-85 torsion spring drawing](img/2f85-torsion-spring.png)

*Spring drawing (mm): 120° free angle, 6-1/6 coils, 0.81 mm wire, 7.75 mm inside
diameter.*

## Removing the gripper's articulation root

The standalone gripper has its own articulation root so it can simulate by
itself. When you mount it on a robot, remove it: the gripper's joints must
belong to the robot's articulation. Two articulation roots is a common cause of
a gripper that won't drive or behaves erratically. The step-by-step is in
[Get started](../../../docs/2-Get_started.md#attach-the-robotiq-gripper-to-the-robot).

![Articulation Root API in the Physics property panel](img/articulation-root-panel.png)

*The Articulation Root block in the Physics panel: delete it (red cross) from
the gripper when mounting it on a robot.*
