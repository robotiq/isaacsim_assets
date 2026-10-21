# Physics tuning for the UR5e + 2F-85 + graspable objects

Notes on the non-default USD attribute values we ended up with after
tuning the gripper for reliable grasping in Isaac Sim 6. None of this is
required for the ROS teleop to *function* — but everything below affects
whether the gripper actually holds an object during motion.

Two categories:

- **Gripper (2F-85)** — variant choice, drive gains, joint topology
- **Objects (cube / cylinder)** — mass, contact offsets, damping, solver
  iterations

## 2F-85 variant: use `Physx_Loop`, not `Physx_Mimic`

The NVIDIA 2F-85 asset has a `Physics` variant set on
`.../Robotiq_2F_85_edit` with three options:

- `None` — no physics constraints; pure kinematic
- `Physx_Mimic` — mimic joints hard-couple the passive fingers to
  `finger_joint` algebraically
- **`Physx_Loop`** ← the one we use

The Physx_Loop variant models the real four-bar linkage as a proper
closed kinematic chain, with the `*_inner_knuckle_joint`s marked
`excludeFromArticulation = True` and solved as maximal-coordinate
loop-closure constraints. Compared to the mimic variant this gives:

- More realistic contact behavior (the fingertip pad can rotate slightly
  under load to conform to the object, like the real hardware)
- Better handling when the mimic would fight a contact force
- Slightly higher solver cost — irrelevant at our scale

The trade-off is small residual compliance at the loop-closure joints
(you can see the inner knuckle wiggle a fraction of a degree under load).
That's fundamental to maximal-coordinate constraints; ways to reduce it
below.

**Important consequence for teleop:** under Physx_Loop, command **only
`finger_joint`**. The remaining finger joints are solved by the
loop-closure constraint; sending explicit targets for them fights the
solver. All frontend scripts in `grippers/demo/teleop/` do this
correctly.

## `finger_joint` drive — what works

The NVIDIA asset defaults are very soft (they assume you'll layer your
own controller on top). We're driving the gripper directly with the
`PhysicsDriveAPI` and need firmer values:

| Attribute | Asset default | Working value | Effect |
|---|---|---|---|
| `drive:angular:physics:stiffness` | 0.17 | 50 | Actually holds the commanded angle under contact load |
| `drive:angular:physics:damping` | 2e-4 | 3.0 | Kills the underdamped wobble when arm accelerates while gripping |
| `drive:angular:physics:maxForce` | 16.5 N·m | 5.0 N·m | Prevents the gripper from crushing the object past its `restOffset` |

`maxForce = 5 N·m` at the 30 mm effective lever arm is ~170 N of finger
force. Enough to hold a 100 g cube through fast arm motion without
tunneling through it.

## Contact offsets on graspable objects

Every rigid body has PhysX contact offset attributes that default to
`-inf` (meaning "use scene default"). For centimeter-scale objects the
scene default is too small — you get visible interpenetration during
grasp. Set explicit values:

| Attribute | Default | Value we use | What it does |
|---|---|---|---|
| `physxCollision:contactOffset` | `-inf` | 0.002 m (2 mm) | PhysX starts generating contact constraints when this close to the surface |
| `physxCollision:restOffset` | `-inf` | 0.0005 m (0.5 mm) | Hard "skin" — objects rest this far apart at zero relative velocity, never compress through it |

The pad meshes on the gripper are inside instance proxies (referenced
NVIDIA asset), so USD blocks authoring offsets there. Setting them on
the object side alone is enough because PhysX pairs the object's offsets
against the pad's scene-default (small) — the pair still sums to a
visible skin.

## Rigid-body properties on graspable objects

Beyond contact offsets, three more attributes matter for stable grasp:

| Attribute | Default | Value | Why |
|---|---|---|---|
| `physics:mass` | 0.0 (!) | 0.1 kg | The primitive-shape assets ship with `mass = 0, density = 0`, which gives PhysX undefined inertia (`centerOfMass = (-inf, -inf, -inf)`). Any contact force accelerates infinitely — objects tunnel through anything. Setting mass fixes this. |
| `physxRigidBody:linearDamping` | 0.0 | 0.1 | Absorbs the small contact-noise oscillation that would otherwise vibrate the object at rest. |
| `physxRigidBody:solverVelocityIterationCount` | 1 | 4 | Better contact-velocity resolution. Reduces jitter during grasp. |

## Articulation solver iterations

The full UR5e articulation runs 32 position / 1 velocity iterations by
default. We bumped **velocity** iterations to 2 to sharpen contact
resolution on the gripper end.

Additionally, for the four bodies attached to the loop-closure joints
(`left_outer_finger`, `left_inner_finger`, `right_outer_finger`,
`right_inner_finger`), we set per-body iterations higher:

```
physxRigidBody:solverPositionIterationCount = 64
physxRigidBody:solverVelocityIterationCount = 8
```

PhysX uses the max iteration count across bodies in a constraint island,
so these four values apply to the island that includes the loop closures.
Result: less visible residual motion at the inner knuckles.

## Scene-level: enable stabilization

```
physxScene:enableStabilization = True
```

One-flag change on `/PhysicsScene`. PhysX runs an extra stabilization
pass that re-converges constraints between frames. Cheap; measurably
tightens the loop closures.

## What we tried and reverted

- **Cranking `finger_joint.stiffness` to hold better** — worked but
  transferred vibration into the arm. K=50, C=3.0 hits the right damping
  ratio for the finger inertia.
- **Turning off the Butterworth smoothing in Servo** — thought Servo's
  `internal_joint_state_` staleness came from the LPF; turns out it's a
  separate integration-tracking issue in Servo's implementation. The LPF
  is already at its minimum (`butterworth_filter_coeff = 1.5`) by
  default.
- **Rewriting the bridge to compute deltas from measured state** —
  aimed at fixing the "first move after Stop→Play" jump. Worked in
  principle but introduced its own edge cases; reverted to the simple
  forward-with-NaN-guard bridge and use the `Circle (○)` gamepad button
  to trigger `stop_servo → start_servo` as a manual resync instead.

## Summary of changed USD attributes

For quick reference, the attributes overridden from the NVIDIA asset
defaults:

```
/World/ur5e/wrist_3_link/Robotiq_2F_85_edit
    Physics variant:  None|Physx_Mimic|Physx_Loop  →  Physx_Loop

.../Robotiq_2F_85/Joints/finger_joint
    drive:angular:physics:stiffness      0.17    →  50.0
    drive:angular:physics:damping        2e-4    →  3.0
    drive:angular:physics:maxForce       16.5    →  5.0

/World/Cube (or your graspable objects)
    physics:mass                          0.0    →  0.1
    physxCollision:contactOffset          -inf   →  0.002
    physxCollision:restOffset             -inf   →  0.0005
    physxRigidBody:linearDamping          0.0    →  0.1
    physxRigidBody:solverVelocityIterationCount   1  →  4

/World/ur5e/root_joint (articulation root)
    physxArticulation:solverVelocityIterationCount   1  →  2

.../{left,right}_{outer,inner}_finger (bodies in the loop-closure island)
    physxRigidBody:solverPositionIterationCount   inherit  →  64
    physxRigidBody:solverVelocityIterationCount   inherit  →  8

/PhysicsScene
    physxScene:enableStabilization       False   →  True
```
