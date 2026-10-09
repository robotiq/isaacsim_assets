# Physics tuning for the UR5e + 2F-85 + graspable objects

The teleop scene (`ur5robot_with_2F-85.usda`) overrides some physics values
of the gripper asset and sets others on the objects to grasp. This page
lists those values and why each one is needed. The gripper asset itself is
not changed: everything below is authored in the demo scene.

None of this is needed for the ROS teleop to *run*. It is what makes the
gripper hold an object while the arm moves.

The values apply to the scene's default `physx` gripper, which uses the
`Physx_compliant` variant. The objects' values apply to any PhysX gripper,
including `Physx_parallel_grip`. For the Newton gripper, see
[Robotiq_2F_85/newton/README.md](../../Robotiq_2F_85/newton/README.md).

## Gripper: `finger_joint` drive

The asset's finger drive is very soft: it assumes you put your own
controller on top. The demo drives the gripper directly through the joint
drive, so it needs firmer values:

| Attribute | Asset (`Physx_compliant`) | Demo scene | Effect |
|---|---|---|---|
| `drive:angular:physics:stiffness` | 0.17 | **50** | Holds the commanded angle under contact load. |
| `drive:angular:physics:damping` | 0.0002 | **3** | Stops the finger wobbling when the arm accelerates during a grasp. |
| `drive:angular:physics:maxForce` | 16.5 N·m | **1 N·m** | Caps the grip force so the fingers don't crush through the object. |

Raising the stiffness further holds better but passes vibration into the
arm. Stiffness 50 with damping 3 is the balance we kept.

Command **only `finger_joint`**. With the compliant variant, the other
finger joints are solved by the closed linkage; sending them targets fights
the solver. All the demo's frontends send `finger_joint` only.

## Gripper: closed-linkage solver iterations

The compliant variant closes the finger linkage with loop-closure joints,
which have a little residual give: you can see the inner knuckles move a
fraction of a degree under load. Two settings reduce it:

- **Per-body iterations** on the four bodies of the linkage
  (`left_outer_finger`, `left_inner_finger`, `right_outer_finger`,
  `right_inner_finger`):
  ```
  physxRigidBody:solverPositionIterationCount = 64
  physxRigidBody:solverVelocityIterationCount = 8
  ```
  PhysX uses the highest count among the bodies linked by a constraint, so
  these apply to the whole linkage.
- **Scene stabilization**, on `/PhysicsScene`:
  ```
  physxScene:enableStabilization = True
  ```
  PhysX runs an extra pass that re-converges constraints between frames.
  It is cheap and visibly tightens the linkage.

The remaining instability of the compliant linkage under PhysX is why the
asset also has Newton variants, which model it more robustly.

## Arm: articulation solver iterations

On the articulation root (`/World/ur5e/root_joint`):

```
physxArticulation:solverPositionIterationCount = 255
physxArticulation:solverVelocityIterationCount = 8
```

With only 1 velocity iteration, contact velocity errors at the fingertips
aren't resolved and the grasp jitters.

## Objects to grasp

Each graspable object needs the values below. `add_props.py` creates its
props with them.

| Attribute | Default | Demo scene | Why |
|---|---|---|---|
| `physics:mass` | 0 | **0.1 kg** (adjust per object) | Primitive shapes often come with mass 0 *and* density 0. PhysX then has no valid inertia, any contact force accelerates the object infinitely, and the fingers pass straight through it. |
| `physxCollision:contactOffset` | scene default | **0.002 m** | Distance at which PhysX starts generating contacts. The scene default is too small for centimetre-scale objects. |
| `physxCollision:restOffset` | scene default | **0.0005 m** | A thin "skin" PhysX keeps between surfaces, so the fingers can't press into the object. Must be smaller than `contactOffset`. |
| `physxRigidBody:linearDamping` | 0 | **0.1** | Without damping, every contact disturbance makes the object oscillate in the grip. |
| `physxRigidBody:solverVelocityIterationCount` | 1 | **4** | Resolves contact velocities better, which reduces jitter in the grip. |

Setting the offsets on the object is enough. The finger pads are inside
the referenced asset, so the scene can't set offsets on them, but PhysX adds
the two sides' offsets together, so the object's values still give a
visible skin.

Before and after images of this fix are in the gripper README,
[Tuning](../../Readme.md#tuning).

## Fast motion: continuous collision detection (CCD)

Physics advances in steps of a few milliseconds. The default collision
detection only checks for overlap at the end of each step, so an object
that moves further than its own thickness in one step can pass through a
finger unseen. As a rule of thumb this happens when
`speed × step time > thickness`: about 0.5 m/s for a 2 mm gap with a 4 ms
step.

If fast arm moves still make objects pass through the fingers, turn on
CCD **on the small, fast object**:

| | `physxRigidBody:enableCCD` | `physxRigidBody:enableSpeculativeCCD` |
|---|---|---|
| How it works | Sweeps the body's motion and finds the time of impact | Enlarges the collision check by the expected motion |
| Cost | High | Low |
| Catches | Straight and rotating motion | Straight motion only |
| Side effects | Can make contacts "sticky" | Almost none |

Use **speculative CCD** first. Turn it on per object, not on static
geometry (it gives them nothing) and not on every body: on a busy scene,
CCD everywhere can multiply the physics cost 2–5×. The demo scene leaves
CCD off; its objects and speeds don't need it.

## What we tried and reverted

- **Much higher `finger_joint` stiffness:** held better, but passed
  vibration into the arm.
- **Turning off MoveIt Servo's smoothing filter** to fix stale joint
  states in Servo: the staleness has another cause, and the filter is
  already at its minimum (`butterworth_filter_coeff = 1.5`).
- **Computing the bridge's deltas from measured joint states**, to fix the
  jump on the first move after Stop → Play: it worked but added its own
  edge cases. The bridge stays simple, and the gamepad's resync button
  restarts Servo instead.

## Summary of changed USD attributes

```
/World (variant set)
    Gripper                                          physx  (gripper Physics = Physx_compliant)

.../Robotiq_2F_85/Joints/finger_joint
    drive:angular:physics:stiffness                  0.17    ->  50
    drive:angular:physics:damping                    0.0002  ->  3
    drive:angular:physics:maxForce                   16.5    ->  1

.../Robotiq_2F_85/{left,right}_{outer,inner}_finger
    physxRigidBody:solverPositionIterationCount      inherit ->  64
    physxRigidBody:solverVelocityIterationCount      inherit ->  8

/World/ur5e/root_joint (articulation root)
    physxArticulation:solverPositionIterationCount           ->  255
    physxArticulation:solverVelocityIterationCount           ->  8

/World/Cube (and every graspable object)
    physics:mass                                     0       ->  0.1 (per object)
    physxCollision:contactOffset                     default ->  0.002
    physxCollision:restOffset                        default ->  0.0005
    physxRigidBody:linearDamping                     0       ->  0.1
    physxRigidBody:solverVelocityIterationCount      1       ->  4

/PhysicsScene
    physxScene:enableStabilization                   False   ->  True
```
