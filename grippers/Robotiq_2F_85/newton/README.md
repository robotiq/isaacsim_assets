# Newton (MuJoCo-Warp) tuning — 2F-85

Newton-specific scene settings and notes for the **`Newton_compliant`** and
**`Newton_parallel_grip`** variants of the unified 2F-85 asset
([`../Robotiq_2F_85.usda`](../Robotiq_2F_85.usda)).
Select `Physics = Newton_compliant` (encompassing grip) or `Newton_parallel_grip`,
ensure the scene has a `PhysicsScene`, sublayer [`scene/Robotiq_Newton_scene_physics.usda`](../../scene/Robotiq_Newton_scene_physics.usda) into the stage on Isaac Sim 6.1+ (it sets the elliptic friction cone; see the simulation guide §5.3), and play. The gripper is **open by default** (`finger_joint` drive target `0`); drive
`finger_joint` toward ~0.8 rad (≈45°) to close.

Everything the solver needs is baked into the variant payloads
([`../payloads/Robotiq_2F_85_newton_compliant_physics.usda`](../payloads/Robotiq_2F_85_newton_compliant_physics.usda),
[`../payloads/Robotiq_2F_85_newton_parallel_grip_physics.usda`](../payloads/Robotiq_2F_85_newton_parallel_grip_physics.usda))
plus the shared Newton layers they sublayer (including the engine-neutral
`Robotiq_2F_85_fingertip_physics.usda`), and rebuilds natively on load — **except** the items below.

## Scene options — sublayer the shared scene layer once

The finger-coupling equality (`solref = [0.002, 1]`, near-rigid) is authored on
`right_outer_knuckle_joint` (`MjcEqualityJointAPI`) and rebuilds natively. The solver's
anti-slip option and the friction cone cannot live in the asset, because the gripper has
no `PhysicsScene`. On Isaac Sim 6.1+, sublayer
[`../../scene/Robotiq_Newton_scene_physics.usda`](../../scene/Robotiq_Newton_scene_physics.usda)
into your stage once. It authors both, on `/PhysicsScene`:

```usda
uniform token mjc:option:cone = "elliptic"
uniform double mjc:option:impratio = 10
```

Or author the same two attributes on your own scene. Two things to get right: the scene needs
`MjcSceneAPI` in its `prepend apiSchemas`, or the importer ignores the attributes; and the
shared layer only merges with a scene at exactly `/PhysicsScene` (with `/World/PhysicsScene`
or `/physicsScene` you would get a second scene prim, so author the attributes on your own).
The cone matters too: see the Simulation Guide, §5.3.

Needs **Isaac Sim 6.1 / Newton 1.5.0** or later, which reads scene `mjc:option:*`
from USD. No script is needed after Stop → Play.

**Isaac Sim 6.0.1 (Newton 1.2.1) is no longer supported.** It reads neither `mjc:option:impratio` nor the
coupling's `solref`/`solimp`, so it needed a runtime script after every Stop → Play; that script is
removed. The last version that still has it is the git tag `last-isaac-6.0-supported`.

## Intentional `diagonalInertia = (0.001, 0.001, 0.001)`

Every gripper link's `physics:diagonalInertia` is `(0.001, 0.001, 0.001)` —
~100× the physical value, and **intentional, not a placeholder**. The extra
rotational inertia stabilizes the soft Newton loop-closure equalities so the weak
parallel-grip spring holds; with physically-small inertias the linkage goes
floppy and the fingertips sag ~20° out of parallel. Treat it like the
`mjc:armature`/`damping` tuning — do **not** replace it with CAD inertias without
re-tuning the loop-closure `solref` (or adding passive-joint armature).
