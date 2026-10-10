# Newton (MuJoCo-Warp) tuning — 2F-140

Newton-specific runtime tuning and notes for the **`Newton_compliant`** and
**`Newton_parallel_grip`** variants of the unified 2F-140 asset
([`../Robotiq_2F_140.usda`](../Robotiq_2F_140.usda)). They mirror the 2F-85's Newton
variants (same layers, joint treatment and MuJoCo tuning) with the 2F-140's geometry.
Select `Physics = Newton_compliant` (encompassing grip) or `Newton_parallel_grip`,
ensure the scene has a `PhysicsScene`, sublayer [`scene/Robotiq_Newton_scene_physics.usda`](../../scene/Robotiq_Newton_scene_physics.usda) into the stage on Isaac Sim 6.1+ (it sets the elliptic friction cone; see the simulation guide §5.3), and play. The gripper is **open by default**
(`finger_joint` drive target `0`); drive `finger_joint` toward 45° (≈0.785 rad) to close.

Everything the solver needs is baked into the payloads — the shared Newton finger linkage
([`../payloads/Robotiq_2F_140_newton_fourbar_physics.usda`](../payloads/Robotiq_2F_140_newton_fourbar_physics.usda)),
the shared Newton bodies/colliders
([`../payloads/Robotiq_2F_140_newton_common_physics.usda`](../payloads/Robotiq_2F_140_newton_common_physics.usda)),
the engine-neutral fingertip bodies and welds
([`../payloads/Robotiq_2F_140_fingertip_physics.usda`](../payloads/Robotiq_2F_140_fingertip_physics.usda),
shared with the PhysX variants), and the thin per-variant overlays — and rebuilds natively on load — **except** the items below.

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

**Isaac Sim 6.0.1 (Newton 1.2.1):** Newton 1.2.1 reads neither `mjc:option:impratio` nor the
coupling's `solref`/`solimp`, so still run
[`apply_gripper_tuning.py`](apply_gripper_tuning.py) after every Stop → Play (Script Editor or
MCP bridge — idempotent). It sets every `JOINT`-type equality in the model, so a stage that
also holds another gripper's coupling (e.g. a Hand-E) gets that one tuned to the 2F values too.

## Intentional `diagonalInertia = (0.001, 0.001, 0.001)`

Every moving finger-chain link (left/right `outer_knuckle`, `outer_finger`,
`inner_knuckle`, `inner_finger`) has `physics:diagonalInertia = (0.001, 0.001, 0.001)` —
much larger than the physical value, and **intentional, not a placeholder** (copied
from the 2F-85). Two bodies are deliberately left out: `base_link` keeps its CAD
inertia from the shared mass layer (it is the fixed/welded base, so its inertia is
inert), and the fingertips keep the real pad inertia (they are welded to `inner_finger`). The extra
rotational inertia stabilizes the soft Newton loop-closure equalities so the weak
parallel-grip spring holds; with physically-small inertias the linkage goes
floppy and the fingertips sag ~20° out of parallel. Treat it like the
`mjc:armature`/`damping` tuning — do **not** replace it with CAD inertias without
re-tuning the loop-closure `solref` (or adding passive-joint armature).

## Isaac Sim source patch — `solreflimit` ×180/π backport (required)

Newton **1.2.1** (bundled with Isaac Sim 6.0.1) over-scales the *angular*
hinge/D6 `mjc:solreflimit` stiffness/damping by **180/π**. Because this asset
bakes `mjc:solreflimit` into the payload (the couplers / joint limits), a stock
Isaac 6.0.1 imports it wrong — authored `mjc:solreflimit = [0.02, 2]` compiles to
`jnt_solref = [-35809, -5729]` instead of the intended `[-625, -100]`.

- **Fix:** backport of upstream Newton PR **#2736** ("Fix USD MJC angular
  `limit_ke/kd` over-scaling on revolute/D6 joints") — pre-multiply the angular
  `limit_*_ke/kd` by π/180.
- **File patched:** `exts/isaacsim.pip.newton/pip_prebundle/newton/_src/usd/schemas.py`
  in the Isaac install (keep a backup alongside, e.g. `schemas.py.orig-solreflimit-bug`).
- ⚠️ **Vendored file** — an Isaac reinstall/upgrade overwrites it; reapply the
  patch, or move to an Isaac build bundling Newton ≥ the one containing #2736.
  Already fixed on Newton `main`, so this is packaging lag, not an open defect.

## 2F-140 mass provenance

There is no standalone Newton 2F-140 model (the 2F-85's Newton masses came from one),
so the Newton variants inherit the 2F-140's **real** mass and CoM straight from the
shared mass layer (`payloads/Robotiq_2F_140_body_mass_physics.usda`, also used by PhysX);
`newton_common` overrides only the isotropic inertia above. If the loop proves unstable
live, lighter link masses are the first knob to try (the 2F-85's Newton links are
~0.02 kg) -- author them in `newton_common` as a Newton-only override.
