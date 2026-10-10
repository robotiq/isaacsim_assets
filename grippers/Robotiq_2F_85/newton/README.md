# Newton (MuJoCo-Warp) tuning — 2F-85

Newton-specific runtime tuning and notes for the **`Newton_compliant`** and
**`Newton_parallel_grip`** variants of the unified 2F-85 asset
([`../Robotiq_2F_85.usda`](../Robotiq_2F_85.usda)).
Select `Physics = Newton_compliant` (encompassing grip) or `Newton_parallel_grip`,
ensure the scene has a `PhysicsScene`, and play. The gripper is **open by default** (`finger_joint` drive target `0`); drive
`finger_joint` toward ~0.8 rad (≈45°) to close.

Everything the solver needs is baked into the variant payloads
([`../payloads/Robotiq_2F_85_newton_compliant_physics.usda`](../payloads/Robotiq_2F_85_newton_compliant_physics.usda),
[`../payloads/Robotiq_2F_85_newton_parallel_grip_physics.usda`](../payloads/Robotiq_2F_85_newton_parallel_grip_physics.usda))
plus the shared Newton layers they sublayer (including the engine-neutral
`Robotiq_2F_85_fingertip_physics.usda`), and rebuilds natively on load — **except** the items below.

## Anti-slip `impratio` — set once on your physics scene

The finger-coupling equality (`solref = [0.002, 1]`, near-rigid) is authored on
`right_outer_knuckle_joint` (`MjcEqualityJointAPI`) and rebuilds natively. The one
setting the asset cannot carry is the solver's anti-slip option, because the
gripper has no `PhysicsScene`. Author it **once** on yours (it needs
`MjcSceneAPI` applied):

```usda
uniform double mjc:option:impratio = 10
```

Needs **Isaac Sim 6.1 / Newton 1.5.0** or later, which reads scene `mjc:option:*`
from USD. No script is needed after Stop → Play.

**Isaac Sim 6.0.1 (Newton 1.2.1):** the build ignores `mjc:option:impratio`, so
still run [`apply_gripper_tuning.py`](apply_gripper_tuning.py) after every
Stop → Play (Script Editor or MCP bridge — idempotent).

## Intentional `diagonalInertia = (0.001, 0.001, 0.001)`

Every gripper link's `physics:diagonalInertia` is `(0.001, 0.001, 0.001)` —
~100× the physical value, and **intentional, not a placeholder**. The extra
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
