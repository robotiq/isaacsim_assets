# Newton (MuJoCo-Warp) tuning — 2F-85

Newton-specific runtime tuning and notes for the **`Newton_compliant`** variant
of the unified 2F-85 asset ([`../Robotiq_2F_85.usda`](../Robotiq_2F_85.usda)).
Select `Physics = Newton_compliant`, ensure the scene has a `PhysicsScene`, and
play. The gripper is **open by default** (`finger_joint` drive target `0`); drive
`finger_joint` toward ~0.8 rad (≈45°) to close.

Everything the solver needs is baked into the payload
([`../payloads/Robotiq_2F_85_newton_compliant_physics.usda`](../payloads/Robotiq_2F_85_newton_compliant_physics.usda))
and rebuilds natively on load — **except** the items below.

## Runtime tuning — `apply_gripper_tuning.py` (each stop → play)

Two settings cannot be authored in USD and must be re-applied to the live
`mujoco_warp` model after every stop → play (run from the Script Editor or the
MCP bridge — idempotent):

1. `opt.impratio = 10` — anti-slip; the build ignores the authored `MjcSceneAPI`
   option.
2. near-rigid finger-coupling equality `solref = [0.002, 1]` — the coupling
   joint-equality has no USD prim (it is synthesized at model build), so it is
   located at runtime by `eq_type == mjEQ_JOINT`.

## Intentional `diagonalInertia = (0.001, 0.001, 0.001)`

Every gripper link's `physics:diagonalInertia` is `(0.001, 0.001, 0.001)` —
~100× the physical value, and **intentional, not a placeholder**. The extra
rotational inertia stabilizes the soft Newton loop-closure equalities so the weak
parallel-grip spring holds; with physically-small inertias the four-bar goes
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
