# Robotiq 2F-85 — Newton gripper asset

Standalone Robotiq 2F-85 gripper tuned for Isaac Sim's **Newton (MuJoCo-Warp)**
physics backend. Self-contained (no arm or environment), gripper at the origin,
with its own `PhysicsArticulationRootAPI` — drop it into a scene and it simulates
natively.

## Provenance

The **physics model** derives from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85)
Robotiq 2F-85 (`2f85.xml`): bodies, masses/inertias, joints, the four-bar
loop-closure, and the contact/friction tuning all come from there. The **visual
meshes** are Robotiq's detailed CAD (`Defeatured_2F_85_*`, the same parts used by
the sibling `Gripper_2F85` asset). The two were combined and imported into Isaac
through the Newton USD importer, then flattened and cleaned into this asset.

## How it was generated

1. Start from the developed Newton gripper (`ur5robot_2F-85gripper_newton_cad.usda`
   in the ROS working tree): the Menagerie 2F-85 imported via Newton, retuned,
   with the Robotiq CAD visuals referenced onto the collision bodies.
2. Reference only the gripper subtree (`.../wrist_3_link/robotiq_2f85_mjcf`) into
   a fresh stage at the origin (robot + environment dropped) and `Stage.Flatten()`.
3. Restore the 6 MuJoCo `<contact><exclude>` self-collision pairs that the
   MJCF→USD import dropped, re-authored as `physics:filteredPairs`
   (base↔driver, base↔spring_link, coupler↔follower — both sides). Without them
   the fingers jam on coupler↔follower self-contact instead of relaxing to the
   parallel-grip pose.
4. Split the heavy inlined mesh geometry into per-body files under `geometry/`
   (sublayered via `geometry/geometry.usda`), leaving a small, readable `.usda`
   for the structure.

## Layout

```
Gripper_2F85_newton/
├── Robotiq_2F85_newton.usda   # structure: prim tree, joints, drives, Mjc tuning, exclusions
├── geometry/
│   ├── geometry.usda           # index — sublayers the per-body meshes
│   └── <body>.usd              # per-body mesh geometry (crate, git-LFS)
├── apply_gripper_tuning.py     # runtime tuning (see below)
└── README.md
```

## Usage

- Load the asset, ensure a `PhysicsScene` exists, and play. The gripper is **open
  by default** (`finger_joint` drive target `0`); drive `finger_joint` toward
  ~0.8 rad (≈45°) to close.
- After each **stop → play**, run **`apply_gripper_tuning.py`** once (Script
  Editor or MCP bridge). It re-applies the only two settings that cannot be
  persisted in USD:
  1. `opt.impratio = 10` — anti-slip;
  2. the near-rigid finger-coupling equality `solref` — the equality has no USD
     prim (it is synthesized at model build).

  Everything else (drive gains, armature, `mjc:damping`, spring stiffness,
  `solreflimit`, loop-closure equalities, contact tuning) is baked into the
  `.usda` and rebuilds natively on load.

## Notes

- Physics is **Newton / MuJoCo-Warp**; the tuning is Newton-specific.
- `geometry/*.usd` are git-LFS tracked; the `.usda` files are plain text and
  diffable.
- **Link `physics:diagonalInertia = (0.001, 0.001, 0.001)` is intentional, not a
  placeholder.** It is ~100× the physical value; the extra rotational inertia
  stabilizes the soft Newton loop-closure equalities so the weak parallel-grip
  spring holds. With physically-small inertias the four-bar goes floppy and the
  fingertips sag ~20° out of parallel. Treat it like the `mjc:armature`/`damping`
  Newton tuning — do **not** replace it with CAD/menagerie inertias without
  re-tuning the loop-closure `solref` (or adding passive-joint armature).

## Local Isaac Sim / Newton patches

This asset was developed and validated against a **locally patched** Isaac Sim.
There are two kinds of patch — one persistent source edit, and two runtime
model edits.

### 1. In-place source patch — `solreflimit` ×180/π backport (required)

The bundled **newton 1.2.1** (Isaac Sim 6.0.1) over-scales the *angular*
hinge/D6 `mjc:solreflimit` stiffness/damping by **180/π**. An authored
`mjc:solreflimit = [0.02, 2]` compiles to `jnt_solref = [-35809, -5729]`
instead of the intended `[-625, -100]`. Because this asset **bakes
`mjc:solreflimit` into the `.usda`** (the couplers/joint limits), a stock Isaac
6.0.1 imports it wrong.

- **Fix:** backport of upstream newton PR **#2736** ("Fix USD MJC angular
  `limit_ke/kd` over-scaling on revolute/D6 joints"). Adds
  `_solref_to_stiffness_per_rad` / `_solref_to_damping_per_rad` (pre-multiply by
  π/180) and routes the angular `limit_*_ke/kd` schema attrs through them.
- **File patched:** `exts/isaacsim.pip.newton/pip_prebundle/newton/_src/usd/schemas.py`
  in the Isaac install (backup saved alongside as `schemas.py.orig-solreflimit-bug`).
- **Patch file:** `solreflimit_2736_backport.patch` (kept with the developer's
  Isaac install). After the patch: authored `[0.02, 2]` → `jnt_solref = [-625, -100]`.
- ⚠️ **Vendored file** — an Isaac Sim reinstall/upgrade overwrites it. Reapply the
  patch, or move to an Isaac build bundling newton ≥ the one containing #2736.
  Already fixed on newton `main`, so this is packaging lag, not an open defect.

### 2. Runtime model patches — `apply_gripper_tuning.py` (each stop → play)

Two settings **cannot** be authored in USD and are applied to the live
`mujoco_warp` model at play time (see `apply_gripper_tuning.py` above):

1. `opt.impratio = 10` — anti-slip; the build ignores the authored `MjcSceneAPI`
   option.
2. near-rigid finger-coupling equality `solref = [0.002, 1]` — the coupling
   joint-equality has no USD prim (synthesized at model build), located by
   `eq_type == mjEQ_JOINT`.
