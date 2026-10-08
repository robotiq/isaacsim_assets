# isaacsim_assets

Public [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) assets
maintained by [Robotiq](https://robotiq.com). This repository is a home for
Robotiq simulation assets and the documentation needed to use them. Three
grippers are published here today, the **2F-85**, the **2F-140** and the **Hand-E**, and the
layout is organized by asset category so other assets (grippers, robots,
tooling, scenes) can be added over time.

## Layout

```
grippers/                         Gripper assets
  GRIPPER_SIMULATION_GUIDE.md     How to simulate Robotiq grippers in Isaac Sim
  images/                         Figures used by the guide
  Robotiq_2F_85/                  2F-85 gripper (PhysX + Newton solver variants)
  Robotiq_2F_140/                 2F-140 gripper (PhysX + Newton solver variants)
  Robotiq_Hand_E/                 Hand-E gripper (PhysX, Fingertip + FingertipMount variants)
  demo/                           Demos, incl. the UR5e teleop for either gripper
LICENSE                           Repository license + third-party attributions
```

New asset categories should be added as sibling top-level directories (e.g.
`robots/`, `scenes/`), each self-contained with its own assets and docs.

## Contents

### Robotiq 2F-85, 2F-140 and Hand-E grippers

The 2F-85, 2F-140 and Hand-E grippers are authored for Isaac Sim 6 with the
same layout: each is a single asset whose `Physics` variant selects the solver
and grip — same body tree, geometry, and collision for all of them — and whose
`Fingertip` variant selects the fingertip.

| | 2F-85 | 2F-140 | Hand-E |
| --- | --- | --- | --- |
| Asset | [`grippers/Robotiq_2F_85/`](grippers/Robotiq_2F_85/) | [`grippers/Robotiq_2F_140/`](grippers/Robotiq_2F_140/) | [`grippers/Robotiq_Hand_E/`](grippers/Robotiq_Hand_E/) |
| `Physics` — PhysX | `Physx_parallel_grip` *(default)*, `Physx_compliant` | `Physx_parallel_grip` *(default)*, `Physx_compliant` | `PhysX` *(default)* |
| `Physics` — Newton / MuJoCo-Warp | `Newton_compliant`, `Newton_parallel_grip` ([tuning](grippers/Robotiq_2F_85/newton/)) | `Newton_compliant`, `Newton_parallel_grip` ([tuning](grippers/Robotiq_2F_140/newton/)) | `Newton` |
| `Fingertip` | `Standard` *(default)*, `Tactile_TSF85` | `Flat_Overmolded` *(default)*, `Silicone`, `V_Groove`, `Tactile_TSF140` | `Std` *(default)*, `Flat_Overmolded`, `Support`, `Extender`, `Bin_Picking` |
| `FingertipMount` | — | — | `Outside` *(default)*, `Inside` (fingertip screw-hole row; `Inside` is 9 mm closer to the centerline) |

The Hand-E has two sliding fingers (50 mm stroke) instead of a linkage, so it has one
`PhysX` and one `Newton` physics variant (one driven finger joint, the other finger follows it).
The Newton coupling is a stiff MuJoCo joint equality baked into the asset, so no runtime tuning
script is needed.

`*_parallel_grip` keeps the fingers parallel (mimic joints on PhysX, a welded
coupler on Newton); `*_compliant` simulates the closed five-bar loop, so the
fingers wrap around the object. The Newton variants model the linkage more
robustly but need a one-shot runtime tuning after each Stop→Play (see the
gripper's `newton/` folder, linked in the table).

The [UR5e teleop demo](grippers/demo/teleop/) runs either gripper on either
solver (`TELEOP_GRIPPER=2F85|2F140`, `TELEOP_PHYSICS=<Physics variant>`).

Start with the
**[Gripper Simulation Guide](grippers/GRIPPER_SIMULATION_GUIDE.md)** — it covers
choosing a variant, mounting the gripper on a robot, tuning it for reliable
grasping, the physics of the mechanism, and the Newton backend.

## Getting the assets

The heavy asset files (`.usd`, `.png`) are stored with
[Git LFS](https://git-lfs.com). Install it before cloning so the real files are
fetched instead of pointer stubs:

```bash
git lfs install
git clone https://github.com/robotiq/isaacsim_assets.git
```

`.usda` files are plain-text USD and are kept diffable (not in LFS).

## License

Original work in this repository (documentation, scripts, and Robotiq-authored
asset content) is released under the **BSD 3-Clause License**.

Some asset files were originally copied or derived from a third party (the NVIDIA
Isaac Sim asset library) and remain under its own license. See
[`LICENSE`](LICENSE) for the full text and attributions, and the `LICENSE` files
bundled next to the affected assets.
