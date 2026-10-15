<!-- docs-site:exclude -->
# isaacsim_assets

📖 See rendered documentation on: https://robotiq.github.io
<!-- docs-site:exclude -->

Public [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) assets
maintained by [Robotiq](https://robotiq.com). This repository is a home for
Robotiq simulation assets and the documentation needed to use them. Three
grippers are published here today, the **2F-85**, the **2F-140** and the **Hand-E**, and the
layout is organized by asset category so other assets (grippers, robots,
tooling, scenes) can be added over time.

<!-- docs-site:exclude -->
## General documentation

- [Setup your scene](./docs/1-Setup_your_scene.md)
- [Get started](./docs/2-Get_started.md)
- [Do a pick and place](./docs/3-Do_a_pick_and_place.md)
- [Control from ROS](./docs/4-Control_from_ROS.md)
- [Connect AI agent to Isaac Sim](./docs/5-Connect_AI_agent_to_Isaac_Sim.md)
- [Tune physics for grasping](./docs/6-Tune_physics_for_grasping.md)

## Layout

```
docs/                             General Isaac Sim guides
grippers/                         Gripper assets files
tests/                            Isaac Sim runtime tests
LICENSE                           license + third-party attributions
```

## Contents

### Robotiq 2F-85, 2F-140 and Hand-E grippers

[`grippers/README.md`](grippers/README.md) lists every gripper asset and its
documentation.

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

The [UR5e teleop demo](grippers/demo/teleop/) runs any of these grippers.

To get started, follow the [general documentation](#general-documentation)
to set up a scene, then open your gripper's guide from
[`grippers/README.md`](grippers/README.md) for its variants and specific
settings.

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
<!-- docs-site:exclude -->
