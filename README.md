# isaacsim_assets

Public [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) assets
maintained by [Robotiq](https://robotiq.com). This repository is a home for
Robotiq simulation assets and the documentation needed to use them. Only the
**2F-85 gripper** is published here today, but the layout is organized by asset
category so other assets (grippers, robots, tooling, scenes) can be added over
time.

## Layout

```
grippers/                         Gripper assets
  GRIPPER_SIMULATION_GUIDE.md     How to simulate Robotiq grippers in Isaac Sim
  images/                         Figures used by the guide
  Gripper_2F85/                   2F-85 for the PhysX backend
  Gripper_2F85_newton/            2F-85 for the Newton (MuJoCo-Warp) backend
LICENSE                           Repository license + third-party attributions
```

New asset categories should be added as sibling top-level directories (e.g.
`robots/`, `scenes/`), each self-contained with its own assets and docs.

## Contents

### Robotiq 2F-85 gripper

A 2F-85 adaptive gripper authored for Isaac Sim 6, provided for both physics
backends:

- **PhysX** — [`grippers/Gripper_2F85/`](grippers/Gripper_2F85/), with
  `parallel_grip` (mimic-joint) and `compliant` (closed five-bar loop) variants.
- **Newton / MuJoCo-Warp** —
  [`grippers/Gripper_2F85_newton/`](grippers/Gripper_2F85_newton/), which models
  the compliant five-bar linkage more robustly; see its
  [README](grippers/Gripper_2F85_newton/README.md).

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

Some asset files were originally copied or derived from third parties (the NVIDIA
Isaac Sim asset library and the MuJoCo Menagerie) and remain under their own
licenses. See [`LICENSE`](LICENSE) for the full text and attributions, and the
`LICENSE` files bundled next to the affected assets.
