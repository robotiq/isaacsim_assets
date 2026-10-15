# Robotiq gripper assets

Isaac Sim 6 assets for Robotiq grippers. Each gripper is one USD file: open or
reference it, then pick the physics engine and fingertip with its variant sets.

## Available grippers

| Gripper | Entry point | Physics variants | Fingertip variants | Documentation |
| --- | --- | --- | --- | --- |
| 2F-85 | [`Robotiq_2F_85/Robotiq_2F_85.usda`](Robotiq_2F_85/Robotiq_2F_85.usda) | `Physx_parallel_grip` *(default)*, `Physx_compliant`, `Newton_compliant`, `Newton_parallel_grip` | `Standard` *(default)*, `Tactile_TSF85` | [2F guide](docs/2F/README.md), [Newton tuning](Robotiq_2F_85/newton/README.md) |
| 2F-140 | [`Robotiq_2F_140/Robotiq_2F_140.usda`](Robotiq_2F_140/Robotiq_2F_140.usda) | `Physx_parallel_grip` *(default)*, `Physx_compliant`, `Newton_compliant`, `Newton_parallel_grip` | `Flat_Overmolded` *(default)*, `Silicone`, `V_Groove`, `Tactile_TSF140` | [2F guide](docs/2F/README.md), [Newton tuning](Robotiq_2F_140/newton/README.md) |
| Hand-E | [`Robotiq_Hand_E/Robotiq_Hand_E.usda`](Robotiq_Hand_E/Robotiq_Hand_E.usda) | `PhysX` *(default)*, `Newton` | `Std` *(default)*, `Flat_Overmolded`, `Support`, `Extender`, `Bin_Picking` (+ `FingertipMount`: `Outside` / `Inside`) | [Hand-E README](docs/HandE/README.md) |

`Physx_*` variants run on the regular Isaac Sim app; `Newton_*` variants need
Isaac Sim started with the Newton engine. See
[Physics engines](docs/2F/README.md#physics-engines).

## Folder layout of a gripper

```
Robotiq_<model>/
  Robotiq_<model>.usda     Entry point with the Physics and Fingertip variant sets
  configuration/           Ready-made builds with one Physics variant fixed
  payloads/                Geometry, kinematics and per-engine physics layers
  parts/, materials/       Meshes and materials
  newton/                  Newton runtime tuning script and notes (2F only)
  PACKAGE-LICENSES/        Third-party licenses for this asset
```

## Documentation

**Gripper guides**

- [2F-85 / 2F-140 guide](docs/2F/README.md): variants, physics engines, tuning, the physics of the mechanism, Newton.
- [Hand-E](docs/HandE/README.md): variants, joints, source specifications and finger force per fingertip.
- Newton tuning: [2F-85](Robotiq_2F_85/newton/README.md), [2F-140](Robotiq_2F_140/newton/README.md).

**Using a gripper in a scene** (general Isaac Sim guides, in [`../docs`](../docs/))

- [Setup your scene](../docs/1-Setup_your_scene.md): a room, a cube and a UR5e or Franka arm.
- [Get started](../docs/2-Get_started.md): attach the gripper to the arm and try it.
- [Do a pick and place](../docs/3-Do_a_pick_and_place.md)
- [Control from ROS](../docs/4-Control_from_ROS.md)
- [Connect an AI agent to Isaac Sim](../docs/5-Connect_AI_agent_to_Isaac_Sim.md)
- [Tune physics for grasping](../docs/6-Tune_physics_for_grasping.md): object mass, friction and contact settings, for any gripper.

**Demo**

- [Teleop demo](demo/teleop/README.md): drive a UR5e with any gripper from a gamepad, the keyboard or the mouse.
  - [Controller profiles](demo/teleop/CONTROLLER_PROFILES.md): add a profile for your gamepad.
  - [Physics tuning](demo/teleop/PHYSICS_TUNING.md)
- [MCP bridge](demo/mcp_bridge/README.md): connect Claude Code to a running Isaac Sim.
- [Recording a Newton video](demo/video_export/NEWTON_VIDEO_RECORDING.md)
