# Learning policies with Isaac Lab (UR5e + Robotiq 2F-85)

Goal: teach the UR5e + 2F-85 of the [teleop demo](../teleop/README.md) to grasp a cube and lift it,
with reinforcement learning (PPO, rsl_rl) in Isaac Lab 3.0. No teleoperation or ROS involved.

**Status: work in progress, not yet run.** The files below were written against Isaac Lab's stock
`contrib/lift` task but have not been simulated yet (Isaac Sim's EULA has to be accepted first).

| File | What |
|---|---|
| `assets/ur5e_2F-85.usda` | The teleop scene's UR5e + 2F-85 weld as one articulation (`/Robot`). |
| `robotiq_rl/assets.py` | Isaac Lab articulation config: arm gains, gripper driver `finger_joint`. |
| `robotiq_rl/lift_cube/` | Task `Robotiq-Lift-Cube-UR5e-2F85`: env config, PPO config, registration. |

## Setup

```bash
git clone https://github.com/isaac-sim/IsaacLab.git ~/robotiq/devel/IsaacLab   # v3.0.0, Isaac Sim 6.2
cd ~/robotiq/devel/IsaacLab && uv sync --extra isaacsim
export OMNI_KIT_ACCEPT_EULA=YES   # only after reading NVIDIA's Omniverse license agreement
```

## Open items

- `TCP_OFFSET` in `lift_cube/env_cfg.py` is an estimate; measure it in the composed stage.
- The gripper uses the asset's default `Physx_parallel_grip` variant; the teleop's finger tuning is not
  carried over yet.
- Table, robot placement and the goal-pose ranges come from the stock Franka task and need adapting to the UR5e's reach.
- Newton backend and the 2F-140 are follow-ups.
