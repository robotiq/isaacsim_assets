# Learning policies with Isaac Lab (UR5e + Robotiq 2F-85)

Goal: teach the UR5e + 2F-85 of the [teleop demo](../teleop/README.md) to grasp a cube and lift it,
with reinforcement learning (PPO, rsl_rl) in Isaac Lab 3.0 on PhysX. No teleoperation or ROS involved.

Task `Robotiq-Lift-Cube-UR5e-2F85`: a 5 cm cube on a table, a random goal point above it; the policy
drives the six arm joints (position targets) and a binary open/close command for the gripper.

| File | What |
|---|---|
| `assets/ur5e_2F-85.usda` | The teleop scene's UR5e + 2F-85 weld as one articulation (`/Robot`). |
| `robotiq_rl/assets.py` | Isaac Lab articulation config: arm gains, gripper driver `finger_joint`. |
| `robotiq_rl/lift_cube/env_cfg.py` | The environment: stock Isaac Lab lift task plus our robot, cube and rewards. |
| `robotiq_rl/lift_cube/mdp.py` | TCP-based reward terms, gated on a grasp (see below). |
| `robotiq_rl/lift_cube/agents.py` | PPO settings. |
| `scripts/probe_scene.py` | Prints bodies, joints, TCP, tool axis and the gripper opening/closing. |
| `scripts/eval_policy.py` | Rolls out an exported policy and prints grasp statistics. |

## Setup

```bash
git clone https://github.com/isaac-sim/IsaacLab.git ~/robotiq/devel/IsaacLab   # v3.0.0, Isaac Sim 6.2
cd ~/robotiq/devel/IsaacLab && uv sync --extra isaacsim
uv pip install --python .venv/bin/python --no-deps -e <this repo>/grippers/demo/rl   # registers the task
uv pip install --python .venv/bin/python moviepy imageio-ffmpeg                      # only for --video
export OMNI_KIT_ACCEPT_EULA=YES   # after accepting NVIDIA's Omniverse license agreement
```

`uv sync` drops the editable install and the video packages again: repeat the two `uv pip install` lines.

## Train, play, evaluate

From the Isaac Lab checkout (training is headless by default; these numbers are for an 8 GB laptop GPU):

```bash
.venv/bin/python scripts/reinforcement_learning/train.py --rl_library rsl_rl \
    --task Robotiq-Lift-Cube-UR5e-2F85 --num_envs 2048 --max_iterations 3000     # ~40 min
.venv/bin/python scripts/reinforcement_learning/play.py --rl_library rsl_rl \
    --task Robotiq-Lift-Cube-UR5e-2F85 --num_envs 1 --video --video_length 300   # also exports policy.pt
.venv/bin/python <this repo>/grippers/demo/rl/scripts/eval_policy.py --policy logs/rsl_rl/lift_cube_ur5e_2f85/<run>/exported/policy.pt
```

Runs go to `logs/rsl_rl/lift_cube_ur5e_2f85/`; videos to `<run>/videos/play/`. The task is only registered
when `robotiq_rl` is imported: the `train.py`/`play.py` scripts find it through the editable install.

## Results

| Run | What changed | Held and lifted (steps) | Goal reached | Verdict |
|---|---|---|---|---|
| v1 (1500 it, 1024 envs) | stock lift rewards | 38% | <1% | pushes and tilts the cube |
| v2 (2500 it, 2048 envs) | + reward for closing near the cube | 91% (by height) | 4% | **reward hack**: wedges the cube in the forearm/wrist crease and carries it with the gripper open, TCP 18 cm away |
| v3 (3000 it, 2048 envs) | lift and goal rewards require the cube between the pads and the gripper partly closed | 89% | 3-10% (noisy) | **real grasp**: TCP 1.8 cm from the cube, gripper ~46% closed, cube carried to the goal height |

Lesson: a lift reward on cube height alone is satisfied without grasping. Gate it on the grasp
(TCP near the cube, gripper closed on it).

## Open items

- Reaching the goal point is still weak (success 3-10% of episodes, noisy): train longer, tune the goal ranges
  and reward weights, or use end-effector (differential IK) actions.
- The gripper uses the asset's default `Physx_parallel_grip` variant; the teleop's finger tuning is not carried over.
- "Held" is a distance and closure proxy; a contact sensor on the pads would be stricter.
- Newton backend and the 2F-140 are follow-ups.
