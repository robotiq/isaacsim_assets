# Learning policies with Isaac Lab (UR5e + Robotiq 2F-85)

Two tasks: a [cube lift](#cube-lift) and a [blind sphere pick with the TSF-85 tactile fingertips](#blind-sphere-pick-with-tactile-fingertips).

## Cube lift

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


## Blind sphere pick with tactile fingertips

`Robotiq-Tactile-Sphere-Pick-UR5e-2F85`: the 2F-85 on `Tactile_TSF85` fingertips (`SensorSim=EffectiveResponse`) has to
pick one of ten 4 cm spheres out of an open box **without knowing where they are**: get the fingertips into the box,
move until the pads feel a sphere, grasp it, judge the grasp, lift it out.

| What | How |
|---|---|
| Scene | UR5e on a table, 36 x 36 cm box (10 cm deep), 10 spheres (4 cm, 30 g) spread at random over the floor. Sphere 0 is always in the zone the tool tip can reach (the gripper base and the open fingers cannot get within ~5-8 cm of the walls), so about half of the spheres are out of reach. The tool tip starts at a random point above and around the box. |
| Actions | Tool-tip motion (relative IK, 3 translations; the three rotation dims have scale 0: the tool stays pointing down) and a continuous gripper target. |
| Actor sees | Joint state, tool-tip position in the box frame, gripper, **two 7 x 4 taxel force maps** (3 steps of history). No sphere positions, no camera. |
| Critic sees | Also the sphere positions and the phase flags (asymmetric PPO). |
| Tactile | Isaac Lab's `ContactSensor` on each pad, filtered on the spheres and on the box parts: the force of each object, spread by a Gaussian around its average contact point onto the TSF-85 taxel grid (layout from the USD markers, `taxels.py`). The contact point lands within ~3 mm of the right taxel; two pads squeezing a sphere read a symmetric ~11 N. |
| Phases | Rewarded in order and gated: enter the box, sweep and touch, grasp (both pads press a sphere, gripper partly closed; balance of the two forces), stable grasp (8 steps), lift; success = the held sphere stays above the rim + 4 cm, over the box, for 1 s. |

Train in chunks, with an evaluation and a single-env video after each (the training holds the GPU lock):

```bash
grippers/demo/rl/scripts/train_in_chunks.sh my_run 400 12 2048 [first_checkpoint]
# <run>/eval.txt: per-phase statistics on 64 envs;  <run>/eval_video/: env 0 of the exported policy
```

Scripted checks of the pieces (they drive the real controller with the sphere's true position, not a policy):
`scripts/probe_tactile.py`, `probe_contact.py`, `probe_grasp.py`; `eval_tactile.py --ablate_tactile` zeroes the tactile
observations to show whether the policy needs to feel.

### Results

Final policy (`tp8`, 3091 PPO iterations in total, 2048 envs, ~4.5 h on an 8 GB laptop GPU), evaluated on 256 envs x 20 s
with the observation noise and the sphere mass randomization on, the tool tip starting at a random point:

| | With the tactile maps | Tactile maps zeroed (`--ablate_tactile`) |
|---|---|---|
| Entered the box | 100 % (step 7) | 100 % |
| Pads felt a sphere | 100 % (median step 14) | 100 % (touched, but never reacts) |
| Stable grasp (both pads, 8 steps) | 99 % (step 29) | **0 %** |
| Held out of the box for 1 s | **254 / 256** | **0 / 256** |

The policy only closes the gripper after it feels the sphere: with the tactile observation removed it touches spheres
just as often and never grasps. How it gets there, from the progress clips (`tp3_c1` -> `tp8`): it sweeps the pads
over the floor, closes on the first sphere they feel, holds it with ~11 N per pad and lifts it straight up above the box.
Easy-version baseline (fixed start, spheres only in the reachable zone): solved in 300 iterations by a reflex that
closes on whatever is under the gripper, which is why the search was made harder.

Not done: spheres of different sizes (the spawner makes one size), friction randomization (the pad's friction combine
mode is `max`, so a lower sphere friction changes nothing), a real-pad calibration of the simulated taxels (the
sensor model is uncalibrated, see `Robotiq_2F_85/tactile/README.md`), and a check on the Newton backend.

### Emptying the box (`scripts/pick_all.py`)

A supervisor around the trained policy: the policy picks, and when the pads sense a held sphere above the rim (pad force
and tool-tip height only, no privileged state) a scripted routine carries it to the table beside the box, opens the
gripper and returns above the box before handing control back to the policy for the next sphere.

| Policy | Spheres picked of 10 (mean, min-max) | Boxes fully emptied |
|---|---|---|
| `tp8`, trained on single picks | 3.4 - 4.2 (2 - 8) | 0 % |
| `tp20` (chunk 3), trained as a sequence of picks | **7.6** (4 - 10), spheres within reach, 32 envs | 9 % |
| | **8.1** (5 - 10), spheres spread over the whole floor as in training, 16 envs | 12 % |

The remaining 1-3 spheres are those the gripper cannot straddle (against a wall, in a corner: the box is unchanged, see
the geometry above), or ones shoved there. Almost none are left inside the reachable zone.

What made the difference was training the actual task: a successful lift parks the sphere outside, puts the arm back
at a random start with the gripper open and the episode goes on (90 s), so shoving the other spheres towards the walls
costs the picks that follow. The reward is one potential-based shaping term, +100 per sphere picked, -15 for each sphere
knocked out of the reachable zone and a small cost for fast-moving spheres. The pick-complete test in training is the one
the supervisor uses (held out of the box, above the rim, 10 steps).

### Lessons (what went wrong on the way)

- **Reward tuning for the multi-pick task cost most of a night.** Each of these was a real failure: hovering under the
  success height (twice, in different forms); a "hold in a band over the box" that the policy flew past, then dithered
  around the edge of; a dense per-step shaping that earned more than the 100 per pick, fixed by making the shaping
  potential-based; a disturbance penalty of 50 per sphere that made the policy stop moving at all (15 works); an action-rate
  penalty that had silently become 3500 per episode against 460 from picks. Look at the size of every reward term in the
  log before trusting a run.
- **The training score and the real routine disagree until they use the same criterion.** Scoring chunks with
  `pick_all.py` (the supervisor itself, 16 envs) was what showed it; the training metric alone was misleading.
- **Slowing the policy down near the floor to touch gently made it much worse** (1.1 picks): it was never trained at
  reduced speed. Gentleness has to be trained.
- **Jobs of one session share the Isaac lock's lease.** A second job started while the training ran and ran
  alongside it, out of GPU memory; the chunk driver then restarted from scratch. It now stops when a chunk leaves no checkpoint.
### Lessons from the single-pick version

- **Hovering under the finish line.** With a dense per-step lift reward and a termination on success, the policy parked
  the sphere 6 mm below the success height: finishing ended the episode and forfeited the reward it was collecting.
  Fix: no termination on success, pay "held out of the box" per step, keep the dense reward rising past the threshold.
- **Weak early shaping.** One run never found a grasp in 400 iterations; larger approach / enter / sweep / touch weights fixed it.
- **Static colliders cannot be contact-filter targets** (the filtered force reads 0 while the unfiltered one does not):
  the box parts are kinematic rigid bodies.
- **Easy version first.** With a fixed start and the spheres in the reachable zone the policy never searches: it
  closes on whatever is under it. The random start and the spread-out spheres make it search.
- GUI training is more than twice as slow as headless here, so watch the per-chunk videos instead.
