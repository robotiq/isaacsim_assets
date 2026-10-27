# Blind sphere picking with tactile fingertips (UR5e + Robotiq 2F-85, Isaac Lab)

Reinforcement learning (PPO, rsl_rl) in Isaac Lab 3.0 on PhysX: a UR5e with a Robotiq 2F-85 on `Tactile_TSF85` fingertips has to
pick spheres out of an open box **without seeing them**, from proprioception and the simulated tactile maps of
[`grippers/tactile_sensors`](../../tactile_sensors/README.md) (the sensor model this task is built on).

| File | What |
|---|---|
| `assets/ur5e_2F-85.usda`, `assets/ur5e_2F-85_tsf85.usda` | The teleop scene's UR5e + 2F-85 weld as one articulation (`/Robot`), and the same with the tactile fingertips. |
| `robotiq_rl/assets.py` | Isaac Lab articulation configs: arm gains, gripper driver `finger_joint`. |
| `robotiq_rl/tactile_pick/` | The task: scene and MDP (`env_cfg.py`, `mdp.py`), PPO settings (`agents.py`), random start poses (`start_poses.npz`). |
| `scripts/` | `train_in_chunks.sh`, `eval_tactile.py`, `pick_all.py`, and scripted checks (`probe_*.py`, `show_scene.py`). |

## The task

`Robotiq-Tactile-Sphere-Pick-UR5e-2F85`: the 2F-85 on `Tactile_TSF85` fingertips (`SensorSim=EffectiveResponse`) has to
pick one of ten 4 cm spheres out of an open box **without knowing where they are**: get the fingertips into the box,
move until the pads feel a sphere, grasp it, judge the grasp, lift it out.

| What | How |
|---|---|
| Scene | UR5e on a table, 36 x 36 cm box (10 cm deep), 10 spheres (4 cm, 30 g) spread at random over the floor. Sphere 0 is always in the zone the tool tip can reach (the gripper base and the open fingers cannot get within ~5-8 cm of the walls), so about half of the spheres are out of reach. The tool tip starts at a random point above and around the box. |
| Actions | Tool-tip motion (relative IK, 3 translations; the three rotation dims have scale 0: the tool stays pointing down) and a continuous gripper target. |
| Actor sees | Joint state, tool-tip position in the box frame, gripper, **two 7 x 4 taxel force maps** (3 steps of history). No sphere positions, no camera. |
| Critic sees | Also the sphere positions and the phase flags (asymmetric PPO). |
| Tactile | The `TactileSensor` of [`grippers/tactile_sensors`](../../tactile_sensors/README.md), filtered on the spheres and on the box parts: the force of each object, spread by a Gaussian around its average contact point onto the TSF-85 taxel grid. The contact point lands within ~3 mm of the right taxel; two pads squeezing a sphere read a symmetric ~11 N. |
| Phases | Rewarded in order and gated: enter the box, sweep and touch, grasp (both pads press a sphere, gripper partly closed; balance of the two forces), stable grasp (8 steps), lift; success = the held sphere stays above the rim + 4 cm, over the box, for 1 s. |

## Setup

```bash
git clone https://github.com/isaac-sim/IsaacLab.git ~/robotiq/devel/IsaacLab   # v3.0.0, Isaac Sim 6.2
cd ~/robotiq/devel/IsaacLab && uv sync --extra isaacsim
uv pip install --python .venv/bin/python --no-deps -e <this repo>/grippers/tactile_sensors -e <this repo>/grippers/demo/rl
uv pip install --python .venv/bin/python moviepy imageio-ffmpeg                      # only for --video
export OMNI_KIT_ACCEPT_EULA=YES   # after accepting NVIDIA's Omniverse license agreement
```

`uv sync` drops the editable installs and the video packages again: repeat the `uv pip install` lines. The task is only registered
when `robotiq_rl` is imported: Isaac Lab's `train.py`/`play.py` find it through the editable install.

## Train and evaluate

Trained policies are not in the repo (about 5 h of training on an 8 GB GPU for the one described below).

Train in chunks, with an evaluation and a single-env video after each (the training holds the GPU lock; `PICK_ALL=1` also scores
each chunk with the emptying routine of `pick_all.py`):

```bash
PICK_ALL=1 grippers/demo/rl/scripts/train_in_chunks.sh my_run 600 12 2048 [first_checkpoint]
# <run>/eval.txt: per-phase statistics on 64 envs;  <run>/eval_video/: env 0 of the exported policy
```

Scripted checks of the pieces (they drive the real controller with the sphere's true position, not a policy):
`scripts/probe_tactile.py`, `probe_grasp.py`, `show_scene.py` (a clip of the scene); `eval_tactile.py --ablate_tactile` zeroes the
tactile observations to show whether the policy needs to feel.

## Results of the single-pick policy

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

## Emptying the box (`scripts/pick_all.py`)

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

## Lessons (what went wrong on the way)

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
## Lessons from the single-pick version

- **Hovering under the finish line.** With a dense per-step lift reward and a termination on success, the policy parked
  the sphere 6 mm below the success height: finishing ended the episode and forfeited the reward it was collecting.
  Fix: no termination on success, pay "held out of the box" per step, keep the dense reward rising past the threshold.
- **Weak early shaping.** One run never found a grasp in 400 iterations; larger approach / enter / sweep / touch weights fixed it.
- **Static colliders cannot be contact-filter targets** (the filtered force reads 0 while the unfiltered one does not):
  the box parts are kinematic rigid bodies.
- **Easy version first.** With a fixed start and the spheres in the reachable zone the policy never searches: it
  closes on whatever is under it. The random start and the spread-out spheres make it search.
- GUI training is more than twice as slow as headless here, so watch the per-chunk videos instead.
