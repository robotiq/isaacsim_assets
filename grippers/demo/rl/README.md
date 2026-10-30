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
git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab   # v3.0.0, Isaac Sim 6.2 (the path is an example; scripts read it from ISAACLAB)
cd ~/IsaacLab && uv sync --extra isaacsim
uv pip install --python .venv/bin/python --no-deps -e <this repo>/grippers/tactile_sensors -e <this repo>/grippers/demo/rl
uv pip install --python .venv/bin/python moviepy imageio-ffmpeg                      # only for --video
export OMNI_KIT_ACCEPT_EULA=YES   # after accepting NVIDIA's Omniverse license agreement
```

`uv sync` drops the editable installs and the video packages again: repeat the `uv pip install` lines. The task is only registered
when `robotiq_rl` is imported: Isaac Lab's `train.py`/`play.py` find it through the editable install.

## Train and evaluate

The policy behind the multi-pick numbers below is in the repo: `robotiq_rl/tactile_pick/policy/policy.pt` (the exported actor, TorchScript, 0.5 MB, Git LFS),
run `tp20` chunk 3, with that run's `env.yaml` and `agent.yaml` next to it (the observation layout it expects). It is what "the shipped policy" means
here. Training checkpoints (`model_*.pt`, with critic and optimizer) are not in the repo (about 5 h of training on an 8 GB GPU for this one).
`tp8`, in the next section, is an earlier single-pick policy and is not in the repo.

Train in chunks, with an evaluation and a single-env video after each (on a shared machine it can queue behind a GPU lock, `ISAAC_LOCK=<wrapper>`, optional; `PICK_ALL=1` also scores
each chunk with the emptying routine of `pick_all.py`):

```bash
PICK_ALL=1 grippers/demo/rl/scripts/train_in_chunks.sh my_run 600 12 2048 [first_checkpoint]
# <run>/eval.txt: per-phase statistics on 64 envs;  <run>/eval_video/: env 0 of the exported policy
```

Scripted checks of the pieces (they drive the real controller with the sphere's true position, not a policy):
`scripts/probe_tactile.py`, `probe_grasp.py`, `show_scene.py` (a clip of the scene); `eval_tactile.py --ablate_tactile` zeroes the
tactile observations to show whether the policy needs to feel.

## Results of the single-pick policy (historical: `tp8`, the earlier single-pick environment; the current multi-pick environment does not reproduce this table)

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

## Tactile contact-distance gate

The demo uses the library's default 8 mm gate (`max_contact_distance`: a contact farther than that from every taxel reads nothing), because it is closer to the real sensor. The policy was trained before the gate existed, so it relies partly on back/side contacts the gate removes. Same policy, same seeded boxes, 256 envs x 600 steps (`scripts/eval_tactile.py --seed 31/32 --gate -1|0.008`):

| Sensor | Successes (seed 31 / 32) |
|---|---|
| gate off (trained with) | 1225 / 1286 |
| gate on (default) | 1095 / 1150 |

Single-pick speed drops ~10%. On the emptying routine (see the next section for how it is measured; reachable layout, 64 envs, seeds 41 / 42) the gate makes little difference: 4.0 / 3.9 picks per box with the gate against 4.1 / 4.4 without. Retraining with the gate on did not help, so the policy in the repo is kept:
- from scratch (1200 iterations): never enters the box;
- warm start from the single-pick policy, gate on, 3000 to 4800 iterations (also with no entropy bonus and a lower learning rate): single-pick successes per 64 envs 272 at best, then falling to 190 - 260 (the ungated policy: ~306);
- the retrained candidates did not beat the policy in the repo on the routine either (measured with the task completing the picks, as in training: 8.0 / 7.8 against 7.3 / 6.7 and 7.3 / 6.6). `--gate -1` evaluates without the gate.

## Emptying the box (`scripts/pick_all.py`)

A supervisor around the trained policy: the policy picks, and when the pads sense a held sphere above the rim (pad force
and tool-tip height only, no privileged state) a scripted routine carries it to the table beside the box, opens the
gripper and returns above the box before handing control back to the policy for the next sphere.

<video src="docs/emptying_the_box.mp4" controls width="800"></video>

[**Video: the routine empties the box** (`docs/emptying_the_box.mp4`, 1080p, 60 fps, RTX render, tactile gate on)](docs/emptying_the_box.mp4): one run (seed 41, one env) that picked 8 of 10 and dropped them beside the box; it is a good run, the mean over 64 envs is 4.0 (range 1 - 7), see below. Recorded with
`pick_all.py --policy robotiq_rl/tactile_pick/policy/policy.pt --num_envs 1 --layout reachable --seed 41 --hq_video out.mp4` (`--hq_fps 120|60|30`; the camera is picked up from the scene, no extra flag).

**How the picks are counted matters.** In training the task itself completes a pick once a sphere has been held out of the box for a second: it parks the sphere and puts the arm at a random start with the gripper open. Earlier versions of this README (and `pick_all.py`) measured the routine with that still on, so it fired while the scripted carry was under way and teleported almost every sphere (7.7 - 8.0 completions per box, for 7.8 - 8.1 picks counted): those 7.6 - 8.1 picks per box were mostly the task's, not the carry's. `pick_all.py` now turns it off (`--env_completes_picks` brings it back) and counts only spheres the supervisor really carried and dropped. Measured that way, with the policy in the repo (64 envs, seeds 41 / 42, gate on):

| Layout | Picks per box of 10 (mean, min - max) | Fully emptied |
|---|---|---|
| spheres within reach (`--layout reachable`) | **4.0 / 3.9** (1 - 7) | 0 % |
| spheres spread over the whole floor, as in training (`--layout floor`) | **4.1 / 4.0** (1 - 7) | 0 % |
| for comparison, the task completing the picks (`--env_completes_picks`), reachable | 8.1 / 7.8 | 12 % / 14 % |

Envs stop after 900 steps without a pick (`--give_up`), usually with spheres still inside the reachable zone (0.7 - 1.1 per box on average, a few envs with many): after the scripted carry the policy often does not find the next sphere. That is the open problem of this demo; the policy was trained with the task's teleport-and-home pick completion, not with the supervisor's carry and drop. `tp8` (single picks only) got 3.4 - 4.2 with the old counting.

With the task completing the picks, the remaining 1-3 spheres are those the gripper cannot straddle (against a wall, in a corner: the box is unchanged, see
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
