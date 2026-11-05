# Finding, grasping and recognising shape sorter pieces by touch (simulation)

Written 2026-11-03 / 04. Code and demo: `grippers/demo/rl/` (task `Robotiq-Tactile-Shape-Pick-UR5e-2F85`); the demo's own description is [`SHAPE_PICK.md`](SHAPE_PICK.md).
Videos: [`docs/double_pieces_demo.mp4`](docs/double_pieces_demo.mp4) (16 to 32 pieces in the box, final policy and recogniser), [`docs/recognition_demo.mp4`](docs/recognition_demo.mp4), [`docs/unknown_demo.mp4`](docs/unknown_demo.mp4).

## 1. Goal

A UR5e with a Robotiq 2F-85 on simulated TSF tactile fingertips works blind (no camera) over a box holding 8 to 16 of the six loose pieces of the shape sorter (rectangle, pentagon, cross, cylinder, star, square). It should

1. find a piece by touch and grasp it,
2. say which of the six pieces it holds from the tactile maps of the grasp, or say "unknown",
3. carry a recognised piece out of the box and put an unknown one back, so that it can be picked again with a better grasp.

Everything is simulation (Isaac Lab 3.0, PhysX) with an uncalibrated tactile model. The accuracies are accuracies on simulated maps, not on a real TSF.

## 2. How it works

| Part | What it does | Where |
|---|---|---|
| Scene | 18 piece slots, 8 to 16 active per reset, random orientation drop, 36 cm box | `robotiq_rl/shape_pick/env_cfg.py`, assets in `assets/shape_sorter/` |
| Tactile model | 7x4 taxels per pad. The force of each (pad, piece) pair is spread over the taxels that touch the piece (signed-distance grids of the six pieces, 2 mm skin, 6 mm gate); walls and floor keep the Gaussian model | `robotiq_rl/shape_pick/tactile_sdf.py`, `grippers/tactile_sensors/.../spread.py` |
| Find and grasp | RL policy (PPO, rsl_rl, MLP 256-128-64, 3-step history), warm-started from the sphere-picking policy, reward shaped for centred, recognisable grasps | task `Robotiq-Tactile-Shape-Pick-UR5e-2F85` |
| Protocol (`--lift_first`) | The policy only finds and grasps. At a confirmed grasp (both pads press, gripper closed on something) the supervisor lifts straight up to a tool height of 20 cm, reads the maps, then carries a recognised piece out or puts an unknown one back | `scripts/pick_all.py` |
| Recogniser | Gradient boosting on 80 features: log(1+N) mean of the last 8 frames of the 56 map values, 22 per-pad footprint descriptors, closure, left/right force ratio. Gaussian noise (sigma 0.03) on loaded taxels in training. Probabilities are temperature-scaled on 5-fold out-of-fold predictions; "unknown" below the lowest confidence at which the cross-validated accuracy of answered pieces is at least 85 % | `robotiq_rl/shape_pick/recognizer.py`, `scripts/fit_recognizer.py` |
| Put-back | Two hardcoded joint poses (above the box centre, then 16 cm tool tip over the centre), reached at 2 rad/s per joint, then release | `PUT_BACK_ABOVE`, `PUT_BACK_RELEASE` in `pick_all.py` |

Policy and recogniser are trained one after the other, not together. The frozen recogniser's probability of the true class is part of the policy's reward (`r_recognise`, paid at pick completion), so the policy learns grasps the recogniser reads well. The recogniser is then refit on grasps recorded from the new policy.

## 3. Results so far (before the overnight run)

### Recognition with scripted, centred grasps (1979 grasps, 5-fold CV, noise added, chance 17 %)

| Tactile map | Accuracy |
|---|---|
| Gripper closure only | 26 - 35 % |
| Gaussian maps | 51 - 57 % |
| Shape-aware maps | 69 - 75 % |
| Shape-aware maps + footprint features, gradient boosting | about 80 % |

### Find and grasp

- The sphere policy, untouched, lifts 104 pieces per 128 envs x 20 s with the shape-aware map and 17 with the Gaussian map.
- After fine-tuning on the pieces (about 6000 PPO iterations): 60 pieces lifted per 64 envs x 20 s; 95 % of envs touch a piece, 81 % hold one, 66 % lift one.
- The "held" test of the sphere task (object centre within 3 cm of the tool point) rejected 94 % of the policy's grasps; widening it to 5.5 cm was what made training work.
- With the recognisability reward, about 2.3 times more lifts per 20 s than before that training (126 - 142 per 64 envs).

### Recognition from the policy's own grasps (3037 grasps)

- 62.5 % cross-validated (60 % balanced), 65.4 % live (206 of 315 lifted pieces).
- Recall per class: rectangle 0.44, pentagon 0.69, cross 0.54, cylinder 0.73, star 0.67, square 0.50. The pieces are held at the edge of the pads, tilted and off-centre, which is why this is below the scripted 80 %.
- With 282 grasps it was 46 %: the curve was still rising with data.

### "Unknown" answer (target 85 %)

- Cross-validated, the calibrated threshold 0.61 answers 52 % of the pieces with 85 % accuracy, against 63 % when it always answers (`docs/accuracy_vs_coverage.png`).
- Live (128 envs x 150 s, seed 21): 296 pieces held and read, 159 named (54 %), 135 correct (84.9 %), 137 put back. Over several runs the share named is 40 - 54 % and the accuracy of the named ones 85 - 95 %.
- Put-back at 2 - 3 rad/s: all pieces are still in the gripper when the release pose is reached (123 of 123). A second try is named about half of the time, almost always correctly.

![accuracy vs coverage](docs/accuracy_vs_coverage.png)

### Final run: 30 000 grasps

| Item | Value |
|---|---|
| Grasps recorded for the recogniser (policy of rc5, lift-first, every piece put back after the reading) | 30 132 (10 runs of 1024 envs, about 4.5 min each), plus 2 x 1024-env runs of the final policy; 8x more than before |
| Cross-validated accuracy, always answering | 74.2 % (was 62.5 % on 3037 grasps); recall per class 0.56 to 0.83 |
| Share answered at 85 % accuracy (cross-validated) | 76.2 % (was 52 %) |
| Policy after 2 more chunks (1000 iterations) with the 30k recogniser's reward, eval 64 envs x 20 s | 133 then 155 lifted pieces (126 - 142 before); 91 % of envs lift one |
| Final recogniser (30k + final-policy grasps): cross-validated | 74.4 % always answering; answers 77.3 % of pieces at 85 % accuracy (threshold 0.51) |
| Live, final policy + final recogniser (128 envs x 8000 steps, seed 99) | 664 pieces held and read; 530 named (79.8 %), 474 of them correct (**89.4 %**); 134 put back; counting unknown as wrong: 71.4 % |
| Put-back (live) | 134 of 134 still in the gripper at the release pose; 104 put back, 65 held and read again, of which 37 named correctly, 8 wrongly, 20 unknown again |

## 4. How to reproduce

Prerequisites: Isaac Lab 3.0 with its own venv (we use `~/robotiq/devel/IsaacLab`), an NVIDIA GPU (8 GB is enough; 1536 envs use about 6.6 GB), the branch above checked out, `robotiq_rl` importable, scikit-learn and joblib in the venv. Large files (`policy.pt`, `recognizer.joblib`, videos) are Git LFS.

```bash
cd grippers/demo/rl
export OMNI_KIT_ACCEPT_EULA=YES
# 0. assets (already in the repo): scripts/convert_shape_sorter.py, scripts/make_piece_sdfs.py, scripts/check_shape_sorter.py

# 1. train the find-and-grasp policy (resume from the sphere-policy checkpoint; recognisability reward on)
export TASK=Robotiq-Tactile-Shape-Pick-UR5e-2F85 EXPERIMENT=tactile_shape_pick SHAPE_CENTRE_W=4 SHAPE_RECOGNISE_W=40
# optional on a shared GPU: export ISAAC_LOCK=<path to the lock wrapper>
scripts/train_in_chunks.sh my_run 500 3 1536 <first checkpoint>

# 2. record the policy's own grasps (every piece is put back after the reading); repeat with different seeds
python scripts/pick_all.py --task $TASK --policy policy.pt --num_envs 1024 --steps 5000 --layout reachable \
    --seed 100 --lift_first --record_putback --record_grasps grasps_100.npz

# 3. fit, calibrate and choose the "unknown" threshold (85 % target)
python scripts/fit_recognizer.py grasps_1*.npz --target 0.85 --out recognizer.joblib
cp recognizer.joblib robotiq_rl/shape_pick/policy/recognizer.joblib   # the training reward reads this file

# 4. evaluate live, with unknown and put-back
python scripts/pick_all.py --task $TASK --policy policy.pt --recognizer recognizer.joblib --num_envs 128 \
    --steps 8000 --give_up 4000 --layout reachable --seed 99 --lift_first --putback_speed 2.0

# 5. video (1080p, pressure map and answer in the panel; "UNKNOWN" in orange when a piece is put back)
python scripts/pick_all.py ... --num_envs 1 --hq_video out.mp4 --hq_pressure_map
```

The final results were made by running these steps in a row: record 10 runs of 1024 envs (30 132 grasps) of the policy of `rc5`, fit, train the policy 2 chunks of 500 iterations with the new recogniser's reward (`rc6`), record 2 more runs of the final policy, refit on everything, evaluate live.

## 5. Lessons and pitfalls

- Measure the funnel (both pads press, closed, held) before tuning PPO hyper-parameters: the 3 cm held test was the real blocker.
- Raw gradient-boosting probabilities are overconfident (96 % stated, 73 % right); calibrate on out-of-fold predictions before choosing an "unknown" threshold.
- The recogniser must be refit on grasps of the current policy; its grasp distribution changes with training.
- Put-back speed matters: at 0.8 rad/s the same run read 95 pieces, against 235 at 2 and 3 rad/s.
- On one GPU, training, recording and sklearn jobs must not run together (an all-cores CPU job slowed training from 7 s to 130 - 190 s per iteration).

## 6. Limits

- Simulation only, uncalibrated tactile model; the recogniser is trained on its output.
- Step 1 lifts about 2 pieces per box in 90 s, against 5.7 spheres for the sphere policy.
- The search is done by the policy; there is no scripted coverage path of the box.
