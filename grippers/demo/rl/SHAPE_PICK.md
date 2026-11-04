# Finding and recognising shape sorter pieces by touch

A second demo on the same robot (UR5e + Robotiq 2F-85 on TSF-85 tactile fingertips, Isaac Lab 3.0, PhysX). The box holds 8 to 16 of
the six loose pieces of the shape sorter, blind (no camera). The robot

1. **finds a piece by touch and grasps it** (the policy of the sphere demo, fine-tuned on the pieces), then
2. **recognises which of the six pieces it holds from the tactile maps of the grasp**, and drops it outside the box.

Task id `Robotiq-Tactile-Shape-Pick-UR5e-2F85`, code in `robotiq_rl/shape_pick/`, assets in `assets/shape_sorter/` (see its README).
Everything is simulation, with the uncalibrated tactile model of `grippers/tactile_sensors`: the recognition accuracy below is the
accuracy on simulated maps, not on a real TSF.

## The pieces and the scene

Rectangle, pentagon, cross, cylinder, star, square: 4 cm thick extrusions of an outline, 4 to 5.5 cm across, 29 to 43 g (wood density,
friction 0.5). Eighteen piece slots of fixed types (`SLOT_TYPES`, three of each type); every reset puts a random subset of 8 to 16 in the box (12 on average), types equally likely,
each dropped from 4.5 cm with a random orientation so it settles on whatever face it falls on (a cylinder can end up on its side). Unused
pieces wait on the ground, out of view. The pieces are spread over the whole floor of the 36 cm box (16 do not fit in the tool tip's reach),
at least 5.8 cm apart, and do not pile up (the tallest is 4.5 cm).
The policy and the recogniser below were trained with 4 to 8 pieces in the box; they were not retrained for the denser box. On 128 envs x 100 s with
8 to 16 pieces they lift and name 266 pieces (2.1 per env, against 1.2 with 4 to 8) and name 64.7 % of them correctly (65.4 % with 4 to 8).

## A tactile model that can see shapes (`robotiq_rl/shape_pick/tactile_sdf.py`)

The sensor model of the library gives, per pad and touched object, the normal force and one averaged contact point, spread as a Gaussian
blob over the 7 x 4 taxels. A blob cannot tell a flat face from a cylinder or a star. For the pieces the force of every (pad, piece) pair is
instead spread over the taxels that touch the piece: the weight of a taxel falls from 1 to 0 over the first 2 mm of its distance to the
surface, the distances coming from signed-distance grids of the six pieces (`scripts/make_piece_sdfs.py`) and the pose of the piece
(`spread_forces_by_distance` in `grippers/tactile_sensors`). The pad is modelled as a thin compliant skin: a flat face loads every taxel it
covers, a cylinder a strip along its line of contact, the tip of a star a few taxels. Walls and floor of the box keep the Gaussian model.
A piece that touches only the edge or the back of the pad (nearest taxel > 6 mm from its surface) reads nothing.

Recognition accuracy with scripted, centred grasps (1979 grasps of random pieces, 5-fold cross-validation, observation noise added; chance 17 %):

| tactile map | accuracy |
|---|---|
| gripper closure only (the piece's width) | 26 - 35 % |
| Gaussian maps | 51 - 57 % |
| shape-aware maps | 69 - 75 % |
| shape-aware maps + footprint features (centre, spread, area, entropy per pad), gradient boosting | **~80 %** |

It is also a much better input for the policy: the sphere policy, untouched, lifts 104 pieces per 128 envs x 20 s with the shape-aware map
and 17 with the Gaussian one.

## Step 1: finding and grasping a piece

The policy of the sphere demo (`robotiq_rl/tactile_pick/policy`) is fine-tuned on the pieces (same observations, shape-aware map as the tactile
input), 2048 envs, PPO, about 6000 PPO iterations (about 5 h of training, in several runs) iterations. Two things made the difference:

- **The "held" test of the sphere task rejected the policy's grasps.** It needs the object's centre within 3 cm of the tool point. A piece
  grasped anywhere along the pads is farther (median 3.5 cm), so the policy was rewarded for 6 % of its grasps and training stalled
  (25 successes per 64 envs x 20 s, below the 52 of the untrained policy). With 5.5 cm: 58. Measure the funnel (both pads press / closed /
  held, `scripts/diag_shape.py`) before touching hyper-parameters.
- A reward for holding the piece near the middle of the pads (`centre` in `SHAPE_TASK`) raised it to 60 - 64 lifted pieces per 64 envs x 20 s (58 before) and the grasps are better centred: the median distance from the tool point to the piece's centre when it is lifted went from 3.5 to 2.5 cm and the grasps are better
  centred.

Result: per 64 envs and 20 s, 60 pieces lifted out of the box; 95 % of the envs touch a piece, 81 % hold one and 66 % lift one; about 1.9 picks per 90 s episode in training (the sphere policy: 5.7).

## Step 2: recognising the piece

When the policy holds a piece above the rim for 10 steps (both pads press, tool above the rim: sensor-only, as in `pick_all.py`) the
supervisor records the last 8 steps of the maps and the closure; `robotiq_rl/shape_pick/recognizer.py` turns them into the footprint
features and a gradient boosting classifier names the piece. Then the usual scripted carry puts it down outside the box.

Recognition from the policy's own grasps is harder than from scripted centred ones: the pieces are held at the edge of the pads,
tilted, off-centre. Trained on 3037 of the policy's grasps: 62.5 % (balanced over the classes 60 %) cross-validated, 65.4 % (206 of 315 lifted pieces) live on fresh boxes (see the
confusion matrix below). With 282 grasps it was 46 %, and the curve was still rising with data.

Confusion matrix of the cross-validation (rows: the piece, columns: the answer):

```
           rectan pentag  cross cylind   star square
rectangle     163     75     59     33     35      7   recall 0.44
pentagon       56    477     78     54     15      7   recall 0.69
cross          42    106    327     83     36      8   recall 0.54
cylinder       26     77     63    498      8     10   recall 0.73
star           30     47     71      8    336     10   recall 0.67
square         11     20     34     13     18     96   recall 0.50
```

## Answering "unknown"

The recogniser is wrong a third of the time, so it can decline. Its raw probabilities are overconfident (it says 96 % and is right 73 % of the
time), so they are temperature-scaled on cross-validated predictions (`recognizer.fit_model`); below the confidence at which the
cross-validated accuracy of the answered pieces drops under 85 % the answer is "unknown" (threshold 0.61 on the calibrated probability).
Cross-validated that answers 52 % of the pieces with 85 % accuracy, against 63 % accuracy when it always answers
([`docs/accuracy_vs_coverage.png`](docs/accuracy_vs_coverage.png): accuracy against coverage, and the calibration, measured before the calibration).

When the answer is unknown the supervisor does not carry the piece out: the arm goes, in joint space, to a fixed pose above the centre of the
box (tool tip at 28 cm, so that the piece clears the rim) and then to a second fixed pose (tool tip at 16 cm over the box centre: the piece
about 12 cm above the floor) and releases it there, so that the policy can pick it up again, with another grasp. The two poses are hardcoded joint
angles (`PUT_BACK_ABOVE`, `PUT_BACK_RELEASE` in `pick_all.py`, from `start_poses.npz`), reached at `--putback_speed` 2 rad/s per joint
(at 0.8 rad/s the same run read 95 pieces, against 235 at 2 and 3 rad/s). `scripts/show_putback_pose.py` renders the arm at the two poses.
Live, on fresh boxes (128 envs x 150 s, 8 to 16 pieces, seed 21, with the earlier Cartesian put-back): 296 pieces held and read, 159 named (54 %), 135 of
them correctly (**84.9 %**), 137 put back. Over several runs the share named is 40 - 54 % and the accuracy of the named ones 85 - 95 %.
Put-back at 2 - 3 rad/s (64 envs x 83 s, three runs): all the pieces are still in the gripper when the release pose is reached (123 of 123), 82 - 88 are put
back and 45 - 52 of them are held again before the run ends (the policy often picks the piece it just released); of those, 18 - 27 are named
correctly, 1 - 2 wrongly and 24 - 30 are unknown again: a second try is named about half of the time, almost always correctly. Unknown pieces
do not count as picks; the statistics are printed by `pick_all.py` (`--recognizer`).

<video src="docs/unknown_demo.mp4" controls width="800"></video>

[`docs/unknown_demo.mp4`](docs/unknown_demo.mp4): the first 30 s of one run (seed 36, one env): a square named at 97 % (correct), then a pentagon whose best guess
would have been a cylinder at 39 %: "unknown", put back in the box.

What did not work: putting the carried piece down, releasing it and grasping it again at the same spot (`pick_all.py --regrasp`, experimental):
the second grasp holds the piece in only 40 % of the attempts, because the piece lies beside the open fingers, not between them; the
footprint of the first grasp is nearly always centred on the pad, so it cannot say where the piece is.

## Final results: 30 000 grasps, lift-first protocol

The numbers above were measured on 3037 grasps of an earlier policy. The final recogniser is trained on 30 132 grasps of the policy
(`pick_all.py --lift_first --record_putback --record_grasps`: the supervisor lifts straight up, reads the maps, and puts every piece back so that the
next grasp can be recorded; 10 runs of 1024 envs, about 4.5 min each), plus two runs of the final policy; the policy was trained 1000 more iterations
(2 chunks) with the reward of that recogniser.

| | 3037 grasps (earlier) | 30 000 grasps (final) |
|---|---|---|
| cross-validated accuracy, always answering | 62.5 % | 74.2 % (74.4 % with the final policy's grasps added) |
| share of pieces answered at 85 % accuracy (cross-validated) | 52 % | 76 - 77 % (threshold 0.51 on the calibrated probability) |
| recall per class (cross-validated) | 0.44 - 0.73 | 0.56 (rectangle) - 0.83 (cylinder) |
| policy: pieces lifted per 64 envs x 20 s | 126 - 142 | 133, then 155 (91 % of the envs lift one) |
| live, 128 envs x 8000 steps, 8 - 16 pieces | 54 % named, 84.9 % of those correct | 664 pieces read, **79.8 % named, 89.4 % of those correct** (474 of 530); 71.4 % counting unknown as wrong |

Live put-back: 134 of 134 pieces are still in the gripper at the release pose; 104 are put back, 65 of them held and read again (37 named correctly, 8 wrongly, 20 unknown again).
Recall when answered: rectangle 0.74, pentagon 0.90, cross 0.75, cylinder 0.90, star 0.95, square 0.88; unknown for 31, 28, 37, 22, 7 and 26 % of them.

The policy and the recogniser are trained one after the other: the frozen recogniser's probability of the true class is part of the policy's reward, and the
recogniser is refit on the new policy's grasps. Data was the limit: 3037 -> 30 132 grasps gave +12 points of accuracy and +24 points of coverage at 85 %.

Twice the pieces (16 to 32 in the box, `SHAPE_SLOTS=36 SHAPE_ACTIVE=16,32 SHAPE_MIN_GAP=0.04`, pieces may touch and stack), same policy and recogniser, one env,
150 s: [`docs/double_pieces_demo.mp4`](docs/double_pieces_demo.mp4) (540p; the render is 1080p, 60 fps). Pieces recognised in green, UNKNOWN in orange when a piece is put back.

<video src="docs/double_pieces_demo.mp4" controls width="800"></video>

## Video

<video src="docs/recognition_demo.mp4" controls width="800"></video>

[`docs/recognition_demo.mp4`](docs/recognition_demo.mp4) (1080p, 60 fps, RTX render): the first 23 s of one run (seed 3, one env, 12 pieces in the box), with the
pressure map of the two fingertips (the maps the policy and the recogniser receive) and the recogniser's answer in the panel: it finds,
lifts and names a cylinder and a star, both correct, and drops them beside the box. It is an excerpt, picked among 6 seeds for having two
correct answers; over many runs the recogniser is right 65 % of the time (above).
Recorded with the `pick_all.py` command below (`--steps 700 --seed 3`, camera `--hq_eye 1.0 -0.55 0.62 --hq_look 0.5 0 0.08`).

## Run it

```bash
# the policy and the recogniser are in robotiq_rl/shape_pick/policy/
python scripts/pick_all.py --task Robotiq-Tactile-Shape-Pick-UR5e-2F85 --policy robotiq_rl/shape_pick/policy/policy.pt \
    --recognizer robotiq_rl/shape_pick/policy/recognizer.joblib --num_envs 1 --steps 4500 --layout reachable \
    --hq_video out.mp4 --hq_pressure_map            # 1080p, 60 fps, the pressure map and the recognised piece on the video
# data and training
python scripts/collect_grasps.py --out grasps.npz                       # scripted grasps (recognition feasibility, scripts/train_recognizer.py)
python scripts/pick_all.py ... --record_grasps grasps.npz               # the policy's own grasps
python scripts/fit_recognizer.py grasps.npz --out recognizer.joblib
TASK=Robotiq-Tactile-Shape-Pick-UR5e-2F85 EXPERIMENT=tactile_shape_pick scripts/train_in_chunks.sh my_run 600 8 2048 <checkpoint>
```

## Limits

- Simulation only; the tactile model is uncalibrated and the recogniser is trained on its output.
- Step 1 lifts about 2 pieces per box in 90 s, much less than the sphere policy (5.7 spheres): the pieces do not roll into the pads.
- The carry-and-drop routine is the scripted one of the sphere demo, with the limits listed there.
