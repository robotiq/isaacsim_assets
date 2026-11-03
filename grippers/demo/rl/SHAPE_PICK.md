# Finding and recognising shape sorter pieces by touch

A second demo on the same robot (UR5e + Robotiq 2F-85 on TSF-85 tactile fingertips, Isaac Lab 3.0, PhysX). The box now holds 4 to 8 of
the six loose pieces of the shape sorter, blind (no camera). The robot

1. **finds a piece by touch and grasps it** (the policy of the sphere demo, fine-tuned on the pieces), then
2. **recognises which of the six pieces it holds from the tactile maps of the grasp**, and drops it outside the box.

Task id `Robotiq-Tactile-Shape-Pick-UR5e-2F85`, code in `robotiq_rl/shape_pick/`, assets in `assets/shape_sorter/` (see its README).
Everything is simulation, with the uncalibrated tactile model of `grippers/tactile_sensors`: the recognition accuracy below is the
accuracy on simulated maps, not on a real TSF.

## The pieces and the scene

Rectangle, pentagon, cross, cylinder, star, square: 4 cm thick extrusions of an outline, 4 to 5.5 cm across, 29 to 43 g (wood density,
friction 0.5). Ten piece slots of fixed types (`SLOT_TYPES`); every reset puts a random subset of 4 to 8 in the box, types equally likely,
each dropped from 4.5 cm with a random orientation so it settles on whatever face it falls on (a cylinder can end up on its side). Unused
pieces wait on the ground, out of view.

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

What did not work: putting the carried piece down, releasing it and grasping it again at the same spot (`pick_all.py --regrasp`, experimental):
the second grasp holds the piece in only 40 % of the attempts, because the piece lies beside the open fingers, not between them; the
footprint of the first grasp is nearly always centred on the pad, so it cannot say where the piece is.

## Video

<video src="docs/recognition_demo.mp4" controls width="800"></video>

[`docs/recognition_demo.mp4`](docs/recognition_demo.mp4) (1080p, 60 fps, RTX render): the first 33 s of one run (seed 42, one env), with the
pressure map of the two fingertips (the maps the policy and the recogniser receive) and the recogniser's answer in the panel: it finds,
lifts and names a pentagon and a rectangle, both correct, and drops them beside the box. It is an excerpt: the next piece of that run, 100 s
later, is named a cylinder and is a square. The accuracy over many runs is the 65 % above; with one env a run only has a few pieces.
Recorded with the `pick_all.py` command below (`--steps 1000 --seed 42`, camera `--hq_eye 1.0 -0.55 0.62 --hq_look 0.5 0 0.08`).

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
