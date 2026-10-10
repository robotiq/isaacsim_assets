# Runtime tests (manual, Isaac Sim)

Tests that load the gripper assets into a running Isaac Sim and simulate them.
They need an NVIDIA RTX GPU and a local Isaac Sim install, so they are **not**
part of the GitHub CI (which only runs the static SimReady validation). Run them
by hand before merging an asset change.

```bash
~/isaacsim/python.sh -m pip install pytest      # once: Isaac's Python ships without pytest
~/isaacsim/python.sh -m pytest tests/runtime -v  # headless
```

To watch the simulation, open the Isaac Sim window instead; `--hold` keeps it
open after the last test until you close it (press **R** in the window to
replay the last test in the same session), and `--slow FACTOR` plays FACTOR
times slower than real time (the physics is unchanged, each frame is just
held on screen longer):

## Reusing one Isaac session

Isaac takes a while to start, so a held session also accepts test runs from
another terminal. Start it once (the `-k` just picks what runs at startup;
`-k nothing_yet` runs nothing), then send it selections with
`isaac_run.py`, which prints the tests' output:

```bash
~/isaacsim/python.sh -m pytest tests/runtime --gui --hold -k nothing_yet   # terminal 1, once
tests/runtime/isaac_run.py "2F_85 and Newton_compliant"                     # terminal 2, repeatedly
tests/runtime/isaac_run.py "parallel and 2F_140" --slow 5
```

`--joints on|off` draws or hides the physics joints in the viewport for the
session (`--show-joints` does the same at startup). The selection is a
simplified `-k`: words joined by ` and ` must all appear in the test id,
`not <word>` excludes. The gripper USD layers are re-read
from disk before every run, so **asset edits are picked up without a
restart**; edits to the test code still need one (the collected tests are
the ones loaded at startup). The session listens on `127.0.0.1:8777`
(`--port` to change).

```bash
~/isaacsim/python.sh -m pytest tests/runtime -v --gui --hold
~/isaacsim/python.sh -m pytest tests/runtime -v --gui --hold --slow 5 -k "2F_85 and Newton_compliant"
```

Adjust `~/isaacsim` to your Isaac Sim install. One headless `SimulationApp` is
started for the whole session (`conftest.py`); each test builds its own stage.

## Saved camera and lights for GUI runs

A preset in `views/` holds the viewport camera pose, the viewport lighting
mode (the Lighting menu: Camera Light, Stage Lights, Lights Off, or a light
rig) and any light prims in the stage. The most specific file wins:

| Scope | File | Applies to |
|---|---|---|
| `case` | `<test>-<params>.json` | one parametrized case |
| `test` | `<test>.json` | every case of that test function |
| `gripper` | `<gripper folder>.json`, e.g. `Robotiq_2F_140.json` | every test of that gripper |
| `default` | `default.json` | everything else |

With `--gui` the preset is applied automatically when the test calls
`view()`; headless runs ignore it. To create or update one, run a single case
with `--save-view [SCOPE]`, frame the view and set the lighting in the Isaac UI
during the hold, then close the window: the current camera, lighting mode and
lights are written to the file of that scope (`case` when omitted).

```bash
~/isaacsim/python.sh -m pytest tests/runtime -v --gui --hold --save-view gripper -k 2F_140
```

The viewport always uses the `RTX - Minimal` render mode (`conftest.py`) in
Kit's default Minimal shading mode (Textured Diffuse, the look the viewport
menu's "RTX - Minimal" gives in the interactive app); the tests are about
physics, not rendering. `SimulationApp` would otherwise force the "Real-Time
2.0 reference" shading mode, which renders a black viewport.

`--screenshot DIR` captures the viewport to `DIR/<test>.png` after each test,
headless or not, with the saved view applied; `--no-view` skips the presets.
Handy for checking a view without opening the window:

```bash
~/isaacsim/python.sh -m pytest tests/runtime -k 2F_85 --screenshot /tmp/shots
```

```bash
~/isaacsim/python.sh -m pytest tests/runtime -v --gui --hold --save-view -k 2F_140
```

Commit the JSON so the view travels with the test.

| Test | What it checks |
|---|---|
| `test_gripper_close.py` | Each gripper on every Physics variant (2F-85 and 2F-140: `Physx_parallel_grip`, `Physx_compliant`, `Newton_parallel_grip`, `Newton_compliant`; Hand-E: `PhysX`, `Newton`) is open by default, its driven joint closes to a target at the datasheet finger speed (150 mm/s, ramped target), and the coupled follower joint tracks it: `finger_joint` / right outer knuckle on the 2F, `left_finger_joint` / `right_finger_joint` on the Hand-E. The engine is switched per case with `SimulationManager.switch_physics_engine`; the Newton 2F cases run the gripper's `newton/apply_gripper_tuning.py` once the sim plays. |
| `test_gripper_parallel.py` | 2F-85 and 2F-140 on all four Physics variants: through a close / open cycle at the datasheet finger speed (no object) the inner fingers (pads) keep their open orientation, sampled every 5 frames: within 2° at rest (closed and reopened) for every variant, and while moving within 2° on the parallel-grip variants and 3° on the compliant ones (the springs deflect under the fingers' inertia). The finger joint must come back to open. |
| `test_gripper_max_speed.py` | 2F-85 and 2F-140, `parallel_grip` and `compliant`, PhysX and Newton: a full close commanded in one step peaks at the 86 deg/s finger joint velocity cap (datasheet 150 mm/s) (within +5 % / -10 %). The Newton cases are `xfail` (strict): the MuJoCo solver ignores joint velocity limits, so Newton runs at ~255 deg/s. |

`_scene.py` holds the shared setup (fresh stage, engine switch, play, Newton tuning) and `move_to`, which ramps a joint target at the gripper's datasheet finger speed (`FINGER_RATE`) instead of stepping it; `test_gripper_max_speed.py` deliberately steps, to saturate the velocity cap.
