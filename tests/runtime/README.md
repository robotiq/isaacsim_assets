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
open after the last test until you close it:

```bash
~/isaacsim/python.sh -m pytest tests/runtime -v --gui --hold
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
| `test_gripper_close.py` | Each gripper, on PhysX and on Newton (`Physx_parallel_grip` / `Newton_parallel_grip` for the 2F-85 and 2F-140, `PhysX` / `Newton` for the Hand-E), is open by default, its driven joint closes to a target, and the coupled follower joint tracks it: `finger_joint` / right outer knuckle on the 2F, `left_finger_joint` / `right_finger_joint` on the Hand-E. The engine is switched per case with `SimulationManager.switch_physics_engine`; the Newton 2F cases run the gripper's `newton/apply_gripper_tuning.py` once the sim plays. |
| `test_gripper_parallel.py` | 2F-85 and 2F-140 on the parallel-grip variants, PhysX and Newton: through a close / open cycle the inner fingers (pads) keep their open orientation within 2°, sampled every 10 frames, and the finger joint comes back to open. |

`_scene.py` holds the shared setup (fresh stage, engine switch, play, Newton tuning).
