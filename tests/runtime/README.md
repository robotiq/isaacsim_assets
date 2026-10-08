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

Each test case can have a preset in `views/` (`<test>-<param>.json`, falling
back to `<test>.json`) holding the viewport camera pose, the viewport lighting
mode (the Lighting menu: Camera Light, Stage Lights, Lights Off, or a light
rig) and any light prims in the stage. With `--gui` the preset is applied
automatically when the test calls `view()`; headless runs ignore it. To create
or update one, run a single case with `--save-view`, frame the view and set
the lighting in the Isaac UI during the hold, then close the window: the
current camera, lighting mode and lights are written to that case's file.

The viewport always uses the `RTX - Minimal` render mode (`conftest.py`); the
tests are about physics, not rendering. Kit is started on its default renderer
and switched to Minimal after startup: started directly in Minimal, its
reference shading mode renders black.

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
| `test_gripper_close.py` | Each gripper (default `Physx_parallel_grip` variant) is open by default, closes to a `finger_joint` target, and the mimic-coupled right knuckle follows. |
