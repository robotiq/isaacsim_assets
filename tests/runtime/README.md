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

With `--gui` the name of the running test (e.g. `test_gripper_pusher[Robotiq_2F_85-Newton_compliant]`) is shown at the top of the viewport. It is a UI overlay, so `--screenshot` and `--record` captures do not include it.

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
| `test_gripper_grasp.py` | Every gripper and Physics variant closes at the datasheet speed on a free 1 kg cube resting on a static holder between the pads (`cubes/<gripper>.json`), toward full close so the cube stops the fingers. Checks: the driven joint stalls short of its target (position drift ≤ 0.1° / 0.05 mm) and the follower tracks it; both fingertips touch the cube (penetration ≤ 1 mm) and each pushes with ≥ 1 N, balanced within 20 % (40 % on the compliant variants, whose soft grip leaves the cube on its holder); no other link touches the cube; the pads stay within 2° of their open orientation; after reopening nothing touches the cube. PhysX runs this test at 1000 Hz (at the default 60 Hz the squeeze is not resolved), and its holder has a low-friction material (`HOLDER_FRICTION`) so it does not take over the pads' left / right difference. Contact forces: PhysX contact report, Newton `efc.force` (contacts within 1 mm only). |
| `test_gripper_fixed_cube.py` | 2F-85 and 2F-140, PhysX and Newton, in two tests: `parallel_grip`, and `compliant` with its own box placement (`fixed_cubes/<gripper>_compliant.json`). Each closes at the datasheet speed on a cube fixed in space (a static box collider, no holder, `fixed_cubes/<gripper>.json`), toward full close so the cube stops the fingers. Checks: `finger_joint` and `right_outer_knuckle_joint` stall within 1° of the baseline recorded in the preset for that variant, at rest (drift ≤ 0.1°); the pads stay within 2° of their open orientation. PhysX runs at 1000 Hz. |
| `test_gripper_fixed_cylinder.py` | 2F-85 and 2F-140 on all four Physics variants: closes at the datasheet speed on a cylinder fixed in space (a static collider, no holder, axis along the gripper's X for every gripper, one placement per gripper in `fixed_cylinders/<gripper>.json`), toward full close so the cylinder stops the fingers. PhysX and Newton share one baseline per grip (the mean of the two engines' recordings). Checks: `finger_joint` and `right_outer_knuckle_joint` within 1° of it (2° on compliant, looser on the 2F-85's: `GRIPPER_TOL`), at rest (drift ≤ 0.1°, looser for the 2F-85 Newton_compliant, which is still ringing down: `CASE_REST_DRIFT`); each fingertip's rotation from its open orientation within 1° (1.5° on compliant, looser on the 2F-85's) of it (about zero on parallel grip; the compliant pads tilt). PhysX runs at 1000 Hz. |
| `test_gripper_pusher.py` | 2F-85 and 2F-140 on all four Physics variants, gripper held open: a flat plate on a prismatic joint (its own articulation; `pushers/<gripper>.json`) starts beyond the fingertips and is driven 50 mm/s toward the base by its displacement, pushing on the fingers with at most 50 N. Checks: everything is at rest; parallel grip stays rigid (the fingers stop the plate short of its end, pads rotate < 1°, knuckle joints within 1° of open); compliant bends (the plate reaches its end within 1 mm, pads rotate > 2°) and matches the baseline PhysX and Newton share within 2° (joints) / 1.5° (pads). PhysX runs at 1000 Hz. |
| `test_gripper_lift.py` | Every gripper and Physics variant, hung upside down (fingers pointing down) from a vertical prismatic joint (`_lift.py`): closes on the grasp test's cube (same `cubes/<gripper>.json` size and gripper-relative position) resting on its small static plate, then lifts it with a constant acceleration up to a peak speed, a cruise, the same deceleration to a stop and a hold (the `LIFT_*` constants in the test). Checks: the gripper follows the commanded lift and peaks near the commanded speed; it travels the commanded distance; the cube never moves more than `SLIP_TOL` / `SLIP_ROT_TOL` in the gripper's base frame (no slip) and rises with it; both fingertips keep touching it and no other link does. PhysX runs at the grasp test's rate (`PHYSX_HZ`). |
| `test_gripper_ros2.py` | 2F-85 and 2F-140 on all four Physics variants: builds the ROS 2 graph of `grippers/ros2/build_gripper_graph.py` and, from a separate ROS 2 process, sends the `JointState` that `topic_based_ros2_control` (`robotiq/ros`, `sim_topic_based:=true`) sends to a simulator (the mimic joints named, a position only for the driven one). Checks: the driven joint reaches the commanded position and the state topic reports that one joint, under its URDF name, with a velocity and the simulation time as stamp. Needs a system ROS 2 under `/opt/ros` (the probe uses `$ROS_DISTRO`'s and gets the ROS domain / RMW variables of the test's environment) and Isaac started with its ROS environment: `source ~/isaacsim/setup_ros_env.sh` first, with `PYTHONPATH` unset if ROS is sourced in that shell (otherwise ROS's `launch_testing` pytest plugin loads into Isaac's Python and fails on `lark`): `env -u PYTHONPATH ~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_ros2.py`. Without that environment Isaac's ROS 2 bridge does not load (`ROS2 Bridge startup failed` in the log) and setting a ROS2Subscriber's message type segfaults Isaac natively; the builder detects this (`Ros2BridgeUnavailable`) and the test skips with the instruction instead of crashing the session. |

`_scene.py` holds the shared setup (fresh stage, engine switch, play, Newton tuning) and `move_to`, which ramps a joint target at the gripper's datasheet finger speed (`FINGER_RATE`) instead of stepping it; `test_gripper_max_speed.py` deliberately steps, to saturate the velocity cap.

## Placing the grasp cube

Each gripper's preset (`cubes/<gripper>.json`) holds the cube's edge length and centre
and the holder's size and X / Y position; the holder's top is always placed at the
cube's bottom. Run one case of that gripper with `--save-cube`, move / scale
`/World/Cube` and `/World/CubeHolder` in the Isaac UI during the hold, press **R** to
replay with the new layout (it is saved first), and close the window to keep it:

```bash
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_grasp.py -v --gui --hold --save-cube -k "2F_85 and Physx_parallel"
```

A non-uniform cube scale is averaged into its edge length and rotations are dropped.
The cube's size is baked into `UsdGeom.Cube.size` (Newton ignores scale on primitive
colliders); the holder is a scaled box mesh, whose scale Newton does apply.

## Placing the fixed cube and recording its baselines

`test_gripper_fixed_cube.py` has its own presets, `fixed_cubes/<gripper>.json` (and `<gripper>_compliant.json` for the compliant test): the
box's size per axis and centre (no holder, stretch it freely with the scale gizmo), plus the recorded joint baselines under
`"expected"`. Place the cube the same way (`--save-cube` saves to `fixed_cubes/` for this
test); moving or resizing it drops the baselines. Then record them for every case:

```bash
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_fixed_cube.py -v --gui --hold --save-cube -k "2F_85 and Physx"
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_fixed_cube.py -v --record-baseline
```

The fixed-cylinder test works the same way with `fixed_cylinders/<gripper>.json`
(`/World/Cylinder`: move and stretch it, its orientation is fixed; one placement per
gripper, shared by its four variants). Its baselines also hold each fingertip's rotation:

```bash
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_fixed_cylinder.py -v --gui --hold --save-cube -k "2F_85 and Physx_parallel"
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_fixed_cylinder.py -v --record-baseline
```

## Placing the pusher plate

`test_gripper_pusher.py` reads `pushers/<gripper>.json`: the plate's size and start
position, and its displacement toward the gripper base. Shape and move
`/World/PusherRig/Pusher` to its start, and move the see-through `/World/PusherEnd` along
Z to where the push should end; `--save-cube` saves both (one placement per gripper,
shared by its four variants). Then record the compliant baselines:

```bash
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_pusher.py -v --gui --hold --save-cube -k "2F_85 and Physx_parallel"
~/isaacsim/python.sh -m pytest tests/runtime/test_gripper_pusher.py -v --record-baseline
```

## Recording videos

`--record DIR` (with `--gui`) saves the viewport every other frame of each test to
`DIR/<test id>/` (30 fps). `make_videos.py` (OpenCV, bundled with Isaac Sim's Python)
turns them into VP9 `.webm` videos: one per case (`cases/`), the Physics variants of each
test and gripper side by side (`grids/`, 2x2 for the 2F, 1x2 for the Hand-E) and all the
grids one after the other (`all_tests.webm`):

```bash
~/isaacsim/python.sh -m pytest tests/runtime --gui --record /tmp/frames
~/isaacsim/python.sh tests/runtime/make_videos.py /tmp/frames /tmp/videos
```
