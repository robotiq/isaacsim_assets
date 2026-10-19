# Teleoperation demo — UR5e + Robotiq grippers in Isaac Sim

Drive a UR5e fitted with a Robotiq 2F-85, 2F-140 or Hand-E with a
**DualSense** gamepad, the **keyboard** or the **mouse**. Everything runs inside
Isaac Sim: no ROS, no extra process. `physx_tick_teleop.py` reads the input
and moves the arm on Isaac Sim's own update tick.

For gripper physics tuning, see [PHYSICS_TUNING.md](PHYSICS_TUNING.md).

## Quick start

From a shell:

```bash
./run_demo.sh
TELEOP_GRIPPER=2F140 TELEOP_PHYSICS=Newton_compliant ./run_demo.sh
```

`run_demo.sh` picks the right Isaac Sim launcher for the physics, and with a
DualSense also starts the haptics (see [Haptics](#haptics-dualsense)).
`./run_demo.sh --help` lists its options.

<details>
<summary>Windows</summary>

`run_demo.sh` is Linux only. On Windows, set the variables and start Isaac Sim
with `open_scene.py` yourself, from PowerShell:

```powershell
$env:TELEOP_GRIPPER = "2F85"              # 2F85, 2F140 or HandE
$env:TELEOP_PHYSICS = "Physx_compliant"   # see Gripper and Physics variants
C:\isaacsim\isaac-sim.bat --exec "C:\path\to\grippers\demo\teleop\open_scene.py"
```

For a `Newton_*` physics, use `isaac-sim.newton.bat` instead of
`isaac-sim.bat`. The other [settings](#settings) are set the same way, with
`$env:NAME = "value"`, except `TELEOP_HAPTICS`, `ISAACSIM_ROOT` and
`ISAACSIM_LAUNCHER`, which only `run_demo.sh` reads.

On Windows the teleop uses the keyboard and the mouse: the DualSense is read
from Linux's `/dev/input/js0`, so the gamepad and the haptics aren't available.

</details>

### What happens

`open_scene.py` opens the scene, selects the gripper and physics, presses
Play and starts the teleop. Wait for **`[physx-autostart] ready.`** in the
console; Isaac Sim's first start takes 1–3 minutes. Then click in the
viewport and drive (see [Controls](#controls)).

### Settings

| Variable | Values (default first) | Effect |
|---|---|---|
| `TELEOP_GRIPPER` | `2F85`, `2F140`, `HandE` | Gripper variant. |
| `TELEOP_PHYSICS` | `Physx_compliant`, `Physx_parallel_grip`, `Newton_compliant`, `Newton_parallel_grip` | Physics variant. `Newton_*` needs Isaac Sim's Newton launcher. The Hand-E has one PhysX and one Newton variant. |
| `TELEOP_RENDER` | `RealTimePathTracing`, `keep` | Renderer; `keep` leaves the scene's own. |
| `TELEOP_KEEP_VIEW` | `0`, `1` | `1` keeps the camera pose saved in the scene. |
| `TELEOP_INPUT` | `auto`, `keyboard` | `auto` uses a DualSense when one is plugged in, else the keyboard. `keyboard` always uses the keyboard (same as `./run_demo.sh --keyboard`). |
| `TELEOP_HAPTICS` | `1`, `0` | `0` drops the contact-force plot and R2 rumble. |
| `TELEOP_FULL_FORCE_N` | `200` | Average pad force (N) at which R2 reaches full stiffness. |
| `READY_TIMEOUT` | `240` | Seconds `run_demo.sh` waits for the scene, counted from the launch. |
| `MCP_EXT_ROOT` | `../mcp_bridge/isaacsim-mcp-server` | Where the Isaac Sim MCP extension is (needed for the haptics). |
| `ISAACSIM_ROOT` / `ISAACSIM_LAUNCHER` | `~/isaacsim` / derived from physics | Isaac Sim install and launcher override. |

## Controls

The speed controls set how fast the **robot** moves and rotates; the gripper
always opens and closes at the same rate. Robot speed starts at its maximum
(250 mm/s and 57°/s); each faster/slower press
changes it by 1.25×, ten presses down to 27 mm/s for fine work. Moves are
**relative to the view**: away from you on screen is away from you, whatever
the camera does. Up and down are always world vertical. Rotations pivot about
the tool tip.

### Keyboard

| Key | Action |
|---|---|
| W / S | move the tool away from / toward you |
| A / D | move the tool left / right |
| Q / E | tool up / down |
| Arrow keys | roll / pitch |
| Z / C | yaw − / yaw + |
| Space (hold) | close the gripper |
| R / F | robot faster / slower |
| T | point the tool down |
| Backspace | restart the simulation (only while the viewport has the focus, or in mouse mode) |
| M | mouse mode on / off (see [Mouse](#mouse)) |

Isaac Sim's own shortcuts keep working; the teleop doesn't block keys.

The keyboard is used when no gamepad is found. To use it with a DualSense
plugged in, run `./run_demo.sh --keyboard` (or set `TELEOP_INPUT=keyboard`).

### Mouse

With the keyboard, press **M** while the viewport has the focus to switch
mouse mode on. The pointer is hidden and locked to the window, and Isaac Sim's
window stops using the mouse: no camera zoom or orbit, no selection, no menus
or panels. Press **M** again to get the pointer and the window back. Mouse mode
also switches off when Isaac Sim loses the focus, for example on Alt-Tab.

| Mouse | Action |
|---|---|
| Move | move the tool |
| Scroll | tool up / down |
| Left button (hold) | close the gripper |
| Right button held + move | roll / pitch |
| Right button held + scroll | yaw |

Sensitivity is set by `MOUSE_FULL_PX` and `SCROLL_TICKS_PER_NOTCH` at the top of
`physx_tick_teleop.py`.

### DualSense

The DualSense is read directly from `/dev/input/js0`. Press
**Options** for the diagram below, drawn on the controller:

![controller map](controls.png)

| Control | Action |
|---|---|
| right stick | move the tool |
| d-pad up / down | tool up / down |
| d-pad left / right | zoom in / out on the tool tip |
| left stick | roll / pitch |
| `L1` / `R1` | yaw − / yaw + |
| `R2` | close the gripper (analog) |
| Triangle / Square | robot faster / slower |
| Cross | point the tool down |
| Create | restart the simulation |
| Options | show the controls |

Button numbering is the standard hid-playstation mapping. On a pad that
numbers them differently, Options also prints the indices currently held,
which is enough to re-map the constants at the top of `physx_tick_teleop.py`.

Controller art: **PS5 Button Icons and Controls** by Zacksly, licensed
[CC BY 3.0](http://creativecommons.org/licenses/by/3.0/) —
<https://zacksly.itch.io>. Vendored unmodified under `third_party/ps5_icons/`;
regenerate the diagram with `python3 make_controls_diagram.py` after changing
a binding.

### Restarting

**Backspace**, the DualSense's **Create** button, or **Stop → Play** in Isaac Sim's toolbar
resets the robot and objects to their starting state. The teleop re-attaches
by itself; the arm can't be driven for about a second.

## Gripper and Physics variants

The scene carries two independent variant sets on `/World` (Stage panel →
select `/World` → *Variants*), so one file can test any gripper on any
physics. `TELEOP_GRIPPER` and `TELEOP_PHYSICS` select them at start.

| `Gripper` | Gripper asset |
| --------- | ------------- |
| `2F85` *(default)* | `Robotiq_2F_85/Robotiq_2F_85.usda` |
| `2F140` | `Robotiq_2F_140/Robotiq_2F_140.usda` |
| `HandE` | `Robotiq_Hand_E/Robotiq_Hand_E.usda` |

The Hand-E has one PhysX and one Newton variant, so the scene's `Physx_*`
physics select its `PhysX` variant and `Newton_*` its `Newton` variant. The
grip drives its `left_finger_joint` over the 25 mm stroke (the right finger
follows), instead of `finger_joint` over 0.78 rad.

| `Physics` | Launch with | After every Stop → Play |
| --------- | ----------- | ----------------------- |
| `Physx_compliant` *(default)* | normal Isaac Sim (`isaac-sim.sh`) | nothing |
| `Physx_parallel_grip` | normal Isaac Sim | nothing |
| `Newton_compliant` | **Newton experience** (`isaac-sim.newton.sh`) | run the gripper's `newton/apply_gripper_tuning.py` |
| `Newton_parallel_grip` | **Newton experience** | run the gripper's `newton/apply_gripper_tuning.py` |

`Gripper` payloads the gripper's unified asset under `wrist_3_link`, deletes
its own `PhysicsArticulationRootAPI` so it merges into the arm articulation,
and welds its `base_link` to `wrist_3_link` with a `FixedJoint`. `Physics`
selects the asset's own `Physics` variant of the same name and adds the
engine's tuning: arm-joint drive damping (plus `mjc:armature` on Newton), and
on PhysX the finger solver iterations and `finger_joint` drive (authored once
per gripper in the `_TeleopPhysxGripperTuning_*` classes, which the PhysX
variants inherit). The scene's `/PhysicsScene` carries `MjcSceneAPI` (inert
under PhysX), so no scene edit is needed when switching. The scene also sublayers
[`grippers/scene/Robotiq_Newton_scene_physics.usda`](../../scene/Robotiq_Newton_scene_physics.usda),
which sets the elliptic friction cone Newton needs on Isaac Sim 6.1+.

`Physics` is listed **before** `Gripper` on purpose: the asset's `Physics`
variant set comes in through the payload inside the `Gripper` variant, and USD
only honours a variant selection authored at least as strongly as that — so
the scene's `Physics` variant must be the stronger arc.

Switch variants by relaunching, not live in a running scene: grippers and
grips have different DOF counts (e.g. compliant 8 vs parallel 6), and the
teleop crashes (CUDA device-side assert) if the articulation changes under it.

**The Newton runtime step is not optional** for the 2F grippers. Two
Newton/MuJoCo settings can't be saved in USD (`opt.impratio` and the finger
loop-closure equality `solref`); run the gripper's
`newton/apply_gripper_tuning.py` from **Window → Script Editor** after each
Stop → Play, or the fingers go floppy. The Hand-E needs no tuning. See
[the Gripper Simulation Guide](../../GRIPPER_SIMULATION_GUIDE.md) for the
details.

## Haptics (DualSense)

With a DualSense, `run_demo.sh` also starts the haptic loop: a
contact-force plot of the two finger pads in the viewport, and resistance on
the `R2` trigger proportional to the grip force. It needs:

- `pip install --user -r requirements.txt` (pydualsense, hidapi) and
  `sudo apt install libhidapi-hidraw0`;
- read/write access to the DualSense hidraw node (a `uaccess` udev rule);
- the Isaac Sim MCP extension, which the plot is injected through: clone it to
  `../mcp_bridge/isaacsim-mcp-server` or set `MCP_EXT_ROOT` (see
  [mcp_bridge/README.md](../mcp_bridge/README.md)).

Without them the teleop still runs; only the plot and the rumble are missing.
`TELEOP_HAPTICS=0` turns them off.

## Files

| File | Role |
|---|---|
| `ur5robot_with_2F-85.usda` | The scene: UR5e, gripper and physics variants, props. Shipped as USD ASCII so tuning changes diff in reviews. |
| `open_scene.py` | Run by Isaac Sim at start (`--exec`): opens the scene, selects variants, presses Play, starts the teleop. |
| `physx_tick_teleop.py` | The teleop: input (DualSense, keyboard, mouse), inverse kinematics, joint targets. |
| `run_demo.sh` | Launcher: Isaac Sim plus the haptics. |
| `trigger_force_feedback.py` | DualSense `R2` resistance from the pad forces. |
| `add_props.py` | Adds the graspable props to the scene. |
| `make_controls_diagram.py` | Regenerates `controls.png`. |
| [PHYSICS_TUNING.md](PHYSICS_TUNING.md) | The gripper and object physics values and why. |

This demo doesn't use ROS.
