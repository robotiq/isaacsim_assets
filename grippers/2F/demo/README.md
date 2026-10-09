# Isaac Sim teleop stack (UR5e + Robotiq 2F-85)

Everything needed to reproduce the manual-teleop setup we use for testing:
a UR5e + Robotiq 2F-85 gripper in Isaac Sim 6, driven from ROS 2 (Humble or Jazzy) via
MoveIt Servo, with keyboard and PS4/PS5 gamepad frontends.

If you just want to grasp things in the sim and drive them around with a
controller, follow the **Quick start** below. If something breaks or you
want the full picture, read **[teleop/README.md](teleop/README.md)**.

## What's in this folder

```
grippers/demo/
├── README.md                 ← this file (setup + quick start)
├── isaac_scripts/            ← general Isaac-side tools (not teleop-specific)
│   └── build_ros_action_graph.py    ← builds the ROS bridge OmniGraph in an open stage
├── teleop/                   ← the manual teleop workflow
│   ├── README.md                    ← detailed doc, data flow, gotchas
│   ├── PHYSICS_TUNING.md            ← USD changes on 2F-85 + cube for stable gripping
│   ├── ur5robot_with_2F-85.usda      ← tuned UR5e + 2F-85 scene, ASCII, text-diffable
│   ├── teleop.launch.py             ← ONE-SHOT: brings up servo + gamepad + haptics together
│   ├── ur5e_servo.launch.py         ← MoveIt Servo + bridge + filter + robot_state_publisher
│   ├── _servo_compat.py             ← Humble/Jazzy differences, resolved from what's installed
│   ├── Dockerfile                   ← OPTIONAL containerised ROS side (either distro)
│   ├── run.sh                       ← docker run wrapper (host networking, gamepad passthrough)
│   ├── entrypoint.sh                ← sources the image's ROS distro
│   ├── smoke_launch_check.py        ← CI check: launch description vs installed packages
│   ├── set_servo_command_type.py    ← manual command-type switch (debugging aid)
│   ├── gamepad.launch.py            ← joy_node + gamepad mapper
│   ├── haptics.launch.py            ← contact-force plot + DualSense R2 force feedback
│   ├── trigger_force_feedback.py    ← gripper pad force → R2 adaptive-trigger resistance
│   ├── requirements.txt             ← host python deps for the haptic node (pydualsense, hidapi)
│   ├── servo_to_isaac_bridge.py     ← Float64MultiArray → JointState on /joint_command
│   ├── joint_states_arm_filter.py   ← strip the 6 gripper joints so Servo only sees the arm
│   ├── sim_reset_watchdog.py        ← reseeds Servo after an Isaac Stop→Play (/clock jump-back)
│   ├── gamepad_teleop.py            ← DualSense (SDL2 layout) → TwistStamped mapping
│   ├── servo_keyboard_teleop.py     ← keyboard variant (raw tty, WASD + arrows)
│   ├── keyboard_jog.py              ← servo-less fallback (direct /joint_command)
│   ├── send_joint_command.py        ← one-shot test publisher / "go to pose" helper
│   └── keydump.py                   ← diagnostic (shows raw terminal keycodes)
├── mcp_bridge/               ← OPTIONAL — Claude Code (or other MCP client) live control
│   ├── README.md                    ← setup guide
│   ├── launch_isaac_with_mcp.sh     ← Isaac Sim launcher with the MCP extension wired in
│   └── mcp.json.example             ← template for your MCP client's config file
└── video_export/            ← record & export a video of the Newton sim motion
    ├── NEWTON_VIDEO_RECORDING.md    ← the write-up (why Stage Recorder fails, the working pipeline)
    └── newton_video_pipeline.py     ← reusable recorder → bake → render → encode pipeline
```

## Prerequisites

### Supported platforms

The stack runs on both combinations below. It does not read `ROS_DISTRO` to
decide how to behave — it inspects the installed `moveit_servo` and
`ur_moveit_config` and adapts, so a from-source build of either also works.
See [`teleop/_servo_compat.py`](teleop/_servo_compat.py) for what differs.

| | Ubuntu 24.04 + ROS 2 Jazzy | Ubuntu 22.04 + ROS 2 Humble |
|---|---|---|
| Status | **Recommended.** Verified end-to-end on RTX A4000. | Supported; CI-verified, not re-run end-to-end since the Jazzy port. |
| Isaac Sim 6.0.0 | Passes NVIDIA's compatibility check; Isaac bundles internal Jazzy ROS 2 libraries (`exts/isaacsim.ros2.core/jazzy`), so both sides speak the same distro and DDS type hashes match. | Isaac Sim 6's original target platform. |
| MoveIt Servo | 2.12+ (rewritten): `servo_node`, requires explicit command-type selection. | Pre-rewrite: `servo_node_main`, accepts any command type at any time. |

Humble publishes no packages for noble, so on a 24.04 machine Jazzy is the only
option — that asymmetry is why the stack moved to Jazzy while keeping Humble
working.

Both are exercised on every change to `grippers/demo/teleop/` by the
`isaac-teleop-launch-smoke` CI job; run it locally with
[`teleop/smoke_launch_check.py`](teleop/smoke_launch_check.py).

Install the packages below and run the launches natively — that's the normal
workflow. If you'd rather not, or want to try the distro you aren't on,
[`teleop/run.sh`](teleop/run.sh) runs the ROS side in a container for either
one; see [teleop/README.md § Running in a container](teleop/README.md#running-in-a-container).
Isaac Sim stays on the host regardless, since it needs the GPU.

### Everything else

- **NVIDIA driver** compatible with Isaac Sim 6 (RTX GPU required)
- **Isaac Sim 6.0.0** installed at `~/isaacsim` (or update the launcher path)
- The `isaacsim.ros2.bridge`, `isaacsim.sensors.physics`, and `isaacsim.core.nodes` Kit extensions enabled (Window → Extensions, check *AUTOLOAD*)
- ROS 2 apt packages — substitute `jazzy` or `humble` for `$ROS_DISTRO`:
  ```bash
  sudo apt install \
      ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-servo \
      ros-$ROS_DISTRO-ur-description ros-$ROS_DISTRO-ur-moveit-config \
      ros-$ROS_DISTRO-joy
  ```
- **(Gamepad only)** A DualShock 4 or DualSense controller connected — the
  Linux kernel's `hid-playstation` driver exposes it on `/dev/input/js0`.
- **(Haptics, optional)** The R2 adaptive-trigger force feedback needs a
  **DualSense** specifically (DualShock 4 has no adaptive triggers), plus
  `pydualsense` + `hidapi` for the system `python3` and read/write access to
  the controller's `hidraw` node (a `uaccess` udev rule for `054c:0ce6`,
  otherwise run as root):
  ```bash
  pip install -r teleop/requirements.txt   # pydualsense + hidapi
  sudo apt install libhidapi-hidraw0
  ```
  Best-effort — `teleop.launch.py` starts it, but the rest of the stack runs
  fine without it.

## Quick start

The shipped scene [`teleop/ur5robot_with_2F-85.usda`](teleop/ur5robot_with_2F-85.usda)
already has the UR5e + 2F-85 articulation with the tuned physics baked in.
If you want to use a different scene, see
[teleop/README.md § Building a compatible stage](teleop/README.md#building-a-compatible-stage).

### Step 1 — Isaac side: create the ROS bridge Action Graph

Open `teleop/ur5robot_with_2F-85.usda` in Isaac Sim (or your own scene).
Then:

- `Window → Script Editor → File → Open` → select
  `isaac_scripts/build_ros_action_graph.py`
- Edit `ARTICULATION_PRIM` at the top if your robot isn't at
  `/World/ur5e/root_joint` (the prim with `PhysicsArticulationRootAPI`
  applied — see the *Articulation root* section in teleop/README.md)
- **Run** (the play button in Script Editor)

You should see `[ROS_JointControl] built, driving /World/ur5e/root_joint` in
the console. Verify by running `ros2 topic list` in a ROS-sourced terminal:
`/clock`, `/joint_states`, `/joint_command` should all appear.

Press **Play** in Isaac Sim's Timeline (the graph only ticks during playback).

### Step 2 — ROS side: launch the whole teleop stack

One command brings up everything: Servo + bridge + filter + robot_state_publisher
immediately, joy_node and the gamepad mapper at 10 s, then the DualSense R2
haptic feedback at 12 s:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch /path/to/grippers/demo/teleop/teleop.launch.py
```

Look for `bridging /forward_position_controller/commands -> /joint_command`
(bridge) and `servo started` (gamepad) — that's the "ready to drive"
handshake.

The R2 haptic feedback is best-effort — if Isaac, the deps, or a DualSense
aren't present it just logs and the rest of the stack is unaffected. **On a
freshly launched Isaac, do one Stop→Play once the stack is up**: PhysX only
starts reporting the gripper pad contacts after a replay, so without it the
trigger stays slack even on a firm grasp.

If you only want the Servo side and will drive it from something other
than the gamepad, use `ur5e_servo.launch.py` alone; if you already have
Servo running and just want the gamepad, use `gamepad.launch.py` — neither
of those starts the haptics, only `teleop.launch.py` does.

### Step 3 — Frontend: pick one

**Gamepad** — already launched by step 2. See the mapping cheat-sheet in
the console output. Left stick: roll/pitch. Right stick: linear X/Y.
D-pad up/down: Z. **L1/R1: yaw −/+**. **R2 (analog): gripper** (released =
open, fully pressed = closed). Triangle/Cross: speed ×/÷ 1.25. Circle: resync
servo after a Stop→Play cycle. Options: go to ready pose. **PS/Share: quit.**
On a DualSense, R2 also pushes back proportionally to grip force (see step 2).

That mapping is for a PS4/PS5 controller. For another gamepad, pick a
**controller profile** — see [Controller profiles](#controller-profiles).

**Keyboard:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 /path/to/grippers/demo/teleop/servo_keyboard_teleop.py \
        --ros-args -p use_sim_time:=true
```
`A/D` linear x, `W/X` linear y, `↑/↓` linear z, `I/K J/L U/O` rotations,
`1`–`6` + `+/-` joint jog, `←/→` gripper, `h` home, `q` quit.

**Fallback if Servo won't cooperate:** `keyboard_jog.py` — direct per-joint
control, bypasses Servo entirely. Same key idea, works whenever the Action
Graph subscriber is alive.

## Controller profiles

Gamepads report their buttons and sticks in different orders and with
different signs, so `gamepad_teleop.py` reads them through a **profile**
(the `PROFILES` table at the top of the file). Pick one with the
`controller` launch argument; it works on `teleop.launch.py` and
`gamepad.launch.py`:

```bash
ros2 launch /path/to/grippers/2F/demo/teleop/teleop.launch.py controller:=elecom
```

| Profile | Controller | Notes |
|---|---|---|
| `dualsense` (default) | PS4 DualShock 4 / PS5 DualSense | Mapping above. PS/Share quits. |
| `elecom` | Elecom wired gamepad (USB `05b8:1004`) | 12 buttons, no analog triggers. No quit button: stop with **Ctrl+C**. |

`elecom` mapping (numbers are the labels printed on the controller):

| Function | Elecom |
|---|---|
| Linear X / Y | right stick |
| Up / down | back **5** (left upper) / back **7** (left lower) |
| Roll / pitch | left stick |
| Yaw − / + | **1** / **4** |
| Gripper | back **8**: hold = closed, release = open |
| Speed × / ÷ 1.25 | **2** / **3** |
| Resync Servo | **9** |
| Ready pose | **10** |

**Adding a profile for your gamepad:**

1. With `joy_node` running (any of the launches above starts it), watch
   what each control sends:
   ```bash
   ros2 topic echo /joy
   ```
   Press each button and push each stick, one at a time, and note which
   index of `buttons` or `axes` changes. Note also the sign of each stick:
   the profiles expect **left = −1** and **up = −1** after the sign is
   applied.
2. Copy the `elecom` entry in `PROFILES`, give it your controller's name,
   and fill in the indices. Use `gripper_axis` for an analog trigger or
   `gripper_button` for a digital one, and `quit=()` if you don't want a
   quit button.
3. Launch with `controller:=<your profile>` and drive the robot. If a stick
   moves the wrong way, flip that axis in `signs`.

Watch out for the quit buttons: the `dualsense` profile quits on buttons
4 and 5 (Share and PS). On another controller those indices are often
ordinary buttons, so the script quits as soon as you press one. Using
your own profile avoids that.

## Running on Windows (Isaac Sim) + WSL (ROS 2)

The stack was written for Linux, with Isaac Sim and ROS 2 on the same
machine. It also runs with **Isaac Sim on Windows** and **the ROS 2 side in
WSL** (Ubuntu 24.04, Jazzy): the demo scripts work unchanged, but the
connection between the two sides and the gamepad need the setup below.
Tested on Windows 11 with Isaac Sim 6.1.

### Connect Isaac Sim and ROS 2 with Zenoh

Plain DDS does not cross between Windows and WSL reliably, so both sides use
the Zenoh middleware. `doc/3-Control from ROS.md` explains why and covers
the details; in short:

1. **WSL networking must be mirrored.** In `%UserProfile%\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   then `wsl --shutdown` and reopen WSL.
2. **Install Zenoh in WSL:** `sudo apt install ros-jazzy-rmw-zenoh-cpp`.
3. **Every WSL terminal** needs, after sourcing ROS:
   ```bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```
4. **Start the Zenoh router** in a WSL terminal and leave it running:
   ```bash
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ```
5. **Start Isaac Sim** from PowerShell, after the router:
   ```powershell
   $env:ZENOH_CONFIG_OVERRIDE = 'connect/endpoints=["tcp/127.0.0.1:7447"];listen/endpoints=["tcp/127.0.0.1:0"]'
   C:\isaacsim\isaac-sim.bat
   ```
   Enable `isaacsim.ros2.bridge` with **Autoload** (Window → Extensions),
   then open `teleop/ur5robot_with_2F-85.usda` with **File → Open** and
   press **Play**. Open the scene by hand: opening it from a script in a
   running Isaac Sim can leave the robot unable to move (see
   *Troubleshooting*).
6. **Check the link** from WSL: `ros2 topic list` shows `/clock`,
   `/joint_command` and `/joint_states`. Then run Step 2 as usual, in a WSL
   terminal, with the path under `/mnt/c/...`.

### Use the Newton gripper

The scene uses the PhysX gripper by default. To run the gripper on the Newton
engine instead:

1. **Start the Newton app**, with the same Zenoh setting:
   ```powershell
   $env:ZENOH_CONFIG_OVERRIDE = 'connect/endpoints=["tcp/127.0.0.1:7447"];listen/endpoints=["tcp/127.0.0.1:0"]'
   C:\isaacsim\isaac-sim.newton.bat
   ```
   The Newton app keeps its own extension settings. Enable
   `isaacsim.ros2.bridge` with **Autoload** in it too.
2. **Open** `teleop/ur5robot_with_2F-85.usda` with **File → Open**.
3. **Select the Newton gripper:** in the Stage panel, select `/World`, and in
   the Property panel set **Variants → Gripper** to `newton`. Use this
   `/World` variant only; don't change the gripper's own **Physics**
   variant (see *Troubleshooting*).
4. **Before pressing Play,** run `teleop/newton_runtime.py` from
   **Window → Script Editor**:
   ```python
   exec(open(r"C:\path\to\grippers\2F\demo\teleop\newton_runtime.py").read())
   ```
   It prints two `[newton-runtime]` lines. Run it again after each scene
   reopen. It is not needed after Stop → Play.
5. Press **Play**, then run Step 2 in WSL as usual.

`newton_runtime.py` fixes two Newton problems. The first stops a crash.

- **Joint states.** The scene's joint-state reader asks the engine for joint
  efforts. Newton has none, and on Isaac Sim 6.1 that crashes Isaac at Play.
  The script swaps that reader for one that reads positions and velocities.
  Under Newton, `/joint_states` therefore has **zero efforts**. The swap
  lives in the session layer, so saving the scene never writes it to the
  file. The variant you picked in step 3 is saved, though: set it back to
  `physx` before saving, or the next person opening the scene gets the
  Newton gripper.
- **Gripper tuning.** It runs `Robotiq_2F_85/newton/apply_gripper_tuning.py`
  automatically after every Play. Without the tuning the fingers go floppy.

### Use a USB gamepad in WSL

WSL does not see USB devices by default. Pass the gamepad through with
[usbipd-win](https://github.com/dorssel/usbipd-win):

1. Find the gamepad's bus ID in PowerShell: `usbipd list`.
2. **Bind it, once, in PowerShell as Administrator:**
   ```powershell
   usbipd bind --busid <BUSID> --force
   ```
   then unplug and replug the gamepad. `--force` is needed for gamepads:
   without it, Windows keeps the device and the attach fails with
   `Device in error state`. While bound this way, Windows itself cannot
   use the gamepad; `usbipd unbind --busid <BUSID>` (as Administrator, then
   replug) gives it back.
3. **Attach it to WSL**, after each replug or WSL restart:
   ```powershell
   usbipd attach --wsl --busid <BUSID>
   ```
4. **Allow your WSL user to read it, once,** in WSL:
   ```bash
   sudo usermod -aG input $USER
   ```
   then open a new WSL terminal. Without it, `joy_node` sees no gamepad
   and reports no error.
5. **Check** in WSL: `ros2 run joy joy_enumerate_devices` lists the
   gamepad. If it is not a PS4/PS5 controller, pick or add a profile (see
   [Controller profiles](#controller-profiles)).

### What to expect

- **`trigger_force_feedback.py … process has died`** at startup: the
  haptics node needs a DualSense and exits without one. The rest of the
  stack runs normally.
- **`start_servo: service not available` / `start_servo failed`** from the
  gamepad node: over Zenoh the gamepad node does not reach Servo's
  start/stop services in time. Servo runs anyway and the robot moves; only
  the resync button is affected (it takes a few seconds and may not
  resync).
- **The Linux helper scripts** (`isaac-demo-dds.sh`,
  `cpu_governor_check.sh`, `mcp_bridge/launch_isaac_with_mcp.sh`) are
  Linux-only and not needed on Windows.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` doesn't show `/joint_states` | Action Graph missing or sim not playing | Rerun `build_ros_action_graph.py`, press Play in Isaac. Check console for the `Joint state from sensor: … must have the same length` error — see [teleop/README.md](teleop/README.md) *"Joint state length mismatch"* gotcha. |
| Gamepad launches but robot doesn't move | Servo not started (state cleared by Stop→Play) | Press **Circle (○)** on the gamepad — this triggers a stop→start resync of Servo. |
| Robot moves without any input, values wrap madly | Servo emitted NaN, drives chased it | Stop→Play the sim to reset joints. The bridge now drops NaN so this shouldn't recur, but if it does, see teleop/README.md gotcha *"Servo drift loop when idle output isn't suppressed"*. |
| First move after Stop→Play feels wrong (arm snaps to a stale pose) | Servo's `internal_joint_state_` is stale from before Stop→Play | Press **Circle (○)** on the gamepad before the first real input, or send any zero-magnitude twist to warm Servo up. |
| Sticks feel sluggish | Servo's `scale.rotational` cap clips you | `ur5e_servo.launch.py` overrides `scale.rotational=3.0`; `scale.linear` keeps the `ur_servo.yaml` default (0.6 m/s). Bump either in the launch file if needed. |
| R2 trigger doesn't stiffen when gripping | Haptics not running, deps/controller missing, or PhysX not yet reporting contacts | Confirm it's a **DualSense** (not DS4) with `pydualsense`/`hidapi` installed. On a fresh Isaac, do one **Stop→Play** so PhysX reports pad contacts. Make sure no stale `trigger_force_feedback.py` from an earlier session is still holding UDP `8770`. Only `teleop.launch.py` starts the haptics. |
| Gamepad worked for a few seconds, then stopped responding | The gamepad node quit: with the `dualsense` profile, buttons 4/5 (Share/PS) quit, and on other gamepads those are ordinary buttons | Use a profile for your controller, `controller:=<profile>` (see [Controller profiles](#controller-profiles)), and relaunch. |
| Gripper doesn't close, arm works | The gripper runs Newton settings under the PhysX engine: either the gripper's own **Physics** variant was set to `Newton_compliant`, or `/World` **Gripper** is `newton` in the regular `isaac-sim.bat` app | Choose the engine with the `/World` **Gripper** variant only, and match the app: `physx` with `isaac-sim.bat`, `newton` with `isaac-sim.newton.bat`. |
| Isaac Sim crashes when you press Play with the Newton gripper | The joint-state reader asks Newton for joint efforts, which Newton doesn't have | Run `newton_runtime.py` before Play (see [Use the Newton gripper](#use-the-newton-gripper)). |
| `/joint_command` is received but the robot never moves | Physics/articulation handle went stale — usually after reopening the stage in an already-running Isaac (e.g. over MCP) | **Relaunch Isaac fresh** and reload the scene — a Stop→Play does *not* recover this. |

Deeper explanations for each of the above in [teleop/README.md](teleop/README.md).

## What this stack does *not* include

- **Planning.** No MoveIt path planning, no `move_group`. Servo is a
  real-time streaming controller only — no obstacle avoidance beyond a
  contact "slow down" decel scale.
- **The real UR driver.** Everything targets Isaac Sim's Action Graph
  `IsaacArticulationController`. Migrating to the real `ur_robot_driver`
  is out of scope but the launch file's `command_out_topic` remap is where
  you'd start.

The shipped USD (`teleop/ur5robot_with_2F-85.usda`) has the arm, gripper,
tuned physics, and ROS bridge Action Graph pre-built. If you want to swap
in a different scene, see teleop/PHYSICS_TUNING.md for what to bake in.
