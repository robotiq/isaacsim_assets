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
servo after a Stop→Play cycle. Options: go to ready pose. On a DualSense, R2
also pushes back proportionally to grip force (see step 2).

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

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` doesn't show `/joint_states` | Action Graph missing or sim not playing | Rerun `build_ros_action_graph.py`, press Play in Isaac. Check console for the `Joint state from sensor: … must have the same length` error — see [teleop/README.md](teleop/README.md) *"Joint state length mismatch"* gotcha. |
| Gamepad launches but robot doesn't move | Servo not started (state cleared by Stop→Play) | Press **Circle (○)** on the gamepad — this triggers a stop→start resync of Servo. |
| Robot moves without any input, values wrap madly | Servo emitted NaN, drives chased it | Stop→Play the sim to reset joints. The bridge now drops NaN so this shouldn't recur, but if it does, see teleop/README.md gotcha *"Servo drift loop when idle output isn't suppressed"*. |
| First move after Stop→Play feels wrong (arm snaps to a stale pose) | Servo's `internal_joint_state_` is stale from before Stop→Play | Press **Circle (○)** on the gamepad before the first real input, or send any zero-magnitude twist to warm Servo up. |
| Sticks feel sluggish | Servo's `scale.rotational` cap clips you | `ur5e_servo.launch.py` overrides `scale.rotational=3.0`; `scale.linear` keeps the `ur_servo.yaml` default (0.6 m/s). Bump either in the launch file if needed. |
| R2 trigger doesn't stiffen when gripping | Haptics not running, deps/controller missing, or PhysX not yet reporting contacts | Confirm it's a **DualSense** (not DS4) with `pydualsense`/`hidapi` installed. On a fresh Isaac, do one **Stop→Play** so PhysX reports pad contacts. Make sure no stale `trigger_force_feedback.py` from an earlier session is still holding UDP `8770`. Only `teleop.launch.py` starts the haptics. |
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
