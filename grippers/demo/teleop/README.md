# Teleoperation — UR5e + Robotiq 2F-85 in Isaac Sim

Deep-dive companion to [the parent `README.md`](../README.md). Covers:

1. **Isaac Sim side** — stage requirements, the ROS 2 bridge extension,
   and the OmniGraph Action Graph that exposes the robot to ROS topics.
2. **ROS 2 side** — MoveIt Servo + bridge + filter + frontends (keyboard
   and gamepad) that turn user input into joint commands.
3. **Building a compatible stage** — what a scene must contain for the
   ROS side to work with it.

For gripper/cube physics tuning, see [PHYSICS_TUNING.md](PHYSICS_TUNING.md).

## Goal

Drive tool0 in cartesian space from a gamepad or keyboard (A/D, W/X for
linear x/y, `↑`/`↓` for z; I/K J/L U/O for angular; `←`/`→` for gripper).
No URCap, no `ros2_control`, no MoveIt planning — just Servo doing
real-time inverse kinematics and our two-line bridge feeding the
resulting joint positions into Isaac via the existing Action Graph.

## Data flow

```
        keyboard
            │
            ▼
┌──────────────────────────┐
│ servo_keyboard_teleop.py │ ── /joint_command ────┐  (gripper bypass)
│  (TwistStamped @ tool0)  │     JointState         │
│  or gamepad_teleop.py    │     (finger_joint)     │
└────────────┬─────────────┘                        │
             │                                      │
   /servo_node/delta_twist_cmds                     │
             │                                      │
             ▼                                      │
┌──────────────────────────┐                        │
│      moveit_servo        │                        │
│  IK in base_link frame,  │                        │
│  singularity scaling,    │                        │
│  joint-limit margin      │                        │
└────────────┬─────────────┘                        │
             │                                      │
  /forward_position_controller/commands             │
             │  Float64MultiArray, 6 doubles        │
             ▼                                      │
┌──────────────────────────┐                        │
│  servo_to_isaac_bridge   │── /joint_command ──────┤  (arm path)
│  attaches joint names    │     JointState         │
│  (6 arm joints)          │                        │
└──────────────────────────┘                        │
                                                    │
                                                    ▼
                              ┌──────────────────────────────────────┐
                              │  Isaac Sim Action Graph              │
                              │  SubscribeJointState →               │
                              │  ArticulationController              │
                              │  (applies arm + gripper, each from   │
                              │   their respective messages)         │
                              └──────────────────────────────────────┘

                       robot pose feedback
┌──────────────────────────┐ ←── /joint_states (14 joints)
│ joint_states_arm_filter  │
│ (strip 8 finger joints)  │ ──► /joint_states_arm (6 joints)
└──────────────────────────┘
                                          │
                                          ▼
                                     moveit_servo
```

Arm and gripper share `/joint_command` but never collide: each published
`JointState` only carries the joints it owns, and `IsaacArticulationController`
applies whichever joint names it sees in each message.

## Isaac Sim side: the Action Graph this teleop relies on

Everything below the dashed line in the data-flow diagram lives inside
Isaac Sim. The robot is visible to ROS only because an OmniGraph Action
Graph in the USD stage bridges Isaac's articulation API to three ROS
topics.

### Stage requirements

**Shipped scene:** [`ur5robot_with_2F-85.usda`](ur5robot_with_2F-85.usda) in
this folder is a ready-to-use scene with all the physics tuning from
[PHYSICS_TUNING.md](PHYSICS_TUNING.md) baked in — open it in Isaac Sim,
run the Action Graph builder script, hit Play, and skip to Day-to-day
usage. It's shipped as USD ASCII (`.usda`) rather than binary crate
(`.usdc`) so diffs of gripper-tuning changes are readable in reviews.
If you want to use a different stage, see
[Building a compatible stage](#building-a-compatible-stage) at the
bottom of this doc.

### Gripper backend variant — PhysX vs Newton

The scene carries a `Gripper` variant set on `/World` (Stage panel →
select `/World` → *Variants* → `Gripper`), so one file can test either
gripper without maintaining a second stage:

| `Gripper` | Gripper asset | Launch with | After every Stop→Play |
| --------- | ------------- | ----------- | --------------------- |
| `physx` *(default)* | `Gripper_2F85/Robotiq_2F_85_edit.usda` (+ its own `Physics` sub-variant) | normal Isaac Sim / `mcp_bridge/launch_isaac_with_mcp.sh` | nothing |
| `newton` | `Gripper_2F85_newton/Robotiq_2F85_newton.usda` | **Newton experience** — `mcp_bridge/launch_isaac_newton_with_mcp.sh` (`isaac-sim.newton.sh`) | run `Gripper_2F85_newton/apply_gripper_tuning.py` once |

Each variant swaps the gripper payload *and* its mount joint. The
`newton` variant deletes the Newton gripper's own
`PhysicsArticulationRootAPI` so it merges into the arm articulation
(finger DOF stays named `finger_joint`), and mounts it on `wrist_3_link`
with a stiff `gripper_mount_lock` revolute. The scene's `/PhysicsScene`
carries `MjcSceneAPI` (inert under PhysX) so no scene edit is needed when
switching.

**The Newton runtime step is not optional.** Two Newton/MuJoCo settings
cannot be persisted in USD (`opt.impratio` and the finger loop-closure
equality `solref`); run `apply_gripper_tuning.py` from the Script Editor
or MCP bridge once after each Stop→Play, or the fingers go floppy. See
[the Newton asset README](../../Gripper_2F85_newton/README.md).

The articulation we drive contains the UR5e arm and the Robotiq 2F-85
gripper as one assembly. The non-obvious detail:

- **Articulation root prim:** `/World/ur5e/root_joint`
  *(a `PhysicsFixedJoint` child of `/World/ur5e`, with
  `PhysicsArticulationRootAPI` applied)*
- **Common mistake:** pointing nodes at `/World/ur5e` itself. Some
  Isaac graph nodes scan downward and find the articulation anyway
  (`isaacsim.core.nodes.IsaacArticulationState`), but the strict
  sensor-physics nodes (`isaacsim.sensors.physics.IsaacReadJointState`)
  silently fail with "no DOFs found" if pointed at the parent. Always
  use the exact root.

The 2F-85 has 8 finger DOFs in this USD. Under the Physx_Loop variant
the 4-bar loop-closure constraint propagates `finger_joint` to the rest,
so the teleop scripts command only `finger_joint` (see "Gripper path"
below).

### Required Kit extensions

`Window → Extensions`, search and enable (`AUTOLOAD` on):

| Extension                       | Why                                                                                          |
| ------------------------------- | -------------------------------------------------------------------------------------------- |
| `isaacsim.ros2.bridge`          | Provides the `ROS2Publish/Subscribe*` and `ROS2PublishClock` OmniGraph nodes.                |
| `isaacsim.core.nodes`           | Provides `IsaacArticulationController`, `IsaacReadSimulationTime`. Usually on by default.    |
| `isaacsim.sensors.physics`      | Provides `IsaacReadJointState` (the *only* node that outputs `jointDofTypes`).               |
| `omni.kit.scripting` *(opt.)*   | Lets you attach a Python Scripting Component to a prim for hot-reloading graph build code.   |

Launch Isaac Sim from a **clean shell** (do not `source /opt/ros/$ROS_DISTRO/setup.bash`
first — its Python 3.10 will collide with Isaac's bundled Python 3.11/3.12
and `rclpy` inside the bridge extension will refuse to load).

### Action Graph topology — `/World/ROS_JointControl`

```
  ┌───────────────────────┐
  │     OnPlaybackTick    │ ─── tick ──┬──────────────┬──────────────┬─────────────────────┐
  │ omni.graph.action.    │            │              │              │                     │
  │   OnPlaybackTick      │            │              │              │                     │
  └───────────────────────┘            │              │              │                     │
                                       │              │              │                     │
  ┌───────────────────────┐            │              │              │                     │
  │ IsaacReadSimulation-  │ ── simTime ─┐             │              │                     │
  │        Time           │             │             │              │                     │
  └───────────────────────┘             │             │              │                     │
                                        │             │              │                     │
                                        ▼             ▼              ▼                     ▼
                                  ┌──────────┐  ┌───────────┐  ┌─────────────┐  ┌──────────────────┐
                                  │ Publish  │  │ Subscribe │  │ Articulation│  │  ReadJointState  │
                                  │  Clock   │  │ JointState│  │ Controller  │  │  (sensors.physics)│
                                  │   →      │  │     →     │  │ targetPrim: │  │      prim:       │
                                  │ /clock   │  │/joint_cmd │  │ /World/ur5e/│  │ /World/ur5e/     │
                                  └──────────┘  └─────┬─────┘  │ root_joint  │  │    root_joint    │
                                                      │        └──────▲──────┘  └─────────┬────────┘
                                                      │  jointNames,  │                   │
                                                      │  positionCmd, │                   │ outputs: jointNames,
                                                      │  velocityCmd, │                   │ jointPositions,
                                                      │  effortCmd    │                   │ jointVelocities,
                                                      └───────────────┘                   │ jointEfforts,
                                                                                          │ jointDofTypes,
                                                                                          │ sensorTime,
                                                                                          │ stageMetersPerUnit
                                                                                          │
                                                                                          ▼
                                                                              ┌────────────────────────┐
                                                                              │   PublishJointState    │
                                                                              │      → /joint_states   │
                                                                              └────────────────────────┘
```

Seven nodes, three external topics:

| Node path                                   | Type                                          |
| ------------------------------------------- | --------------------------------------------- |
| `/World/ROS_JointControl/OnTick`            | `omni.graph.action.OnPlaybackTick`            |
| `/World/ROS_JointControl/SimTime`           | `isaacsim.core.nodes.IsaacReadSimulationTime` |
| `/World/ROS_JointControl/PubClock`          | `isaacsim.ros2.bridge.ROS2PublishClock`       |
| `/World/ROS_JointControl/SubJointState`     | `isaacsim.ros2.bridge.ROS2SubscribeJointState`|
| `/World/ROS_JointControl/ArtController`     | `isaacsim.core.nodes.IsaacArticulationController` |
| `/World/ROS_JointControl/ReadJointState`    | `isaacsim.sensors.physics.IsaacReadJointState`|
| `/World/ROS_JointControl/PubJointState`     | `isaacsim.ros2.bridge.ROS2PublishJointState`  |

**Topics it produces / consumes:**

| Topic           | Direction       | Type                       | Purpose                                       |
| --------------- | --------------- | -------------------------- | --------------------------------------------- |
| `/clock`        | Isaac → ROS     | `rosgraph_msgs/Clock`      | Sim time — every ROS node needs `use_sim_time:=true` to consume this. |
| `/joint_states` | Isaac → ROS     | `sensor_msgs/JointState`   | 14 joints (6 UR5e + 8 finger). Filter strips to 6 for Servo.          |
| `/joint_command`| ROS → Isaac     | `sensor_msgs/JointState`   | Position commands. Action Graph applies whichever joints are named.   |

### The big Isaac-side gotcha: `ReadJointState`, not `ArticulationState`

In Isaac Sim 6, `ROS2PublishJointState` requires its `jointDofTypes`
input to be the same length as `jointNames`/`jointPositions`/etc.
If any of these arrays has a different length, the node throws
`Joint state from sensor: ... must have the same length.` *every
compute* and **the ROS topic never appears at all**.

Two candidate "reader" nodes exist; only one of them works:

- `isaacsim.core.nodes.IsaacArticulationState` — permissive about
  prim paths but **does not output `jointDofTypes`**. Pairing it with
  `ROS2PublishJointState` gives the silent length-mismatch failure.
- `isaacsim.sensors.physics.IsaacReadJointState` — outputs all 7
  fields including `jointDofTypes`. **Use this one.** Strict about
  the articulation root (see above).

The mistake is invisible from outside Isaac: `ros2 topic list` simply
won't show `/joint_states`. The fix is visible only in
`Window → Console` as the length-mismatch error.

### Building the graph

You can construct `/World/ROS_JointControl` three ways. Pick whichever
fits the moment:

1. **GUI (`Window → Visual Scripting → Action Graph`)** — exhaustive
   manual wiring per the topology table above. Fine the first time,
   tedious afterwards.

2. **Python (Script Editor)** — paste the `og.Controller.edit()` call.
   We ship this at `../isaac_scripts/build_ros_action_graph.py`.
   Open it via `Window → Script Editor → File → Open`, hit Run. The
   script first removes any existing `/World/ROS_JointControl` so it's
   idempotent against the graph the shipped scene already has.

3. **MCP (Claude does it)** — launch Isaac Sim from `../mcp_bridge/launch_isaac_with_mcp.sh`,
   open Claude Code in this repo, and ask. Claude has
   `mcp__isaac-sim__create_action_graph` wired and the topology
   documented above.

### Verifying the Isaac side is healthy

With the stage loaded, the graph built, and the simulation **playing**
(it has to be playing — `OnPlaybackTick` only fires during playback):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash       # in a separate shell
ros2 topic list -t                       # /clock /joint_command /joint_states all present?
ros2 topic echo /joint_states --once     # 14 joints, sec ≈ thousands (sim time)
ros2 topic echo /clock --once            # advancing
```

If `/joint_states` is missing but `/joint_command` and `/clock` are
present, you're hitting the `IsaacReadJointState` gotcha — check the
Console for the length-mismatch error and re-verify the reader node
type and its `prim` input (must be the articulation root).

If nothing is listed: simulation isn't playing, or the
`isaacsim.ros2.bridge` extension isn't loaded.

## Components

All in this folder:

| File                          | Role                                                                                   |
| ----------------------------- | -------------------------------------------------------------------------------------- |
| `ur5e_servo.launch.py`        | Brings up `robot_state_publisher`, `servo_node`, filter, bridge — patched configs.     |
| `joint_states_arm_filter.py`  | `/joint_states` (12) → `/joint_states_arm` (6). Servo only knows UR5e arm joints.      |
| `servo_to_isaac_bridge.py`    | `/forward_position_controller/commands` (Float64MultiArray) → `/joint_command` (JointState) with joint names attached. |
| `servo_keyboard_teleop.py`    | Reads keyboard, publishes TwistStamped/JointJog to Servo, plus direct gripper writes.  |
| `keyboard_jog.py`             | Servo-less fallback. Pure per-joint nudge straight to `/joint_command`. No IK, no Servo. |
| `sim_reset_watchdog.py`       | Watches `/clock` for a jump backwards (Isaac Stop→Play) and reseeds Servo, so teleop resumes from the reset pose. Started by `ur5e_servo.launch.py`. |

Local helpers that absorb the Humble/Jazzy differences:

| File | Role |
|---|---|
| `_servo_compat.py`      | Every distro difference lives here — servo executable name, `kinematics.yaml` layout, planning-group name, declared-parameter filtering, idle protection, command-type switching. Detects what is installed rather than reading `ROS_DISTRO`. |
| `smoke_launch_check.py` | Builds the launch description and checks it against the installed packages. Run by the `isaac-teleop-launch-smoke` CI job on both distros; no GPU or Isaac needed. |
| `set_servo_command_type.py` | Manual `switch_command_type` helper for debugging. Nothing launches it — the frontends select their own type. |
| `Dockerfile` / `run.sh` / `entrypoint.sh` | Optional containerised ROS side, either distro. See [Running in a container](#running-in-a-container). |

Haptic feedback (DualSense only, optional — included by `teleop.launch.py`):

| File | Role |
|---|---|
| `haptics.launch.py`          | Injects the gripper contact-force plot into the running Isaac session, then starts the trigger node. Best-effort: if Isaac or the controller isn't there it logs and the rest of the stack is unaffected. |
| `trigger_force_feedback.py`  | Reads the pad-force UDP stream on `127.0.0.1:8770` and drives R2's adaptive-trigger resistance from it — squeeze in sim, the trigger stiffens. |
| `requirements.txt`           | Host python deps for the above (`pydualsense`, `hidapi`); also needs `libhidapi-hidraw0` and hidraw access. Not installed in the container image — see [Running in a container](#running-in-a-container). |

External pieces we lean on (apt packages for the distro you installed — see
[the support matrix](../README.md#supported-platforms)):

- `ur_description` — URDF/xacro for the UR5e (no gripper in this URDF)
- `ur_moveit_config` — SRDF defining the manipulator planning group, plus `kinematics.yaml`, `joint_limits.yaml`, `ur_servo.yaml`. Note the group is called `ur5e_manipulator` on Humble but `ur_manipulator` on Jazzy, whose SRDF macro takes no `name` parameter at all — the launch reads the name out of the generated SRDF rather than assuming either.
- `moveit_servo` — the realtime cartesian/joint jog node

## Setup walk-through

We're driving the **existing** UR ROS 2 driver's MoveIt config but
without any of its hardware-side scaffolding (`ros2_control`, the
URCap reverse-interface, RViz). The launch keeps four nodes alive:

1. **`robot_state_publisher`** — parses the UR5e URDF, publishes `/tf` based
   on the joint states the arm filter feeds. Servo reads TF to know where
   `tool0` is relative to `base_link`.

2. **`joint_states_arm_filter`** — strips the 6 Robotiq finger joints out of
   `/joint_states` (which Isaac publishes with all 12 DOFs). Without this,
   Servo's `CurrentStateMonitor` complains about every gripper joint not
   being in the URDF and ends up with an incomplete robot state.

3. **`servo_node`** — the MoveIt 2 cartesian/joint jog server. Loaded with:
   - URDF + SRDF (UR5e arm only)
   - `ur_servo.yaml` plus three patches the upstream file is missing:
     - `move_group_name = ur5e_manipulator` (SRDF macro produces this; default `ur_manipulator` doesn't exist)
     - `joint_topic = /joint_states_arm` (skip the gripper-polluted topic)
     - `is_primary_planning_scene_monitor = True` (no move_group running to provide one)
     - `override_velocity_scaling_factor = 1.0` (param defaults to 0.0 if absent — see Gotcha #2)

4. **`servo_to_isaac_bridge`** — Servo publishes `Float64MultiArray` (just an
   array of 6 doubles, no joint names) to `/forward_position_controller/commands`.
   Isaac's Action Graph needs `JointState` with explicit names. The bridge
   converts on the fly.

All four nodes run with `use_sim_time: True` so their clocks track
Isaac's `/clock` topic. See Gotcha #1 for why this is non-optional.

## The three gotchas we hit (and how to spot them)

These all manifested as "Servo seems to run but the robot doesn't move."
Same symptom, three independent causes. If teleop ever stops working,
work the checklist in this order.

### 1. `use_sim_time` mismatch

**Symptom:** Servo's main loop logs `run_duration` warnings (so it's
running), `/servo_node/status` reports `0` (NO_WARNING), but
`/forward_position_controller/commands` is silent — not even a hold
message — and no joint motion in Isaac.

**Cause:** Isaac publishes `/clock` with sim time (`sec ≈ 2000`). Any
ROS node started without `use_sim_time: True` stamps its messages with
wall time (`sec ≈ 1.78e9`). Servo reads the stamp on each incoming
`TwistStamped` / `JointJog`, sees a value billions of seconds in its
"future" relative to its sim-time clock, and the stale-message filter
silently drops the input.

**Fix:** every node in the launch sets `use_sim_time: True`. When you
run `servo_keyboard_teleop.py` directly, pass
`--ros-args -p use_sim_time:=true`.

**Quirk:** with `use_sim_time: True`, `node.get_clock().now()` returns
`(0, 0)` until the **first** `/clock` message arrives. The keyboard
script calls `wait_for_sim_time()` at startup to block until the clock
has populated; otherwise the first keypress would be stamped 0 and
also dropped as stale.

### 2. `override_velocity_scaling_factor = 0.0`

**Symptom:** Servo is publishing now (Gotcha #1 was fixed), the joint
positions in `/forward_position_controller/commands` evolve smoothly,
but every command is exactly the current joint state — delta = 0.

**Cause:** `moveit_servo` declares this parameter with default
`0.0`. The shipped `ur_servo.yaml` doesn't override it, so every
outgoing velocity gets multiplied by zero before the position update.
The robot dutifully "moves to where it already is" 250 times a second.

**Fix:** `servo_yaml["override_velocity_scaling_factor"] = 1.0` in
the launch. Verify at runtime with
`ros2 param get /servo_node moveit_servo.override_velocity_scaling_factor`.

### 3. Cartesian commands at a singularity

**Symptom:** Robot at home pose (`shoulder_lift = -π/2, wrist_1 = -π/2,
others = 0`). WASD does nothing visible. Joint jog also produces near-zero
deltas. `/servo_node/collision_velocity_scale` is `1.0` (so it's not a
collision halt).

**Cause:** Home pose puts the wrist directly above the base; the
Jacobian's condition number blows up. Servo's singularity scaling
(controlled by `lower_singularity_threshold`/`hard_stop_singularity_threshold`)
attenuates the joint output toward zero as the threshold is approached.

**Fix:** don't start teleop from home. Move to a non-singular pose first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 ./send_joint_command.py ready
```

If you drift back into a singularity during teleop, the EE will go
unresponsive. Use the per-joint jog (`1`–`6` + `+`/`-`) to escape — that
path bypasses cartesian IK so the singularity scaling is less harsh.

### 4. Servo drift loop when idle output isn't suppressed

**Symptom:** Robot moves without any keypress. With Servo started and no
input, you see `/forward_position_controller/commands` publishing at
~250 Hz with slowly-drifting joint values. Left alone long enough, the
arm drifts into a singular configuration, Servo emits `.nan` for every
joint target, the joint drives in Isaac chase NaN with no limit
enforcement, and the wrist joints wrap many full turns (we saw
`wrist_2 = 177 rad`). The keyboard is innocent — `delta_twist_cmds` is
silent the entire time.

**Cause:** with idle suppression off (the upstream default in
`ur_servo.yaml`), Servo runs a timer at `publish_period` (4 ms) that
fires regardless of whether any input arrived. Each tick computes
`next = low_pass_filter(current_state + delta)` and publishes it. When
there's no input, `delta = 0`, so `next = filter(current_state)`. The
filter lags actual state slightly, the drives chase the lag, current
state drifts, the filter follows the drift — positive-feedback runaway
that eventually crosses a singularity threshold and produces NaN.

**Fix:** silence must mean an idle robot. The two Servo generations spell
that differently, and `_servo_compat.apply_idle_protection()` picks
whichever the installed one declares:

| | Humble (pre-rewrite Servo) | Jazzy (MoveIt 2.12+) |
|---|---|---|
| Mechanism | `low_latency_mode: true` | `incoming_command_timeout: 0.1` + `halt_all_joints_in_cartesian_mode` + `halt_all_joints_in_joint_mode` |
| Behaviour | Publishes only in response to an incoming `Twist`/`JointJog`, emits halt frames after `incoming_command_timeout: 0.1 s`, then goes silent. | The rewritten Servo halts rather than republishing once input goes quiet; the halt flags make it stop *every* joint when the timeout fires. |

Either way: no keypress = no message = the action graph holds its last
received target and the robot stops.

`low_latency_mode` does not exist in `moveit_servo` 2.12+, and the
`halt_all_joints_*` flags do not exist on Humble — so setting the wrong pair is
**silent**, since undeclared parameters are ignored without a warning. That is
why `apply_idle_protection()` raises rather than falling through to a default
when it cannot establish which mechanism applies: an unprotected Servo launches
perfectly happily and only misbehaves once you leave it idle. The
`isaac-teleop-launch-smoke` CI job asserts exactly one mechanism ends up set, on
both distros.

Note `ur_moveit_config` ships an `ur_servo.yaml` containing **both**
`low_latency_mode` and `incoming_command_timeout` on Jazzy, so the config file
is no guide to which Servo you have — only the parameters `moveit_servo`
declares are.

**Defense in depth:** the bridge (`servo_to_isaac_bridge.py`) validates
every value against ±2π / per-joint limits before forwarding to
`/joint_command` and drops messages containing `NaN`/`inf`/out-of-range
positions. Drop count is logged every 50 drops. If you ever see those
warnings, you've hit a Servo NaN bug or a singularity edge case — the
action graph won't act on garbage but you'll want to investigate.

## Running in a container

Entirely optional — the stack runs natively on both distros, and that is the
normal way to use it. The container is for:

- trying the other distro without touching your workstation (a Jazzy machine
  reproducing a Humble bug, or the reverse);
- a clean room when you suspect your local ROS install rather than the code.

Isaac Sim always stays on the host: it needs the GPU, and the container only
talks to it over ROS topics.

```bash
./run.sh                     # full stack on Jazzy (default)
./run.sh --distro humble     # ...on Humble instead
./run.sh --servo-only        # Servo half only; drive it from elsewhere
./run.sh --keyboard          # Servo half, then the keyboard frontend
./run.sh --smoke             # smoke_launch_check.py; no Isaac or GPU needed
./run.sh -- bash             # poke around inside
```

`run.sh` builds the image on first use (one per distro) and re-uses it after.
This folder is bind-mounted at `/workspace`, so editing a teleop script doesn't
need a rebuild.

Two settings it applies that matter, both the same as
[`resources/gello/run.sh`](../../gello/run.sh):

- `--network host`, so container and host nodes discover each other.
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. FastDDS's default shared-memory transport
  **silently** drops container↔host traffic, because the container runs as root
  and the host-side Isaac process doesn't. Symptom: `ros2 topic list` shows the
  topics but `ros2 topic echo` never prints.

**Haptics doesn't work from the container.** `teleop.launch.py` includes
`haptics.launch.py`, which needs `pydualsense`/`hidapi` and raw hidraw access to
the DualSense; the image ships neither and `run.sh` passes no hidraw device. The
include is best-effort, so the rest of the stack comes up normally and the
haptic node just logs and gives up. Run the haptic loop natively if you want it.

Export `ROS_DOMAIN_ID` before calling `run.sh` if you use a non-default one;
it's passed through. The gamepad arrives via `--device /dev/input/js0`
(override with `JOY_DEV=`), and is optional for every mode but the default.

The image's package list has to stay in step with the
`isaac-teleop-launch-smoke` CI job, which installs the same set directly.

## Day-to-day usage

Native install. (From a container, `run.sh` runs these launches for you.)

Commands below assume you're **cd'd into this `teleop/` folder** of this
resource. That keeps launch paths short. Alternatively set
`export ISAAC_SIM_TELEOP=/path/to/grippers/demo/teleop` in your shell
rc.

**Terminal A — bring up the stack:**

```bash
# clean shell, then:
source /opt/ros/$ROS_DISTRO/setup.bash
cd .../grippers/demo/teleop
ros2 launch ur5e_servo.launch.py
```

You should see four processes start. Within ~5 seconds:

- `/robot_state_publisher` listing URDF segments
- `/servo_node` reporting `Loaded robot model in …`
- `/joint_states_arm_filter` quietly running
- `/servo_to_isaac_bridge` logging
  `bridging /forward_position_controller/commands -> /joint_command`

No error messages about unknown joints — if you see those, the arm
filter isn't reaching Servo (check that `joint_topic` is set to
`/joint_states_arm`).

**Get the robot off home (one-time per session):**

```bash
python3 ./send_joint_command.py ready
```

**Terminal B — pick one frontend:**

### Gamepad (recommended: DualShock 4 / DualSense)

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch ./gamepad.launch.py
```

Starts `joy_node` at 30 Hz autorepeat and `gamepad_teleop`. Waits for sim
time, calls `/servo_node/start_servo`, then prints the mapping. Default:

| Control | Action |
|---|---|
| **Right stick** | linear x/y (tool0; x sign flipped for intuitive push‑forward = +x) |
| **D-pad up/down** | linear -z / +z |
| **Left stick** | angular roll / pitch |
| **L1 / R1** | yaw − / yaw + |
| **R2 (analog)** | gripper (released = open → fully pressed = closed) |
| **Triangle (△)** | speed × 1.25 |
| **Cross (×)** | speed ÷ 1.25 |
| **Circle (○)** | resync servo (stop→start + zero-twist warmup) — use after Stop→Play |
| **Options (☰)** | go to ready pose |
| **PS / Share** | quit |

Axis/button assignments live at the top of `gamepad_teleop.py`; change
them if your driver reports different indices. Run `keydump.py` to see
raw keycodes from any input, or `ros2 topic echo /joy` for gamepad axes.

### Keyboard

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 ./servo_keyboard_teleop.py \
        --ros-args -p use_sim_time:=true
```

On startup it waits for sim time to populate, calls
`/servo_node/start_servo`, and prints the control reference.

| Keys      | Action                                              |
| --------- | --------------------------------------------------- |
| `A`/`D`   | linear ±x (tool0)                                   |
| `W`/`X`   | linear ±y                                           |
| `↑`/`↓`   | linear ±z                                           |
| `I`/`K`   | rotate around +y / -y                               |
| `J`/`L`   | rotate around +z / -z                               |
| `U`/`O`   | rotate around +x / -x                               |
| `1`–`6`   | select arm joint                                    |
| `+`/`-`   | jog selected joint                                  |
| `→` / `←` | open / close gripper                                |
| `h`       | go to ready pose                                    |
| `q` / `^C`| quit (restores terminal tty)                        |

The teleop sends one Twist per keypress. For continuous motion, hold
the key down — your terminal's key-repeat (~30 Hz typical) is well
inside Servo's `incoming_command_timeout: 0.1 s`.

**Buttons that call Servo services** (Circle resync, Options ready-pose) are
recorded in `on_joy` and executed by the main loop between spins, not inside
the callback: a node can't be spun reentrantly, so a service call made from
within `on_joy` would sit unanswered until it timed out.

**Switching between cartesian and per-joint jog:** Jazzy's Servo accepts exactly
one command type at a time and starts with none selected, rejecting everything
with `Command type has not been set, cannot accept input`. The frontend handles
this for you — moving between the twist keys and `+`/`-` calls
`/servo_node/switch_command_type` on the transition (and only on the
transition). Humble's Servo took both types at any time, so nothing is called
there. If you drive Servo from your own publisher on Jazzy, you have to make
that call yourself; `set_servo_command_type.py [twist|joint_jog|pose]` does it.

## Gripper path — why it doesn't fight Servo

The gripper goes around Servo entirely:

- Servo only knows about the UR5e arm (URDF has no 2F-85 segments).
- The keyboard/gamepad script's `send_gripper()` builds a `JointState`
  listing only the driven **`finger_joint`** and publishes it to
  `/joint_command` directly.
- Servo continues to publish `Float64MultiArray` → bridge → `/joint_command`
  with only the 6 arm joints.
- The Isaac Action Graph's `SubscribeJointState → ArticulationController`
  applies each incoming message to **only** the joints it names. Arm and
  gripper messages list disjoint joint sets, so they update independently.

We command **only `finger_joint`** (not the other finger DOFs). Under
the 2F-85 Physx_Loop variant, the four-bar loop-closure constraint
solves for the remaining finger positions automatically. Commanding
more would fight the loop solver. See [PHYSICS_TUNING.md](PHYSICS_TUNING.md)
for details on the Loop vs Mimic variant choice.

## Clean shutdown

`q` in Terminal B exits cleanly (the `KeyReader` context manager
restores cooked tty mode). `Ctrl-C` in Terminal A kills the four
managed nodes. If something hangs:

```bash
pkill -f "servo_to_isaac_bridge|robot_state_publisher|servo_node_main|joint_states_arm_filter|ros2 launch.*ur5e_servo"
```

Beware that this also kills any *other* `ros2 launch` you have going.

## Fallback: `keyboard_jog.py`

If MoveIt Servo isn't cooperating (extension didn't load, parameter
regression in a future Humble package, sim-time drama), use the
fallback:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 ./keyboard_jog.py
```

It publishes a 7-joint `JointState` (6 arm + `finger_joint`) to
`/joint_command` at 30 Hz, holding the latest target and nudging by
`±0.02 rad` per keypress. No IK, no Servo, no URDF — works wherever the
Action Graph subscriber is alive. `1`–`6` + `7` joint select, `+`/`-`
nudge, `o`/`c` open/close gripper, `h` returns to home pose.

The trade-off: per-joint only, no cartesian motion. Useful for
verifying the Action Graph + bridge still work end-to-end without
introducing Servo as a variable.

## Diagnostic cheat sheet

When teleop stops behaving, check in this order:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# 1. Are the four launch nodes alive?
ros2 node list | grep -E "servo_node|servo_to_isaac_bridge|joint_states_arm_filter|robot_state_publisher"

# 2. Is Servo started?
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

# 3. Is Servo actually publishing?
ros2 topic hz /forward_position_controller/commands     # expect ~250 Hz

# 4. Are stamps in sim time, not wall time?
ros2 topic echo /joint_states_arm --field header.stamp --once   # sec should be small (~thousands)

# 5. Singularity scaling clamping us?
ros2 topic echo /servo_node/collision_velocity_scale --once     # 1.0 = no scaling

# 6. Stray twist publishers from old runs?
ros2 topic info /servo_node/delta_twist_cmds                    # publisher_count should be 1

# 7. Anything in the Isaac console (via MCP bridge)?
#    Look for "no DOFs found", "Joint X not found", etc.
```

## Building a compatible stage

For the ROS bridge to work, the USD stage must contain:

1. **A UR5e articulation with a Robotiq 2F-85 gripper**, either as one
   combined articulation or with the gripper mounted on the arm's tool
   flange. NVIDIA ships the individual assets on the Isaac Sim asset
   library (`Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5e/` and
   `Assets/Isaac/6.0/Isaac/Robots/Robotiq/2F-85/`).

2. **The articulation root prim** — the prim with
   `PhysicsArticulationRootAPI` applied. On the NVIDIA UR assets this is
   typically `<parent>/root_joint`, a `PhysicsFixedJoint` child. Note
   the exact prim path; you'll need it for the Action Graph build script.

3. **A `PhysicsScene` prim** somewhere at the root (usually `/PhysicsScene`
   auto-added by Isaac).

4. **The 2F-85 in its `Physx_Loop` variant** — the loop-closure variant
   is what the rest of this doc assumes. Set via the `Physics` variant
   set on the gripper's edit prim. See PHYSICS_TUNING.md for why this
   matters vs the `Physx_Mimic` variant.

5. **Enough joint drive on `finger_joint`** to actually close on things.
   The NVIDIA asset defaults are very soft (`stiffness=0.17, damping=2e-4`).
   PHYSICS_TUNING.md documents the values we ended up with.

Once the stage is loaded and simulation is playing, run
`../isaac_scripts/build_ros_action_graph.py` (edit `ARTICULATION_PRIM` at
the top to match your root prim path) via `Window → Script Editor`. This
creates the `/World/ROS_JointControl` OmniGraph. Verify with
`ros2 topic list` — `/joint_states`, `/joint_command`, `/clock` should
all appear.
