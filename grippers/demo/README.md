# Isaac Sim teleop demo (UR5e + Robotiq grippers)

Drive a UR5e with a Robotiq gripper (2F-85, 2F-140 or Hand-E) in Isaac Sim 6 with a
gamepad or keyboard. Details, data flow and gotchas: [teleop/README.md](teleop/README.md).

## Prerequisites

- RTX GPU + NVIDIA driver, Isaac Sim 6.0.0 at `~/isaacsim` (override with `ISAACSIM_ROOT`)
- ROS 2 Jazzy (default; `TELEOP_ROS_DISTRO=humble` for Humble):
  ```bash
  sudo apt install ros-jazzy-moveit ros-jazzy-moveit-servo ros-jazzy-ur-description \
                   ros-jazzy-ur-moveit-config ros-jazzy-joy
  ```
- Optional, but **a DualSense gamepad is strongly recommended** for the full experience
  (keyboard works without it). It appears as `/dev/input/js0`, and its R2 trigger gives
  force feedback on grip; that needs `pip install -r teleop/requirements.txt` and
  `libhidapi-hidraw0`. A DualShock 4 drives the arm but has no force feedback.

## Start

From a **clean shell** (no `/opt/ros` sourced — the script sources ROS where needed):

```bash
./teleop/run_demo.sh
```

It starts Isaac with the scene open and playing, then the teleop stack. Isaac's first
boot takes ~90–150 s. Ctrl-C stops everything.

## Options

Flags:

| Flag | Effect |
|---|---|
| *(none)* | Tick-paced teleop inside Isaac (gamepad, keyboard fallback) + haptics. **Default.** |
| `--servo` | Old MoveIt Servo stack + gamepad (needs a gamepad). |
| `--keyboard` | Servo stack + keyboard frontend. |
| `--no-frontend` | Isaac + Servo only; drive it yourself. |
| `--no-isaac` | Attach to an Isaac you already have running. |
| `--/a/b=c` | Forwarded to Isaac as a Kit setting. `--` forwards any other args. |

Environment variables:

| Variable | Values (default first) | Effect |
|---|---|---|
| `TELEOP_GRIPPER` | `2F85`, `2F140`, `HandE` | Gripper variant. |
| `TELEOP_PHYSICS` | `Physx_compliant`, `Physx_parallel_grip`, `Newton_compliant`, `Newton_parallel_grip` | Physics variant. `Newton_*` automatically launches Isaac's Newton experience (`isaac-sim.newton.sh`). Hand-E has one PhysX and one Newton variant. |
| `TELEOP_HAPTICS` | `1`, `0` | `0` drops the contact-force plot and R2 rumble. |
| `MCP_EXT_ROOT` | `mcp_bridge/isaacsim-mcp-server` | MCP extension path. Required for the plot and R2 rumble; see [mcp_bridge/README.md](mcp_bridge/README.md). |
| `ISAACSIM_ROOT` / `ISAACSIM_LAUNCHER` | `~/isaacsim` / derived from physics | Isaac install and launcher override. |
| `TELEOP_ROS_DISTRO` | `jazzy`, `humble` | ROS distro to source. |
| `JOY_DEV` | `/dev/input/js0` | Gamepad device. |

Example: `TELEOP_GRIPPER=2F140 TELEOP_PHYSICS=Newton_compliant ./teleop/run_demo.sh`

## Controls

Gamepad: left stick roll/pitch, right stick linear X/Y, D-pad up/down Z, L1/R1 yaw,
**R2 gripper** (pushes back with grip force on a DualSense), Triangle/Cross speed ×/÷ 1.25,
Circle resync after a Stop→Play, Options ready pose. Keyboard (`--keyboard`): see the
console output.

On a freshly launched Isaac, do one **Stop→Play** once everything is up, or PhysX won't
report pad contacts and the R2 trigger stays slack.

## Troubleshooting

- `Error: ROS_DISTRO is set` — run from a shell that hasn't sourced ROS.
- No R2 rumble or plot — check `MCP_EXT_ROOT` and the DualSense deps above.
- Robot ignores `/joint_command` after reopening the stage in a running Isaac — relaunch Isaac.

Anything else: [teleop/README.md](teleop/README.md). Logs: `/tmp/physx_teleop_isaac.log`.
