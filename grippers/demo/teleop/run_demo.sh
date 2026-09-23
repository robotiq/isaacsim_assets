#!/usr/bin/env bash
# One command to run the PhysX UR5e + 2F-85 teleop demo.
#
# Brings up both halves the README walks through by hand: Isaac Sim with the
# scene open and playing, then `ros2 launch teleop.launch.py` for the ROS side
# (servo now, gamepad at +10s, haptics at +12s). The staging lives in the launch
# file, not here.
#
#   ./run_demo.sh                 # tick-paced teleop inside Isaac (default)
#   ./run_demo.sh --servo         # the old MoveIt Servo stack + gamepad
#   ./run_demo.sh --keyboard      # Servo stack + keyboard frontend
#   ./run_demo.sh --no-frontend   # Isaac + Servo running, drive it yourself
#   ./run_demo.sh --no-isaac      # attach to an Isaac you already have running
#
# THE DEFAULT NO LONGER USES ROS FOR CONTROL. physx_tick_teleop.py runs inside
# Isaac on Kit's update tick, so controller, physics and clock share one
# timebase. MoveIt Servo is paced by a WALL clock, and against a sim at RTF
# ~0.45 that produced a ~3.7 Hz limit cycle -- the arm shook about three times
# harder than it advanced (measured: 1.9% of motion energy at 3-5 Hz with
# Servo, 0.0% without). --servo restores the old path for comparison.
#
# Extra Kit arguments are forwarded to Isaac. Anything of the form --/a/b=c is
# recognised as a Kit setting and passed straight through; use -- before
# anything else. They land ahead of --exec, which is greedy and must stay last:
#   ./run_demo.sh --no-frontend --/app/profilerBackend=cpu
#
# Benchmarking (see measure_rtf.py):
#   TELEOP_TCPS=30 TELEOP_PHYSX_HZ=60 ./run_demo.sh --no-frontend
# applies the timing before play, so no stop -> play cycle is needed.
#
#   TELEOP_HAPTICS=0 ./run_demo.sh
# drops the contact-force plot and R2 rumble. The plot redraws an
# omni.ui Plot every frame INSIDE Isaac, so it is the first thing to
# bisect when frames are scarce.
# applies the timing before play, so no stop -> play cycle is needed.
#
# WHY THIS SCRIPT EXISTS RATHER THAN THREE TERMINALS
# --------------------------------------------------
# The two halves need incompatible environments. Isaac must run from a shell
# where /opt/ros has NOT been sourced -- its setup_ros_env.sh only wires up the
# bundled Jazzy when ROS_DISTRO is unset, and a system ROS Python collides with
# Isaac's bundled interpreter. The ROS half obviously needs the opposite. So
# this script stays clean itself, launches Isaac from its own environment, and
# sources ROS only inside the subshells that need it.
#
# Both halves source isaac-demo-dds.sh. That is not cosmetic here: MoveIt Servo
# publishes to /forward_position_controller/commands, which on the shared office
# graph already had two REMOTE subscribers on live robot drivers. This is the
# stack that hazard is about.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DDS_FILE="$SCRIPT_DIR/../isaac-demo-dds.sh"

ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim}"
ISAACSIM_LAUNCHER="${ISAACSIM_LAUNCHER:-isaac-sim.sh}"      # PhysX, not Newton
LAUNCHER="$ISAACSIM_ROOT/$ISAACSIM_LAUNCHER"
ROS_SETUP="/opt/ros/${TELEOP_ROS_DISTRO:-jazzy}/setup.bash"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

# haptics.launch.py (pulled in by teleop.launch.py at +12s) injects the gripper
# contact-force plot through the MCP extension's TCP socket on 8766, and THAT
# injection is what starts the UDP 8770 force broadcast the R2 rumble reads. So
# without the extension there is no plot AND no rumble -- Isaac has to be
# launched with it, not plain.
MCP_BRIDGE="$SCRIPT_DIR/../mcp_bridge"
MCP_EXT_ROOT="${MCP_EXT_ROOT:-$MCP_BRIDGE/isaacsim-mcp-server}"

ISAAC_LOG="${ISAAC_LOG:-/tmp/physx_teleop_isaac.log}"
SERVO_LOG="${SERVO_LOG:-/tmp/physx_teleop_servo.log}"

CLOCK_TIMEOUT="${CLOCK_TIMEOUT:-240}"       # Isaac's first boot is ~90-150s
SERVO_TIMEOUT="${SERVO_TIMEOUT:-60}"

FRONTEND=tick
START_ISAAC=1
ISAAC_EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --servo)       FRONTEND=gamepad; shift ;;
    --keyboard)    FRONTEND=keyboard; shift ;;
    --no-frontend) FRONTEND=none; shift ;;
    --no-isaac)    START_ISAAC=0; shift ;;
    -h|--help)     sed -n '2,16p' "${BASH_SOURCE[0]}"; exit 0 ;;
    --)            shift; ISAAC_EXTRA=("$@"); break ;;
    # Anything Kit-shaped is forwarded rather than rejected: Kit settings all
    # look like --/path/to/key=value, so they are unambiguous and it is a
    # nuisance to need `--` for them.
    --/*)          ISAAC_EXTRA+=("$1"); shift ;;
    *) echo "unknown argument: $1" >&2
       echo "(Kit settings --/a/b=c are forwarded; use -- for anything else)" >&2
       exit 2 ;;
  esac
done

# This shell must stay clean, because Isaac inherits it.
if [[ -n "${ROS_DISTRO:-}" ]]; then
  echo "Error: ROS_DISTRO=$ROS_DISTRO is set -- run this from a CLEAN shell." >&2
  echo "Isaac needs /opt/ros NOT sourced; this script sources it where needed." >&2
  exit 1
fi
for f in "$DDS_FILE" "$ROS_SETUP" "$SCRIPT_DIR/ur5e_servo.launch.py"; do
  [[ -r "$f" ]] || { echo "Error: missing $f" >&2; exit 1; }
done
if [[ "$START_ISAAC" == "1" && ! -x "$LAUNCHER" ]]; then
  echo "Error: Isaac Sim launcher not found at: $LAUNCHER" >&2
  exit 1
fi
if [[ "$FRONTEND" == "gamepad" && ! -e "$JOY_DEV" ]]; then
  echo "Error: no gamepad at $JOY_DEV -- use ./run_demo.sh --keyboard" >&2
  exit 1
fi
# The tick teleop reads the pad itself and falls back to the keyboard, so a
# missing gamepad is a note rather than an error.
if [[ "$FRONTEND" == "tick" && ! -e "$JOY_DEV" ]]; then
  echo "Note:       no gamepad at $JOY_DEV -- teleop will use the keyboard"
fi

PIDS=()
cleanup() {
  local st=$?
  trap - EXIT INT TERM
  echo
  echo "Shutting down..."
  # Reverse order, SIGINT first: ros2 launch only tears its children down
  # cleanly on SIGINT, and SIGKILL would orphan the nodes it spawned.
  for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
    kill -INT "${PIDS[i]}" 2>/dev/null || true
  done
  for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
    for _ in $(seq 20); do
      kill -0 "${PIDS[i]}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL "${PIDS[i]}" 2>/dev/null || true
  done
  exit "$st"
}
trap cleanup EXIT INT TERM

# Run a command with ROS + DDS sourced. `set +u` because ROS's setup.bash
# trips over unbound variables under `set -u`.
ros_env() {
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  source "$DDS_FILE"
  set -u
  "$@"
}

echo "Isaac Sim:  $LAUNCHER"
echo "Scene:      $SCRIPT_DIR/ur5robot_with_2F-85.usda"
echo "ROS:        $ROS_SETUP"
echo "Frontend:   $FRONTEND"

# shellcheck source=../cpu_governor_check.sh
source "$SCRIPT_DIR/../cpu_governor_check.sh"

if [[ "$START_ISAAC" == "1" ]]; then
  if [[ -d "$MCP_EXT_ROOT/isaac.sim.mcp_extension" ]]; then
    ISAAC_CMD=("$MCP_BRIDGE/launch_isaac_with_mcp.sh")
    MCP=1
    echo "MCP ext:    $MCP_EXT_ROOT"
  else
    ISAAC_CMD=("$LAUNCHER")
    MCP=0
    echo "MCP ext:    NOT FOUND at $MCP_EXT_ROOT"
    echo "            -> no contact-force plot and no R2 rumble (haptics needs"
    echo "               the extension's TCP 8766). See mcp_bridge/README.md,"
    echo "               or set MCP_EXT_ROOT."
  fi
  echo "Starting Isaac (log: $ISAAC_LOG) -- first boot takes ~90-150s..."
  (
    set +u
    source "$DDS_FILE"
    set -u
    export TELEOP_DIR="$SCRIPT_DIR"
    # Only the default path wants the in-Isaac teleop; the Servo paths drive
    # the same articulation from ROS and must not fight it.
    if [[ "$FRONTEND" == "tick" ]]; then export TELEOP_TICK=1
    else export TELEOP_TICK=0; fi
    export MCP_EXT_ROOT ISAACSIM_ROOT ISAACSIM_LAUNCHER
    # --exec is greedy, so it goes last. launch_isaac_with_mcp.sh forwards
    # trailing arguments to the Isaac launcher unchanged.
    exec "${ISAAC_CMD[@]}" "${ISAAC_EXTRA[@]}" --exec "$SCRIPT_DIR/open_scene.py"
  ) >"$ISAAC_LOG" 2>&1 &
  PIDS+=("$!")
else
  echo "Skipping Isaac launch (--no-isaac)"
fi

# /clock only ticks once the timeline is PLAYING, so this single check proves
# Isaac is up, the scene is open and physics is running -- not just that a
# process exists.
echo -n "Waiting for Isaac's /clock "
deadline=$(( SECONDS + CLOCK_TIMEOUT ))
until ros_env timeout 5 ros2 topic echo /clock --once >/dev/null 2>&1; do
  if [[ "$START_ISAAC" == "1" ]] && ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    echo; echo "Error: Isaac exited. Last lines of $ISAAC_LOG:" >&2
    tail -20 "$ISAAC_LOG" >&2; exit 1
  fi
  if (( SECONDS > deadline )); then
    echo; echo "Error: no /clock after ${CLOCK_TIMEOUT}s." >&2
    echo "Isaac may be up but not PLAYING -- check $ISAAC_LOG for" >&2
    echo "'[physx-autostart]' lines." >&2
    exit 1
  fi
  echo -n "."
  sleep 2
done
echo " ok"

# haptics fires at +12s and needs this socket; a warning now beats a silent
# missing plot later. Non-fatal: the teleop itself does not use MCP.
if [[ "${MCP:-0}" == "1" ]]; then
  echo -n "Waiting for the MCP socket on 8766 "
  for _ in $(seq 30); do
    if (exec 3<>/dev/tcp/127.0.0.1/8766) 2>/dev/null; then
      exec 3>&- 2>/dev/null || true
      MCP_UP=1; break
    fi
    echo -n "."
    sleep 1
  done
  if [[ "${MCP_UP:-0}" == "1" ]]; then
    echo " ok"
  else
    echo " NOT UP"
    echo "WARNING: no MCP socket on 8766 -- the contact-force plot will not be" >&2
    echo "         injected and the R2 rumble will stay silent. Check" >&2
    echo "         $ISAAC_LOG for isaac.sim.mcp_extension errors." >&2
  fi
fi

wait_for_servo() {
  local deadline=$(( SECONDS + SERVO_TIMEOUT ))
  echo -n "Waiting for servo_node "
  until ros_env ros2 node list 2>/dev/null | grep -q servo_node; do
    if (( SECONDS > deadline )); then echo " timeout"; return 1; fi
    echo -n "."
    sleep 2
  done
  echo " ok"
  return 0
}

# Servo cannot jog the arm out of its singular home pose, so this is a one-time
# per-session nudge. It runs in the background for the foreground-launch modes:
# teleop.launch.py owns the terminal, and this has to happen after servo_node
# appears but before the gamepad comes up at +10s.
ready_when_servo_up() {
  if wait_for_servo; then
    echo "Moving the arm to 'ready'..."
    ros_env python3 "$SCRIPT_DIR/send_joint_command.py" ready \
      || echo "WARNING: send_joint_command.py ready failed" >&2
  else
    echo "WARNING: servo_node never appeared; skipped the move to 'ready'." >&2
    echo "         See $SERVO_LOG" >&2
  fi
}

case "$FRONTEND" in
  tick)
    # Control lives inside Isaac. The only reason to touch ROS here is haptics:
    # the contact-force plot is injected over MCP and the R2 rumble reads the
    # UDP stream that injection starts. Neither needs Servo.
    if [[ "${TELEOP_HAPTICS:-1}" == "0" ]]; then
      echo "Teleop runs inside Isaac. Haptics disabled. Ctrl-C stops everything."
      while true; do sleep 3600; done
    fi
    echo "Teleop runs inside Isaac; starting haptics only (plot + R2 rumble)."
    echo "Ctrl-C stops everything."
    ros_env ros2 launch "$SCRIPT_DIR/haptics.launch.py"
    ;;
  gamepad)
    # teleop.launch.py IS the full stack: ur5e_servo now, gamepad at +10s,
    # haptics at +12s (contact-force plot + DualSense R2 force feedback).
    # Composing those here in bash instead would duplicate the staging and
    # drop haptics, which is what an earlier version of this script did.
    echo "Starting the full teleop stack (servo + gamepad + haptics)."
    echo "Ctrl-C stops everything."
    ready_when_servo_up &
    PIDS+=("$!")
    ros_env ros2 launch "$SCRIPT_DIR/teleop.launch.py"
    ;;
  keyboard)
    # No gamepad half here, so the servo launch goes to a log and the keyboard
    # frontend takes the terminal -- it needs raw mode.
    echo "Starting Servo stack (log: $SERVO_LOG)..."
    ( ros_env ros2 launch "$SCRIPT_DIR/ur5e_servo.launch.py" ) >"$SERVO_LOG" 2>&1 &
    PIDS+=("$!")
    if ! wait_for_servo; then
      echo "Error: servo_node did not appear. Last lines of $SERVO_LOG:" >&2
      tail -30 "$SERVO_LOG" >&2
      exit 1
    fi
    echo "Moving the arm to 'ready'..."
    ros_env python3 "$SCRIPT_DIR/send_joint_command.py" ready
    echo "Starting keyboard frontend. Ctrl-C stops everything."
    ros_env python3 "$SCRIPT_DIR/servo_keyboard_teleop.py"
    ;;
  none)
    echo "Starting Servo stack only. Ctrl-C stops everything."
    ready_when_servo_up &
    PIDS+=("$!")
    ros_env ros2 launch "$SCRIPT_DIR/ur5e_servo.launch.py"
    ;;
esac
