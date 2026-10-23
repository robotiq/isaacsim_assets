#!/usr/bin/env bash
# One command to run the UR5e teleop demo on Linux. No ROS.
#
# Starts Isaac Sim with the scene open and playing, and physx_tick_teleop.py
# driving the arm from inside Isaac (DualSense gamepad or keyboard). Then,
# with a DualSense, the haptics: the gripper contact-force plot and the R2
# trigger force feedback.
#
#   ./run_demo.sh                 # start everything
#   ./run_demo.sh --keyboard      # drive with the keyboard even with a gamepad plugged in
#   ./run_demo.sh --no-isaac      # attach the haptics to an Isaac already running
#
#   TELEOP_GRIPPER=2F140 ./run_demo.sh                 # the 2F-140 (default 2F85)
#   TELEOP_GRIPPER=HandE ./run_demo.sh                 # the Hand-E (PhysX / Newton)
#   TELEOP_PHYSICS=Newton_compliant ./run_demo.sh      # the Newton gripper
#   TELEOP_GRIPPER=2F140 TELEOP_PHYSICS=Newton_parallel_grip ./run_demo.sh
#   TELEOP_SENSORSIM=EffectiveResponse ./run_demo.sh   # the 2F-85 with the TSF-85 tactile sensor
#                                                      # (Raycast | EffectiveResponse) and its live heatmap
#   TELEOP_HAPTICS=0 ./run_demo.sh                     # no plot, no R2 rumble
#   TELEOP_INPUT=keyboard ./run_demo.sh                # same as --keyboard
#   TELEOP_FULL_FORCE_N=100 ./run_demo.sh              # R2 at full stiffness from 100 N (default 200)
#   READY_TIMEOUT=600 ./run_demo.sh                    # wait longer for Isaac (default 240 s)
#   MCP_EXT_ROOT=/path/to/isaacsim-mcp-server ./run_demo.sh   # MCP extension elsewhere
#
# READY_TIMEOUT counts from the launch, including any time the launcher spends
# before Isaac starts. MCP_EXT_ROOT defaults to ../mcp_bridge/isaacsim-mcp-server.
#
# TELEOP_GRIPPER picks the scene's Gripper variant (2F85 | 2F140 | HandE) and
# TELEOP_PHYSICS its Physics variant (Physx_compliant | Physx_parallel_grip |
# Newton_compliant | Newton_parallel_grip, default Physx_compliant). The Hand-E has
# one PhysX and one Newton variant, so any Physx_* / Newton_* maps onto those. The
# physics picks the launcher, because the Newton_* physics only simulates under the
# Newton solver, and that is a separate Isaac experience rather than a flag. Set
# ISAACSIM_LAUNCHER to override it.
#
# Extra Kit arguments are forwarded to Isaac. Anything of the form --/a/b=c is
# recognised as a Kit setting and passed straight through; use -- before
# anything else. They land ahead of --exec, which is greedy and must stay last:
#   ./run_demo.sh --/app/profilerBackend=cpu
#
# Benchmarking: TELEOP_TCPS=30 TELEOP_PHYSX_HZ=60 ./run_demo.sh applies the
# timing before play, so no stop -> play cycle is needed (see open_scene.py).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim}"
# The scene used to have one combined Gripper variant (physx | newton, both the
# 2F-85); keep the old TELEOP_GRIPPER_VARIANT working by mapping it onto the
# two variant sets.
if [[ -n "${TELEOP_GRIPPER_VARIANT:-}" ]]; then
  case "$TELEOP_GRIPPER_VARIANT" in
    physx)  TELEOP_PHYSICS="${TELEOP_PHYSICS:-Physx_compliant}" ;;
    newton) TELEOP_PHYSICS="${TELEOP_PHYSICS:-Newton_compliant}" ;;
  esac
  TELEOP_GRIPPER="${TELEOP_GRIPPER:-2F85}"
  echo "Note: TELEOP_GRIPPER_VARIANT is deprecated; using TELEOP_GRIPPER=$TELEOP_GRIPPER TELEOP_PHYSICS=$TELEOP_PHYSICS"
fi
TELEOP_GRIPPER="${TELEOP_GRIPPER:-2F85}"
TELEOP_PHYSICS="${TELEOP_PHYSICS:-Physx_compliant}"
export TELEOP_GRIPPER TELEOP_PHYSICS
# The Physics variant and the physics engine are two halves of one choice, so
# the launcher is derived from the Physics variant. An explicit
# ISAACSIM_LAUNCHER still wins.
case "$TELEOP_PHYSICS" in
  Newton_*) VARIANT_LAUNCHER=isaac-sim.newton.sh ;;
  *)        VARIANT_LAUNCHER=isaac-sim.sh ;;
esac
if [[ -n "${ISAACSIM_LAUNCHER:-}" ]]; then LAUNCHER_SRC="explicit"
else LAUNCHER_SRC="from Physics '$TELEOP_PHYSICS'"; fi
ISAACSIM_LAUNCHER="${ISAACSIM_LAUNCHER:-$VARIANT_LAUNCHER}"
LAUNCHER="$ISAACSIM_ROOT/$ISAACSIM_LAUNCHER"

# The haptics inject the gripper contact-force plot through the MCP extension's
# TCP socket on 8766, and THAT injection is what starts the UDP 8770 force
# broadcast the R2 rumble reads. So without the extension there is no plot AND
# no rumble -- Isaac has to be launched with it, not plain. The extension is
# loaded whenever it is present, haptics or not: AI agents and scripts use the
# same socket.
MCP_BRIDGE="$SCRIPT_DIR/../mcp_bridge"
MCP_EXT_ROOT="${MCP_EXT_ROOT:-$MCP_BRIDGE/isaacsim-mcp-server}"
HAPTICS="${TELEOP_HAPTICS:-1}"
# auto: DualSense if one is plugged in, else the keyboard. keyboard: always the
# keyboard. Read by physx_tick_teleop.py inside Isaac.
TELEOP_INPUT="${TELEOP_INPUT:-auto}"

ISAAC_LOG="${ISAAC_LOG:-/tmp/physx_teleop_isaac.log}"
READY_TIMEOUT="${READY_TIMEOUT:-240}"       # Isaac's first boot is ~90-150s

START_ISAAC=1
ISAAC_EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --keyboard)    TELEOP_INPUT=keyboard; shift ;;
    --no-isaac)    START_ISAAC=0; shift ;;
    -h|--help)     sed -n '2,/^[^#]/{/^#/s/^# \{0,1\}//p}' "${BASH_SOURCE[0]}"; exit 0 ;;
    --)            shift; ISAAC_EXTRA=("$@"); break ;;
    # Kit settings all look like --/path/to/key=value, so they are unambiguous.
    --/*)          ISAAC_EXTRA+=("$1"); shift ;;
    *) echo "unknown argument: $1" >&2
       echo "(Kit settings --/a/b=c are forwarded; use -- for anything else)" >&2
       exit 2 ;;
  esac
done

if [[ "$START_ISAAC" == "1" && ! -x "$LAUNCHER" ]]; then
  echo "Error: Isaac Sim launcher not found at: $LAUNCHER" >&2
  exit 1
fi
# The input is chosen inside Isaac when the teleop starts, so it can't be
# changed for an Isaac that is already running.
if [[ "$START_ISAAC" == "0" && "$TELEOP_INPUT" == "keyboard" ]]; then
  echo "Note:       --keyboard / TELEOP_INPUT=keyboard has no effect with --no-isaac;"
  echo "            the running Isaac keeps the input it picked at start."
  TELEOP_INPUT=auto
fi

PIDS=()
cleanup() {
  local st=$?
  trap - EXIT INT TERM
  echo
  echo "Shutting down..."
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

echo "Isaac Sim:  $LAUNCHER  ($LAUNCHER_SRC)"
echo "Gripper:    $TELEOP_GRIPPER"
echo "Physics:    $TELEOP_PHYSICS"
echo "Input:      $TELEOP_INPUT"
echo "Scene:      $SCRIPT_DIR/ur5robot_with_2F-85.usda"
if [[ "$TELEOP_PHYSICS" == Newton_* && "$ISAACSIM_LAUNCHER" != *newton* ]]; then
  echo "Note:       Physics '$TELEOP_PHYSICS' with a launcher that is not the Newton"
  echo "               experience -- the gripper will not simulate."
fi

# shellcheck source=../cpu_governor_check.sh
source "$SCRIPT_DIR/../cpu_governor_check.sh"

if [[ "$START_ISAAC" == "1" ]]; then
  if [[ -d "$MCP_EXT_ROOT/isaac.sim.mcp_extension" ]]; then
    ISAAC_CMD=("$MCP_BRIDGE/launch_isaac_with_mcp.sh")
    echo "MCP ext:    $MCP_EXT_ROOT"
  else
    ISAAC_CMD=("$LAUNCHER")
    if [[ "$HAPTICS" != "0" ]]; then
      echo "MCP ext:    NOT FOUND at $MCP_EXT_ROOT"
      echo "            -> no contact-force plot and no R2 rumble (haptics needs"
      echo "               the extension's TCP 8766). See mcp_bridge/README.md,"
      echo "               or set MCP_EXT_ROOT."
      HAPTICS=0
    fi
  fi
  echo "Starting Isaac (log: $ISAAC_LOG) -- first boot takes ~90-150s..."
  # Empty the log HERE, before the wait below starts reading it: the subshell
  # only truncates it when it starts, so a "ready." from the previous run
  # could otherwise be matched.
  : >"$ISAAC_LOG"
  (
    export TELEOP_DIR="$SCRIPT_DIR" TELEOP_TICK=1
    export MCP_EXT_ROOT ISAACSIM_ROOT ISAACSIM_LAUNCHER TELEOP_GRIPPER TELEOP_PHYSICS TELEOP_INPUT
    # --exec is greedy, so it goes last. launch_isaac_with_mcp.sh forwards
    # trailing arguments to the Isaac launcher unchanged.
    exec "${ISAAC_CMD[@]}" "${ISAAC_EXTRA[@]}" --exec "$SCRIPT_DIR/open_scene.py"
  ) >"$ISAAC_LOG" 2>&1 &
  PIDS+=("$!")

  # open_scene.py prints "ready." once the scene is open, playing and the
  # teleop is running.
  echo -n "Waiting for the scene "
  deadline=$(( SECONDS + READY_TIMEOUT ))
  until grep -q -E "\[physx-autostart\] (ready\.|FAILED|TIMED OUT|teleop did NOT)" "$ISAAC_LOG" 2>/dev/null; do
    if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
      echo; echo "Error: Isaac exited. Last lines of $ISAAC_LOG:" >&2
      tail -20 "$ISAAC_LOG" >&2; exit 1
    fi
    if (( SECONDS > deadline )); then
      echo; echo "Error: the scene was not ready after ${READY_TIMEOUT}s; see $ISAAC_LOG." >&2
      echo "Isaac is left running (READY_TIMEOUT sets the wait). Ctrl-C stops it." >&2
      while true; do sleep 3600; done
    fi
    echo -n "."
    sleep 2
  done
  # On a failed start, keep Isaac open: the error is on screen and the teleop
  # can be retried from the Script Editor.
  if ! grep -q "\[physx-autostart\] ready\." "$ISAAC_LOG"; then
    echo; echo "Error: the scene did not start. Last lines of $ISAAC_LOG:" >&2
    tail -20 "$ISAAC_LOG" >&2
    echo "Isaac is left running so you can read the error. Ctrl-C stops it." >&2
    while true; do sleep 3600; done
  fi
  echo " ok"
else
  echo "Skipping Isaac launch (--no-isaac)"
fi

if [[ "$HAPTICS" == "0" ]]; then
  echo "Teleop runs inside Isaac. Haptics off. Ctrl-C stops everything."
  while true; do sleep 3600; done
fi

# Haptics: inject the contact-force plot (one-shot; it also starts the UDP force
# stream), then drive the DualSense R2 from that stream. Neither stops the demo
# when it fails -- the teleop runs inside Isaac regardless.
echo "Teleop runs inside Isaac; starting haptics (plot + R2 rumble). Ctrl-C stops everything."

# "ready." comes from open_scene.py, not from the extension, so the socket can
# still be coming up. Without it the plot is not injected and the UDP force
# stream never starts, so warn rather than fail silently.
echo -n "Waiting for the MCP socket on 8766 "
MCP_UP=0
for _ in $(seq 30); do
  if (exec 3<>/dev/tcp/127.0.0.1/8766) 2>/dev/null; then
    MCP_UP=1; break
  fi
  echo -n "."
  sleep 1
done
if [[ "$MCP_UP" == "1" ]]; then
  echo " ok"
  python3 "$MCP_BRIDGE/make_contact_plot.py" \
    || echo "WARNING: contact-force plot not injected; see $ISAAC_LOG." >&2
  # The tactile heatmap, when the scene has a sensor (TELEOP_SENSORSIM); does nothing otherwise.
  python3 "$MCP_BRIDGE/make_tactile_sensor_plot.py" --if-present \
    || echo "WARNING: tactile heatmap not injected; see $ISAAC_LOG." >&2
else
  echo " NOT UP"
  echo "WARNING: no MCP socket on 8766 -- the contact-force plot will not be" >&2
  echo "         injected and the R2 rumble will stay silent. Check" >&2
  echo "         $ISAAC_LOG for isaac.sim.mcp_extension errors." >&2
fi

# With the keyboard, R2 doesn't drive the gripper, so it gets no resistance
# either; the plot stays.
if [[ "$TELEOP_INPUT" == "keyboard" ]]; then
  echo "Keyboard input: no R2 force feedback."
  while true; do sleep 3600; done
fi
# Ctrl-C (130) or a SIGTERM (143) is the user stopping the demo, not a failure,
# so it gets no warning.
st=0
python3 "$SCRIPT_DIR/trigger_force_feedback.py" --full-force-N "${TELEOP_FULL_FORCE_N:-200}" || st=$?
if (( st != 0 && st != 130 && st != 143 )); then
  echo "WARNING: R2 force feedback stopped (needs a DualSense, pydualsense and hidapi)." >&2
fi
while true; do sleep 3600; done
