#!/usr/bin/env bash
# Convenience wrapper around `docker run` for the Isaac teleop ROS side.
#
# Optional -- the stack runs natively on both distros. This is for trying the
# other one without touching your workstation, or reproducing a distro-specific
# bug. Isaac Sim itself always stays on the host: it needs the GPU, and the
# container only talks to it over ROS topics.
#
# Usage:
#   ./run.sh                        # full stack (Servo + bridge + filter + gamepad)
#   ./run.sh --distro jazzy         # ...on Jazzy instead of the default Humble
#   ./run.sh --servo-only           # Servo half only; drive it from elsewhere
#   ./run.sh --keyboard             # Servo half, then the keyboard frontend
#   ./run.sh --smoke                # run smoke_launch_check.py and exit
#   ./run.sh -- <command>           # anything else inside the container
set -euo pipefail

cd "$(dirname "$0")"

# Default to humble: it matches the host ROS and Isaac Sim's ROS 2 bridge. On
# jazzy, MoveIt Servo receives /joint_states_arm (data confirmed flowing into the
# container) but never accepts it -- "Waiting to receive robot state update"
# forever. That's a jazzy MoveIt state-monitor difference, not a transport issue.
# Use `--distro jazzy` only to reproduce it.
ROS_DISTRO_ARG="${ROS_DISTRO_ARG:-humble}"
if [[ "${1:-}" == "--distro" ]]; then
    ROS_DISTRO_ARG="$2"
    shift 2
fi
case "${ROS_DISTRO_ARG}" in
    humble|jazzy) ;;
    *) echo "unsupported distro '${ROS_DISTRO_ARG}' -- expected humble or jazzy" >&2; exit 2 ;;
esac

IMG="${IMG:-robotiq/isaac_teleop:${ROS_DISTRO_ARG}}"

docker build -q -t "${IMG}" --build-arg "ROS_DISTRO=${ROS_DISTRO_ARG}" -f Dockerfile . >/dev/null

# The gamepad frontend reads /dev/input/js0 (hid-playstation). Absent is fine
# for every mode except the default one, so this never hard-fails here.
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
DEVICE_ARGS=()
if [[ -e "${JOY_DEV}" ]]; then
    # ROS 2 joy_node is SDL-based and needs BOTH device classes:
    #  1. /dev/input/event*  — evdev, so SDL sees the pad at all. Passing only js0
    #     leaves /joy silent (joy_node never opens it, nothing moves). Bind-mount
    #     all of /dev/input and allow the input class (char major 13) via cgroup.
    #  2. /dev/hidraw*  — so SDL's HIDAPI PS5/DS4 driver binds the pad and exposes
    #     the STANDARD "PS5 Controller" axis layout gamepad_teleop is written for.
    #     Without hidraw, SDL falls back to a raw evdev layout with different axis
    #     order/signs -> the sticks map wrong and rest-offset trigger axes creep,
    #     i.e. it moves but incoherently.
    # --group-add by NUMERIC gid (the "input" group name isn't in the osrf/ros
    # image, so `--group-add input` would make `docker run` fail).
    DEVICE_ARGS=(-v /dev/input:/dev/input --device-cgroup-rule 'c 13:* rmw')
    input_gid="$(getent group input | cut -d: -f3)"
    [[ -n "${input_gid}" ]] && DEVICE_ARGS+=(--group-add "${input_gid}")
    for hid in /dev/hidraw*; do
        [[ -e "${hid}" ]] && DEVICE_ARGS+=(--device "${hid}:${hid}")
    done
fi

case "${1:-}" in
    --servo-only)
        shift
        CMD_ARGS=(ros2 launch /workspace/ur5e_servo.launch.py "$@")
        ;;
    --keyboard)
        shift
        # servo_keyboard_teleop.py wants the terminal in raw mode, so it runs in
        # the foreground and the Servo half goes to the background.
        CMD_ARGS=(bash -c '
            ros2 launch /workspace/ur5e_servo.launch.py &
            sleep 10
            exec python3 /workspace/servo_keyboard_teleop.py')
        ;;
    --smoke)
        shift
        # Needs no Isaac Sim and no GPU -- same check the CI job runs.
        CMD_ARGS=(python3 /workspace/smoke_launch_check.py)
        ;;
    --)
        shift
        CMD_ARGS=("$@")
        ;;
    *)
        if [[ ! -e "${JOY_DEV}" ]]; then
            echo "No gamepad at ${JOY_DEV} -- plug one in, or use ./run.sh --keyboard" >&2
            exit 1
        fi
        CMD_ARGS=(ros2 launch /workspace/teleop.launch.py "$@")
        ;;
esac

# Only needed if you reach for rqt/plotjuggler inside the container; the teleop
# frontends are terminal-only.
xhost +local:docker >/dev/null 2>&1 || true

TTY_ARGS=()
if [[ -t 0 ]]; then
    TTY_ARGS=(-it)
fi

# haptics.launch.py starts ../mcp_bridge/make_contact_plot.py (the finger-force
# plot, which injects into Isaac over the MCP bridge). That script lives beside
# this teleop/ dir, so mount it at /mcp_bridge to match the launch's resolved
# /workspace/../mcp_bridge path -- otherwise the plot node dies "No such file".
MCP_BRIDGE_ARGS=()
if [[ -d ../mcp_bridge ]]; then
    MCP_BRIDGE_ARGS=(-v "$(realpath ../mcp_bridge):/mcp_bridge:ro")
fi

# --network host + UDPv4: FastDDS's default shared-memory transport silently
# drops container<->host traffic, because the container runs as root and the
# host-side Isaac process doesn't. Same fix as resources/gello/run.sh.
docker run --rm "${TTY_ARGS[@]}" \
    --name "isaac_teleop_${ROS_DISTRO_ARG}" \
    --network host \
    --ipc host \
    "${DEVICE_ARGS[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd):/workspace:ro" \
    "${MCP_BRIDGE_ARGS[@]}" \
    "${IMG}" "${CMD_ARGS[@]}"
