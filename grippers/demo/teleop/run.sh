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
#   ./run.sh --distro humble        # ...on Humble instead of the default Jazzy
#   ./run.sh --servo-only           # Servo half only; drive it from elsewhere
#   ./run.sh --keyboard             # Servo half, then the keyboard frontend
#   ./run.sh --smoke                # run smoke_launch_check.py and exit
#   ./run.sh -- <command>           # anything else inside the container
set -euo pipefail

cd "$(dirname "$0")"

ROS_DISTRO_ARG="${ROS_DISTRO_ARG:-jazzy}"
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
    DEVICE_ARGS=(--device "${JOY_DEV}:${JOY_DEV}" --group-add input)
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
    "${IMG}" "${CMD_ARGS[@]}"
