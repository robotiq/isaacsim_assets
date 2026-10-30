#!/usr/bin/env bash
# Run every test of this repo before a SimReady Central submission.
#
#   1. tests/unit            (pure Python: tactile model)
#   2. tests/runtime         (every gripper in a real Isaac Sim, except the ROS 2 test)
#   3. tests/runtime/test_gripper_ros2.py (needs Isaac's ROS environment, run apart)
#
# The runtime tests need an NVIDIA RTX GPU and Isaac Sim 6.2 or later. They are
# not part of the GitHub CI (see tests/runtime/README.md), which is why this gate
# runs locally. It exits non-zero if anything fails, and also if the ROS 2 test is
# skipped (no system ROS under /opt/ros, or Isaac's ROS bridge did not load):
# a skip would quietly drop that coverage. Pass --allow-ros-skip to accept it.
#
# Usage:
#   ci/simready/run_pre_submit_tests.sh [--gui] [--allow-ros-skip] [-- PYTEST_ARGS...]
#     --gui            open the Isaac window to watch the runs (no --hold: the
#                      gate must end on its own)
#     PYTEST_ARGS      extra pytest arguments for the runtime suite, e.g. -k 2F_85
#                      (a partial run is NOT a pre-submission gate)
#
# Environment:
#   ISAAC_SIM   Isaac Sim install to use. Default: the newest ~/isaacsim* whose
#               VERSION is 6.2 or later.
#
# Where the Isaac lock hooks are installed (~/robotiq/devel/isaac-lock), each Isaac
# start queues for the GPU by itself; otherwise make sure no other Isaac is running.
set -uo pipefail

GUI=() ALLOW_ROS_SKIP=0 EXTRA=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --gui) GUI=(--gui); shift ;;
    --allow-ros-skip) ALLOW_ROS_SKIP=1; shift ;;
    --) shift; EXTRA=("$@"); break ;;
    -h|--help) sed -n '2,/^set -uo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
cd "$(git rev-parse --show-toplevel)"

version_ok() {  # $1 = install dir; true if VERSION >= 6.2
  local v; v=$(cut -d- -f1 "$1/VERSION" 2>/dev/null) || return 1
  local maj=${v%%.*} rest=${v#*.}; local min=${rest%%.*}
  [ -n "$maj" ] && [ -n "$min" ] && { [ "$maj" -gt 6 ] || { [ "$maj" -eq 6 ] && [ "$min" -ge 2 ]; }; }
}
if [ -z "${ISAAC_SIM:-}" ]; then
  for d in $(ls -d "$HOME"/isaacsim* 2>/dev/null | sort -rV); do
    [ -x "$d/python.sh" ] && version_ok "$d" && { ISAAC_SIM="$d"; break; }
  done
fi
[ -n "${ISAAC_SIM:-}" ] && [ -x "$ISAAC_SIM/python.sh" ] \
  || { echo "No Isaac Sim 6.2+ found: set ISAAC_SIM=<install dir>" >&2; exit 2; }
version_ok "$ISAAC_SIM" || { echo "$ISAAC_SIM is Isaac $(cat "$ISAAC_SIM/VERSION"): the runtime tests need 6.2+" >&2; exit 2; }
PY="$ISAAC_SIM/python.sh"
# A ROS sourced in this shell puts its pytest plugins (launch_testing) on PYTHONPATH,
# which then load into Isaac's Python and fail on `lark`: run everything without it.
unset PYTHONPATH
echo "Isaac Sim: $ISAAC_SIM ($(cat "$ISAAC_SIM/VERSION"))"
"$PY" -c "import pytest" 2>/dev/null || "$PY" -m pip install -q pytest

OUT=$(mktemp -d)
status=0
run() { echo; echo "== $1 =="; shift; "$@"; local rc=$?; [ $rc -eq 0 ] || status=1; return $rc; }

run "1/3 unit tests" "$PY" -m pytest tests/unit -q -rs

run "2/3 runtime tests (all grippers)" "$PY" -m pytest tests/runtime -v -rs "${GUI[@]}" \
  --ignore=tests/runtime/test_gripper_ros2.py --junitxml="$OUT/runtime.xml" "${EXTRA[@]}"

# The ROS 2 test needs Isaac started with its ROS environment and, if a ROS is
# sourced in the shell, PYTHONPATH unset (tests/runtime/README.md).
distro="${ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | sort | tail -1)}"
run "3/3 ROS 2 test (ROS_DISTRO=${distro:-none})" bash -c '
  export ROS_DISTRO="$1"; source "$2/setup_ros_env.sh" >/dev/null
  exec env -u PYTHONPATH "$2/python.sh" -m pytest tests/runtime/test_gripper_ros2.py -v -rs "${@:3}"
' _ "$distro" "$ISAAC_SIM" "${GUI[@]}" --junitxml="$OUT/ros2.xml"
skipped=$(grep -o 'skipped="[0-9]*"' "$OUT/ros2.xml" 2>/dev/null | head -1 | tr -dc 0-9)
if [ "${skipped:-0}" -gt 0 ]; then
  if [ "$ALLOW_ROS_SKIP" = 1 ]; then echo "WARN: $skipped ROS 2 test(s) skipped (allowed by flag)"
  else echo "FAIL: $skipped ROS 2 test(s) skipped: fix the ROS environment, or pass --allow-ros-skip"; status=1; fi
fi
[ ${#EXTRA[@]} -gt 0 ] && echo "NOTE: runtime suite filtered by '${EXTRA[*]}': not a full pre-submission run"

echo
if [ $status -eq 0 ]; then echo "ALL PRE-SUBMISSION TESTS PASSED"; else echo "PRE-SUBMISSION TESTS FAILED: do not submit"; fi
exit $status
