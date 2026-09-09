#!/usr/bin/env bash
# Launch Isaac Sim 6 with the NEWTON physics backend AND the MCP extension.
# Thin wrapper over launch_isaac_with_mcp.sh: selects the isaacsim.exp.full.newton
# experience via ISAACSIM_LAUNCHER and points MCP_EXT_ROOT at the local extension clone.
# Both env vars are overridable from the environment.
# Run from a CLEAN shell (do NOT source /opt/ros/humble first — Py 3.10/3.12 mismatch breaks rclpy).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ISAACSIM_LAUNCHER="${ISAACSIM_LAUNCHER:-isaac-sim.newton.sh}"
export MCP_EXT_ROOT="${MCP_EXT_ROOT:-/home/louschr/robotiq/ROS/isaac_sim/isaacsim-mcp-server}"

exec "$SCRIPT_DIR/launch_isaac_with_mcp.sh" "$@"
