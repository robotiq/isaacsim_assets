#!/usr/bin/env bash
# Launch Isaac Sim 6 with the isaac.sim.mcp_extension enabled so Claude Code
# (or any MCP client) can drive the running Kit process.
#
# Run from a CLEAN shell — do NOT source /opt/ros/humble/setup.bash first,
# because its Python 3.10 collides with Isaac's bundled Python 3.11/3.12 and
# rclpy inside the bridge extension will refuse to load.
#
# Three paths are configurable via env vars:
#   ISAACSIM_ROOT       default: $HOME/isaacsim           (where isaac-sim*.sh lives)
#   MCP_EXT_ROOT        default: <this dir>/isaacsim-mcp-server
#                                                         (clone target of the extension repo)
#   ISAACSIM_LAUNCHER   default: isaac-sim.sh             (PhysX backend)
#                       set to isaac-sim.newton.sh for the Newton physics backend
#                       (boots the isaacsim.exp.full.newton experience). The MCP
#                       ext-folder/enable flags below are forwarded to it unchanged,
#                       so you get Newton + the MCP bridge in one session.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim}"
MCP_EXT_ROOT="${MCP_EXT_ROOT:-$SCRIPT_DIR/isaacsim-mcp-server}"
ISAACSIM_LAUNCHER="${ISAACSIM_LAUNCHER:-isaac-sim.sh}"

if [[ ! -x "$ISAACSIM_ROOT/$ISAACSIM_LAUNCHER" ]]; then
  echo "Error: Isaac Sim launcher not found at: $ISAACSIM_ROOT/$ISAACSIM_LAUNCHER" >&2
  echo "Set ISAACSIM_ROOT to your Isaac Sim install directory," >&2
  echo "and/or ISAACSIM_LAUNCHER to a launcher that exists there." >&2
  exit 1
fi

if [[ ! -d "$MCP_EXT_ROOT/isaac.sim.mcp_extension" ]]; then
  echo "Error: MCP extension not found at: $MCP_EXT_ROOT/isaac.sim.mcp_extension" >&2
  echo "Clone it with:" >&2
  echo "  git clone --depth 1 https://github.com/whats2000/isaacsim-mcp-server $MCP_EXT_ROOT" >&2
  echo "or set MCP_EXT_ROOT to point at an existing checkout." >&2
  exit 1
fi

echo "Isaac Sim:    $ISAACSIM_ROOT/$ISAACSIM_LAUNCHER"
echo "MCP ext dir:  $MCP_EXT_ROOT"

exec "$ISAACSIM_ROOT/$ISAACSIM_LAUNCHER" \
  --ext-folder "$MCP_EXT_ROOT" \
  --enable "isaac.sim.mcp_extension" \
  "$@"
