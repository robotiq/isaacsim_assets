# Isaac Sim MCP bridge — plugging Claude Code into a live Kit session

Optional add-on to the teleop stack. Sets up a **Model Context Protocol**
(MCP) bridge so Claude Code — or any MCP client — can drive a running
Isaac Sim 6 process directly: build Action Graphs, inspect prims, step
physics, read/write joint state, save the stage, and so on, all from
inside a conversation without pasting Python into the Script Editor.

**You do not need this to run the teleop.** The teleop stack in
`../teleop/` runs entirely from ROS. This directory only matters if you
want an AI agent to programmatically edit or introspect the stage while
you're using it.

## Architecture

```
┌─────────────────────────┐         ┌──────────────────────────────────┐
│       Claude Code       │         │           Isaac Sim 6            │
│ (your conversation)     │         │   (~/isaacsim, GUI process)      │
│                         │         │                                  │
│  reads .mcp.json ──┐    │         │   ┌────────────────────────────┐ │
│                    │    │         │   │  isaac.sim.mcp_extension   │ │
│                    ▼    │  stdio  │   │  (loaded via --ext-folder) │ │
│   isaacsim-mcp-server ─────────── │ │  TCP listen 127.0.0.1:8766 │ │
│   (PyPI, in a venv)     │ MCP RPC │   └─────────────┬──────────────┘ │
│                         │         │                 │                │
│                         │         │       og.Controller / omni.usd   │
│                         │         │                 ▼                │
│                         │         │              live stage          │
└─────────────────────────┘         └──────────────────────────────────┘
```

Two processes, two roles:

| Side          | What runs there                                                                                         |
| ------------- | ------------------------------------------------------------------------------------------------------- |
| Claude Code   | `isaacsim-mcp-server` (Python CLI). Speaks MCP over stdio to the client, opens a TCP socket to Isaac on each tool call. |
| Isaac Sim     | `isaac.sim.mcp_extension` Kit extension. TCP server on `localhost:8766`. Receives RPCs, executes against the live stage. |

Source: <https://github.com/whats2000/isaacsim-mcp-server>
(PyPI: `isaacsim-mcp-server`)

## What's in this folder

```
mcp_bridge/
├── README.md                     ← this file
├── launch_isaac_with_mcp.sh      ← Isaac Sim launcher with --ext-folder + --enable already wired
├── mcp.json.example              ← template for the client-side .mcp.json config
├── make_contact_plot.py          ← host-side tool: overlay a live gripper pad contact-force plot
└── remove_contact_plot.py        ← host-side tool: tear that plot back down
```

`make_contact_plot.py` / `remove_contact_plot.py` are standalone diagnostic
clients: they open the extension's `127.0.0.1:8766` socket directly (no MCP
server needed) and inject a scrolling left/right pad contact-force plot into the
running viewport. Works on both the Newton and PhysX backends — see the module
docstring in the script. Handy while tuning gripper grasp physics.

**Not** checked in:

- The `isaacsim-mcp-server` git checkout (a few MB, cloned per-machine)
- The Python venv holding the server package (per-machine)
- Your actual `.mcp.json` (contains machine-specific absolute paths)

## Setup — from scratch

Once per developer machine.

### 1. Install the server-side CLI in a venv

The PyPI package is host-side only (Claude's process runs it via stdio).
Put it in an isolated venv so it doesn't collide with system Python.
Anywhere works, pick a stable location — `~/mcp-servers/isaacsim/.venv`
is a sensible default:

```bash
python3 -m venv ~/mcp-servers/isaacsim/.venv
~/mcp-servers/isaacsim/.venv/bin/pip install --upgrade pip
~/mcp-servers/isaacsim/.venv/bin/pip install isaacsim-mcp-server
```

Note the entry point `~/mcp-servers/isaacsim/.venv/bin/isaacsim-mcp-server`
— you'll paste that path into your `.mcp.json`.

### 2. Clone the extension repo

The extension that runs *inside* Isaac Sim ships in the same source
repo but is not on PyPI. Clone it wherever suits you — the launcher
defaults to expecting it as a sibling of the launcher:

```bash
git clone --depth 1 https://github.com/whats2000/isaacsim-mcp-server \
    "$(pwd)/isaacsim-mcp-server"
```

(Run from this folder to use the default location, or clone elsewhere
and set `MCP_EXT_ROOT` — see below.)

### 3. Configure your MCP client

Copy `mcp.json.example` to `.mcp.json` in your project root (or into
`~/.claude.json` for a user-global entry), and edit the `command` path
to point at the venv binary from step 1:

```json
{
  "mcpServers": {
    "isaac-sim": {
      "command": "/home/YOU/mcp-servers/isaacsim/.venv/bin/isaacsim-mcp-server"
    }
  }
}
```

A project-scoped `.mcp.json` at the working directory root is usually
cleaner than a user-global one — the bridge auto-activates when you
launch Claude Code from that directory.

### 4. Start Isaac Sim with the extension loaded

```bash
# clean shell — do NOT source /opt/ros/humble first
./launch_isaac_with_mcp.sh
```

In Isaac Sim's console (`Window → Console`) look for
`MCP socket listening on localhost:8766`. If it's not there, the
extension didn't load — check `Window → Extensions` for
`isaac.sim.mcp_extension` and verify the `--ext-folder` path.

The launcher accepts two env-var overrides:

- `ISAACSIM_ROOT` — Isaac Sim install location (default `$HOME/isaacsim`)
- `MCP_EXT_ROOT` — where you cloned the extension (default: sibling
  `isaacsim-mcp-server/` directory next to the launcher)

### 5. Start Claude Code and verify

Start Claude Code from a directory whose `.mcp.json` (or ancestor's)
includes the `isaac-sim` entry. On the first tool call, expect:

```
mcp__isaac-sim__get_scene_info → {
  "assets_root_path": "…/Isaac/6.0",
  "stage_path": "/.../your_scene.usd",
  "prim_count": …
}
```

That's the handshake — the bridge is up.

## Day-to-day usage

Just re-run the launcher whenever you want an MCP-enabled Isaac Sim
session:

```bash
./launch_isaac_with_mcp.sh
```

The MCP server (Claude side) is launched on demand by the MCP client
whenever a tool is called — you don't need to start it manually.

## Caveats discovered the hard way

- **Officially targets Isaac Sim 5.x**, but works on 6.0.0 for
  everything we've exercised: Action Graph CRUD, prim inspection,
  physics step, joint observation, `execute_script`. If a named tool
  fails with a renamed API in 6.x, fall back to
  `mcp__isaac-sim__execute_script` and call the Python API directly.

- **Prefer named tools over `execute_script`.** Some client harnesses
  flag `execute_script` when the parameters were inferred from prior
  MCP query results ("code execution against shared infra with
  unverified params"). Use `create_action_graph`, `edit_action_graph`,
  `get_*`, `step_simulation`, etc. wherever they fit.

- **`get_isaac_logs` is the single best diagnostic.** Many graph and
  physics errors never surface in tool responses — they only go to
  Isaac's console. Reading the log after each non-obvious failure
  shortened our Action Graph debug loop dramatically.

- **The launcher requires a clean shell.** Sourcing
  `/opt/ros/humble/setup.bash` first pulls Humble's Python 3.10 onto the
  path; Isaac Sim ships Python 3.11/3.12 and `rclpy` inside the bridge
  extension will refuse to load. Source ROS only in the *consumer*
  terminals (where you run `ros2 topic …`).

- **Extension version skew.** As of writing, the PyPI package (server)
  was 0.5.2 and the bundled extension manifest read 0.4.1. Both worked
  together. If you upgrade the PyPI package and tool calls start
  failing, re-pull the repo and check that `extension.toml`'s expected
  protocol version still matches.
