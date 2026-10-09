# Connecting an AI agent to Isaac Sim via MCP

It is possible to connect an AI agent like **Claude** to Isaac Sim using a
MCP **Model Context Protocol** server. Once connected, the agent can
build Action Graphs, inspect prims, step physics, read/write joint state, run
Python against the stage, and save. This is a convenient way to speed up
your development with Isaac Sim.

```text
┌─────────────┐         ┌─────────────────────┐                  ┌────────────────────────┐
│ Claude Code │──stdio─▶│ isaacsim-mcp-server │──localhost:8766─▶│ extension in Isaac Sim │
└─────────────┘         └─────────────────────┘                  └────────────────────────┘
```


The Linux launchers for this repo are described in
[`grippers/demo/mcp_bridge/README.md`](../grippers/demo/mcp_bridge/README.md).

**Setup (once per machine):**

Everything goes in one folder in your home directory:

1. Install the MCP server.

   ```bash
   # Linux
   uv venv ~/mcp-servers/isaacsim/.venv --python 3.12
   uv pip install --python ~/mcp-servers/isaacsim/.venv isaacsim-mcp-server
   ```

   ```powershell
   # Windows (PowerShell)
   uv venv $HOME\mcp-servers\isaacsim\.venv --python 3.12
   uv pip install --python $HOME\mcp-servers\isaacsim\.venv isaacsim-mcp-server
   ```

   This Create a mcp-servers folder in your home folder and save MCP server files
   in it.

2. Clone the Isaac Sim extension from GitHub repository into the `extension`
   folder next to the venv.

   ```bash
   # Linux
   ext=~/mcp-servers/isaacsim/extension
   if [ -e "$ext" ]; then
     read -rp "$ext already exists. Replace it? [y/N] " answer
     [ "$answer" = y ] && rm -rf "$ext"
   fi
   if [ -e "$ext" ]; then
     echo "Kept the existing extension."
   else
     git clone --depth 1 https://github.com/whats2000/isaacsim-mcp-server "$ext"
   fi
   ```

   ```powershell
   # Windows (PowerShell)
   $ext = "$HOME\mcp-servers\isaacsim\extension"
   if (Test-Path $ext) {
       $answer = Read-Host "$ext already exists. Replace it? [y/N]"
       if ($answer -eq 'y') { Remove-Item -Recurse -Force $ext }
   }
   if (Test-Path $ext) {
       Write-Host "Kept the existing extension."
   } else {
       git clone --depth 1 https://github.com/whats2000/isaacsim-mcp-server $ext
   }
   ```

   Full instructions, including other MCP clients, are in the
   [upstream README](https://github.com/whats2000/isaacsim-mcp-server#quick-start).

3. Register the server with Claude Code, replacing any existing `isaac-sim`
   entry:

   ```bash
   # Linux
   claude mcp add --scope user isaac-sim -- ~/mcp-servers/isaacsim/.venv/bin/isaacsim-mcp-server
   ```

   ```powershell
   # Windows (PowerShell)
   claude mcp add --scope user isaac-sim -- $HOME\mcp-servers\isaacsim\.venv\Scripts\isaacsim-mcp-server.exe
   ```

4. Launch Isaac Sim with the extension enabled:

   ```bash
   # Linux
   <ISAACSIM_ROOT>/isaac-sim.sh --ext-folder ~/mcp-servers/isaacsim/extension --enable isaac.sim.mcp_extension
   ```

   ```powershell
   # Windows (PowerShell)
   <ISAACSIM_ROOT>\isaac-sim.bat --ext-folder $HOME\mcp-servers\isaacsim\extension --enable isaac.sim.mcp_extension
   ```

   On Linux, [`grippers/demo/mcp_bridge/launch_isaac_with_mcp.sh`](../grippers/demo/mcp_bridge/launch_isaac_with_mcp.sh)
   adds these flags for you. It looks for the extension in its own folder by
   default, so set `MCP_EXT_ROOT=~/mcp-servers/isaacsim/extension` first.

5. Check it works. Isaac's console (**Window → Console**) should show
   `Isaac Sim MCP server started on localhost:8766`, and `claude mcp list`
   should show `isaac-sim` as **Connected**. Then start a **new** Claude Code
   session, since MCP tools only load at session start, and ask it to call
   `get_scene_info`.

Two gotchas:

- **Linux: launch Isaac from a clean shell.** Do *not* `source /opt/ros/humble`
  first, or Humble's Python 3.10 shadows Isaac's 3.11/3.12 and the extension
  refuses to load. Source ROS only in the consumer terminals.
- **`get_isaac_logs` is the best diagnostic.** Many graph and physics errors
  never appear in tool responses, only in Isaac's console.