# TSF-85 tactile sensor: try it in 5 minutes

Grip an object with the Robotiq 2F-85 and watch a **live 7×4 tactile heatmap** per finger, driving the
gripper with a **gamepad trigger** or from Isaac. The sensor has two models (`SensorSim`), described in
[`../tactile/README.md`](../tactile/README.md): `EffectiveResponse`
(this scene's default, any step rate, PhysX and Newton) and `Raycast` (PhysX, needs ≥ 240 Hz).

| Tool | What it does |
| --- | --- |
| `mcp_bridge/make_tactile_sensor_plot.py` | The live heatmap, for either model: the force each taxel carries (N), or `--quantity response` (the effective contact response r), or `--quantity depth` (mm, Raycast) |
| `mcp_bridge/gamepad_gripper.py` | Open/close the gripper with the right trigger (release = open, press = close) |
| `mcp_bridge/tune_tactile_zones.py` | Move/resize the taxel grid on the pad |

## One-time setup

**Isaac Sim 6** and the **MCP bridge** (an extension that lets these scripts talk to a running Isaac): follow
[`mcp_bridge/README.md`](mcp_bridge/README.md) once. A gamepad is optional (tested with a Sony DualSense).

## Run it

From this folder (`grippers/demo/`).

1. **Launch Isaac with the bridge**, from a clean shell (no ROS sourced):
   `cd mcp_bridge && ./launch_isaac_with_mcp.sh`. The console shows `MCP socket listening on localhost:8766`.
2. **Open the scene while stopped:** File → Open → `grippers/demo/tactile_demo_scene.usda`. It has the 2F-85 with
   `Fingertip=Tactile_TSF85` and `SensorSim=EffectiveResponse`, zero gravity, 240 Hz, and a kinematic cube between
   the fingers. Pick `/World/Camera` in the viewport's camera menu (or select the gripper and press F).
3. **Show the heatmap before Play:** `python3 mcp_bridge/make_tactile_sensor_plot.py`, then press **Play**.
   The window shows two 7×4 grids (left/right finger, row 0 at the fingertip end) in dark → red → yellow → white.
   Attaching it while playing leaves the pad contact reports off until the next Play (`--restart` does the
   Stop → Play for you).
4. **Drive the gripper:** `python3 mcp_bridge/gamepad_gripper.py` (Ctrl-C to stop), then squeeze the cube. Without
   a gamepad, open **Window → Physics → Physics Inspector** and drag the `finger_joint` Drive Target slider.
5. **Optional:** `python3 mcp_bridge/tune_tactile_zones.py` moves and resizes the grid live; "Print values" gives
   the numbers to put in `MOUNTS` of `../tactile/generate_tactile_zones.py`.

For the teleop demo, `TELEOP_SENSORSIM=EffectiveResponse ./run_demo.sh` (see [`teleop/README.md`](teleop/README.md))
selects the sensor at scene open and shows the same heatmap on its own.

## What am I looking at?

Per taxel, the **force in newtons** that patch of the pad carries (the contact force spread over the contact
patch, or the raycast depth through the pad's spring). It is simulated from the physics engine's contact data
and is not calibrated to the real sensor's counts. `--quantity response` shows r of Shang et al.
(arXiv 2610.10510), a phenomenological response with no unit; `--quantity depth` shows the Raycast depth.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| Heatmap black during a firm grip | The timeline must be playing, and the plot attached before Play (or `--restart`). |
| Trigger does nothing | The timeline must be playing; check the gamepad is connected; re-run `gamepad_gripper.py` after a Stop → Play. |
| Trigger opens instead of closes | Swap `OPEN_RAD`/`CLOSED_RAD` at the top of `gamepad_gripper.py`. |
| `ConnectionRefusedError` | Isaac is not running with the bridge (step 1). |
| A tool can't find the gripper | The tuning and trigger tools assume `/World/Gripper`; set `GRIPPER_ROOT` at the top for another path. |
| Remove the tools' windows | `make_tactile_sensor_plot.py --off`, `gamepad_gripper.py --off`, `tune_tactile_zones.py --off`, `remove_contact_plot.py` |
