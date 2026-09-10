# Recording & Exporting a Video of the Newton Sim Motion

How we captured a video of the UR5e + Robotiq 2F‑85 (Newton / MuJoCo‑Warp)
performing a teleoperated grasp, and why the built‑in tools didn't work.

**Result:** `grasp_recording.mp4` — 1280×720, path‑traced, ~18 s.

---

## TL;DR

1. The **Stage Recorder does not work** with Newton — it records a frozen scene.
2. During a live Newton sim, the motion lives in **Fabric**, not USD.
3. We wrote a small recorder that reads the live **Fabric** transforms each frame
   and bakes them into **USD timeSamples**.
4. We scrub that animation frame‑by‑frame, capture each viewport frame to PNG,
   and encode to MP4 with OpenCV.

---

## Why the Stage Recorder fails (root cause)

During a **live Newton simulation, USD is bypassed**. Both the robot's
*inputs* (joint drive targets) and *outputs* (link poses) flow through
**Fabric**, not USD:

| Data | Where the live value is | What USD holds |
|------|-------------------------|----------------|
| Link world pose | `omni:fabric:worldMatrix` (Fabric) — **live** | `xformOp:*` / `_worldPosition` — **stale rest pose** |
| Joint drive target | Fabric — **live** | USD attribute write is **ignored** |

The **Stage Recorder samples the stale USD side**, so every recording came out
frozen (~0–2 cm of drift) no matter how much the robot actually moved on screen.
This is not fixable through Stage Recorder options.

*How we proved it:* commanded a large joint motion, then compared —
`UsdGeom.Xformable.ComputeLocalToWorldTransform()` (USD) stayed at the rest pose
`(-0.20, -0.22, 0)` while Fabric's `omni:fabric:worldMatrix` held the real,
moving pose. Two different links even reported the *same* stale USD position.

---

## The working pipeline

### 1. Record the live motion from Fabric

Install a persistent recorder (kept in `sys.modules['_fabric_rec']` so it
survives across MCP script calls). It subscribes to the app update stream and,
each frame, reads `omni:fabric:worldMatrix` for every robot prim via **usdrt**:

```python
import usdrt
rst = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
# each frame, for each tracked prim:
m = rst.GetPrimAtPath(path).GetAttribute("omni:fabric:worldMatrix").Get()
# m is 16 floats, ROW-MAJOR, translation in the LAST ROW -> elements [12,13,14]
```

Tracked all prims under `/World/ur5e` that carry `omni:fabric:worldMatrix`
(~147). Armed it (`active=True`), drove the grasp with the gamepad teleop,
then disarmed (`active=False`). Result: 351 frames, 131 prims moving,
gripper fingers traveling ~7 cm — real motion.

### 2. Bake to USD timeSamples

Stop physics so Fabric isn't overwritten, and disable re‑sim:

```python
simulation.stop()
carb.settings.get_settings().set("/app/player/playSimulations", False)
```

For each prim that moved (>2 mm), author `xformOp:transform` timeSamples as the
**absolute world matrix** (row‑major → `Gf.Matrix4d(*m)`), using
`!resetXformStack!` so the pose ignores the parent chain.

⚠️ **Gotcha — the reset token gets silently dropped.** `!resetXformStack!` is
*not* preserved if you author it in a sublayer, or via
`SetXformOpOrder(..., resetXformStack=True)`. It only survives when set as a
**raw token array on the root or session layer**:

```python
prim.GetAttribute("xformOpOrder").Set(
    Vt.TokenArray(["!resetXformStack!", "xformOp:transform"]))   # on SESSION layer
```

The stage root here is anonymous (`anon:...World1.usd`), so **session‑layer
edits are non‑destructive** to the `.usda` cad file. We put the timeSamples in a
sublayer (`grasp_recording_fabric.usda`) but the reset xformOpOrder on the
session layer.

*Verify the bake:* `ComputeLocalToWorldTransform(tc).ExtractTranslation()` must
equal the captured `[12,13,14]` values.

### 3. Set the timeline range

```python
stage.SetStartTimeCode(0); stage.SetEndTimeCode(N-1); stage.SetTimeCodesPerSecond(10)
tl.set_start_time(0); tl.set_end_time((N-1)/10.0)
```

⚠️ Stage‑metadata (`startTimeCode`, etc.) can only be set on the **root or
session** layer. And if the timeline *end‑time in seconds* is too short,
scrubbing **clamps** and the later frames appear frozen — set both the
timeCode range and the seconds range.

Once these are right, scrubbing the timeline pushes USD → Fabric and the
**render animates** (confirmed by watching `omni:fabric:worldMatrix` change per
frame, and by eye).

### 4. Clean the scene

```python
carb.settings.get_settings().set("/persistent/physics/visualizationDisplayJoints", False)  # green joint gizmos
omni.usd.get_context().get_selection().clear_selected_prim_paths()
```

### 5. Capture frames

Scrub each frame and capture the viewport to PNG:

```python
import omni.kit.viewport.utility as vpu
vp = vpu.get_active_viewport()
for fr in range(N):
    tl.set_current_time(fr/10.0)
    for _ in range(11): app.update()          # let path tracing converge
    vpu.capture_viewport_to_file(vp, "/tmp/grasp_frames/f%04d.png" % fr)
    for _ in range(4): app.update()           # flush the async capture
```

- Viewport resolution: **1280×720**; render mode: **RealTimePathTracing**.
- Do it in **batches** (~175 frames) to avoid the MCP socket timeout.

### 6. Encode

No `ffmpeg` on this machine — use the system OpenCV:

```python
import cv2, glob
frames = sorted(glob.glob("/tmp/grasp_frames/f*.png"))
h, w = cv2.imread(frames[0]).shape[:2]
vw = cv2.VideoWriter("grasp_recording.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 20, (w, h))
for f in frames: vw.write(cv2.imread(f))
vw.release()
```

The PNG frames are kept on disk, so **fps / speed is re‑encodable anytime**
without re‑rendering.

---

## Artifacts

| File | What it is |
|------|-----------|
| `grasp_recording.mp4` | Final video (720p, path‑traced, ~18 s @ 20 fps) |
| `grasp_recording_fabric.usda` | Baked motion (reset+world `xformOp:transform` timeSamples) |
| `/tmp/grasp_frames/f0000–f0350.png` | Rendered frames (re‑encode at any fps) |

---

## Reusable script — `newton_video_pipeline.py`

The whole pipeline is packaged in `newton_video_pipeline.py` (run its functions
inside Isaac via the MCP socket / Script Editor; `encode_mp4` runs outside with
system OpenCV). Functions: `install_recorder` → `arm`/`stop` → `bake` →
`prep_render`/`set_pathtracing` → `render_frames` → `encode_mp4`.

**CAVEAT for reusing a saved baked layer next session:** the `!resetXformStack!`
order is written to the (unsaved) **session** layer, so a baked `.usda` alone
won't replay correctly on reload. Call `setup_replay("<layer>.usda")` first — it
re-sublayers the file and re-applies the reset order on the session layer.

## Maximum-realism render settings

The fast modes (`MinimalRendering` / `RealTimePathTracing`, ~9 samples/frame,
4 bounces) are NOT the best quality. For the most realistic output use full
offline path tracing (`set_pathtracing(True)`):

- `/rtx/rendermode = PathTracing`
- `/rtx/pathtracing/maxBounces = 16` (accurate global illumination)
- **~96 update iterations per frame** (sample accumulation) + OptiX denoiser
- Cost at 1080p: **~0.9 s/frame** (~24 min for a 48 s clip). 4K is still feasible.

## Notes & options

- **Capture rate ≈ 10 fps** (the sim's render rate). Encoding at 20 fps → ~2×
  real‑time; encode at 10 fps for true real‑time (35 s).
- **Higher quality:** re‑capture at higher resolution and/or more `app.update()`
  iterations per frame for more path‑tracing convergence.
- After previewing, restore live sim: re‑enable `/app/player/playSimulations`,
  `simulation.play()`, then re‑run `apply_gripper_tuning.py`.
- See also `NEWTON_USD_PERSISTENCE.md` for the runtime tuning that also doesn't
  round‑trip through USD.
