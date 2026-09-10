"""
Newton sim video pipeline — record live motion, bake to USD, render frames.

Why this exists: the Isaac Stage Recorder does NOT work with Newton (it samples
stale USD; the live motion lives in Fabric). This module reads the live Fabric
transforms, bakes them to USD timeSamples, and renders them frame-by-frame.
See NEWTON_VIDEO_RECORDING.md for the full write-up.

HOW TO RUN: these functions must run INSIDE Isaac (they use omni/usdrt/pxr).
Send them over the MCP socket (simulation.execute_script) or paste into Isaac's
Script Editor. Encoding (encode_mp4) runs OUTSIDE Isaac with system OpenCV.

Typical session:
    # live sim playing + teleop running
    install_recorder()                 # track robot + cube + cylinder
    arm()                              # -> drive the motion with the gamepad
    stop()                            # freezes the buffer
    N = bake("/home/louschr/robotiq/ROS/my_take.usda")   # stops physics, bakes
    prep_render(N)                    # timeline range, 1080p, gizmos off
    render_frames("/tmp/my_take", 0, N)                  # scrub + capture PNGs
    # then OUTSIDE isaac:  encode_mp4("/tmp/my_take", "my_take.mp4", fps=30)

To REUSE a saved baked layer next session (see CAVEAT in setup_replay):
    setup_replay("/home/louschr/robotiq/ROS/grasp_recording_fabric_3.usda")
    render_frames(...)
"""

import sys, types, os


# ---------------------------------------------------------------- recorder ----
def install_recorder(scope_prefixes=("/World/ur5e", "/World/Cube", "/World/Cylinder")):
    """Install a persistent Fabric-transform recorder in sys.modules['_fabric_rec'].
    Tracks every prim under scope_prefixes that carries omni:fabric:worldMatrix."""
    import omni.kit.app, omni.usd
    import usdrt
    from usdrt import Usd as RtUsd

    sid = omni.usd.get_context().get_stage_id()
    rst = RtUsd.Stage.Attach(sid)
    ust = omni.usd.get_context().get_stage()

    def mat16(m):
        # usdrt Gf.Matrix4d -> 16 floats, ROW-MAJOR (translation in LAST ROW: [12,13,14])
        try:    return [float(m[i][j]) for i in range(4) for j in range(4)]
        except Exception:
            return [float(x) for row in m for x in row]

    paths = []
    for p in ust.Traverse():
        pth = str(p.GetPath())
        if not any(pth == s or pth.startswith(s) for s in scope_prefixes):
            continue
        rp = rst.GetPrimAtPath(pth)
        if not rp:
            continue
        a = rp.GetAttribute("omni:fabric:worldMatrix")
        if a and a.IsValid() and a.HasValue():
            paths.append(pth)

    mod = sys.modules.get("_fabric_rec") or types.ModuleType("_fabric_rec")
    sys.modules["_fabric_rec"] = mod
    mod.rst = rst; mod.paths = paths; mod.frames = []; mod.active = False; mod.mat16 = mat16

    def _on_update(e, _m=mod):
        if not _m.active:
            return
        fr = {}
        for pth in _m.paths:
            at = _m.rst.GetPrimAtPath(pth).GetAttribute("omni:fabric:worldMatrix")
            if at and at.HasValue():
                fr[pth] = _m.mat16(at.Get())
        _m.frames.append(fr)

    bus = omni.kit.app.get_app().get_update_event_stream()
    if getattr(mod, "sub", None) is not None:
        try: mod.sub.unsubscribe()
        except Exception: pass
    mod.sub = bus.create_subscription_to_pop(_on_update, name="fabric_rec")
    print("recorder installed; tracked prims:", len(paths))
    return len(paths)


def arm():
    m = sys.modules["_fabric_rec"]; m.frames = []; m.active = True
    print("ARMED")


def stop():
    m = sys.modules["_fabric_rec"]; m.active = False
    print("STOPPED; frames:", len(m.frames))
    return len(m.frames)


# -------------------------------------------------------------------- bake ----
def bake(out_layer_path, tcps=30.0, bake_all=True, min_move=0.002):
    """Stop physics and bake the recorded motion into USD timeSamples.

    timeSamples go into a sublayer (out_layer_path). The absolute-world pose is
    made parent-independent with '!resetXformStack!'.

    GOTCHA: '!resetXformStack!' is DROPPED if authored in a sublayer, so the
    reset xformOpOrder is written to the SESSION layer via a RAW Vt.TokenArray.
    (This is why saved layers need setup_replay() to be reusable next session.)
    """
    import omni.usd, carb.settings
    from pxr import Usd, UsdGeom, Sdf, Gf, Vt

    carb.settings.get_settings().set("/app/player/playSimulations", False)
    import omni.timeline
    omni.timeline.get_timeline_interface().stop()

    mod = sys.modules["_fabric_rec"]; F = mod.frames; N = len(F)
    st = omni.usd.get_context().get_stage()
    anim = Sdf.Layer.FindOrOpen(out_layer_path) or Sdf.Layer.CreateNew(out_layer_path)
    anim.Clear()
    root = st.GetRootLayer()
    if out_layer_path not in root.subLayerPaths:
        root.subLayerPaths.insert(0, out_layer_path)
    st.SetEditTarget(Usd.EditTarget(anim))

    target = []
    for pth in mod.paths:
        p = st.GetPrimAtPath(pth)
        if not p or not UsdGeom.Xformable(p):        continue
        if p.GetTypeName() in ("Material", "Scope", "Shader"): continue
        if pth not in F[0]:                           continue
        if not bake_all:
            import numpy as np
            pts = np.array([(f[pth][12], f[pth][13], f[pth][14]) for f in F if pth in f])
            if float(np.linalg.norm(pts.max(0) - pts.min(0))) < min_move:
                continue
        target.append(pth)

    for pth in target:
        xf = UsdGeom.Xformable(st.GetPrimAtPath(pth))
        op = next((o for o in xf.GetOrderedXformOps()
                   if o.GetOpType() == UsdGeom.XformOp.TypeTransform), None) or xf.AddTransformOp()
        attr = op.GetAttr()
        for i, f in enumerate(F):
            if pth in f:
                attr.Set(Gf.Matrix4d(*f[pth]), Usd.TimeCode(i))

    _apply_reset_and_timecodes(st, target, N, tcps)
    anim.startTimeCode = 0; anim.endTimeCode = float(N - 1); anim.timeCodesPerSecond = tcps
    anim.Save()
    st.SetEditTarget(Usd.EditTarget(root))
    print("baked", len(target), "prims x", N, "frames ->", out_layer_path)
    return N


def _apply_reset_and_timecodes(st, target, N, tcps):
    from pxr import Usd, Vt
    RESET = Vt.TokenArray(["!resetXformStack!", "xformOp:transform"])
    sess = st.GetSessionLayer()
    st.SetEditTarget(Usd.EditTarget(sess))
    for pth in target:
        st.GetPrimAtPath(pth).GetAttribute("xformOpOrder").Set(RESET)
    st.SetStartTimeCode(0); st.SetEndTimeCode(N - 1); st.SetTimeCodesPerSecond(tcps)
    st.SetEditTarget(Usd.EditTarget(st.GetRootLayer()))


def setup_replay(layer_path, tcps=30.0):
    """Reuse a SAVED baked layer (next session). Sublayers it AND re-applies the
    reset xformOpOrder on the session layer (which is NOT saved in the file)."""
    import omni.usd, carb.settings
    from pxr import Usd, Sdf, UsdGeom
    carb.settings.get_settings().set("/app/player/playSimulations", False)
    st = omni.usd.get_context().get_stage()
    root = st.GetRootLayer()
    if layer_path not in root.subLayerPaths:
        root.subLayerPaths.insert(0, layer_path)
    anim = Sdf.Layer.FindOrOpen(layer_path)
    N = int(anim.endTimeCode) + 1
    # prims that carry timeSamples in the layer need the reset order re-applied
    target = []
    for p in st.Traverse():
        s = anim.GetAttributeAtPath(str(p.GetPath()) + ".xformOp:transform")
        if s and s.GetInfo("timeSamples"):
            target.append(str(p.GetPath()))
    _apply_reset_and_timecodes(st, target, N, tcps)
    print("replay ready:", len(target), "prims,", N, "frames")
    return N


# ------------------------------------------------------------------ render ----
def prep_render(N, tcps=30.0, resolution=(1920, 1080)):
    import omni.usd, omni.timeline, omni.kit.app, carb.settings
    import omni.kit.viewport.utility as vpu
    carb.settings.get_settings().set("/persistent/physics/visualizationDisplayJoints", False)
    omni.usd.get_context().get_selection().clear_selected_prim_paths()
    tl = omni.timeline.get_timeline_interface()
    tl.set_time_codes_per_second(tcps); tl.set_start_time(0.0)
    tl.set_end_time((N - 1) / tcps); tl.set_looping(False)   # NB: seconds range must cover all frames or scrub CLAMPS
    vp = vpu.get_active_viewport()
    try: vp.resolution = resolution
    except Exception: pass
    for _ in range(10): omni.kit.app.get_app().update()
    print("render prepped:", vp.resolution, "0..%.2fs" % tl.get_end_time())


def set_pathtracing(enable=True, spp_bounces=(96, 16)):
    """enable=True -> full offline PathTracing (most realistic). False -> RealTimePathTracing."""
    import carb.settings
    S = carb.settings.get_settings()
    if enable:
        S.set("/rtx/rendermode", "PathTracing")
        S.set("/rtx/pathtracing/maxBounces", spp_bounces[1])
        S.set("/rtx/pathtracing/totalSpp", 512)
        S.set("/rtx/pathtracing/optixDenoiser/enabled", True)
    else:
        S.set("/rtx/rendermode", "RealTimePathTracing")
    print("rendermode:", S.get("/rtx/rendermode"))


def render_frames(out_dir, start, end, tcps=30.0, sub=96):
    """Scrub timeline start..end and capture each frame. sub = update iters per
    frame (path-tracing sample accumulation; ~96 with denoiser = clean 1080p)."""
    import omni.timeline, omni.kit.app
    import omni.kit.viewport.utility as vpu
    os.makedirs(out_dir, exist_ok=True)
    tl = omni.timeline.get_timeline_interface(); app = omni.kit.app.get_app()
    vp = vpu.get_active_viewport()
    for fr in range(start, end):
        tl.set_current_time(fr / tcps)
        for _ in range(sub): app.update()
        vpu.capture_viewport_to_file(vp, os.path.join(out_dir, "f%05d.png" % fr))
        for _ in range(4): app.update()
    print("rendered", start, "..", end - 1, "->", out_dir)


# ---- run OUTSIDE Isaac (system python + opencv; no ffmpeg on this box) --------
def encode_mp4(frames_dir, out_path, fps=30):
    import cv2, glob
    frames = sorted(glob.glob(os.path.join(frames_dir, "f*.png")))
    h, w = cv2.imread(frames[0]).shape[:2]
    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for f in frames: vw.write(cv2.imread(f))
    vw.release()
    print("wrote", out_path, len(frames), "frames @", fps, "fps")
