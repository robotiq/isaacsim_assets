"""
Fingertip showcase: a close camera orbit around the Hand-E while its Fingertip variant is switched.

Run INSIDE Isaac (send over the MCP socket, or the Script Editor), with the demo open and the gripper
positioned where you want it. The simulation is stopped first so the arm sits still while the variants change.

    exec(open(".../rotate_camera_around_object.py").read())
    render_orbit("/tmp/fp_preview", resolution=(960, 540), pathtracing=False, sub=3)      # fast preview
    render_orbit("/tmp/fp_final",   resolution=(1920, 1080), pathtracing=True, sub=96)    # final

Each fingertip is shown for SECONDS_EACH seconds at FPS. The first frame uses the viewport camera exactly as it is when
the script starts; from there the camera zooms in a little (to ZOOM_END of its starting distance from the gripper) and
sweeps SWEEP_DEG degrees around it over the whole clip.
"""
import math
import os

GRIPPER = "/World/ur5e/wrist_3_link/Robotiq_Hand_E_edit"
VARIANTS = ("Std", "Flat_Overmolded", "Bin_Picking", "Extender", "Support")
SECONDS_EACH = 2.0
FPS = 30
SWEEP_DEG = 260.0
ZOOM_END = 0.72         # final camera distance as a fraction of the starting distance
FOCUS_HEIGHT = 0.20     # look-at height inside the gripper's bounding box (0 = fingertip ends, 1 = top)


def _focus_point(stage):
    """Centre of the fingertip region: the lower part of the gripper's bounding box."""
    from pxr import Usd, UsdGeom
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeWorldBound(stage.GetPrimAtPath(GRIPPER)).ComputeAlignedRange()
    lo, hi = box.GetMin(), box.GetMax()
    return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + FOCUS_HEIGHT * (hi[2] - lo[2]))


def _set_camera(eye, target):
    """Point the active viewport camera. (Authoring a transform op by hand is ignored: Kit's camera keeps its
    translate/rotate/scale ops, so use Isaac's helper.)"""
    import omni.kit.viewport.utility as vu
    import numpy as np
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array(eye, dtype=float), target=np.array(target, dtype=float),
                    camera_prim_path=str(vu.get_active_viewport().camera_path))


def _current_camera():
    """(eye, forward) of the active viewport camera, in world space."""
    import omni.kit.viewport.utility as vu
    import omni.usd
    from pxr import Gf, UsdGeom
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.Xformable(stage.GetPrimAtPath(str(vu.get_active_viewport().camera_path))).ComputeLocalToWorldTransform(0)
    return m.ExtractTranslation(), m.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()


def _camera_path(eye0, fwd0, focus):
    """Return f(t) -> (eye, target) for t in 0..1: starts exactly at the current view, then zooms and orbits."""
    from pxr import Gf
    F = Gf.Vec3d(*focus)
    rel = eye0 - F
    r0 = rel.GetLength()
    az0 = math.atan2(rel[1], rel[0])
    el0 = math.asin(rel[2] / r0)
    # the point the starting camera actually looks at (the closest point on its view ray to the focus)
    target0 = eye0 + fwd0 * max(0.05, Gf.Dot(F - eye0, fwd0))
    smooth = lambda x: x * x * (3 - 2 * x)

    def at(t):
        r = r0 * (1 + (ZOOM_END - 1) * smooth(t))
        az = az0 + math.radians(SWEEP_DEG) * t
        eye = F + Gf.Vec3d(r * math.cos(el0) * math.cos(az), r * math.cos(el0) * math.sin(az), r * math.sin(el0))
        blend = smooth(min(1.0, t / 0.15))              # ease from the starting view direction to looking at the focus
        target = target0 * (1 - blend) + F * blend
        return eye, target
    return at


def frame_plan():
    """(number of frames, variant of each frame)."""
    per = int(round(SECONDS_EACH * FPS))
    n = per * len(VARIANTS)
    return n, [VARIANTS[min(i // per, len(VARIANTS) - 1)] for i in range(n)]


def render_orbit(out_dir, resolution=(960, 540), pathtracing=False, sub=3, frames=None, start_view=None):
    """Render the orbit. frames: optional iterable of frame indices (e.g. [0, 70, 299]) for a test.
    start_view: ((ex, ey, ez), (fx, fy, fz)) eye and forward of frame 0; default = the viewport camera as it is now
    (pass it explicitly when re-running, because a render leaves the viewport camera at its last pose)."""
    import omni.kit.app, omni.timeline, omni.usd
    import omni.kit.viewport.utility as vu
    app = omni.kit.app.get_app()
    os.makedirs(out_dir, exist_ok=True)
    omni.timeline.get_timeline_interface().stop()
    for _ in range(30): app.update()
    stage = omni.usd.get_context().get_stage()
    vp = vu.get_active_viewport()
    try: vp.resolution = resolution
    except Exception: pass
    import carb.settings
    S = carb.settings.get_settings()
    S.set("/persistent/physics/visualizationDisplayJoints", False)
    omni.usd.get_context().get_selection().clear_selected_prim_paths()
    if pathtracing:
        S.set("/rtx/rendermode", "PathTracing"); S.set("/rtx/pathtracing/maxBounces", 16)
        S.set("/rtx/pathtracing/totalSpp", 512); S.set("/rtx/pathtracing/optixDenoiser/enabled", True)
    else:
        S.set("/rtx/rendermode", "RealTimePathTracing")
    from pxr import Gf
    if start_view is None:
        eye0, fwd0 = _current_camera()                         # the view the user set, read BEFORE anything moves it
    else:
        eye0, fwd0 = Gf.Vec3d(*start_view[0]), Gf.Vec3d(*start_view[1]).GetNormalized()
    vs = stage.GetPrimAtPath(GRIPPER).GetVariantSets().GetVariantSet("Fingertip")
    n, plan = frame_plan()
    # One fixed look-at point for the whole clip, measured with the Std tip (the middle length), so the camera
    # does not jump when the longer Extender / shorter Support tip changes the bounding box.
    vs.SetVariantSelection("Std")
    for _ in range(20): app.update()
    focus = _focus_point(stage)
    path = _camera_path(eye0, fwd0, focus)
    current = None
    for i in (range(n) if frames is None else frames):
        variant = plan[i]
        if variant != current:
            vs.SetVariantSelection(variant); current = variant
            for _ in range(20): app.update()                    # let the new fingertip compose and load
        eye, target = path(i / max(1, n - 1))
        _set_camera(tuple(eye), tuple(target))
        for _ in range(sub): app.update()
        vu.capture_viewport_to_file(vp, os.path.join(out_dir, "f%05d.png" % i))
        for _ in range(4): app.update()
    print("rendered", n if frames is None else list(frames), "->", out_dir, "| variants:", VARIANTS)
