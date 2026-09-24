#!/usr/bin/env python3
"""Open the PhysX teleop scene at Isaac startup, select the gripper variant, play.

Run by run_demo.sh through Kit's --exec, i.e. INSIDE Isaac.

There is no post-play runtime step on the PhysX path: the Action Graph is baked into the scene and the physx
gripper variant needs no tuning after Stop -> Play. Open, select, play, done.

The variant IS set explicitly rather than trusted to its default. The scene
carries both gripper backends behind a `Gripper` variant set on /World, and the
selection is saved IN the file -- so whoever last switched it to `newton` and
saved decides what you get. Selecting it here costs nothing and makes the run
reproducible. Set TELEOP_GRIPPER_VARIANT=newton to go the other way (then you
also need the Newton experience and its runtime tuning step).

As in autostart.py, the work happens on Kit's update tick and never calls
omni.kit.app.update(): pumping frames from inside a callback re-enters Kit's
main loop and corrupts its asyncio state.
"""
import os
import sys
import time

SCENE = "ur5robot_with_2F-85.usda"
VARIANT = os.environ.get("TELEOP_GRIPPER_VARIANT", "physx")

# Benchmark knobs, applied BEFORE play so no stop -> play cycle is needed --
# the PhysX substep rate is only picked up when the scene starts playing, and
# stop -> play cycles measurably degrade a session's frame-time tail.
#   TELEOP_TCPS      stage timeCodesPerSecond; sim seconds per frame = 1/this
#   TELEOP_PHYSX_HZ  physxScene:timeStepsPerSecond, the PhysX substep rate
# Unset means "leave the scene's own values alone".
TCPS = os.environ.get("TELEOP_TCPS")
PHYSX_HZ = os.environ.get("TELEOP_PHYSX_HZ")

# Start the in-Isaac tick-paced teleop once physics is stepping. Set
# TELEOP_TICK=0 to leave it out (e.g. when driving through MoveIt Servo).
TICK_TELEOP = os.environ.get("TELEOP_TICK", "1") != "0"

# Renderer. The scene opens in MinimalRendering, which does simplified lighting
# and shadows -- soft shadows and dome fill authored in the scene barely reach
# it, so tuning the lights looked like it did nothing.
#
# Measured on this machine, same scene, 10 s samples:
#     MinimalRendering      59.6 FPS, GPU 13%
#     RealTimePathTracing   58.6 FPS, GPU 85%
#
# One frame. The work moves onto a GPU that was sitting idle while the CPU was
# the bottleneck, so the better lighting is very nearly free here. Set
# TELEOP_RENDER to another mode, or to "keep", to leave the scene's own.
RENDER_MODE = os.environ.get("TELEOP_RENDER", "RealTimePathTracing")
TICK_FILE = "physx_tick_teleop.py"

# Default view: above the work area, looking DOWN at it. The pose saved in the
# scene sits at z = 0.097 m -- table height -- tilted about 4 degrees below
# horizontal, so zooming out along the camera-to-tool line walks the camera into
# the table and then under it. From up here, backing off just gets you further
# above the bench. TELEOP_KEEP_VIEW=1 leaves whatever the scene saved.
# Close enough to see the fingers. physx_tick_teleop.py re-frames on the actual
# fingertip bodies once physics is stepping, which it can do precisely; this is
# what you get in the moment before that, and with TELEOP_TICK=0.
VIEW_EYE = (0.42, -0.62, 0.52)
VIEW_TARGET = (0.0, -0.12, 0.22)
_SETTLE_FRAMES = 30

_WARMUP_FRAMES = 5
_TIMEOUT_S = 180.0


def _teleop_dir():
    for cand in (os.environ.get("TELEOP_DIR"),
                 os.path.dirname(os.path.abspath(__file__))
                 if "__file__" in globals() else None):
        if cand and os.path.isfile(os.path.join(cand, SCENE)):
            return cand
    raise RuntimeError(
        "cannot locate teleop/ -- set TELEOP_DIR to the directory holding "
        + SCENE)


# The Newton/MJWarp solver pre-allocates a fixed contact buffer, 200 per
# world by default. add_props.py puts a tube, four cubes and three cylinders
# on the bench, and that took the peak to 203 -- so every contact past the cap
# was dropped, with 2394 warnings in a single session and correspondingly
# degraded grasp fidelity on Newton.
#
# Newton takes max(user, its own geometry estimate), so raising this can only
# ever help. njmax goes up alongside it: MuJoCo constraints scale with contact
# count, so leaving it at its 1200 default would just move the ceiling instead
# of removing it.
NEWTON_NCONMAX = int(os.environ.get("TELEOP_NEWTON_NCONMAX", "512"))
NEWTON_NJMAX = int(os.environ.get("TELEOP_NEWTON_NJMAX", "2400"))


def _raise_newton_contact_limit():
    """Give the MJWarp solver room for the props, before its buffers are sized.

    Gated on being able to acquire a Newton stage rather than on VARIANT: the
    gripper variant and the physics engine are chosen separately -- the engine
    comes from the experience (isaac-sim.newton.sh) -- so asking Newton itself
    is the only honest test. On PhysX the extension is not loaded and this is
    a no-op.

    Ordering matters. It has to run after the engine is on Newton, because
    switching resets the stage and discards earlier edits, and before play,
    because that is when the buffers are allocated.
    """
    try:
        import isaacsim.physics.newton as newton_ext
    except Exception:
        return "not a Newton run; contact limit untouched"
    try:
        ns = newton_ext.acquire_stage()
        if ns is None:
            return "Newton stage not acquired; contact limit left at default"
        cfg = ns.cfg.solver_cfg
        was_n, was_j = cfg.nconmax, cfg.njmax
        cfg.nconmax = max(was_n or 0, NEWTON_NCONMAX)
        cfg.njmax = max(was_j or 0, NEWTON_NJMAX)
        return ("Newton contact limits: nconmax %s -> %s, njmax %s -> %s"
                % (was_n, cfg.nconmax, was_j, cfg.njmax))
    except Exception:
        import traceback
        return ("could not raise the Newton contact limit:\n"
                + traceback.format_exc())


def _select_variant(stage):
    """Returns a human-readable note about what it did."""
    prim = stage.GetPrimAtPath("/World")
    if not prim or not prim.IsValid():
        return "no /World prim; variant NOT selected"
    vset = prim.GetVariantSets().GetVariantSet("Gripper")
    if not vset or not vset.GetVariantNames():
        return "no Gripper variant set; nothing to select"
    names = list(vset.GetVariantNames())
    if VARIANT not in names:
        return "variant %r not in %s; left at %r" % (VARIANT, names,
                                                     vset.GetVariantSelection())
    was = vset.GetVariantSelection()
    if was == VARIANT:
        return "Gripper variant already %r" % VARIANT
    vset.SetVariantSelection(VARIANT)
    return "Gripper variant %r -> %r" % (was, VARIANT)


def _set_render_mode():
    """Returns a note about what it did."""
    if RENDER_MODE.lower() == "keep":
        return "renderer left as the scene opened it"
    try:
        import carb.settings

        s = carb.settings.get_settings()
        was = s.get("/rtx/rendermode")
        s.set("/rtx/rendermode", RENDER_MODE)
        return "renderer %s -> %s" % (was, s.get("/rtx/rendermode"))
    except Exception as exc:                      # noqa: BLE001
        return "renderer unchanged (%s)" % exc


def _look_at(stage, eye, target):
    """Point the perspective camera at target from eye.

    Writes translate + rotateXYZ rather than a transform matrix, because that
    is the op set Kit's own camera manipulator uses on /OmniverseKit_Persp --
    swapping it for a matrix works until someone drags in the viewport.

    USD cameras look down their own -Z with +Y up, and xformOp:rotateXYZ
    composes as Rz*Ry*Rx, which is what the extraction below assumes.
    """
    import math

    cam = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if not cam or not cam.IsValid():
        return "no /OmniverseKit_Persp; view left as saved"

    ex, ey, ez = eye
    tx, ty, tz = target
    fx, fy, fz = tx - ex, ty - ey, tz - ez
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    if n < 1e-9:
        return "degenerate view; left as saved"
    fx, fy, fz = fx / n, fy / n, fz / n
    zx, zy, zz = -fx, -fy, -fz                    # camera +Z is backwards
    # right = up_world x z, renormalised; guards the straight-down case
    rx, ry, rz = -zy, zx, 0.0
    rn = math.sqrt(rx * rx + ry * ry + rz * rz)
    if rn < 1e-6:
        rx, ry, rz, rn = 1.0, 0.0, 0.0, 1.0
    rx, ry, rz = rx / rn, ry / rn, rz / rn
    ux, uy, uz = zy * rz - zz * ry, zz * rx - zx * rz, zx * ry - zy * rx

    b = math.asin(max(-1.0, min(1.0, -rz)))
    a = math.atan2(uz, zz)
    c = math.atan2(ry, rx)

    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(cam)
    ops = {o.GetOpName(): o for o in xf.GetOrderedXformOps()}
    t = ops.get("xformOp:translate")
    r = ops.get("xformOp:rotateXYZ")
    if t is None or r is None:
        return "camera uses an unexpected op set; view left as saved"
    t.Set(Gf.Vec3d(float(ex), float(ey), float(ez)))
    r.Set(Gf.Vec3f(math.degrees(a), math.degrees(b), math.degrees(c)))
    drop = math.degrees(math.asin(max(-1.0, min(1.0, -fz))))
    return "view set: eye %s looking %.0f deg down" % (str(eye), drop)


def _apply_timing(stage):
    """Apply the benchmark overrides. Returns notes to print."""
    notes = []
    if TCPS:
        import omni.timeline

        stage.SetTimeCodesPerSecond(float(TCPS))
        omni.timeline.get_timeline_interface().set_time_codes_per_second(
            float(TCPS))
        notes.append("timeCodesPerSecond -> %s (sim %.2f ms/frame)"
                     % (TCPS, 1000.0 / float(TCPS)))
    if PHYSX_HZ:
        # Found by traversal, not a hardcoded path: it is /PhysicsScene here,
        # NOT /World/PhysicsScene, and guessing wrong fails silently enough to
        # cost an afternoon.
        scenes = [p for p in stage.Traverse()
                  if p.GetTypeName() == "PhysicsScene"]
        if not scenes:
            notes.append("WARNING: no PhysicsScene prim; substep rate NOT set")
        for prim in scenes:
            attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
            if attr:
                attr.Set(int(PHYSX_HZ))
                notes.append("%s substep -> %s Hz" % (prim.GetPath(), PHYSX_HZ))
    return notes


def _start_tick_teleop(d):
    """exec the teleop module, the way the Script Editor would.

    Not `import`: the module self-starts at import and stops any previous
    instance first, so a cached import would silently do nothing the second
    time. It traps its own exceptions and only prints ERR, so success is read
    back from carb rather than inferred from the absence of a raise.
    """
    import carb

    path = os.path.join(d, TICK_FILE)
    g = {"__name__": "physx_tick_teleop", "__file__": path}
    with open(path) as fh:
        exec(compile(fh.read(), path, "exec"), g)   # noqa: S102
    return bool(getattr(carb, "_physx_tick_teleop", None))


def install():
    import omni.kit.app

    d = _teleop_dir()
    state = {"phase": "warmup", "frames": 0, "started": time.monotonic()}
    sub = {"handle": None}

    def _done(msg, ok=True):
        print("[physx-autostart] " + msg)
        if ok:
            print("[physx-autostart] ready.")
        sub["handle"] = None

    def _on_update(_e):
        state["frames"] += 1
        if time.monotonic() - state["started"] > _TIMEOUT_S:
            _done("TIMED OUT in phase '%s'" % state["phase"], ok=False)
            return
        try:
            if state["phase"] == "warmup":
                if state["frames"] >= _WARMUP_FRAMES:
                    state["phase"] = "open"
                return

            if state["phase"] == "open":
                import omni.timeline
                import omni.usd

                path = os.path.join(d, SCENE)
                ctx = omni.usd.get_context()
                current = (ctx.get_stage_url() or "").replace("file://", "")
                if os.path.normpath(current) != os.path.normpath(path):
                    # Synchronous, returns a bare bool -- the (ok, err) tuple
                    # belongs to open_stage_async().
                    if not ctx.open_stage(path):
                        raise RuntimeError("could not open " + path)
                    print("[physx-autostart] opened", path)
                else:
                    print("[physx-autostart] scene already open")

                print("[physx-autostart] " + _select_variant(ctx.get_stage()))
                for note in _apply_timing(ctx.get_stage()):
                    print("[physx-autostart] " + note)
                print("[physx-autostart] " + _set_render_mode())
                if os.environ.get("TELEOP_KEEP_VIEW", "0") == "0":
                    print("[physx-autostart] "
                          + _look_at(ctx.get_stage(), VIEW_EYE, VIEW_TARGET))

                print("[physx-autostart] " + _raise_newton_contact_limit())

                tl = omni.timeline.get_timeline_interface()
                if not tl.is_playing():
                    tl.play()
                print("[physx-autostart] timeline playing")
                if not TICK_TELEOP:
                    _done("teleop skipped (TELEOP_TICK=0)")
                    return
                # The articulation cannot be initialised until physics has
                # actually stepped, so give it a few frames before starting.
                state["phase"] = "teleop"
                state["frames"] = 0
                return

            if state["phase"] == "teleop":
                if state["frames"] < _SETTLE_FRAMES:
                    return
                if _start_tick_teleop(d):
                    _done("teleop running (tick-paced, no Servo)")
                else:
                    _done("teleop did NOT start -- see the ERR traceback above",
                          ok=False)
        except Exception as exc:                      # noqa: BLE001
            import traceback
            traceback.print_exc()
            _done("FAILED: %s" % exc, ok=False)

    sub["handle"] = (omni.kit.app.get_app()
                     .get_update_event_stream()
                     .create_subscription_to_pop(_on_update,
                                                 name="physx_teleop_autostart"))
    print("[physx-autostart] scheduled -- %s (variant %r)" % (d, VARIANT))
    return sub


_SUB = install()
