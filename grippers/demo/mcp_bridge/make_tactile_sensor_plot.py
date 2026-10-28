"""
Live 7x4 heatmap of the TSF-85 tactile sensor in a running Isaac Sim session, for either
``SensorSim`` variant of the 2F-85 or 2F-140. The injected code runs the readers of
grippers/tactile/ (``make_tactile_sensor``) and shows them in an omni.ui window.

Run from a host shell (NOT inside Isaac) against a session with the MCP extension up
(see README.md, TCP 127.0.0.1:8766), with the gripper loaded with Fingertip=Tactile_TSF85
and a SensorSim, and the timeline playing:

    python3 make_tactile_sensor_plot.py                  # create / recreate
    python3 make_tactile_sensor_plot.py --off            # remove
    python3 make_tactile_sensor_plot.py --quantity response   # force (N, default) | response (r) | depth (mm, Raycast)
    python3 make_tactile_sensor_plot.py --if-present     # quietly do nothing without a tactile sensor
    python3 make_tactile_sensor_plot.py --restart        # Stop -> Play after attaching (see below)

The gripper is found from the ``tactile_zone_visuals`` prims, so it works in the teleop scene as in the
test scenes. The engine is Newton if its model is built, else PhysX.

EffectiveResponse reads the pad's contact reports, which PhysX only enables at the next Play, and a stalled
gripper falls asleep without reports. Attach the plot while stopped, start the scene with
``TELEOP_SENSORSIM=EffectiveResponse`` (open_scene.py does it before play), or pass ``--restart``.
Only contacts with something outside the gripper count.
"""
import json
import socket
import sys
from pathlib import Path

GRIPPER_PACKAGE_DIR = str(Path(__file__).resolve().parents[2])  # contains tactile/


def send(cmd, t=70):
    s = socket.socket(); s.settimeout(t); s.connect(("127.0.0.1", 8766))
    s.sendall(json.dumps(cmd).encode()); buf = b""
    while True:
        c = s.recv(65536)
        if not c:
            break
        buf += c
        try:
            return json.loads(buf.decode())
        except Exception:
            continue
    return {}


QUANTITY = sys.argv[sys.argv.index("--quantity") + 1] if "--quantity" in sys.argv else "force"
if QUANTITY not in ("force", "response", "depth"):
    sys.exit(f"--quantity must be force, response or depth, not {QUANTITY!r}")

if "--off" in sys.argv:
    code = r'''
import carb
p = getattr(carb, "_tactile_sensor_plot", None)
if p:
    try: p._sub = None
    except Exception: pass
    try: p.window.destroy()
    except Exception: pass
    carb._tactile_sensor_plot = None
    print("tactile sensor plot removed")
else:
    print("no tactile sensor plot")
'''
else:
    code = r'''
import sys, traceback
try:
    import carb, omni.usd
    PKG = %r
    IF_PRESENT = %r
    QUANTITY = %r
    RESTART = %r
    if PKG not in sys.path:
        sys.path.insert(0, PKG)
    for m in [m for m in sys.modules if m == "tactile" or m.startswith("tactile.")]:
        del sys.modules[m]  # pick up edits to the package
    from tactile import TactilePlot, make_tactile_sensor

    old = getattr(carb, "_tactile_sensor_plot", None)
    if old:
        try: old._sub = None
        except Exception: pass
        try: old.window.destroy()
        except Exception: pass

    stage = omni.usd.get_context().get_stage()
    marks = [p for p in stage.Traverse() if p.GetName() == "tactile_zone_visuals"]
    if not marks:
        if IF_PRESENT:
            print("no tactile sensor in the stage; no plot")
        else:
            raise RuntimeError("no tactile_zone_visuals in the stage: select the TSF fingertip (Tactile_TSF85 / "
                               "Tactile_TSF140) and a SensorSim")
    else:
        # .../<gripper>/<side>_inner_finger/tactile_zone_visuals -> .../<gripper>
        root = str(marks[0].GetPath().GetParentPath().GetParentPath())
        engine = "physx"
        try:
            import isaacsim.physics.newton as ipn
            if ipn.acquire_stage().model is not None:
                engine = "newton"
        except Exception:
            pass
        from pxr import PhysxSchema, Usd
        pads = [stage.GetPrimAtPath(root + "/" + n) for n in ("left_fingertip", "right_fingertip")]
        was_ready = all(p.HasAPI(PhysxSchema.PhysxContactReportAPI) for p in pads)
        with Usd.EditContext(stage, stage.GetSessionLayer()):  # the contact-report schemas must not dirty the scene
            sensor = make_tactile_sensor(stage, root, engine, quantity=QUANTITY)
        plot = TactilePlot()
        plot.show(sensor)
        carb._tactile_sensor_plot = plot
        import omni.timeline
        tl = omni.timeline.get_timeline_interface()
        if engine == "physx" and type(sensor).__name__ == "EffectiveTactile" and not was_ready and tl.is_playing():
            if RESTART:
                tl.stop(); tl.play()
                print("restarted the timeline so that the pad contact reports are on")
            else:
                print("WARNING: attached while playing, the pad contact reports start at the next Play; the map reads "
                      "zero until then. Stop -> Play, or rerun with --restart")
        print("OK: tactile sensor plot up:", type(sensor).__name__, "root", root, "engine", engine)
except Exception:
    print("ERR:\n" + traceback.format_exc())
''' % (GRIPPER_PACKAGE_DIR, "--if-present" in sys.argv, QUANTITY, "--restart" in sys.argv)

r = send({"type": "simulation.execute_script", "params": {"code": code}})
out = r.get("result", r) if isinstance(r, dict) else r
if isinstance(out, dict):
    print("\n".join(l for l in out.get("stdout", "").splitlines() if not l.startswith("[mcp]")))
    if out.get("stderr"):
        print("STDERR:", out["stderr"][-800:])
else:
    print(r)
