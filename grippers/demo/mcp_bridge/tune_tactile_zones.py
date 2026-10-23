"""
Live tuning panel for the TSF-85 taxel grid: adjust the POSITION and SIZE of the 7x4
markers in a running Isaac Sim session and see them move, then bake the printed values
into grippers/Robotiq_2F_85/tools/generate_tactile_zones.py. The readers place the taxels
from these markers live, so nothing else needs updating while tuning.

Injects an omni.ui window with sliders (7 rows along local Z, 4 cols along X):
  Center Z   — array center along the finger length (rows axis), metres
  Center X   — array center across the finger (cols axis), metres
  Grip face  — |Y| offset to the contact face (left uses -Y, right +Y), metres
  Length     — 7-row span, metres
  Width      — 4-col span, metres
  Thickness  — zone box thickness (contact normal), metres

"Print values" dumps the current numbers to paste into the generator.

    python3 tune_tactile_zones.py          # open the panel
    python3 tune_tactile_zones.py --off    # close it

Requires the gripper loaded with Fingertip=Tactile_TSF85 and a SensorSim.
Set GRIPPER_ROOT for your scene.
"""
import json
import socket
import sys

GRIPPER_ROOT = "/World/Gripper"
# Seed values = current generator constants (metres). 7 rows along local Z,
# 4 cols along local X; cz = rows centre, cx = cols centre, y = grip-face |Y|.
SEED = dict(cx=0.0, cz=0.1324, y=0.0416, h=0.038, w=0.0235, th=0.004)


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


if "--off" in sys.argv:
    code = r'''
import carb
w = getattr(carb, "_tzt", None)
if w:
    try: w["win"].destroy()
    except: pass
    carb._tzt = None
    print("tuning panel closed")
else:
    print("no tuning panel")
'''
else:
    code = r'''
import traceback
try:
    import omni.ui as ui, omni.usd, omni.timeline, carb
    from pxr import Gf, UsdGeom

    ROWS, COLS, GAP = 7, 4, 0.0004
    G = "%s"
    FING = {"left": G + "/Robotiq_2F_85/left_inner_finger",
            "right": G + "/Robotiq_2F_85/right_inner_finger"}
    stage = omni.usd.get_context().get_stage()
    P = dict(cx=%r, cz=%r, y=%r, h=%r, w=%r, th=%r)

    def offsets(pad_y):
        # 7 rows along Z, 4 cols along X (matches generate_tactile_zones.py)
        row_z = (P["h"] - GAP * (ROWS - 1)) / ROWS
        col_x = (P["w"] - GAP * (COLS - 1)) / COLS
        zmax = P["cz"] + P["h"] * 0.5
        xmin = P["cx"] - P["w"] * 0.5
        out = []
        for r in range(ROWS):
            for c in range(COLS):
                z = zmax - ((r + 0.5) * row_z + r * GAP)
                x = xmin + ((c + 0.5) * col_x + c * GAP)
                out.append((x, pad_y, z))
        return out, col_x, row_z

    def _set(prim, tr, sc):
        if not prim or not prim.IsValid():
            return
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*tr))
        prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(*sc))

    def update_visuals():
        for side, sgn in (("left", -1.0), ("right", 1.0)):
            offs, zx, zz = offsets(P["y"] * sgn)
            for i, (x, y, z) in enumerate(offs):
                r, c = divmod(i, COLS)
                _set(stage.GetPrimAtPath(FING[side] + "/tactile_zone_visuals/zone_r%%d_c%%d" %% (r, c)),
                     (x, y, z), (zx, P["th"], zz))

    prev = getattr(carb, "_tzt", None)
    if prev:
        try: prev["win"].destroy()
        except: pass

    win = ui.Window("Tactile Grid Tuning", width=380, height=340)
    # (key, label, min, max, step[m])  -- step sets the drag resolution
    SPECS = [("cz", "Center Z (rows/length)", 0.08, 0.18, 0.0002),
             ("cx", "Center X (cols/across)", -0.03, 0.03, 0.0002),
             ("y",  "Grip face |Y|", 0.03, 0.06, 0.0002),
             ("h",  "Length (7-row span)", 0.02, 0.055, 0.0005),
             ("w",  "Width (4-col span)", 0.015, 0.035, 0.0005),
             ("th", "Thickness", 0.001, 0.01, 0.0002)]
    with win.frame:
        with ui.VStack(spacing=6, height=0):
            ui.Label("Drag a field (fine step) or type a value; visuals update live.",
                     height=16, style={"color": 0xff888888})
            for key, label, lo, hi, step in SPECS:
                with ui.HStack(height=22, spacing=6):
                    ui.Label("%%s  [%%+.4f]" %% (label, P[key]), width=170, style={"color": 0xffcccccc})
                    m = ui.SimpleFloatModel(P[key])
                    d = ui.FloatDrag(model=m, min=lo, max=hi, step=step)
                    d.precision = 5
                    def _mk(k, model):
                        def _cb(mm):
                            P[k] = float(mm.get_value_as_float()); update_visuals()
                        model.add_value_changed_fn(_cb)
                    _mk(key, m)
            ui.Button("Print values", height=26, clicked_fn=lambda: print(
                "TUNED: cx=%%.6f cz=%%.6f y=%%.6f h=%%.6f w=%%.6f th=%%.6f" %%
                (P["cx"], P["cz"], P["y"], P["h"], P["w"], P["th"])))
    carb._tzt = {"win": win, "P": P}
    update_visuals()
    print("tuning panel open")
except Exception:
    print("ERR:\n" + traceback.format_exc())
''' % (GRIPPER_ROOT, SEED["cx"], SEED["cz"], SEED["y"], SEED["h"], SEED["w"], SEED["th"])

r = send({"type": "simulation.execute_script", "params": {"code": code}})
out = r.get("result", r) if isinstance(r, dict) else r
if isinstance(out, dict):
    print("\n".join(l for l in out.get("stdout", "").splitlines() if not l.startswith("[mcp]")))
    if out.get("stderr"):
        print("STDERR:", out["stderr"][-800:])
else:
    print(r)
