"""
Live tuning panel for the TSF-85 taxel grid: adjust the POSITION and SIZE of the 7x4
markers in a running Isaac Sim session and see them move, then bake the printed values
into grippers/tactile/generate_tactile_zones.py (MOUNTS). The readers place the taxels
from these markers live, so nothing else needs updating while tuning.

Injects an omni.ui window with sliders (7 rows along local Z, 4 cols along X):
  Center Z   — array center along the finger length (rows axis), metres
  Center X   — array center across the finger (cols axis), metres
  Grip face  — |Y| offset to the contact face (left uses -Y, right +Y), metres
  Length     — 7-row span, metres
  Width      — 4-col span, metres
  Thickness  — zone box thickness (contact normal), metres

"Print values" dumps the current numbers to paste into the generator.

    python3 tune_tactile_zones.py                             # the 2F-85
    python3 tune_tactile_zones.py --gripper Robotiq_2F_140    # the 2F-140
    python3 tune_tactile_zones.py --off                       # close it

Requires the gripper loaded with its TSF fingertip (Tactile_TSF85 / Tactile_TSF140) and a SensorSim.
Set GRIPPER_ROOT for your scene.
"""
import argparse
import json
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # grippers/, for tactile.generate_tactile_zones
from tactile import generate_tactile_zones as gen  # noqa: E402  (stdlib only)

GRIPPER_ROOT = "/World/Gripper"
ap = argparse.ArgumentParser(description="TSF-85 taxel grid tuning panel")
ap.add_argument("--gripper", default="Robotiq_2F_85", choices=list(gen.MOUNTS))
ap.add_argument("--off", action="store_true", help="close the panel")
ARGS = ap.parse_args()
# Seed values = the gripper's current generator values (metres). 7 rows along local Z,
# 4 cols along local X; cz = rows centre, cx = cols centre, y = grip-face |Y|.
_M = gen.MOUNTS[ARGS.gripper]
SEED = dict(cx=_M.center_x, cz=_M.center_z, y=_M.pad_y, h=gen.SENSING_HEIGHT, w=gen.SENSING_WIDTH,
            th=gen.ZONE_THICKNESS)
# Slider ranges around the seed: (key, label, min, max, step[m]), the step sets the drag resolution.
SPECS = [("cz", "Center Z (rows/length)", SEED["cz"] - 0.05, SEED["cz"] + 0.05, 0.0002),
         ("cx", "Center X (cols/across)", -0.03, 0.03, 0.0002),
         ("y", "Grip face |Y|", SEED["y"] - 0.015, SEED["y"] + 0.015, 0.0002),
         ("h", "Length (7-row span)", 0.02, 0.055, 0.0005),
         ("w", "Width (4-col span)", 0.015, 0.035, 0.0005),
         ("th", "Thickness", 0.001, 0.01, 0.0002)]


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


if ARGS.off:
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
    FING = {"left": G + "/%s/left_inner_finger",
            "right": G + "/%s/right_inner_finger"}
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

    for side, path in FING.items():
        if not stage.GetPrimAtPath(path + "/tactile_zone_visuals").IsValid():
            raise RuntimeError("no taxel markers at %%s/tactile_zone_visuals: load the gripper with its TSF "
                               "fingertip and a SensorSim, or pass the matching --gripper" %% path)

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

    win = ui.Window("Tactile Grid Tuning: %s", width=380, height=340)
    SPECS = %r
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
''' % (GRIPPER_ROOT, ARGS.gripper, ARGS.gripper, SEED["cx"], SEED["cz"], SEED["y"], SEED["h"], SEED["w"],
       SEED["th"], ARGS.gripper, SPECS)

r = send({"type": "simulation.execute_script", "params": {"code": code}})
out = r.get("result", r) if isinstance(r, dict) else r
if isinstance(out, dict):
    print("\n".join(l for l in out.get("stdout", "").splitlines() if not l.startswith("[mcp]")))
    if out.get("stderr"):
        print("STDERR:", out["stderr"][-800:])
else:
    print(r)
