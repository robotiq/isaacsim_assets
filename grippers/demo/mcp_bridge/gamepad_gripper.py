"""
Drive the 2F-85 gripper with a gamepad's RIGHT TRIGGER, in a running Isaac Sim
session — same mapping as the teleop demo's R2 axis (release = open, fully
pressed = closed, proportional), commanding only the driven finger_joint.

Why a host-side loop (not a one-shot injector): writes to an isaacsim.core
Articulation only take effect from a *direct* execute_script call over the MCP
bridge — the same write from a Kit app-update / physics-step callback is silently
ignored, and USD drive-attr edits are not read back live. So, like the teleop
(which streams /joint_command continuously), this streams the finger target at a
fixed rate through the bridge. The articulation handle is created ONCE on a fresh
physics view (repeatedly recreating it invalidates the view).

Usage — with an Isaac session up (isaac.sim.mcp_extension on :8766), the gripper
loaded (any variant: the script only drives finger_joint) and the timeline PLAYING:

    python3 gamepad_gripper.py          # stream trigger -> gripper until Ctrl-C
    python3 gamepad_gripper.py --off    # open the gripper and stop

Direction: finger_joint 0.0 rad = OPEN, 0.78 rad = CLOSED (same convention as the
teleop). If your build maps the sign the other way, swap OPEN_RAD/CLOSED_RAD.
"""
import json
import socket
import sys
import time

GRIPPER_ROOT = "/World/Gripper"
OPEN_RAD, CLOSED_RAD = 0.0, 0.78     # release -> OPEN_RAD, full press -> CLOSED_RAD
DEADZONE = 0.03
HZ = 15.0


def send(cmd, t=10):
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


def run(code):
    r = send({"type": "simulation.execute_script", "params": {"code": code}})
    out = r.get("result", r) if isinstance(r, dict) else r
    return out.get("stdout", "") if isinstance(out, dict) else str(r)


SETUP = r'''
import carb, carb.input, omni.appwindow, omni.timeline
from isaacsim.core.prims import Articulation
tl = omni.timeline.get_timeline_interface()
if not tl.is_playing():
    tl.play()
a = Articulation("%s/Robotiq_2F_85"); a.initialize()
carb._grart = a
carb._grdof = a.dof_names.index("finger_joint")
carb._grinp = carb.input.acquire_input_interface()
carb._grgp = omni.appwindow.get_default_app_window().get_gamepad(0)
print("SETUP_OK dof=%%d" %% a.num_dof)
''' % GRIPPER_ROOT

STEP = r'''
import carb, numpy as np
a = getattr(carb, "_grart", None)
if a is None:
    print("NOART")
else:
    amt = float(carb._grinp.get_gamepad_value(carb._grgp, carb.input.GamepadInput.RIGHT_TRIGGER))
    if amt < %r:
        amt = 0.0
    tgt = %r + amt * (%r - %r)
    a.set_joint_position_targets(np.array([[tgt]]), joint_indices=[carb._grdof])
    print("%%.2f %%.3f" %% (amt, tgt))
''' % (DEADZONE, OPEN_RAD, CLOSED_RAD, OPEN_RAD)

OFF = r'''
import carb, numpy as np
a = getattr(carb, "_grart", None)
if a is not None:
    a.set_joint_position_targets(np.array([[%r]]), joint_indices=[carb._grdof])
print("opened + streaming stopped")
''' % OPEN_RAD


if "--off" in sys.argv:
    print(run(OFF).strip())
    sys.exit(0)

print(run(SETUP).strip())
print(f"streaming RIGHT TRIGGER -> finger_joint at {HZ:.0f} Hz "
      f"(release=open {OPEN_RAD}, press=close {CLOSED_RAD}). Ctrl-C to stop.")
period = 1.0 / HZ
last_print = 0.0
try:
    while True:
        out = run(STEP).strip()
        now = time.time()
        if now - last_print > 0.5:      # throttle console echo
            print("  trigger/target:", out)
            last_print = now
        time.sleep(period)
except KeyboardInterrupt:
    print("\nstopping; opening gripper")
    print(run(OFF).strip())
