#!/usr/bin/env python3
"""
Drive the DualSense right trigger (R2) adaptive resistance from the live
gripper pad contact force — closing the teleop haptic loop.

The Isaac contact-force plot (../mcp_bridge/make_contact_plot.py)
UDP-broadcasts the two inner-finger pad forces as an ASCII "fL fR" datagram to
127.0.0.1:8770 every physics frame. This node reads that stream and sets R2 to a
Rigid resistance proportional to the AVERAGE of the two forces, so squeezing an
object in sim makes the trigger physically stiffen under the finger.

Pairs naturally with gamepad_teleop.py, where R2 also *commands* the gripper:
press R2 to close -> fingers contact the object -> R2 pushes back.

Prerequisites:
  * The contact-force plot running in Isaac (so the UDP stream exists).
  * pydualsense + hidapi  (pip install pydualsense hidapi).
  * Read/write access to the DualSense hidraw node — a `uaccess` udev rule for
    idVendor==054c/idProduct==0ce6, otherwise the open needs root.

Run alongside the teleop stack:
    python3 trigger_force_feedback.py                 # sensible defaults
    python3 trigger_force_feedback.py --full-force-N 20 --deadband-N 0.5

--full-force-N is the average pad force at which R2 reaches maximum stiffness;
tune it empirically against the forces you see in the plot for a firm grasp.
"""
import argparse
import socket
import time

from pydualsense import pydualsense, TriggerModes

TRIGGER_MAX = 255  # DualSense force parameter range is 0..255


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8770,
                    help="UDP port the contact-force plot broadcasts on")
    ap.add_argument("--full-force-N", type=float, default=200.0,
                    help="avg pad force (N) mapped to full R2 stiffness")
    ap.add_argument("--deadband-N", type=float, default=0.3,
                    help="avg force below this releases the trigger")
    ap.add_argument("--ema", type=float, default=0.3,
                    help="smoothing factor 0..1 (higher = snappier, noisier)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.2)

    ds = pydualsense()
    ds.init()
    ds.triggerR.setMode(TriggerModes.Rigid)
    print("[haptic] R2 stiffness <- avg gripper pad force | "
          "full=%.1f N  deadband=%.1f N  port=%d"
          % (args.full_force_N, args.deadband_N, args.port))

    smooth = 0.0
    last_val = -1
    try:
        while True:
            try:
                data, _ = sock.recvfrom(256)
                parts = data.split()
                avg = 0.5 * (float(parts[0]) + float(parts[1]))
            except socket.timeout:
                avg = 0.0            # stream stalled -> relax the trigger
            except (ValueError, IndexError):
                continue             # malformed datagram, skip

            smooth += args.ema * (avg - smooth)

            if smooth < args.deadband_N:
                val = 0
            else:
                frac = smooth / args.full_force_N
                val = int(max(0, min(TRIGGER_MAX, frac * TRIGGER_MAX)))

            if val != last_val:
                if val == 0:
                    ds.triggerR.setMode(TriggerModes.Off)
                    ds.triggerR.setForce(1, 0)
                else:
                    ds.triggerR.setMode(TriggerModes.Rigid)
                    ds.triggerR.setForce(1, val)   # index 0 = start pos (0), 1 = force
                last_val = val

            time.sleep(0.01)         # ~100 Hz update; pydualsense writes in its own thread
    except KeyboardInterrupt:
        pass
    finally:
        ds.triggerR.setMode(TriggerModes.Off)
        ds.triggerR.setForce(1, 0)
        time.sleep(0.1)
        ds.close()
        print("\n[haptic] trigger released, controller closed.")


if __name__ == "__main__":
    main()
