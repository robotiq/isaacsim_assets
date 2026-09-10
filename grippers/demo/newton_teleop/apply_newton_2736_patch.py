#!/usr/bin/env python3
"""Backport Newton PR #2736 into the Newton bundled with Isaac Sim.

    python3 apply_newton_2736_patch.py            # apply (idempotent)
    python3 apply_newton_2736_patch.py --revert   # restore the .orig backup
    python3 apply_newton_2736_patch.py --check    # report status only

WHY THIS IS NEEDED
------------------
Newton's USD importer divides angular joint LIMIT GAINS by pi/180, assuming they
were authored per-degree. `mjc:solreflimit` is always PER-RADIAN, so authored
values come out ~57x too stiff. On the 2F-85 that mis-scales the coupler joint
limits, and the symptoms are severe:

  * the passive four-bar joints buzz 3-4 deg peak-to-peak AT REST with nothing
    commanded (left_follower 3.24 deg, right_follower 3.85 deg, coupler 2.01 deg,
    reversing on ~61% of samples)
  * newton:timeStepsPerSecond <= 500 diverges to NaN within seconds

Measured on this machine after patching (UR5e + 2F-85, arm kinematic):

  | metric                | unpatched | patched |
  |-----------------------|-----------|---------|
  | left_follower at rest | 3.24 deg  | 0.001 deg |
  | right_follower        | 3.85 deg  | 0.001 deg |
  | 500 Hz timestep       | NaN       | stable    |
  | RTF at 500 Hz         | (NaN)     | 0.486     |

So the "timeStepsPerSecond must stay at 1000" rule was a CONSEQUENCE of the
unpatched importer, not a real constraint. With the patch, 500 Hz is both stable
and faster (RTF 0.420 -> 0.486).

The fix is upstream in Newton PR #2736, but Isaac Sim 6.0.0 bundles newton
1.2.0, which predates it. THE PATCH LIVES IN THE ISAAC INSTALL, NOT IN GIT --
re-run this after any Isaac Sim or Newton reinstall/upgrade, or the limits
silently regress and the buzz returns.

Only the REVOLUTE path is patched, which is what the 2F-85 uses. The D6 /
angular-axes path further down the same file has the identical bug; patch it too
if you hit a scene that needs it.
"""
import argparse
import glob
import os
import shutil
import sys

REL = "exts/isaacsim.pip.newton/pip_prebundle/newton/_src/utils/import_usd.py"
MARK = "Newton PR #2736 backport"

# (original, replacement) -- the builder DEFAULT is pre-multiplied by deg->rad so
# that the later division cancels for defaults; an AUTHORED per-radian value only
# gets divided, which is the 57x error. Remove both halves: defaults stay
# correct, authored per-radian values now survive.
EDITS = [
    ('                key="limit_angular_ke" if key == UsdPhysics.ObjectType.RevoluteJoint else "limit_linear_ke",\n'
     '                default=default_joint_limit_ke * limit_gains_scaling,',
     '                key="limit_angular_ke" if key == UsdPhysics.ObjectType.RevoluteJoint else "limit_linear_ke",\n'
     '                # ' + MARK + ': was `* limit_gains_scaling`.\n'
     '                default=default_joint_limit_ke,'),
    ('                key="limit_angular_kd" if key == UsdPhysics.ObjectType.RevoluteJoint else "limit_linear_kd",\n'
     '                default=default_joint_limit_kd * limit_gains_scaling,',
     '                key="limit_angular_kd" if key == UsdPhysics.ObjectType.RevoluteJoint else "limit_linear_kd",\n'
     '                # ' + MARK + ': was `* limit_gains_scaling`.\n'
     '                default=default_joint_limit_kd,'),
    ('                joint_params["limit_ke"] /= DegreesToRadian\n'
     '                joint_params["limit_kd"] /= DegreesToRadian',
     '                # ' + MARK + ' (Isaac 6.0.0 bundles newton 1.2.0, pre-fix).\n'
     '                # These were `/= DegreesToRadian`. Limit POSITIONS are authored\n'
     '                # in degrees and are still converted above, but limit GAINS from\n'
     '                # mjc:solreflimit are PER-RADIAN; dividing them by pi/180 scaled\n'
     '                # them ~57x, ringing the 2F-85 coupler limits (3-4 deg buzz at\n'
     '                # rest) and diverging at timeStepsPerSecond <= 500.'),
]


def target(isaac_root):
    p = os.path.join(isaac_root, REL)
    if os.path.isfile(p):
        return p
    hits = glob.glob(os.path.join(isaac_root, "**", "import_usd.py"), recursive=True)
    hits = [h for h in hits if "newton" in h and "_src/utils" in h]
    if not hits:
        sys.exit("could not find newton's import_usd.py under " + isaac_root)
    return hits[0]


def is_applied(isaac_root=None):
    """True if the backport is present in the installed Newton.

    For callers that need to gate on this rather than report it -- see
    setup_demo.py, which refuses to start an unpatched demo.
    """
    root = isaac_root or os.path.expanduser("~/isaacsim")
    return MARK in open(target(root)).read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--isaac-root", default=os.path.expanduser("~/isaacsim"))
    ap.add_argument("--revert", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    path = target(args.isaac_root)
    backup = path + ".orig-preNewton2736"
    src = open(path).read()
    applied = MARK in src

    if args.check:
        print("file    :", path)
        print("patched :", applied)
        print("backup  :", "present" if os.path.exists(backup) else "MISSING")
        return 0

    if args.revert:
        if not os.path.exists(backup):
            sys.exit("no backup at " + backup)
        shutil.copyfile(backup, path)
        print("reverted from", backup)
        return 0

    if applied:
        print("already patched:", path)
        return 0

    if not os.path.exists(backup):
        shutil.copyfile(path, backup)
        print("backup ->", backup)

    for old, new in EDITS:
        if old not in src:
            sys.exit("pattern not found -- Newton version may differ:\n" + old[:120])
        src = src.replace(old, new)
    open(path, "w").write(src)
    print("patched", path)
    print("RESTART Isaac Sim for this to take effect (the module is already imported).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
