#!/usr/bin/env python3
"""Bring the Newton demo up in one call, or refuse and say why.

Run INSIDE Isaac Sim, launched with the Newton experience:

    python3 isaac_rpc.py script setup_demo.py                 # MCP bridge

or, from the Script Editor, where __file__ does not exist:

    import os, sys
    os.environ["NEWTON_DEMO_DIR"] = ".../grippers/demo/newton_teleop"
    exec(open(os.environ["NEWTON_DEMO_DIR"] + "/setup_demo.py").read())
    main()

It performs, in order:

  1. verify Newton PR #2736 is backported into this Isaac install
  2. open ur5e_2F85_newton.usda
  3. press play
  4. apply the two gripper settings that cannot live in USD

and stops at the first failure instead of leaving a half-configured stage.

WHY THIS EXISTS
---------------
Every step below fails SILENTLY when skipped: an unpatched Newton rings the
coupler limits (3-4 deg buzz) and diverges below 1000 Hz; a stage that is not
playing has no mujoco_warp Model to tune; and without the tuning the gripper
slips and the finger coupling goes soft. None of that raises -- it just grips
badly, which is indistinguishable from bad tuning.

Step 4 is NOT once-per-session: it has to be re-run after every stop -> play,
because the model is rebuilt each time. Call retune() for that.

The arm's Physics variant and the gripper's articulation root are NOT set here
-- author_scene.py bakes both into the scene. If you find yourself setting them
by hand, the scene is stale; re-author it.
"""
import os
import sys

SCENE_NAME = "ur5e_2F85_newton.usda"


def _here():
    """Directory of this script, or None under Script Editor (no __file__)."""
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return None


def _repo_dir():
    """newton_teleop/ directory, however this script was invoked."""
    for cand in (_here(), os.environ.get("NEWTON_DEMO_DIR")):
        if cand and os.path.isfile(os.path.join(cand, SCENE_NAME)):
            return cand
    raise RuntimeError(
        "cannot locate newton_teleop/ -- run this as a file, or set "
        "NEWTON_DEMO_DIR to the directory holding " + SCENE_NAME)


def check_patch(isaac_root=None):
    d = _repo_dir()
    if d not in sys.path:
        sys.path.insert(0, d)
    import apply_newton_2736_patch as patch

    if patch.is_applied(isaac_root):
        print("[setup] Newton #2736 backport: present")
        return
    raise RuntimeError(
        "Newton PR #2736 is NOT backported into this Isaac install.\n"
        "  The 2F-85's coupler limits will read ~57x too stiff: the passive\n"
        "  joints buzz 3-4 deg at rest and the sim diverges below 1000 Hz.\n"
        "  Fix:  python3 apply_newton_2736_patch.py   (then RESTART Isaac)")


def open_scene():
    import omni.usd

    path = os.path.join(_repo_dir(), SCENE_NAME)
    ctx = omni.usd.get_context()
    current = (ctx.get_stage_url() or "").replace("file://", "")
    if os.path.normpath(current) == os.path.normpath(path):
        print("[setup] scene already open")
        return
    ok, err = ctx.open_stage(path)
    if not ok:
        raise RuntimeError("could not open %s: %s" % (path, err))
    print("[setup] opened", path)


def play():
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    if not tl.is_playing():
        tl.play()
    print("[setup] timeline playing")


def retune():
    """Re-apply the runtime-only gripper settings. Safe to call repeatedly.

    Needed after EVERY stop -> play: those two values live in the built
    mujoco_warp model, which is discarded and rebuilt on each play.
    """
    gripper_dir = os.path.normpath(
        os.path.join(_repo_dir(), "..", "..", "Gripper_2F85_newton"))
    if gripper_dir not in sys.path:
        sys.path.insert(0, gripper_dir)
    import apply_gripper_tuning

    apply_gripper_tuning.apply()


def main(isaac_root=None):
    check_patch(isaac_root)
    open_scene()
    play()
    # The model only exists once physics has stepped, so this must follow play().
    retune()
    print("[setup] ready -- run newton_kinematic_teleop.py to drive it.")
    print("[setup] after any stop -> play, call retune() again.")
    return 0


if __name__ == "__main__":
    main()
