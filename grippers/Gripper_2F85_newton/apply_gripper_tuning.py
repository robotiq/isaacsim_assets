"""
Re-apply ONLY the two 2F-85 gripper settings that cannot be saved in the USD.

Everything else (finger drive gains, armature, damping, spring stiffness,
maxForce, and the stiff coupler limits) is now baked into
Robotiq_2F85_newton.usda and rebuilds natively on load.

These two can't live in USD and must be re-applied at runtime after each
stop -> play:
  1. opt.impratio = 10   -- global solver option; the build ignores authored
                            MjcSceneAPI options (authored iterations=200 ran as 100).
  2. rigid driver coupling -- the finger-coupling joint-equality has no USD prim
                            (it's synthesized during model build), so its
                            near-rigid solref=[0.002,1] can't be authored. The
                            row is located by eq_type == mjEQ_JOINT (not a hard-
                            coded index), so it stays correct if the asset is
                            loaded alongside an arm or a second gripper.

Run after the sim is playing (Script Editor or MCP bridge). Idempotent.
"""
import gc
import math
import numpy as np


def apply():
    models = []
    for o in gc.get_objects():
        try:
            mod, name = type(o).__module__, type(o).__name__
        except Exception:
            continue
        if isinstance(mod, str) and "mujoco_warp" in mod and name == "Model":
            models.append(o)
    if not models:
        print("apply_gripper_tuning: no mujoco_warp Model found (is the sim playing?)")
        return

    for mjm in models:
        # 1) impratio = 10 (anti-slip)
        a = mjm.opt.impratio_invsqrt.numpy()
        a[...] = 1.0 / math.sqrt(10.0)
        mjm.opt.impratio_invsqrt.assign(a)

        # 2) near-rigid finger-coupling equality.
        # Locate the JOINT-type equality row(s) by eq_type == mjEQ_JOINT rather
        # than a hardcoded index, so this stays correct when the gripper is
        # dropped into a scene with other equalities (arm, second gripper). A
        # multi-gripper scene has one JOINT equality per gripper; tune them all.
        MJEQ_JOINT = 2  # mujoco mjtEq.mjEQ_JOINT
        et = mjm.eq_type.numpy()
        et = et[0] if et.ndim > 1 else et  # per-equality types (same across worlds)
        joint_rows = [i for i in range(et.shape[0]) if int(et[i]) == MJEQ_JOINT]
        if not joint_rows:
            print("apply_gripper_tuning: no JOINT-type equality found; "
                  "skipping driver coupling")
        else:
            sr = mjm.eq_solref.numpy()
            si = mjm.eq_solimp.numpy()
            def eqrow(arr, i, vals):
                v = np.asarray(vals)
                if arr.ndim == 3: arr[:, i, :len(v)] = v
                else: arr[i, :len(v)] = v
            for i in joint_rows:
                eqrow(sr, i, [0.002, 1.0])
                eqrow(si, i, [0.9999, 0.99999, 0.0001, 0.5, 2.0])
            mjm.eq_solref.assign(sr)
            mjm.eq_solimp.assign(si)

    print(f"apply_gripper_tuning: applied impratio=10 + rigid driver coupling "
          f"to {len(models)} model(s). (Everything else is in the USD.)")


apply()
