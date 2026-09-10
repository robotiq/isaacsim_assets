#!/usr/bin/env python3
"""In-Isaac kinematic teleop for the UR5e + Newton 2F-85 scene.

Run inside Isaac (Script Editor or MCP bridge):
    python3 isaac_rpc.py script newton_kinematic_teleop.py
Stop with:
    import carb; carb._kinteleop["stop"]()

WHY
---
Newton needs a 1 ms timestep for the 2F-85's soft loop-closure equalities.
Measured on an RTX A4000 laptop:

    arm dynamic     (nv 32)  RTF 0.090     11x slower than realtime
    arm physics off (nv 26)  RTF 0.453     5x faster, gripper fidelity intact

The arm has ZERO colliders, so it pays full constraint-solve cost for nothing.
It is therefore removed from physics (`Physics` variant -> "None") and driven
kinematically here, while the gripper stays a fully dynamic Newton articulation.

MoveIt Servo cannot drive this scene: its loop runs on the WALL clock (measured
1726 commands per SIMULATED second against a configured 250) while the sim runs
at RTF ~0.1, giving a ~3 Hz limit cycle no gain change removes. Jazzy also
deleted `low_latency_mode`, which used to pace Servo by its input. This driver
runs on Kit's update tick, so controller, physics and clock share one timebase
and the oscillation cannot occur.

FORWARD KINEMATICS
------------------
    T_i_world(q) = ARM_BASE . Rz(180deg) . F_i(q)

F_i is plain URDF FK from ur_description's joint origins/axes (every axis is
local Z). The Rz(180) is a LEFT-multiplied base rotation -- that was the whole
trick. Trying to absorb the frame difference with right-multiplied per-link
offsets fails (~1.3 m error, ~8 cm even after searching 4096 sign/offset
combinations), because a base-frame difference cannot be expressed as a
per-link offset. Verified against live physics poses: worst link error 0.1 mm
at zero config, 0.02 mm at the working start pose.

The arm links are FLAT SIBLINGS under /World/ur5e (not a nested chain), so each
link's LOCAL transform is its arm-relative pose and is written directly.

Reads /dev/input/js0 directly -- no ROS. Isaac's bundled Python and ROS 2's
rclpy are different interpreters; mixing them is what the repo's launcher warns
about.

CONTROLS (DualSense, same mapping as gamepad_teleop.py)
    right stick    linear x / y        left stick   roll / pitch
    d-pad up/down  linear z            L1 / R1      yaw
    R2             gripper (analog)    Triangle/Cross  speed x1.4 / /1.4
"""
import math
import os
import struct as _struct
import traceback

import numpy as np

import carb
import omni.kit.app
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ARM = "/World/ur5e"
LINKS = ["shoulder_link", "upper_arm_link", "forearm_link",
         "wrist_1_link", "wrist_2_link", "wrist_3_link"]
GRIPPER = "/World/ur5e/wrist_3_link/gripper_2f85"

# ur_description UR5e joint origins (xyz, rpy); every joint axis is local Z
ORIGINS = [((0, 0, 0.1625), (0, 0, 0)),
           ((0, 0, 0), (1.570796327, 0, 0)),
           ((-0.425, 0, 0), (0, 0, 0)),
           ((-0.3922, 0, 0.1333), (0, 0, 0)),
           ((0, -0.0997, 0), (1.570796327, 0, 0)),
           ((0, 0.0996, 0), (1.570796326589793, 3.141592653589793,
                             3.141592653589793))]
START_DEG = [-0.00006633832, -62.5, 107.499954, -134.49998, -94.50019, -33.984856]
Q_LIMIT = math.radians(360.0)
MAX_DQ_PER_TICK = math.radians(2.0)   # rad/tick cap, keeps the gripper from jumping

# ---- floor guard -----------------------------------------------------------
# With the arm kinematic the gripper base has INFINITE stiffness: contact forces
# cannot push it back, so driving the fingers into the floor has nowhere to go
# except the finger constraints, and they explode. Rather than filter out
# gripper/floor contact (we WANT to see the fingers deflect), the commanded
# DOWNWARD motion is blocked once the pads are actually touching. This is a
# feedback clamp on measured pad height, not a geometric limit, so it stays
# correct when the wrist rotates and the lowest point changes.
FLOOR_Z = 0.0
PAD_BODIES = ["left_pad", "left_silicone_pad", "right_pad", "right_silicone_pad"]
PAD_BELOW_ORIGIN = 0.015    # pad geometry extends this far below its body origin
CONTACT_BUDGET = 0.004      # allowed penetration -> visible finger deflection

AXIS_LX, AXIS_LY, AXIS_L2, AXIS_RX, AXIS_RY, AXIS_R2, AXIS_DX, AXIS_DY = range(8)
BTN_X, BTN_O, BTN_TRI = 0, 1, 2
BTN_L1, BTN_R1 = 4, 5
DEADZONE = 0.12
SPEED_LINEAR = 0.25
SPEED_ANGULAR = 1.0
GRIP_OPEN, GRIP_CLOSED = 0.0, 0.80
# Slew-rate limit on the gripper target (rad/s of SIM time). A fast R2 squeeze
# steps the target 0 -> 0.80 in a couple of ticks, and that step input into the
# stiff four-bar diverges. Ramping the target slowly reproduces the conditions
# under which a manual sweep closes cleanly all the way to 0.787.
GRIP_RATE = 0.8
JS = _struct.Struct("IhBB")


def _T(x):
    M = np.eye(4)
    M[0:3, 3] = x
    return M


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[1, 1] = c; M[1, 2] = -s; M[2, 1] = s; M[2, 2] = c
    return M


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[0, 0] = c; M[0, 2] = s; M[2, 0] = -s; M[2, 2] = c
    return M


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[0, 0] = c; M[0, 1] = -s; M[1, 0] = s; M[1, 1] = c
    return M


_ORIG = [_T(xyz) @ _Rz(rpy[2]) @ _Ry(rpy[1]) @ _Rx(rpy[0]) for xyz, rpy in ORIGINS]
_BASEROT = _Rz(math.pi)          # the left-multiplied base rotation


def fk(q):
    """Arm-relative pose of each link (list of 4x4), q in radians."""
    out, M = [], np.eye(4)
    for i in range(6):
        M = M @ _ORIG[i] @ _Rz(q[i])
        out.append(_BASEROT @ M)
    return out


def _dz(v):
    return 0.0 if abs(v) < DEADZONE else v


def _np2gf(M):
    return Gf.Matrix4d(*[float(M[r][c]) for r in range(4) for c in range(4)]).GetTranspose()


def _quat_from_R(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s, 0.25 * s)


class Pad:
    def __init__(self, dev="/dev/input/js0"):
        self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = [0.0] * 12
        self.axes[AXIS_L2] = 1.0
        self.axes[AXIS_R2] = 1.0
        self.buttons = [0] * 16
        self.edges = [0] * 16

    def poll(self):
        self.edges = [0] * 16
        while True:
            try:
                d = os.read(self.fd, JS.size)
            except (BlockingIOError, OSError):
                return
            if not d or len(d) < JS.size:
                return
            _t, val, typ, num = JS.unpack(d)
            if typ & 0x02 and num < len(self.axes):
                self.axes[num] = val / 32767.0
            elif typ & 0x01 and num < len(self.buttons):
                if val and not self.buttons[num]:
                    self.edges[num] = 1
                self.buttons[num] = val

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass


class Teleop:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.q = [math.radians(d) for d in START_DEG]
        self.scale = 1.0
        self.grip = GRIP_OPEN
        self.r2_rest = None       # calibrated on first real trigger reading
        self._errs = 0

        # one matrix op per arm link so a full pose can be written each tick
        self.ops = []
        for n in LINKS:
            prim = self.stage.GetPrimAtPath(ARM + "/" + n)
            if not prim or not prim.IsValid():
                raise RuntimeError("missing link " + n)
            xf = UsdGeom.Xformable(prim)
            op = next((o for o in xf.GetOrderedXformOps()
                       if o.GetOpName().endswith("transform")), None)
            if op is None:
                local = xf.GetLocalTransformation()
                xf.ClearXformOpOrder()
                op = xf.AddTransformOp()
                op.Set(local)
            self.ops.append(op)

        from isaacsim.core.prims import SingleArticulation
        self.art = SingleArticulation(GRIPPER)
        self.art.initialize()
        dn = list(self.art.dof_names)
        self.fj = dn.index("finger_joint") if "finger_joint" in dn else None

        # constant gripper offset relative to the flange, from current state
        p, qt = self.art.get_world_pose()
        p = [float(v) for v in p]
        qt = [float(v) for v in qt]
        Gw = np.eye(4)
        w, x, y, z = qt
        Gw[0:3, 0:3] = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        Gw[0:3, 3] = p
        # The offset MUST be computed in the configuration the gripper's build
        # pose corresponds to -- the freshly loaded arm sits at its ZERO pose,
        # not at START_DEG. Using START_DEG here produced a bogus 0.29 m offset
        # that flung the gripper onto the floor as soon as the arm moved.
        self.armbase = self._armbase()
        flange_build = self.armbase @ fk([0.0] * 6)[5]
        self.grip_off = np.linalg.inv(flange_build) @ Gw

        # bind the lowest gripper bodies for the floor guard
        from isaacsim.core.prims import SingleRigidPrim
        self.pads = []
        for p in Usd.PrimRange(self.stage.GetPrimAtPath(GRIPPER)):
            if p.GetName() in PAD_BODIES and p.HasAPI(UsdPhysics.RigidBodyAPI):
                try:
                    r = SingleRigidPrim(str(p.GetPath())); r.initialize()
                    self.pads.append(r)
                except Exception:
                    pass
        print("[kin_teleop] floor guard tracking %d pad bodies" % len(self.pads))

        self.pad = Pad()
        self._write(self.q)
        self._sub = (omni.kit.app.get_app().get_update_event_stream()
                     .create_subscription_to_pop(self._tick, name="kin_teleop"))
        print("[kin_teleop] running. right stick=xy  dpad=z  L1/R1=yaw  R2=gripper")

    def _armbase(self):
        prim = self.stage.GetPrimAtPath(ARM)
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        M = np.array([[m[r][c] for c in range(4)] for r in range(4)]).T
        return M

    def _write(self, q, vw=None):
        """Write arm link poses + gripper root pose, and the MATCHING velocity.

        Setting only the pose teleports the articulation root: the solver sees a
        position jump with zero velocity, the jointed fingers are dragged
        instantaneously, constraint forces spike and the model explodes. Newton
        needs the root velocity to be consistent with the motion, so the
        commanded world twist is published alongside the pose (and zeroed when
        idle, otherwise the gripper keeps "coasting" in the solver's view).
        """
        Ls = fk(q)
        for op, M in zip(self.ops, Ls):
            op.Set(_np2gf(M))
        flange = self.armbase @ Ls[5]
        Gw = flange @ self.grip_off
        self.art.set_world_pose(
            position=np.array(Gw[0:3, 3], dtype=np.float32),
            orientation=np.array(_quat_from_R(Gw[0:3, 0:3]), dtype=np.float32))
        # TORCH tensors, not numpy: the GPU backend calls unsqueeze() on these,
        # so numpy raises TypeError. This was previously wrapped in a bare
        # `except: pass`, which silently swallowed that error -- the root was
        # pose-teleported with ZERO velocity on every update, and the resulting
        # discontinuity shocked the finger constraints every frame (visible as
        # jitter in all joints). Never silently swallow these again.
        import torch
        cur = self.art.get_joint_positions()
        dev = cur.device if hasattr(cur, "device") else "cpu"
        v6 = np.zeros(6) if vw is None else np.asarray(vw, dtype=float)
        try:
            self.art.set_linear_velocity(
                torch.tensor(v6[0:3], dtype=torch.float32, device=dev))
            self.art.set_angular_velocity(
                torch.tensor(v6[3:6], dtype=torch.float32, device=dev))
        except Exception:
            self._velerr = getattr(self, "_velerr", 0) + 1
            if self._velerr == 1:
                print("[kin_teleop] velocity write FAILED:\n" + traceback.format_exc())
        return flange

    def _pads_touching(self):
        """True when the lowest pad surface has reached the floor."""
        if not self.pads:
            return False
        try:
            lo = min(float(r.get_world_pose()[0][2]) for r in self.pads)
        except Exception:
            return False
        return (lo - PAD_BELOW_ORIGIN) <= (FLOOR_Z + CONTACT_BUDGET)

    def _jac(self, eps=1e-5):
        F0 = self.armbase @ fk(self.q)[5]
        p0, R0 = F0[0:3, 3], F0[0:3, 0:3]
        J = np.zeros((6, 6))
        for i in range(6):
            qp = list(self.q)
            qp[i] += eps
            F1 = self.armbase @ fk(qp)[5]
            J[0:3, i] = (F1[0:3, 3] - p0) / eps
            W = (F1[0:3, 0:3] @ R0.T - np.eye(3)) / eps
            J[3:6, i] = [W[2, 1], W[0, 2], W[1, 0]]
        return J, R0

    def _tick(self, e):
        try:
            dt = e.payload.get("dt", 1.0 / 60.0) or 1.0 / 60.0
            self.pad.poll()
            a, b = self.pad.axes, self.pad.buttons
            if self.pad.edges[BTN_TRI]:
                self.scale = min(4.0, self.scale * 1.4)
                print("[kin_teleop] speed x%.2f" % self.scale)
            if self.pad.edges[BTN_X]:
                self.scale = max(0.25, self.scale / 1.4)
                print("[kin_teleop] speed x%.2f" % self.scale)

            sl, sa = SPEED_LINEAR * self.scale, SPEED_ANGULAR * self.scale
            tw = np.array([-_dz(a[AXIS_RY]) * sl, _dz(a[AXIS_RX]) * sl,
                           -a[AXIS_DY] * sl,
                           _dz(a[AXIS_LX]) * sa, _dz(a[AXIS_LY]) * sa,
                           (b[BTN_R1] - b[BTN_L1]) * sa])
            if np.any(np.abs(tw) > 1e-9):
                J, R0 = self._jac()
                w = np.zeros(6)
                w[0:3] = R0 @ tw[0:3]
                w[3:6] = R0 @ tw[3:6]
                # floor guard: allow contact, forbid being driven through it
                if w[2] < 0.0 and self._pads_touching():
                    w[2] = 0.0
                dq = np.linalg.pinv(J, rcond=1e-3) @ w
                # cap per-tick joint motion: near a singularity pinv can return
                # very large dq, which teleports the gripper far in one frame
                # and destabilises the finger constraints
                step = np.clip(dq * dt, -MAX_DQ_PER_TICK, MAX_DQ_PER_TICK)
                self.q = [max(-Q_LIMIT, min(Q_LIMIT, self.q[i] + step[i]))
                          for i in range(6)]
                self._write(self.q, vw=w)
                self._moving = True
            elif getattr(self, "_moving", False):
                self._write(self.q)          # one final write with zero velocity
                self._moving = False

            # Trigger polarity is NOT reliable: this DualSense rests at -1.0
            # while gamepad_teleop.py documents +1.0 as released. Hard-coding
            # either one makes the gripper slam shut at startup. Calibrate the
            # rest value from the first real reading and measure deflection.
            r2 = a[AXIS_R2]
            if self.r2_rest is None and abs(r2) > 0.5:
                self.r2_rest = r2
                print("[kin_teleop] R2 rest calibrated at %+.2f" % r2)
            amt = 0.0 if self.r2_rest is None else min(1.0, abs(r2 - self.r2_rest) * 0.5)
            want = GRIP_OPEN + amt * (GRIP_CLOSED - GRIP_OPEN)
            # rate-limit rather than jumping straight to `want`
            lim = GRIP_RATE * dt
            step = max(-lim, min(lim, want - self.grip))
            if self.fj is not None and abs(step) > 1e-5:
                self.grip += step
                tgt = self.grip
                from isaacsim.core.utils.types import ArticulationAction
                # Command ONLY finger_joint. The coupler / follower / spring_link
                # joints are PASSIVE -- they are solved by the four-bar
                # loop-closure equalities. Sending a full joint_positions vector
                # puts position targets on those passive joints, which fights the
                # equalities and blows the gripper up on the first close.
                # gamepad_teleop.py documents exactly this ("Commanding more
                # fought those constraints") and I ignored it.
                # torch tensors, not numpy: the GPU pipeline's backend calls
                # .to(device) on joint_indices, so numpy raises
                # AttributeError: 'numpy.ndarray' object has no attribute 'to'
                import torch
                cur = self.art.get_joint_positions()
                dev = cur.device if hasattr(cur, "device") else "cpu"
                self.art.apply_action(ArticulationAction(
                    joint_positions=torch.tensor([tgt], dtype=torch.float32, device=dev),
                    joint_indices=torch.tensor([self.fj], dtype=torch.long, device=dev)))
        except Exception:
            # Do NOT stop on a transient error. The previous version called
            # stop() here, which unsubscribed and silently killed the controller
            # mid-drive after a single hiccup. Log the first few and carry on.
            self._errs = getattr(self, "_errs", 0) + 1
            if self._errs <= 3:
                print("[kin_teleop] tick error %d:\n%s" % (self._errs, traceback.format_exc()))
            elif self._errs == 20:
                print("[kin_teleop] 20 tick errors; still running, further ones muted")

    def stop(self):
        try:
            self._sub.unsubscribe()
        except Exception:
            pass
        self.pad.close()
        print("[kin_teleop] stopped")


_prev = getattr(carb, "_kinteleop", None)
if _prev:
    try:
        _prev["stop"]()
    except Exception:
        pass
try:
    _t = Teleop()
    carb._kinteleop = {"obj": _t, "stop": _t.stop}
except Exception:
    print("ERR:\n" + traceback.format_exc())
