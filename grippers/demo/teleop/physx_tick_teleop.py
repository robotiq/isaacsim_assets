#!/usr/bin/env python3
"""Tick-paced Cartesian teleop for the UR5e + 2F-85 on the PhysX backend.

Runs INSIDE Isaac. Start it from the Script Editor:

    exec(open(".../grippers/demo/teleop/physx_tick_teleop.py").read())

Stop it with:

    import carb; carb._physx_tick_teleop["stop"]()

WHY THIS EXISTS -- IT REPLACES MoveIt Servo
-------------------------------------------
Servo drove the arm with a ~3.7 Hz limit cycle: the arm shook roughly three
times harder than it advanced. Measured from screen capture, power in the
3-5 Hz band, same scene and same frame rate:

    gamepad through Servo   1.9% of motion energy at 3-5 Hz
    keyboard_jog (no Servo) 0.0%

The cause is a timebase mismatch, not a gain. Servo's control loop is paced by
a WALL-clock rate, so it keeps emitting commands at its configured rate however
slowly the sim runs -- here RTF ~0.45, so it commands roughly twice the motion
per simulated second that the sim can deliver, sees stale feedback, and
overshoots. No gain change removes it: publish_period,
override_velocity_scaling_factor, command magnitude and use_smoothing were all
tried. use_sim_time does not help either -- it fixes message STAMPS, not the
loop's pacing.

This runs on Kit's update tick instead, so controller, physics and clock share
one timebase and the oscillation cannot occur.

HOW IT DRIVES THE ARM
---------------------
The arm is a real PhysX articulation, so this writes joint position TARGETS and
lets the drives track them, rather than posing links directly. Less code, and
contact behaves.

The jog integrates its own target open-loop rather than servoing to measured
positions. That is deliberate: closing the loop on measured state at this frame
rate is what produces the overshoot in the first place. The drives do the
closing, in the physics step, where the feedback is not stale.

Reads /dev/input/js0 directly. No ROS: Isaac's bundled Python and rclpy are
different interpreters, which is also why the launcher insists on a clean shell.
"""
import math
import os
import struct as _struct
import time
import traceback

import numpy as np

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdGeom

ARM = "/World/ur5e"
ARTICULATION = "/World/ur5e/root_joint"
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
FINGER_JOINT = "finger_joint"

# ur_description UR5e joint origins (xyz, rpy); every joint axis is local Z.
ORIGINS = [((0, 0, 0.1625), (0, 0, 0)),
           ((0, 0, 0), (1.570796327, 0, 0)),
           ((-0.425, 0, 0), (0, 0, 0)),
           ((-0.3922, 0, 0.1333), (0, 0, 0)),
           ((0, -0.0997, 0), (1.570796327, 0, 0)),
           ((0, 0.0996, 0), (1.570796326589793, 3.141592653589793,
                             3.141592653589793))]

Q_LIMIT = math.radians(360.0)
MAX_DQ_PER_TICK = math.radians(3.0)
# How far the integrated target may run ahead of where the arm actually is.
# Without this the target winds up without limit whenever the arm is blocked --
# resting on the floor, say -- and the Jacobian is then evaluated at a pose the
# robot is nowhere near, so later commands come out in the wrong direction and
# the arm lunges when it finally comes free. This bounds the target; it is NOT
# closing the control loop at frame rate, which is the Servo failure this file
# exists to avoid.
MAX_LAG = math.radians(15.0)
# Gain (1/s) pulling the tool back onto its intended orientation. With zero
# commanded angular velocity the pseudo-inverse holds orientation only to FIRST
# ORDER, so over finite steps -- and with rcond truncating near singularities --
# it drifts with nothing correcting it: measured 24 degrees over a 34 cm
# vertical move. This closes a loop on the TARGET's own orientation, which is
# kinematic and evaluated at tick rate, so it is not the frame-rate feedback on
# measured physics that made Servo oscillate.
ORI_GAIN = 12.0
# Orientation error below which the hold is considered satisfied. Above it the
# arm moves even with the sticks at rest, which is what makes "point tool down"
# a command rather than a preference that only takes effect once you jog.
ORI_TOL = math.radians(0.4)
GRIP_OPEN, GRIP_CLOSED = 0.0, 0.80
# rad/s, in SIM time. The target is ramped rather than stepped because a step
# input destabilises the four-bar.
#
# Raising this to 2.0 did close the gripper 2.3x faster on the clock (2.13 s ->
# 0.91 s to reach 0.75 rad) but made the grasp unstable in use, which is the
# failure the ramp exists to prevent -- the speed was measured in free air,
# where the four-bar is not loaded. The measurement was right and the change
# was still wrong.
GRIP_RATE = 0.8

AXIS_LX, AXIS_LY, AXIS_L2, AXIS_RX, AXIS_RY, AXIS_R2, AXIS_DX, AXIS_DY = range(8)
BTN_X, BTN_O, BTN_TRI = 0, 1, 2
BTN_SQUARE = 3
BTN_L1, BTN_R1 = 4, 5
# hid-playstation js mapping for the DualSense: 13 buttons, 0..12. The device
# here reports exactly 13, and 0/1/2/4/5 above already match, so the rest follow.
# HELP prints the live button state, which is how to check this on another pad.
BTN_CREATE, BTN_OPTIONS, BTN_PS = 8, 9, 10
BTN_L3, BTN_R3 = 11, 12

# Options alone. It was L1+R1+Options, on the reasoning that a restart should
# be hard to hit by accident -- but Options is not used for anything else and
# sits where no thumb rests mid-jog, so the combo was protecting against a
# press that does not happen. The cost was a three-finger reach for the one
# control you want when the arm is already in trouble.
# Options is the menu button on a console, so it is where people look for a
# control list; Create is the odd one out and does the disruptive thing.
RESTART_BTN = BTN_CREATE
# Cross, not L3. Clicking a thumbstick nudges the stick on the way down, so the
# tool lurched as it levelled -- the two are physically the same control.
TOOL_DOWN_BTN = BTN_X
HELP_BTN = BTN_OPTIONS          # the menu button, right of the touchpad

CONTROLS = [
    ("right stick",      "horizontal move"),
    ("d-pad up/down",    "move the tool up/down (always world vertical)"),
    ("d-pad left/right", "zoom the view on the tool tip"),
    ("left stick",       "roll / pitch the tool"),
    ("L1 / R1",          "yaw the tool"),
    ("Cross",            "point tool down"),
    ("R2",               "open / close the gripper (analog)"),
    ("Create",           "restart the sim"),
    ("Options",          "show or hide this list"),
]
# Re-read the viewport camera at most this often. It is a USD traversal, and
# the camera only changes when someone moves the view.
_CAM_REFRESH_FRAMES = 15
ZOOM_RATE = 1.2          # fraction of the remaining distance per second
ZOOM_MIN = 0.12          # m; do not let the camera reach the tool
ZOOM_MAX = 8.0           # m
ZOOM_MIN_HEIGHT = 0.20   # m; never let the camera sink to bench level

# The links whose midpoint is the tool tip. Zoom and the opening view both aim
# here: the FLANGE is roughly 0.2 m behind the fingers, which is the whole part
# you are trying to look at when you zoom in.
# Distance from the wrist flange to the pads, along the tool's local +z.
#
# There is no prim to read this from. left_fingertip, right_fingertip and the
# *_inner_finger bodies all report origins AT the tool origin -- measured in a
# live session, the midpoint of the "fingertip" bodies sits 13.5 mm from the
# flange on a gripper 160 mm long, and in tool frame it is (0, 0, 0.0002).
# Aiming there is aiming at the flange, which is what zoom kept doing.
#
# The gripper's rendered bounds do reach the right place: projected onto the
# tool axis they extend 0.1609 m. That is measured at startup, with this as the
# fallback when the bounds cannot be read.
TOOL_LENGTH = 0.155
TOOL_LENGTH_RANGE = (0.05, 0.30)     # reject a nonsense measurement
GRIPPER_PRIM = "Robotiq_2F_85_edit"

FRAME_DIST = 0.75        # m from the tip for the opening view
FRAME_ELEV = 22.0        # degrees above horizontal, so zooming out clears the bench
_RESTART_STOP_FRAMES = 30       # let the stop settle before playing again
_RESTART_SETTLE_FRAMES = 45     # let physics step before reseeding the target
DEADZONE = 0.12
SPEED_LINEAR = 0.25
SPEED_ANGULAR = 1.0

JS = _struct.Struct("IhBB")

KEYBOARD_HELP = ("W/S=x  A/D=y  Q/E=z  arrows=roll/pitch  Z/C=yaw  "
                 "SPACE=close gripper  R/F=speed")


def _T(x):
    M = np.eye(4); M[0:3, 3] = x; return M


def _Rx(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4)
    M[1, 1] = c; M[1, 2] = -s; M[2, 1] = s; M[2, 2] = c; return M


def _Ry(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4)
    M[0, 0] = c; M[0, 2] = s; M[2, 0] = -s; M[2, 2] = c; return M


def _Rz(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4)
    M[0, 0] = c; M[0, 1] = -s; M[1, 0] = s; M[1, 1] = c; return M


_ORIG = [_T(xyz) @ _Rz(rpy[2]) @ _Ry(rpy[1]) @ _Rx(rpy[0]) for xyz, rpy in ORIGINS]

# The asset's link frames differ from ur_description's by a base rotation, and
# it is LEFT-multiplied -- it cannot be absorbed into per-link offsets on the
# right. Without it FK is off by 0.47 m here; with it the error is 0.0000 m
# against live physics poses. Omitting it is not a small error: it puts the
# wrist 0.47 m out, and no amount of right-multiplied per-link offsets can
# absorb it.
_BASEROT = _Rz(math.pi)


def fk(q):
    """Arm-relative pose of each link (list of 4x4), q in radians."""
    out, M = [], np.eye(4)
    for i in range(6):
        M = M @ _ORIG[i] @ _Rz(q[i])
        out.append(_BASEROT @ M)
    return out


def _rotvec(Rm):
    """Axis * angle as a 3-vector (log map of a rotation matrix)."""
    c = max(-1.0, min(1.0, (np.trace(Rm) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    return v / (2.0 * math.sin(ang)) * ang


def _expmap(v):
    """Rotation matrix from an axis * angle 3-vector."""
    ang = float(np.linalg.norm(v))
    if ang < 1e-12:
        return np.eye(3)
    k = v / ang
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)


def _tool_down(R_now):
    """Nearest orientation with the tool pointing straight down.

    The tool's own z axis is taken to world -Z, and the remaining freedom --
    rotation about vertical -- is kept as close to the current pose as
    possible, so the wrist does not spin to get there.
    """
    down = np.array([0.0, 0.0, -1.0])
    # Keep the current tool x, projected onto the plane perpendicular to down.
    x = R_now[:, 0] - down * float(np.dot(R_now[:, 0], down))
    if np.linalg.norm(x) < 1e-6:            # tool x was already vertical
        x = R_now[:, 1] - down * float(np.dot(R_now[:, 1], down))
    x = x / np.linalg.norm(x)
    y = np.cross(down, x)
    return np.column_stack([x, y, down])


def _dz(v):
    return 0.0 if abs(v) < DEADZONE else v


class Pad:
    """The DualSense, read straight from the joystick device.

    Reopens itself when the device node is replaced. This is not defensive
    programming for its own sake: trigger_force_feedback.py drives the same
    controller over hidraw, and when it exits -- which it does on every
    run_demo.sh shutdown -- the pad is reset and the kernel re-enumerates the
    joystick. The old descriptor stays readable and simply never delivers
    another event, so the teleop looked completely healthy (subscription alive,
    zero errors, axes present) while ignoring the controller entirely. It cost
    an afternoon to find; noticing it here costs one stat call a second.
    """

    RECHECK_EVERY = 1.0          # seconds between inode checks

    def __init__(self, dev="/dev/input/js0"):
        self.dev = dev
        self.fd = -1
        self._ino = None
        self._checked = 0.0
        self.axes = [0.0] * 12
        self.axes[AXIS_L2] = 1.0
        self.axes[AXIS_R2] = 1.0
        self.buttons = [0] * 16
        self.edges = [0] * 16
        self._open()

    def _open(self):
        self.close()
        self.fd = os.open(self.dev, os.O_RDONLY | os.O_NONBLOCK)
        self._ino = os.fstat(self.fd).st_ino
        self._checked = time.monotonic()

    def _reopen_if_replaced(self):
        now = time.monotonic()
        if now - self._checked < self.RECHECK_EVERY:
            return
        self._checked = now
        try:
            # A new inode at the same path means the device was re-created and
            # our descriptor now points at something nothing writes to.
            if os.stat(self.dev).st_ino != self._ino:
                self._open()
                print("[physx_teleop] controller re-enumerated; reopened %s"
                      % self.dev)
        except Exception:
            pass

    def poll(self):
        self.edges = [0] * 16
        self._reopen_if_replaced()
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
            if self.fd >= 0:
                os.close(self.fd)
        except Exception:
            pass
        self.fd = -1


class Keyboard:
    """Same axes/buttons/edges contract as Pad, so _tick cannot tell them apart."""

    AXIS_KEYS = {"W": (AXIS_RY, -1.0), "S": (AXIS_RY, +1.0),
                 "D": (AXIS_RX, +1.0), "A": (AXIS_RX, -1.0),
                 "Q": (AXIS_DY, -1.0), "E": (AXIS_DY, +1.0),
                 "RIGHT": (AXIS_LX, +1.0), "LEFT": (AXIS_LX, -1.0),
                 "DOWN": (AXIS_LY, +1.0), "UP": (AXIS_LY, -1.0)}
    BUTTON_KEYS = {"C": BTN_R1, "Z": BTN_L1, "R": BTN_TRI, "F": BTN_X}
    GRIP_KEY = "SPACE"

    def __init__(self):
        import carb.input
        import omni.appwindow

        self._ci = carb.input
        self._iface = carb.input.acquire_input_interface()
        self._kb = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = self._iface.subscribe_to_keyboard_events(self._kb, self._on_key)
        self._down = set()
        self.axes = [0.0] * 12
        self.axes[AXIS_L2] = 1.0
        self.axes[AXIS_R2] = 1.0
        self.buttons = [0] * 16
        self.edges = [0] * 16

    def _on_key(self, event, *_):
        try:
            name = getattr(event.input, "name", None) or \
                str(event.input).rsplit(".", 1)[-1]
            T = self._ci.KeyboardEventType
            if event.type == T.KEY_PRESS:
                self._down.add(name)
            elif event.type == T.KEY_RELEASE:
                self._down.discard(name)
        except Exception:
            pass
        return False        # do not consume; Kit shortcuts keep working

    def poll(self):
        prev = list(self.buttons)
        self.axes = [0.0] * 12
        self.axes[AXIS_L2] = 1.0
        self.axes[AXIS_R2] = -1.0 if self.GRIP_KEY in self._down else 1.0
        for k, (i, v) in self.AXIS_KEYS.items():
            if k in self._down:
                self.axes[i] = v
        self.buttons = [0] * 16
        for k, i in self.BUTTON_KEYS.items():
            if k in self._down:
                self.buttons[i] = 1
        self.edges = [1 if (self.buttons[i] and not prev[i]) else 0
                      for i in range(16)]

    def close(self):
        try:
            self._iface.unsubscribe_to_keyboard_events(self._kb, self._sub)
        except Exception:
            pass


def _acquire_input():
    try:
        p = Pad()
        print("[physx_teleop] input: DualSense on /dev/input/js0")
        return p
    except Exception as exc:
        print("[physx_teleop] no gamepad (%s)" % exc)
        print("[physx_teleop] input: KEYBOARD -- " + KEYBOARD_HELP)
        return Keyboard()


class Teleop:
    def __init__(self):
        from isaacsim.core.prims import SingleArticulation

        self.stage = omni.usd.get_context().get_stage()
        self.art = SingleArticulation(ARTICULATION)
        self.art.initialize()
        names = list(self.art.dof_names)
        self.idx = [names.index(n) for n in ARM_JOINTS]
        self.fj = names.index(FINGER_JOINT) if FINGER_JOINT in names else None

        q = self._read(self.art.get_joint_positions())
        # Seed the target from the CURRENT measured pose, then integrate it
        # open-loop. Re-reading measured state every tick would close a loop at
        # frame rate over a sim running at RTF ~0.45 -- which is the Servo
        # failure this replaces.
        self.q = [float(q[i]) for i in self.idx]
        self.grip = float(q[self.fj]) if self.fj is not None else GRIP_OPEN
        self.r2_rest = None
        self._moving = False

        # Static: the arm base is fixed to the world, so reading it once is safe
        # even with Fabric active (which reports AUTHORED transforms for
        # physics-driven prims -- fine here, wrong for moving links).
        m = UsdGeom.Xformable(self.stage.GetPrimAtPath(ARM)) \
            .ComputeLocalToWorldTransform(0)
        self.armbase = np.array([[m[r][c] for c in range(4)]
                                 for r in range(4)]).T

        # The orientation the tool should be holding. Commanded rotation moves
        # it; translation must not.
        self.R_des = (self.armbase @ fk(self.q)[5])[0:3, 0:3]

        self._restart = None        # None, or frames elapsed since the combo
        self._recover = None        # ditto, for an externally driven Play
        self._tl_sub = None

        # How far the tool reaches, measured from the gripper's own rendered
        # bounds projected onto the tool axis. Done once: it is fixed geometry.
        self.tool_len = TOOL_LENGTH
        try:
            # UsdGeom is imported at module level -- importing it here too would
            # make it local to __init__ and shadow the earlier use above.
            from pxr import Usd

            grip = None
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ARM)):
                if prim.GetName() == GRIPPER_PRIM:
                    grip = prim
                    break
            if grip is not None:
                cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                          ["default", "render", "proxy", "guide"])
                rng = cache.ComputeWorldBound(grip).ComputeAlignedRange()
                F = self.armbase @ fk(self.q)[5]
                axis = F[0:3, 0:3] @ np.array([0.0, 0.0, 1.0])
                reach = max(float(np.dot(np.array(c) - F[0:3, 3], axis))
                            for c in (rng.GetMin(), rng.GetMax()))
                if TOOL_LENGTH_RANGE[0] <= reach <= TOOL_LENGTH_RANGE[1]:
                    self.tool_len = reach
        except Exception:
            pass
        print("[physx_teleop] tool length %.3f m along the tool axis"
              % self.tool_len)

        self.pad = _acquire_input()
        self._sub = (omni.kit.app.get_app().get_update_event_stream()
                     .create_subscription_to_pop(self._tick,
                                                 name="physx_tick_teleop"))
        # Stop/Play from Isaac's own toolbar has to be survivable: it is a
        # completely ordinary thing to do mid-demo, and it invalidates the
        # articulation handle exactly as our own restart does.
        try:
            self._tl_sub = (omni.timeline.get_timeline_interface()
                            .get_timeline_event_stream()
                            .create_subscription_to_pop(
                                self._on_timeline,
                                name="physx_tick_teleop_timeline"))
        except Exception:
            print("[physx_teleop] timeline events unavailable; GUI Stop/Play "
                  "will NOT be recovered from:\n" + traceback.format_exc())
        self._frame_tip()
        print("[physx_teleop] running on Kit's update tick (no Servo, no ROS)")
        print("[physx_teleop] press Options for the control list")

    @staticmethod
    def _read(v):
        return np.asarray(v.detach().cpu() if hasattr(v, "detach") else v,
                          dtype=float).ravel()

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

    def _view_basis(self):
        """(right, forward) in world, from the active viewport camera.

        Forward is the camera's view direction FLATTENED onto the horizontal
        plane, so pushing the stick away drives the tool across the floor
        rather than into it -- vertical stays on the d-pad. Falls back to world
        axes if the viewport cannot be read, which keeps the teleop usable in a
        headless or unusual layout rather than failing.
        """
        self._cam_age = getattr(self, "_cam_age", _CAM_REFRESH_FRAMES) + 1
        if self._cam_age < _CAM_REFRESH_FRAMES and getattr(self, "_cam", None):
            return self._cam
        self._cam_age = 0
        basis = (np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        try:
            from omni.kit.viewport.utility import get_active_viewport

            cam = self.stage.GetPrimAtPath(str(get_active_viewport().camera_path))
            m = UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(0)
            M = np.array([[m[r][c] for c in range(4)] for r in range(4)]).T
            right = M[0:3, 0]
            fwd = -M[0:3, 2]                  # USD cameras look down -Z
            right[2] = 0.0
            fwd[2] = 0.0
            if np.linalg.norm(right) > 1e-6 and np.linalg.norm(fwd) > 1e-6:
                basis = (right / np.linalg.norm(right), fwd / np.linalg.norm(fwd))
        except Exception:
            pass
        self._cam = basis
        return basis

    def _toggle_help(self):
        """Show or hide the control diagram, drawn INSIDE the viewport.

        Not a ui.Window. Kit hides floating windows in fullscreen, which is how
        the demo is actually run, so a window is invisible exactly when someone
        needs the controls. ViewportWindow.get_frame() draws into the viewport
        itself and survives.

        Degrades in three steps: the image, then a text list if controls.png is
        missing, then a console dump if omni.ui is unavailable -- the shortcut
        always does something.
        """
        img = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "controls.png") if "__file__" in globals() else None
        try:
            import omni.ui as ui
            from omni.kit.viewport.utility import get_active_viewport_window

            frame = getattr(self, "_help_frame", None)
            if frame is not None:
                frame.visible = not frame.visible
                return

            vp_win = get_active_viewport_window()
            if vp_win is None:
                raise RuntimeError("no active viewport window")
            frame = vp_win.get_frame("physx_teleop_help")
            have_img = bool(img) and os.path.isfile(img)
            with frame:
                # Centred, and sized as a fraction of the viewport so it stays
                # readable whatever the window is.
                with ui.ZStack():
                    ui.Rectangle(style={"background_color": 0xd0101014})
                    with ui.VStack(spacing=8):
                        ui.Spacer()
                        with ui.HStack(height=0):
                            ui.Spacer()
                            if have_img:
                                ui.Image(img, width=1100, height=594,
                                         fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                            else:
                                with ui.VStack(width=520, height=0, spacing=5):
                                    ui.Label("UR5e + 2F-85 teleop",
                                             style={"font_size": 20,
                                                    "color": 0xff66ccff})
                                    for name, what in CONTROLS:
                                        with ui.HStack(height=24, spacing=10):
                                            ui.Label(name, width=170,
                                                     style={"color": 0xffffff00})
                                            ui.Label(what)
                            ui.Spacer()
                        ui.Spacer()
            self._help_frame = frame
        except Exception:
            print("[physx_teleop] controls:")
            for name, what in CONTROLS:
                print("   %-18s %s" % (name, what))

    def _tool_tip(self):
        """World position of the pads: the flange, pushed out along the tool axis.

        Computed rather than read off a prim, because none of the gripper's
        prims are where their names suggest -- see TOOL_LENGTH.
        """
        F = self.armbase @ fk(self.q)[5]
        return F[0:3, 3] + F[0:3, 0:3] @ np.array([0.0, 0.0, self.tool_len])

    def _aim_view(self, eye, target):
        """Point the perspective camera at target from eye."""
        try:
            from pxr import Gf, UsdGeom

            cam = self.stage.GetPrimAtPath("/OmniverseKit_Persp")
            if not cam or not cam.IsValid():
                return False
            f = np.asarray(target, dtype=float) - np.asarray(eye, dtype=float)
            n = float(np.linalg.norm(f))
            if n < 1e-9:
                return False
            f = f / n
            z = -f                                  # USD cameras look down -Z
            r = np.array([-z[1], z[0], 0.0])
            rn = float(np.linalg.norm(r))
            r = np.array([1.0, 0.0, 0.0]) if rn < 1e-6 else r / rn
            u = np.cross(z, r)
            b = math.asin(max(-1.0, min(1.0, -r[2])))
            a = math.atan2(u[2], z[2])
            c = math.atan2(r[1], r[0])
            xf = UsdGeom.Xformable(cam)
            ops = {o.GetOpName(): o for o in xf.GetOrderedXformOps()}
            t, rot = ops.get("xformOp:translate"), ops.get("xformOp:rotateXYZ")
            if t is None or rot is None:
                return False
            t.Set(Gf.Vec3d(*[float(v) for v in eye]))
            rot.Set(Gf.Vec3f(math.degrees(a), math.degrees(b), math.degrees(c)))
            return True
        except Exception:
            return False

    def _frame_tip(self):
        """Put the opening view on the fingertips, from in front and above."""
        tip = self._tool_tip()
        el = math.radians(FRAME_ELEV)
        eye = tip + np.array([0.0, -math.cos(el), math.sin(el)]) * FRAME_DIST
        if self._aim_view(eye, tip):
            print("[physx_teleop] view framed on the tool tip (%.2f m)"
                  % FRAME_DIST)

    def _zoom(self, amount, dt):
        """Dolly the viewport camera toward or away from the tool tip.

        Moves along the camera-to-tool line rather than the camera's own axis,
        so the tip stays where it is on screen instead of sliding off as you
        approach. Geometric rather than linear -- a fixed fraction of the
        remaining distance per second -- so it stays controllable both when
        far out and when close in.

        Writing back has to respect however the camera was authored: Kit's
        perspective camera carries a transform matrix, while an authored camera
        may use translate/orient. Anything else is left alone rather than
        guessed at.
        """
        try:
            from omni.kit.viewport.utility import get_active_viewport

            cam = self.stage.GetPrimAtPath(str(get_active_viewport().camera_path))
            m = UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(0)
            M = np.array([[m[r][c] for c in range(4)] for r in range(4)]).T
            pos = M[0:3, 3]
            tip = self._tool_tip()
            to_tip = tip - pos
            dist = float(np.linalg.norm(to_tip))
            if dist < 1e-6:
                return
            step = dist * (1.0 - math.exp(-ZOOM_RATE * abs(amount) * dt))
            new_dist = dist - step if amount > 0 else dist + step
            new_dist = max(ZOOM_MIN, min(ZOOM_MAX, new_dist))
            # Backing away along this line descends whenever the camera sits
            # below the tool, which is how it ends up inside the table. Cap the
            # distance at whatever keeps it above ZOOM_MIN_HEIGHT instead.
            u = (pos - tip) / dist
            if u[2] < -1e-6:
                limit = (ZOOM_MIN_HEIGHT - tip[2]) / u[2]
                if limit > 0:
                    new_dist = min(new_dist, limit)
            new_pos = tip + u * new_dist
            # Re-aim, do not just dolly. Moving along the camera-to-tip line
            # keeps the tip centred ONLY if the camera already looks exactly at
            # it; after the view has been orbited by hand it does not, so
            # backing off walked the fingers out of frame.
            if self._aim_view(new_pos, tip):
                return

            op = cam.GetAttribute("xformOp:transform")
            if op and op.Get() is not None:
                Gf = __import__("pxr").Gf
                mat = Gf.Matrix4d(op.Get())
                mat.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in new_pos]))
                op.Set(mat)
                return
            op = cam.GetAttribute("xformOp:translate")
            if op and op.Get() is not None:
                Gf = __import__("pxr").Gf
                op.Set(Gf.Vec3d(*[float(v) for v in new_pos]))
        except Exception:
            self._zoomerr = getattr(self, "_zoomerr", 0) + 1
            if self._zoomerr == 1:
                print("[physx_teleop] zoom unavailable:\n" + traceback.format_exc())

    def _actual(self):
        """Measured joint positions, or None while the handle is unusable.

        After a stop the physics simulation view is gone, and
        get_joint_positions() does not raise -- it returns a DEGENERATE array
        (size 1). So a try/except around the read catches nothing, and the
        IndexError lands on the indexing below it instead, outside the guard.
        That is what killed the controller after a GUI Stop -> Play: the first
        failure printed a traceback, and every tick after it failed silently.
        """
        try:
            act = self._read(self.art.get_joint_positions())
        except Exception:
            return None
        need = max(self.idx + ([self.fj] if self.fj is not None else []))
        return act if act.size > need else None

    def _reseed(self):
        """Re-read the arm and adopt it as the target. Used after a restart."""
        act = self._actual()
        if act is None:
            raise RuntimeError("articulation not readable yet")
        self.q = [float(act[j]) for j in self.idx]
        self.grip = float(act[self.fj]) if self.fj is not None else GRIP_OPEN
        self.R_des = (self.armbase @ fk(self.q)[5])[0:3, 0:3]
        self.r2_rest = None

    def _service_restart(self):
        """Drive the stop -> play -> reseed sequence, one step per tick.

        Split across ticks because the timeline needs frames to act on a stop,
        and physics has to step before the articulation can be read back. Doing
        it inline would mean pumping Kit's loop from inside its own callback,
        which corrupts its asyncio state.

        Two things here are load-bearing, both learned the hard way:

        * stop() INVALIDATES the articulation handle. It has to be
          re-initialised after play() or every read raises.
        * the phase must always end. The first version left self._restart set
          when reseeding raised, and _tick returns early while it is set -- so
          one failure meant the controller was dead until Isaac restarted.
          Comparisons are >= and the state is cleared in a finally.
        """
        tl = omni.timeline.get_timeline_interface()
        n = self._restart
        self._restart = n + 1
        try:
            if n == 0:
                tl.stop()
                print("[physx_teleop] restarting sim...")
            elif n == _RESTART_STOP_FRAMES:
                tl.play()
            elif n >= _RESTART_STOP_FRAMES + _RESTART_SETTLE_FRAMES:
                try:
                    self.art.initialize()      # handle is stale after stop/play
                    self._reseed()
                    # Put the view back too. A restart is for when things have
                    # gone wrong, and by then the camera has usually been
                    # orbited somewhere unhelpful as well.
                    self._frame_tip()
                    print("[physx_teleop] sim restarted; target and view reset")
                finally:
                    self._restart = None
        except Exception:
            self._restart = None               # never wedge the controller
            print("[physx_teleop] restart FAILED:\n" + traceback.format_exc())

    def _on_timeline(self, e):
        """Note an externally driven Play so the next tick can re-attach.

        Only PLAY matters. The handle is already dead by the time STOP is
        observed, and nothing may touch the articulation until physics has
        stepped again -- so this records the event and gets out, rather than
        doing the work in the callback.

        Our own Create-button restart drives stop() and play() itself and
        repairs the handle in _service_restart, so its events are ignored
        here; reacting to them as well would run the recovery twice.
        """
        try:
            if self._restart is not None:
                return
            if e.type == int(omni.timeline.TimelineEventType.PLAY):
                self._recover = 0
        except Exception:
            pass

    def _service_recover(self):
        """Re-attach to the articulation after an externally driven Play.

        Deferred a few frames for the same reason the restart sequence is:
        physics has to step before the articulation can be read back.

        The view is deliberately NOT reframed. A Create-button restart is for
        when things have gone wrong and the camera is usually lost too, but a
        Stop/Play is routine -- yanking the view would be its own annoyance.
        """
        n = self._recover
        self._recover = n + 1
        if n < _RESTART_SETTLE_FRAMES:
            return
        try:
            self.art.initialize()      # handle is stale after stop/play
            self._reseed()
            print("[physx_teleop] timeline play -- articulation re-attached")
        except Exception:
            print("[physx_teleop] re-attach FAILED:\n" + traceback.format_exc())
        finally:
            self._recover = None       # never wedge the controller

    def _clamp_to_actual(self):
        """Keep the target within MAX_LAG of the measured pose (anti-windup)."""
        act = self._actual()
        if act is None:
            return
        # Scale the whole lag vector, never clamp per joint. Clamping joints
        # independently changes the ratio between them, which rotates the
        # resulting Cartesian motion -- the same bug that was in the step cap.
        # From the "ready" pose that turned a commanded 0.33 m descent into
        # 0.30 m sideways with 34 degrees of tumble.
        err = np.array([self.q[k] - float(act[j])
                        for k, j in enumerate(self.idx)])
        peak = float(np.max(np.abs(err))) if err.size else 0.0
        if peak > MAX_LAG:
            err = err * (MAX_LAG / peak)
            self.q = [float(act[j]) + err[k] for k, j in enumerate(self.idx)]

    def _apply(self, positions, indices):
        from isaacsim.core.utils.types import ArticulationAction

        self.art.apply_action(ArticulationAction(
            joint_positions=np.asarray(positions, dtype=np.float32),
            joint_indices=np.asarray(indices, dtype=np.int32)))

    def _tick(self, e):
        try:
            # SIM dt, not wall dt. This is the whole point: the jog integrates
            # in the same timebase physics advances in, so commanded motion can
            # never outrun the simulation however slowly it renders.
            dt = e.payload.get("dt", 1.0 / 60.0) or 1.0 / 60.0
            self.pad.poll()
            a, b = self.pad.axes, self.pad.buttons

            if self._restart is not None:
                self._service_restart()
                return
            if self._recover is not None:
                self._service_recover()
                return
            if self.pad.edges[RESTART_BTN]:
                self._restart = 0
                return

            # Point the tool straight down. This only moves the orientation
            # the hold is aiming at -- the existing correction then rotates
            # there over a few ticks, rather than stepping the joints.
            if self.pad.edges[HELP_BTN]:
                self._toggle_help()
                # Also dump the raw state: the button indices below are the
                # standard hid-playstation mapping, and this is how to check
                # them on a pad that numbers them differently.
                print("[physx_teleop] buttons down: %s"
                      % [i for i, v in enumerate(b) if v])

            if self.pad.edges[TOOL_DOWN_BTN]:
                self.R_des = _tool_down(
                    (self.armbase @ fk(self.q)[5])[0:3, 0:3])
                print("[physx_teleop] tool down")

            # d-pad left/right dollies the view; it is not arm motion, so it
            # runs before the twist and does not feed the Jacobian.
            if abs(a[AXIS_DX]) > 0.5:
                self._zoom(a[AXIS_DX], dt)

            sl, sa = SPEED_LINEAR, SPEED_ANGULAR
            # Right stick is VIEW-relative: push away and the tool goes away
            # from you on screen, whatever the camera is doing. Previously it
            # was world x/y, so orbiting the view left the stick pointing the
            # wrong way. Vertical stays world, on the d-pad.
            right, fwd = self._view_basis()
            lin = (fwd * (-_dz(a[AXIS_RY]) * sl)
                   + right * (_dz(a[AXIS_RX]) * sl)
                   + np.array([0.0, 0.0, -a[AXIS_DY] * sl]))
            tw = np.array([lin[0], lin[1], lin[2],
                           _dz(a[AXIS_LX]) * sa, _dz(a[AXIS_LY]) * sa,
                           (b[BTN_R1] - b[BTN_L1]) * sa])

            # The hold has to be able to drive the arm ON ITS OWN. Gating all
            # of this on stick input meant "point tool down" set the target and
            # then waited: R_des changed, nothing acted on it, and the button
            # looked dead unless you happened to be jogging at the same time.
            R_cur = (self.armbase @ fk(self.q)[5])[0:3, 0:3]
            ori_err = _rotvec(self.R_des @ R_cur.T)
            if np.any(np.abs(tw) > 1e-9) or float(np.linalg.norm(ori_err)) > ORI_TOL:
                J, R0 = self._jac()
                w = np.zeros(6)
                # LINEAR motion is world-referenced, ROTATION is tool-referenced.
                #
                # Rotating the linear twist by R0 (as the rotation is) would
                # express it in the tool frame, so "up" would mean "along the
                # tool's z axis" -- which points world-up only while the wrist
                # happens to be oriented that way. Roll the wrist and up/down
                # starts moving the TCP sideways. Leaving it in world means up
                # is always up, whatever the wrist is doing.
                #
                # Rotation stays tool-referenced because that IS the intuitive
                # frame for it: roll/pitch/yaw about the tool's own axes.
                w[0:3] = tw[0:3]
                w[3:6] = R0 @ tw[3:6]

                # Commanded rotation redefines what "held" means; translation
                # leaves R_des alone, so any drift shows up as error below.
                if np.any(np.abs(w[3:6]) > 1e-9):
                    self.R_des = _expmap(w[3:6] * dt) @ self.R_des
                    ori_err = _rotvec(self.R_des @ R_cur.T)
                w[3:6] = w[3:6] + ORI_GAIN * ori_err
                # Damped least squares rather than a truncated pseudo-inverse.
                # pinv(rcond=1e-3) DISCARDS small singular values, and the
                # directions it throws away are exactly the ones holding
                # orientation near an awkward configuration -- measured drift
                # grew from ~0 to ~7 deg as the arm got there. DLS keeps every
                # direction and merely damps the ill-conditioned ones.
                # lam is small on purpose. Damping trades exactness for
                # conditioning, and what it gives up first is the angular part
                # of the request -- i.e. holding orientation. Measured over one
                # vertical move: lam 0.05 -> 4.7 deg of hold error, lam 0.01 ->
                # 1.4 deg.
                lam = 0.01
                JT = J.T
                dq = JT @ np.linalg.solve(J @ JT + (lam ** 2) * np.eye(6), w)
                # Near a singularity pinv returns very large dq. Cap it, but
                # scale the WHOLE vector rather than clipping each joint: per
                # joint clipping changes the ratio between joints, which rotates
                # the resulting Cartesian motion away from what was asked for.
                step = dq * dt
                peak = float(np.max(np.abs(step))) if step.size else 0.0
                if peak > MAX_DQ_PER_TICK:
                    step = step * (MAX_DQ_PER_TICK / peak)
                self.q = [max(-Q_LIMIT, min(Q_LIMIT, self.q[i] + step[i]))
                          for i in range(6)]
                self._clamp_to_actual()
                self._apply(self.q, self.idx)
                self._moving = True
            elif self._moving:
                self._apply(self.q, self.idx)
                self._moving = False

            # Trigger polarity is not reliable across controllers, so calibrate
            # the rest value from the first real reading instead of assuming it.
            r2 = a[AXIS_R2]
            if self.r2_rest is None and abs(r2) > 0.5:
                self.r2_rest = r2
                print("[physx_teleop] R2 rest calibrated at %+.2f" % r2)
            amt = 0.0 if self.r2_rest is None else min(1.0, abs(r2 - self.r2_rest) * 0.5)
            want = GRIP_OPEN + amt * (GRIP_CLOSED - GRIP_OPEN)
            lim = GRIP_RATE * dt
            delta = max(-lim, min(lim, want - self.grip))
            if self.fj is not None and abs(delta) > 1e-5:
                self.grip += delta
                # ONLY finger_joint. The rest of the 2F-85 chain is coupled;
                # driving it directly fights the linkage.
                self._apply([self.grip], [self.fj])
        except Exception:
            self._errs = getattr(self, "_errs", 0) + 1
            if self._errs == 1:
                print("[physx_teleop] tick FAILED:\n" + traceback.format_exc())

    def stop(self):
        try:
            self._sub = None
            self._tl_sub = None
        except Exception:
            pass
        self.pad.close()
        print("[physx_teleop] stopped")


_prev = getattr(carb, "_physx_tick_teleop", None)
if _prev:
    try:
        _prev["stop"]()
    except Exception:
        pass
try:
    _t = Teleop()
    carb._physx_tick_teleop = {"obj": _t, "stop": _t.stop}
except Exception:
    print("ERR:\n" + traceback.format_exc())
