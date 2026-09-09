#!/usr/bin/env python3
"""
Gamepad teleop for the UR5e + Robotiq 2F-85 in Isaac Sim (via MoveIt Servo).

Subscribes to /joy and translates axes/buttons into the same Servo + gripper
streams the keyboard teleop uses. Designed for a DualShock 4 / DualSense
(Sony Wireless Controller) under the hid-playstation kernel driver. If your
controller maps differently, change AXIS_* / BUTTON_* constants below.

Default mapping (tool0 frame) — see HELP below for the authoritative version:
  Right stick    → linear x/y  (tool0; x sign flipped)
  D-pad up/down  → linear -z / +z
  Left stick     → angular x/y (roll / pitch)
  L1 / R1        → yaw (- / +)
  R2 (analog)    → gripper position (released = open, fully pressed = closed)
  Triangle / Cross → speed × 1.25 / ÷ 1.25
  Circle (○)     → resync servo (stop→start + warmup) after Stop→Play
  Options (☰)    → go to ready pose
  PS / Share     → quit

Servo's incoming_command_timeout is 0.1 s, so we publish only when sticks are
out of deadzone (joy_node's autorepeat_rate keeps the messages flowing while
sticks are held). Releasing all sticks => Servo halts => robot stops.

Run via the gamepad.launch.py wrapper, which also starts joy_node.
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _servo_compat as compat  # noqa: E402


# --- Controller mapping (PS5 DualSense via joy_node's SDL2 GameController) ---
# This controller enumerates through the SDL2 backend: 6 axes, 17 buttons,
# with the D-pad reported as BUTTONS (not axes). An older hid-playstation /
# joydev layout instead gave 8 axes (triggers on 2/5, D-pad on axes 6/7) —
# that mismatch made on_joy() bail every frame, so nothing moved.
AXIS_LX     = 0   # left stick X  (left -1, right +1)
AXIS_LY     = 1   # left stick Y  (up   -1, down  +1)
AXIS_RX     = 2   # right stick X
AXIS_RY     = 3   # right stick Y
AXIS_L2     = 4   # left trigger  (unpressed +1, fully pressed -1)
AXIS_R2     = 5   # right trigger (same convention)
# Buttons (SDL2 GameController order)
BTN_X          = 0    # cross
BTN_O          = 1    # circle
BTN_SQ         = 2    # square
BTN_TRI        = 3    # triangle
BTN_SHARE      = 4
BTN_PS         = 5    # guide
BTN_OPTIONS    = 6
BTN_L1         = 9
BTN_R1         = 10
BTN_DPAD_UP    = 11
BTN_DPAD_DOWN  = 12
BTN_DPAD_LEFT  = 13
BTN_DPAD_RIGHT = 14

# Tuning
DEADZONE       = 0.12    # axis values below this magnitude are zeroed
TRIGGER_THRESH = 0.05    # treat trigger as "pressed" only above this (0..1)
SPEED_LINEAR   = 1.0     # base scaling factor (multiplied by speed_scale at runtime)
SPEED_ANGULAR  = 3.0     # base rad/s — rotations feel slow at 1.0 because a 1m
                         # lever arm moves much less than a 1 m/s linear motion
SPEED_STEP     = 1.25    # multiplier per Triangle/X press
SPEED_MIN      = 0.1
SPEED_MAX      = 5.0
# R2 analog trigger drives the gripper directly: released = fully open,
# fully pressed = fully closed, proportional in between.
GRIPPER_OPEN   = 0.0     # released
GRIPPER_CLOSED = 0.78    # fully pressed

ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
GRIPPER_JOINTS = [
    # Only the driven joint should be commanded — the rest follow from the
    # 4-bar loop and coupling constraints built into the USD. Commanding
    # more fought those constraints.
    "finger_joint",
]
READY_POSE = [0.0, -1.2, 1.2, -1.57, -1.57, 0.0]


def deadzone(v):
    return 0.0 if abs(v) < DEADZONE else v


def trigger_amount(raw):
    """Convert raw trigger axis [+1=unpressed, -1=fully pressed] to [0..1].
    (hid-playstation driver convention — verified on this controller.)"""
    return max(0.0, (1.0 - raw) * 0.5)


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_teleop")
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.start_cli   = self.create_client(Trigger, "/servo_node/start_servo")
        self.pause_cli   = self.create_client(Trigger, "/servo_node/pause_servo")
        self.unpause_cli = self.create_client(Trigger, "/servo_node/unpause_servo")
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.frame = "tool0"
        self.prev_buttons = []
        self.prev_gripper = None  # last commanded gripper position (R2 analog)
        self.r2_seen = False      # R2 has reported a real reading yet (startup guard)
        self.quit = False
        self.speed_scale = 1.0
        # Buttons that call services can't do their work inside on_joy: that
        # runs inside rclpy.spin_once, and neither spin_until_future_complete
        # nor a poll loop can service a response while the only spin is blocked
        # on us. on_joy records the request; main() runs it between spins.
        self.pending_action = None
        self.get_logger().info("gamepad_teleop ready — waiting for /joy")

    # --- helpers ---------------------------------------------------------
    def wait_for_sim_time(self, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().to_msg().sec > 0:
                self.get_logger().info("sim time available")
                return True
        self.get_logger().warn("sim time never populated")
        return False

    def _call(self, client, label, timeout=2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{label}: service not available")
            return False
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        result = fut.result()
        return bool(result and result.success)

    def start_servo(self):
        ok = self._call(self.start_cli, "start_servo")
        self.get_logger().info("servo started" if ok else "start_servo failed")
        # This frontend only ever sends TwistStamped. On Jazzy, Servo boots with
        # no command type selected and rejects every message until told which
        # one to accept; on Humble this is a no-op.
        compat.switch_command_type(self, "twist", spinning=False)

    def resync_servo(self):
        """Fully reset Servo's internal state by stop→start, then warm up its
        integration with a few zero-twist messages so internal_joint_state_
        snaps to the actual current joint state (rather than the stale value
        left over from before the last Stop→Play)."""
        # Build a one-shot stop_servo client (cached on first use).
        if not hasattr(self, "_stop_cli"):
            self._stop_cli = self.create_client(Trigger, "/servo_node/stop_servo")
        self._call(self._stop_cli, "stop_servo")
        self._call(self.start_cli, "start_servo")
        # stop→start may clear the selected command type, and the warm-up burst
        # below is itself twists — reselect before sending them.
        compat.switch_command_type(self, "twist", spinning=False)
        # Warm-up burst: 5 zero-twist messages spaced ~10 ms apart.
        for _ in range(5):
            self.send_twist(0, 0, 0, 0, 0, 0)
            time.sleep(0.01)
        self.get_logger().info("servo resynced (stop→start + zero-twist warmup)")

    def send_twist(self, lx, ly, lz, ax, ay, az):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.twist.linear.x  = float(lx)
        msg.twist.linear.y  = float(ly)
        msg.twist.linear.z  = float(lz)
        msg.twist.angular.x = float(ax)
        msg.twist.angular.y = float(ay)
        msg.twist.angular.z = float(az)
        self.twist_pub.publish(msg)

    def send_gripper(self, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(GRIPPER_JOINTS)
        msg.position = [float(position)] * len(GRIPPER_JOINTS)
        self.joint_pub.publish(msg)

    def run_pending_action(self):
        """Run whatever on_joy asked for, from outside the spin callback."""
        action, self.pending_action = self.pending_action, None
        if action == "resync":
            self.resync_servo()
        elif action == "ready":
            self.get_logger().info("going to ready pose…")
            self.goto_ready()
            self.get_logger().info("ready")

    def goto_ready(self):
        """Pause Servo, hold-publish ready for 2 s, unpause."""
        self._call(self.pause_cli, "pause_servo")
        msg = JointState()
        msg.name = ARM_JOINTS + GRIPPER_JOINTS
        msg.position = list(READY_POSE) + [GRIPPER_OPEN] * len(GRIPPER_JOINTS)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(msg)
            time.sleep(0.02)
        self._call(self.unpause_cli, "unpause_servo")

    def button_edge(self, buttons, idx):
        """True only on the frame the button transitions 0 -> 1."""
        if idx >= len(buttons):
            return False
        was = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return buttons[idx] == 1 and was == 0

    # --- main callback ---------------------------------------------------
    def on_joy(self, msg: Joy):
        ax = msg.axes
        bt = msg.buttons

        # Defensive: axes can be empty briefly at connect
        needed = max(AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, AXIS_L2, AXIS_R2)
        if len(ax) <= needed:
            return

        def btn(i):
            return bt[i] if i < len(bt) else 0

        scale_lin = SPEED_LINEAR  * self.speed_scale
        scale_ang = SPEED_ANGULAR * self.speed_scale

        # Linear x/y from RIGHT stick; z from D-pad up/down buttons (digital,
        # full speed when held). x/z are sign-flipped relative to the raw input
        # to match the tool0 frame convention from the operator's POV in the GUI.
        lin_x = -deadzone(ax[AXIS_RY]) * scale_lin
        lin_y = deadzone(ax[AXIS_RX]) * scale_lin
        dpad_y = btn(BTN_DPAD_UP) - btn(BTN_DPAD_DOWN)   # +1 up, -1 down
        lin_z = -dpad_y * scale_lin

        # Angular: LEFT stick → roll (about x) + pitch (about y);
        # L1 / R1 bumpers → yaw (about z), R1 = +yaw, L1 = -yaw (digital, full speed)
        ang_x = deadzone(ax[AXIS_LX]) * scale_ang   # roll
        ang_y = deadzone(ax[AXIS_LY]) * scale_ang   # pitch
        r1 = bt[BTN_R1] if BTN_R1 < len(bt) else 0
        l1 = bt[BTN_L1] if BTN_L1 < len(bt) else 0
        ang_z = (r1 - l1) * scale_ang

        # Publish twist only when anything is non-zero — keeps Servo idle when
        # sticks are at rest (paired with low_latency_mode=True on Servo).
        if any(abs(v) > 1e-6 for v in (lin_x, lin_y, lin_z, ang_x, ang_y, ang_z)):
            self.send_twist(lin_x, lin_y, lin_z, ang_x, ang_y, ang_z)

        # Gripper: R2 analog trigger → proportional position.
        # Released (amount 0.0) = fully open; fully pressed (1.0) = fully closed.
        # Startup guard: some drivers report the trigger axis as 0 until first
        # touched (which would map to half-closed). Only track R2 once it has
        # reported a real reading (|raw| > 0.5), i.e. seen released (+1) or pressed (-1).
        r2_raw = ax[AXIS_R2]
        if not self.r2_seen and abs(r2_raw) > 0.5:
            self.r2_seen = True
        if self.r2_seen:
            amt = trigger_amount(r2_raw)   # 0..1
            target = GRIPPER_OPEN + amt * (GRIPPER_CLOSED - GRIPPER_OPEN)
            if self.prev_gripper is None or abs(target - self.prev_gripper) > 0.01:
                self.send_gripper(target)
                self.prev_gripper = target
        if self.button_edge(bt, BTN_TRI):
            self.speed_scale = min(SPEED_MAX, self.speed_scale * SPEED_STEP)
            self.get_logger().info(f"speed: {self.speed_scale:.2f}×")
        if self.button_edge(bt, BTN_X):
            self.speed_scale = max(SPEED_MIN, self.speed_scale / SPEED_STEP)
            self.get_logger().info(f"speed: {self.speed_scale:.2f}×")
        if self.button_edge(bt, BTN_O):
            self.pending_action = "resync"
        if self.button_edge(bt, BTN_OPTIONS):
            self.pending_action = "ready"
        if self.button_edge(bt, BTN_PS) or self.button_edge(bt, BTN_SHARE):
            self.get_logger().info("PS/Share pressed — quitting")
            self.quit = True

        self.prev_buttons = list(bt)


HELP = """\
Gamepad teleop (Sony DualShock 4 / DualSense)

  Right stick    →  linear x/y  (tool0; x sign flipped)
  D-pad up/down  →  linear -z / +z  (sign flipped)
  Left stick     →  angular x/y (roll / pitch)
  L1 / R1        →  yaw (- / +)
  R2 (analog)    →  gripper (released = open → fully pressed = closed)
  Triangle (△)   →  speed × 1.25
  Cross (×)      →  speed ÷ 1.25
  Circle (○)     →  resync servo (stop→start + warmup) — use after Stop→Play
  Options (☰)    →  go to ready pose
  PS / Share     →  quit
"""


def main():
    rclpy.init()
    node = GamepadTeleop()
    print(HELP)
    node.wait_for_sim_time()
    node.start_servo()
    try:
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending_action:
                node.run_pending_action()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
