#!/usr/bin/env python3
"""
Keyboard teleop for MoveIt Servo (UR5e in Isaac Sim).

Sends TwistStamped on /servo_node/delta_twist_cmds for cartesian jog,
or JointJog on /servo_node/delta_joint_cmds for individual joint jog.
Calls /servo_node/start_servo once on startup so Servo begins streaming.

Controls (authoritative version in the HELP string below):
  Cartesian linear (tool0):  A/D = ±x, W/X = ±y, ↑/↓ = ±z
  Cartesian angular (tool0): I/K = pitch, J/L = yaw, U/O = roll
  Joint jog (per-joint nudge): 1..6 select joint, +/- to nudge
  Gripper: →/← open / close (0.0 / 0.78 rad on finger_joint)
  h: go to ready pose (pauses Servo briefly, drives arm, resumes)
  q or Ctrl-C: quit

The magnitude per keystroke is bounded by Servo's `scale` config; this
node sends ±1.0 unit commands and lets Servo scale.
"""
import os
import sys
import select
import termios
import time
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _servo_compat as compat  # noqa: E402

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Ready pose — same one send_joint_command.py uses.
# Off-singularity so Servo's cartesian mode is well-conditioned right after.
READY_POSE = [0.0, -1.2, 1.2, -1.57, -1.57, 0.0]

# 2F-85 finger DOFs. Servo doesn't touch these — we publish them directly to
# /joint_command alongside Servo's arm output. The Articulation Controller in
# Isaac applies whichever joints each message names, so arm/gripper don't fight.
GRIPPER_JOINTS = [
    # With the Physx_Loop variant active, only the driven joint should be
    # commanded — the rest follow from the 4-bar loop and mimic constraints
    # built into the USD. Commanding all six fought those constraints.
    "finger_joint",
]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.78

HELP = """\
Cartesian linear (frame = tool0):
  A/D   +x/-x
  W/X   +y/-y
  ↑/↓   +z/-z
Cartesian angular (frame = tool0):
  I/K  +rot_y     J/L  +rot_z     U/O  +rot_x
Joint jog:
  1..6 select joint, +/- nudge (- is the key, _ for shift+'-')
Gripper:
  →    open (0.0 rad)
  ←    close (0.78 rad)
Goto:
  h    move to ready pose (pauses servo briefly)
Quit:
  q or Ctrl-C
"""


class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def getch(self, timeout=0.05):
        """Returns a single-character key or a multi-byte arrow label
        ('UP'/'DOWN'/'LEFT'/'RIGHT'). Arrow keys arrive as either
        `ESC [ A/B/C/D` (normal keypad) or `ESC O A/B/C/D` (application
        keypad — common under GNOME Terminal, tmux, some SSH setups).
        A bare ESC press still returns '\\x1b' (no follow-up bytes)."""
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return ""
        c = sys.stdin.read(1)
        if c != "\x1b":
            return c
        # The follow-up bytes for arrow keys can arrive >50 ms after ESC on
        # some terminals (tmux, slow tty pipelines). Use a generous timeout
        # — costs a bit of latency on a bare ESC press, but we don't bind ESC.
        r2, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not r2:
            return "\x1b"
        seq = sys.stdin.read(1)
        # Read second char of the sequence (also with a tolerant timeout).
        r3, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r3:
            seq += sys.stdin.read(1)
        arrows = {
            "[A": "UP",   "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT",
            "OA": "UP",   "OB": "DOWN", "OC": "RIGHT", "OD": "LEFT",
        }
        if seq in arrows:
            return arrows[seq]
        # Unknown escape sequence — surface it so we can extend the table.
        # Drain any remaining bytes (e.g. modified arrows like ESC [ 1 ; 2 A)
        # so the next read starts clean.
        tail = ""
        while True:
            r4, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not r4:
                break
            tail += sys.stdin.read(1)
        return f"UNKNOWN:\\x1b{seq}{tail}"


class ServoKeyboard(Node):
    def __init__(self):
        super().__init__("servo_keyboard_teleop")
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.jog_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", 10
        )
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.pause_cli = self.create_client(Trigger, "/servo_node/pause_servo")
        self.unpause_cli = self.create_client(Trigger, "/servo_node/unpause_servo")
        self.gripper_pub = self.create_publisher(JointState, "/joint_command", 10)
        # Shared with gripper writes — same topic, different joint names per message.
        self.arm_pub = self.gripper_pub
        self.frame = "tool0"
        self.selected_joint = 0
        # Jazzy's Servo accepts exactly ONE command type at a time, so this
        # frontend — which sends both twists and joint jogs — has to switch as
        # the user moves between them. Cached so the service is only called on
        # an actual transition; None means "nothing selected yet". On Humble
        # switching is a no-op and both message types are always accepted.
        self._command_type = None

    def wait_for_sim_time(self, timeout_sec=10.0):
        """With use_sim_time=true, the clock returns 0,0 until the first /clock
        message lands. Sending Twist messages stamped 0,0 makes Servo drop them
        as stale, so block until sim time has populated."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now().to_msg()
            if now.sec > 0:
                self.get_logger().info(f"sim time available: {now.sec}.{now.nanosec:09d}")
                return True
        self.get_logger().warn("sim time never populated; messages may be dropped")
        return False

    def start_servo(self):
        if not self.start_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/servo_node/start_servo not available; assuming already running")
            return
        fut = self.start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        result = fut.result()
        if result and result.success:
            self.get_logger().info(f"servo started: {result.message}")
        else:
            self.get_logger().warn(f"start_servo: {result.message if result else 'no response'}")

    def _ensure_command_type(self, want):
        """Select `want` on Servo if it isn't already the active type."""
        if self._command_type == want:
            return
        # main() spins this node from a background thread.
        if compat.switch_command_type(self, want, spinning=True):
            self._command_type = want

    def send_twist(self, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0)):
        self._ensure_command_type("twist")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.twist.linear.x = float(lin[0])
        msg.twist.linear.y = float(lin[1])
        msg.twist.linear.z = float(lin[2])
        msg.twist.angular.x = float(ang[0])
        msg.twist.angular.y = float(ang[1])
        msg.twist.angular.z = float(ang[2])
        self.twist_pub.publish(msg)

    def send_jog(self, idx, direction):
        self._ensure_command_type("joint_jog")
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.joint_names = [JOINTS[idx]]
        msg.velocities = [float(direction)]
        msg.duration = 0.0
        self.jog_pub.publish(msg)

    def send_gripper(self, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(GRIPPER_JOINTS)
        msg.position = [float(position)] * len(GRIPPER_JOINTS)
        self.gripper_pub.publish(msg)

    def _call_trigger(self, client, label, timeout=2.0):
        if not client.service_is_ready():
            self.get_logger().warn(f"{label}: service not ready")
            return False
        fut = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while time.time() < deadline and not fut.done():
            time.sleep(0.02)
        return bool(fut.done() and fut.result() and fut.result().success)

    def goto_ready(self, hold_seconds=2.0, rate_hz=50):
        """Pause Servo so its 250 Hz hold-position output doesn't fight us,
        publish the ready pose at high rate so the Articulation Controller
        actually drives there, then unpause Servo (which now picks up the
        new current state as its hold target)."""
        self._call_trigger(self.pause_cli, "pause_servo")
        msg = JointState()
        msg.name = JOINTS + GRIPPER_JOINTS
        msg.position = list(READY_POSE) + [GRIPPER_OPEN] * len(GRIPPER_JOINTS)
        t0 = time.time()
        period = 1.0 / rate_hz
        while time.time() - t0 < hold_seconds:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.arm_pub.publish(msg)
            time.sleep(period)
        self._call_trigger(self.unpause_cli, "unpause_servo")


KEY_TWIST = {
    # linear, tool0
    "d":    ("lin", (1, 0, 0)),    # +x
    "a":    ("lin", (-1, 0, 0)),   # -x
    "w":    ("lin", (0, 1, 0)),    # +y
    "x":    ("lin", (0, -1, 0)),   # -y
    "UP":   ("lin", (0, 0, 1)),    # +z
    "DOWN": ("lin", (0, 0, -1)),   # -z
    # angular, tool0
    "o": ("ang", (1, 0, 0)),
    "u": ("ang", (-1, 0, 0)),
    "i": ("ang", (0, 1, 0)),
    "k": ("ang", (0, -1, 0)),
    "l": ("ang", (0, 0, 1)),
    "j": ("ang", (0, 0, -1)),
}


def main():
    rclpy.init()
    node = ServoKeyboard()
    # Sim time has to be ready before either start_servo or any message is sent.
    node.wait_for_sim_time()
    node.start_servo()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(HELP)
    print(f"selected joint: {JOINTS[node.selected_joint]}")
    try:
        with KeyReader() as kb:
            while rclpy.ok():
                k = kb.getch()
                if not k:
                    continue
                if k in ("q", "\x03"):
                    break
                if k in KEY_TWIST:
                    kind, vec = KEY_TWIST[k]
                    if kind == "lin":
                        node.send_twist(lin=vec)
                    else:
                        node.send_twist(ang=vec)
                elif k in "123456":
                    node.selected_joint = int(k) - 1
                    print(f"selected joint: {JOINTS[node.selected_joint]}")
                elif k == "+":
                    node.send_jog(node.selected_joint, +1)
                elif k in ("-", "_"):
                    node.send_jog(node.selected_joint, -1)
                elif k == "RIGHT":
                    node.send_gripper(GRIPPER_OPEN)
                    print("gripper: open")
                elif k == "LEFT":
                    node.send_gripper(GRIPPER_CLOSED)
                    print("gripper: closed")
                elif k == "h":
                    print("going to ready pose…")
                    node.goto_ready()
                    print("ready")
                elif k.startswith("UNKNOWN:"):
                    print(f"\nunbound key (raw bytes): {k[8:]!r}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
