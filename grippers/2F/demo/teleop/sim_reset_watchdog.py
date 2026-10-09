#!/usr/bin/env python3
"""
Make teleop survive an Isaac sim Stop -> Play cleanly.

Pressing Stop then Play in Isaac resets the robot to its authored start pose
and restarts /clock from zero. Two pieces of stale state otherwise fight that
reset:

  1. Isaac's action graph LATCHES the last /joint_command and re-applies it
     every frame. So on replay the arm resets, then slowly marches back toward
     the pre-stop pose the stale command still targets — with no joystick input
     at all.
  2. MoveIt Servo keeps integrating twist from its last internal setpoint (the
     pre-stop pose), so the first joystick nudge after replay snaps the arm
     back there. (The same /clock jump-back also latched Servo into NaN.)

This node watches /clock. On a backward jump (the signature of a replay) it:

  * FREEZES the arm: for a short window it republishes the current (reset) pose
    onto /joint_command at high rate, overriding the graph's latched target so
    the arm holds the reset pose instead of drifting back.
  * RESEEDS Servo via stop_servo + start_servo, which re-initialises
    ServoCalcs from the current joint state.

Result: after Stop -> Play the arm stays put, and teleop resumes from the
reset pose. Idempotent and debounced: one reset handled at a time.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# A backward jump larger than this (s) means "sim was reset", not clock jitter.
BACKWARD_JUMP_S = 0.5
# Wait this long after the jump before sampling the hold pose, so /joint_states
# has updated to the post-reset pose rather than the last pre-stop pose.
CAPTURE_DELAY_S = 0.15
# Republish the hold pose for this long, long enough for the graph's latched
# command to be overridden and the drives to settle at the reset pose.
FREEZE_S = 2.0
FREEZE_RATE_HZ = 100.0

ARM = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class SimResetWatchdog(Node):
    def __init__(self):
        super().__init__("sim_reset_watchdog")
        self.last_t = None
        self.latest = {}                 # joint name -> position
        self.reset_t = None              # wall time the current reset was detected
        self.hold = None                 # captured reset pose (arm order) or None
        self.freezing = False
        self.reseeding = False

        # Isaac publishes /clock as BEST_EFFORT; match it or receive nothing.
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(
            JointState, "/joint_states", self.on_joints, qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.stop_cli = self.create_client(Trigger, "/servo_node/stop_servo")
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.create_timer(1.0 / FREEZE_RATE_HZ, self.on_freeze_tick)
        self.get_logger().info(
            "watching /clock for sim resets (freezes arm + reseeds Servo on replay)"
        )

    def on_joints(self, msg: JointState):
        self.latest.update({name: pos for name, pos in zip(msg.name, msg.position)})

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.last_t is not None and t < self.last_t - BACKWARD_JUMP_S:
            self.get_logger().warn(
                f"sim reset detected (clock {self.last_t:.2f}s -> {t:.2f}s); "
                f"freezing arm + reseeding Servo"
            )
            self.reset_t = self._now()
            self.hold = None
            self.freezing = True
            self.reseed()
        self.last_t = t

    def on_freeze_tick(self):
        if not self.freezing or self.reset_t is None:
            return
        elapsed = self._now() - self.reset_t
        if elapsed < CAPTURE_DELAY_S:
            return                                   # let /joint_states settle
        # Release check first, so the freeze always ends after FREEZE_S even if
        # the pose was never captured (e.g. /joint_states never arrived).
        if elapsed > FREEZE_S:
            self.freezing = False
            self.get_logger().info("freeze released — teleop live from reset pose")
            return
        if self.hold is None:
            if not all(j in self.latest for j in ARM):
                return                               # no pose yet; wait
            self.hold = [self.latest[j] for j in ARM]
            self.get_logger().info(
                "holding reset pose (deg): "
                + ", ".join(f"{math.degrees(p):.1f}" for p in self.hold)
            )
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ARM
        out.position = list(self.hold)
        self.cmd_pub.publish(out)

    def reseed(self):
        if self.reseeding:
            return
        self.reseeding = True

        def after_start(_):
            self.get_logger().info("Servo reseeded from current state")
            self.reseeding = False

        def after_stop(_):
            # Non-blocking readiness check: never stall the single-threaded
            # executor waiting on the service (which would freeze on_clock,
            # on_joints, and on_freeze_tick). Clients are created at
            # construction; the servo services are local and normally up.
            if not self.start_cli.service_is_ready():
                self.get_logger().error("start_servo unavailable")
                self.reseeding = False
                return
            self.start_cli.call_async(Trigger.Request()).add_done_callback(after_start)

        if not self.stop_cli.service_is_ready():
            self.get_logger().error("stop_servo unavailable")
            self.reseeding = False
            return
        self.stop_cli.call_async(Trigger.Request()).add_done_callback(after_stop)


def main():
    rclpy.init()
    node = SimResetWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
