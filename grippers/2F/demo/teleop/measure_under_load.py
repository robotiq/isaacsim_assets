#!/usr/bin/env python3
"""Measure how commanding motion affects Isaac's frame rate.

    source /opt/ros/$ROS_DISTRO/setup.bash
    source ../isaac-demo-dds.sh
    python3 measure_under_load.py            # 10s idle / 10s moving / 10s idle

Needs Isaac playing and ur5e_servo.launch.py up. Does NOT need a gamepad: it
drives Servo itself on /servo_node/delta_twist_cmds, with the same topic, frame
(tool0) and sim-time stamping gamepad_teleop.py uses -- so the load is
reproducible instead of depending on how hard someone pushes a stick.

Three phases, so the idle number is measured either side of the moving one.
That matters because sessions degrade: without the trailing idle phase you
cannot tell a real motion cost from drift that would have happened anyway.

Stamps come from the node clock with use_sim_time, because Servo drops
messages whose stamps are far from its own clock -- wall-time stamps are
billions of seconds in its future and vanish silently (README gotcha #1).
"""
import statistics as st
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

PHASE_S = 10.0
TWIST_HZ = 50.0
SPEED = 0.05          # m/s along tool0 x -- small, to stay clear of limits
FRAME = "tool0"


class Probe(Node):
    def __init__(self):
        super().__init__("measure_under_load")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.ts, self.cmds = [], []
        self.create_subscription(Clock, "/clock",
                                 lambda m: self.ts.append(time.monotonic()), 50)
        self.create_subscription(
            Float64MultiArray, "/forward_position_controller/commands",
            lambda m: self.cmds.append(time.monotonic()), 50)
        self.twist = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.moving = False

    def wait_sim_time(self, timeout=30.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > 0:
                return True
        return False

    def start_servo(self):
        if not self.start_cli.wait_for_service(timeout_sec=5.0):
            print("  WARNING: /servo_node/start_servo absent -- is Servo up?")
            return False
        self.start_cli.call_async(Trigger.Request())
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def pump(self):
        """Publish twist at TWIST_HZ while self.moving."""
        period = 1.0 / TWIST_HZ
        while rclpy.ok():
            if self.moving:
                m = TwistStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = FRAME
                m.twist.linear.x = SPEED
                self.twist.publish(m)
            time.sleep(period)

    def phase(self, label, moving):
        self.ts.clear(); self.cmds.clear()
        self.moving = moving
        t0 = time.monotonic()
        while time.monotonic() - t0 < PHASE_S:
            rclpy.spin_once(self, timeout_sec=0.005)
        self.moving = False
        d = sorted((b - a) * 1000 for a, b in zip(self.ts, self.ts[1:]))
        if len(d) < 5:
            print("  %-14s no /clock data" % label)
            return None
        wall = self.ts[-1] - self.ts[0]
        fps = len(d) / wall
        print("  %-14s FPS %5.1f   RTF %.3f   p50 %5.1f  p90 %5.1f  p99 %6.1f"
              "   servo %5.1f Hz"
              % (label, fps, fps * (1 / 60.0), d[len(d) // 2],
                 d[int(len(d) * .9)], d[int(len(d) * .99)],
                 len(self.cmds) / wall))
        return fps


def main():
    rclpy.init()
    n = Probe()
    if not n.wait_sim_time():
        print("ERROR: no /clock. Is Isaac playing?")
        return 1
    n.start_servo()
    threading.Thread(target=n.pump, daemon=True).start()

    print("phases of %.0fs (twist %.0f Hz, %.2f m/s along %s):"
          % (PHASE_S, TWIST_HZ, SPEED, FRAME))
    a = n.phase("idle (before)", False)
    b = n.phase("MOVING", True)
    c = n.phase("idle (after)", False)

    if a and b and c:
        idle = (a + c) / 2
        print("")
        print("  idle mean FPS %.1f -> moving %.1f   (%+.1f%%)"
              % (idle, b, 100 * (b - idle) / idle))
        drift = 100 * (c - a) / a
        print("  session drift across the run: %+.1f%% (idle before vs after)"
              % drift)
        if abs(drift) > 10:
            print("  ^ drift is large; treat the motion number with suspicion")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
