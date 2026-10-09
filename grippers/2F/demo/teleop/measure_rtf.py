#!/usr/bin/env python3
"""Measure real-time factor and frame jitter from Isaac's /clock.

    source /opt/ros/$ROS_DISTRO/setup.bash
    source ../isaac-demo-dds.sh
    python3 measure_rtf.py [seconds]

Reads only /clock, so it needs Isaac playing but no teleop and no Servo.

WHAT THE NUMBERS MEAN
---------------------
Isaac uses fixed time stepping: each rendered frame advances sim time by
exactly one time code, i.e. 1/timeCodesPerSecond. So

    RTF = sim-seconds-per-frame x FPS

and RTF < 1 means frames cost more wall time than the sim time they carry.

p50 vs p99 is the jitter. The main run loop is rate-limited, so a frame that
overruns its budget lands on the next boundary -- 2x, 3x, 6x the interval. That
quantisation is what makes arm motion look stepped rather than merely slow, and
it is invisible in a mean.

MEASURE IN THE FIRST MINUTE AFTER A FRESH BOOT. Sessions degrade: repeated
stop -> play cycles and script execution measurably rot the frame-time tail
(observed here: identical settings, p90 40.6 ms -> 91.3 ms within one session),
so numbers from a long-lived session are not comparable with each other.
"""
import statistics as st
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    n = Node("measure_rtf")
    ts, sim = [], []
    n.create_subscription(
        Clock, "/clock",
        lambda m: (ts.append(time.monotonic()),
                   sim.append(m.clock.sec + m.clock.nanosec / 1e9)), 50)

    print("sampling /clock for %.0fs ..." % dur)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.005)

    if len(ts) < 10:
        print("ERROR: only %d /clock messages. Is Isaac PLAYING?" % len(ts))
        rclpy.shutdown()
        return 1

    d = sorted((b - a) * 1000 for a, b in zip(ts, ts[1:]))
    wall = ts[-1] - ts[0]
    simd = sim[-1] - sim[0]
    spf = simd / len(d) * 1000

    print("")
    print("  sim per frame  %8.2f ms   (= 1/%.0f s)" % (spf, 1000 / spf))
    print("  FPS            %8.1f" % (len(d) / wall))
    print("  RTF            %8.3f   %s" % (
        simd / wall, "<-- 1.0 is real time"))
    print("")
    print("  frame interval p50 %6.1f  p90 %6.1f  p99 %6.1f  max %6.1f ms"
          % (d[len(d) // 2], d[int(len(d) * .9)], d[int(len(d) * .99)], d[-1]))
    print("  stdev %.1f ms   frames over 2x median: %.1f%%"
          % (st.pstdev(d),
             100 * sum(1 for x in d if x > 2 * d[len(d) // 2]) / len(d)))
    print("")
    print("  (jitter is p99/p50; RTF is the speed. They are separate problems.)")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
