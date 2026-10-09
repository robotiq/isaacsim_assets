#!/usr/bin/env python3
"""Print the raw bytes your terminal sends for each key press, in raw-tty
mode (same mode the teleop uses). Press q or Ctrl-C to exit."""
import select
import sys
import termios
import tty

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
try:
    tty.setcbreak(fd)
    print("press keys (try arrows), q to quit")
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 1.0)
        if not r:
            continue
        burst = sys.stdin.read(1)
        # Drain anything else that arrived in the same burst
        while True:
            r2, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not r2:
                break
            burst += sys.stdin.read(1)
        print(f"  bytes={burst!r:30s}  hex={' '.join(f'{b:02x}' for b in burst.encode()):20s}  len={len(burst)}")
        if burst == "q":
            break
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
