#!/usr/bin/env python3
"""Run tests in an Isaac Sim session that is being held open.

Start the session once:
    ~/isaacsim/python.sh -m pytest tests/runtime --gui --hold -k nothing_yet
then run any collected test in it, as often as you like:
    tests/runtime/isaac_run.py "2F_85 and Newton_compliant"
    tests/runtime/isaac_run.py "parallel and 2F_140" --slow 5

The selection works like a simplified pytest -k: words joined by " and " must
all appear in the test id; "not <word>" excludes. Asset layers are re-read
from disk before each run, so asset edits are picked up without a restart
(test code edits still need one). Plain Python 3, no dependencies.
"""

import argparse
import json
import socket
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("k", nargs="?", default="", help="test selection, e.g. '2F_85 and Newton_compliant' (omit to only change settings)")
    ap.add_argument("--slow", type=float, default=None, help="play FACTOR times slower than real time for this run")
    ap.add_argument("--joints", choices=["on", "off"], default=None, help="draw the physics joints in the viewport")
    ap.add_argument("--port", type=int, default=8777)
    ap.add_argument("--timeout", type=float, default=600.0, help="seconds to wait for the run")
    args = ap.parse_args()

    req = {"k": args.k}
    if args.slow is not None:
        req["slow"] = args.slow
    if args.joints is not None:
        req["joints"] = args.joints == "on"
    try:
        with socket.create_connection(("127.0.0.1", args.port), timeout=5.0) as conn:
            conn.sendall((json.dumps(req) + "\n").encode())
            conn.settimeout(args.timeout)
            data = b""
            while not data.endswith(b"\n"):
                chunk = conn.recv(65536)
                if not chunk:
                    break
                data += chunk
    except ConnectionRefusedError:
        print(f"no held Isaac session on 127.0.0.1:{args.port}: start one with "
              "`~/isaacsim/python.sh -m pytest tests/runtime --gui --hold ...`", file=sys.stderr)
        return 2
    reply = json.loads(data.decode() or "{}")
    if "error" in reply:
        print(reply["error"], file=sys.stderr)
        for nodeid in reply.get("available", []):
            print("  ", nodeid, file=sys.stderr)
        return 1
    sys.stdout.write(reply.get("output", ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
