#!/usr/bin/env python3
"""Run the runtime tests of the grippers this branch changed and record the result (see README.md).

    ci/runtime/stamp.py                      # the scopes changed since origin/main
    ci/runtime/stamp.py --scope Robotiq_2F_85 --gui

Writes ci/runtime/stamps/<scope>-<hash>.json; commit it in the PR.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import platform
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stamplib  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def dirty_files(cfg: dict, scope: str) -> list[str]:
    prefixes = cfg["shared"] + cfg["scopes"][scope]["paths"]
    out = stamplib.git(ROOT, "status", "--porcelain", "-z", "--no-renames", "--", *prefixes).decode()
    paths = [entry[3:] for entry in out.split("\0") if entry]
    return [p for p in paths if stamplib.in_scope(cfg, scope, p)]


def run_tests(isaac: Path, scope: str, gui: bool, junit: Path) -> int:
    cmd = [str(isaac / "python.sh"), "-m", "pytest", "tests/runtime", "--gripper", scope, "-v", "-rfEx"]
    cmd += ["--junitxml", str(junit)] + (["--gui"] if gui else [])
    print("+", " ".join(cmd), flush=True)
    # A sourced ROS puts its Python packages on PYTHONPATH, and pytest would load ROS's plugins into Isaac's
    # Python; the ROS 2 test runs its probe in its own clean environment, so the stamp run needs none of it.
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    return subprocess.run(cmd, cwd=ROOT, env=env).returncode


def read_junit(junit: Path) -> dict:
    counts = {"passed": 0, "failed": 0, "errors": 0, "xfailed": 0, "skipped": 0}
    tests = {}
    for case in ET.parse(junit).getroot().iter("testcase"):
        test_id = f"{case.get('classname')}::{case.get('name')}"
        outcome = "passed"
        for child in case:
            if child.tag == "failure":
                outcome = "failed"
            elif child.tag == "error":
                outcome = "errors"
            elif child.tag == "skipped":
                outcome = "xfailed" if child.get("type") == "pytest.xfail" else "skipped"
        counts[outcome] += 1
        tests[test_id] = outcome
    return {**counts, "tests": dict(sorted(tests.items()))}


def environment(isaac: Path) -> dict:
    version = (isaac / "VERSION").read_text().strip() if (isaac / "VERSION").exists() else "unknown"
    try:
        gpu = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"], capture_output=True, text=True
        )
        gpu_line = gpu.stdout.strip().splitlines()[0] if gpu.returncode == 0 and gpu.stdout.strip() else "unknown"
    except FileNotFoundError:
        gpu_line = "unknown"
    return {
        "isaac_sim": version,
        "gpu": gpu_line,
        "os": platform.platform(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scope", action="append", help="scope to stamp (repeatable); default: those changed vs --base")
    parser.add_argument("--base", default="origin/main", help="branch the changes are measured against")
    parser.add_argument("--isaacsim", default=os.environ.get("ISAACSIM_ROOT", "~/isaacsim"), help="Isaac Sim install")
    parser.add_argument("--gui", action="store_true", help="run the tests with the Isaac Sim window")
    args = parser.parse_args()

    cfg = stamplib.load_config((ROOT / stamplib.CONFIG).read_bytes())
    head = stamplib.git(ROOT, "rev-parse", "HEAD").decode().strip()
    if args.scope:
        unknown = sorted(set(args.scope) - set(cfg["scopes"]))
        if unknown:
            sys.exit(f"unknown scope(s) {unknown}; known: {list(cfg['scopes'])}")
        scopes = args.scope
    else:
        base = stamplib.git(ROOT, "merge-base", args.base, "HEAD").decode().strip()
        scopes = stamplib.changed_scopes(ROOT, cfg, base, head)
    if not scopes:
        print("Nothing to stamp: this branch changes no file the runtime tests depend on.")
        return 0
    print("Scopes to stamp:", ", ".join(scopes))

    author = stamplib.git(ROOT, "config", "user.email", check=False).decode().strip() or os.environ.get("USER", "unknown")
    isaac = Path(args.isaacsim).expanduser()

    for scope in scopes:
        dirty = dirty_files(cfg, scope)
        if dirty:
            sys.exit(f"{scope}: uncommitted changes in files the tests depend on, commit them first: {dirty[:5]}")
        digest = stamplib.input_hash(stamplib.scope_files(ROOT, cfg, scope, head))
        with tempfile.TemporaryDirectory() as tmp:
            junit = Path(tmp) / "junit.xml"
            code = run_tests(isaac, scope, args.gui, junit)
            if not junit.exists():
                sys.exit(f"{scope}: pytest produced no report (exit code {code})")
            result = read_junit(junit)
        if code or result["failed"] or result["errors"] or not result["passed"]:
            sys.exit(f"{scope}: the tests did not pass ({result['failed']} failed, {result['errors']} errors); no stamp")

        stamp = {
            "version": 1,
            "scope": scope,
            "input_hash": digest,
            "commit": head,
            "author": author,
            "date": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "environment": environment(isaac),
            "result": result,
        }
        path = ROOT / stamplib.stamp_path(scope, digest)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(stamp, indent=2) + "\n")
        print(f"{scope}: stamped {path.relative_to(ROOT)} ({result['passed']} passed, {result['xfailed']} xfailed)")
        # Only the stamp of the current content is ever read; remove the scope's older ones so they don't pile up.
        old = [
            p for p in stamplib.git(ROOT, "ls-files", "-z", "--", f"{stamplib.STAMPS_DIR}/{scope}-*.json").decode().split("\0")
            if p and p != str(path.relative_to(ROOT))
        ]
        if old:
            stamplib.git(ROOT, "rm", "-q", "--", *old)
            print(f"  removed {len(old)} older stamp(s) of {scope}")

    print("\nCommit the stamp file(s) in the PR (git add -A ci/runtime/stamps, so the removals go with them).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
