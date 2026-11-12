#!/usr/bin/env python3
"""Run the runtime tests of the grippers this branch changed and record the result (see README.md).

    ci/runtime/stamp.py                      # the scopes changed since origin/main
    ci/runtime/stamp.py --scope Robotiq_2F_85 --gui
    ci/runtime/stamp.py --root /path/to/checkout --scope Robotiq_2F_85 --shards Robotiq_2F_85=4 --no-skips
    ci/runtime/stamp.py --scope Robotiq_2F_85 --shards Robotiq_2F_85=4 --keep-partial out/   # keep what failed
    ci/runtime/stamp.py --complete out/Robotiq_2F_85-<hash>.partial.json   # re-run the failures here, stamp

Writes ci/runtime/stamps/<scope>-<hash>.json; commit it in the PR.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import platform
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stamplib  # noqa: E402

DEFAULT_ROOT = HERE.parents[1]


def dirty_files(root: Path, cfg: dict, scope: str) -> list[str]:
    prefixes = cfg["shared"] + cfg["scopes"][scope]["paths"]
    out = stamplib.git(root, "status", "--porcelain", "-z", "--no-renames", "--", *prefixes).decode()
    paths = [entry[3:] for entry in out.split("\0") if entry]
    return [p for p in paths if stamplib.in_scope(cfg, scope, p)]


def clean_env() -> dict[str, str]:
    # A sourced ROS puts its Python packages on PYTHONPATH, and pytest would load ROS's plugins into Isaac's
    # Python; the ROS 2 test runs its probe in its own clean environment, so the stamp run needs none of it.
    return {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}


def shard_gpu(index: int, gpus: int) -> int | None:
    return index % gpus if gpus >= 1 else None


def shard_env(index: int, gpus: int = 0, plugin: bool = False) -> dict[str, str]:
    # Shards run side by side: each gets its own ROS domain so their ROS 2 tests cannot hear one another.
    env = {**clean_env(), "ROS_DOMAIN_ID": str(20 + index), "ROS_LOCALHOST_ONLY": "1"}
    if plugin:
        env["PYTHONPATH"] = str(HERE)  # for -p stamp_shard, and nothing else
    gpu = shard_gpu(index, gpus)
    if gpu is not None:
        env["CUDA_VISIBLE_DEVICES"] = str(gpu)
    return env


def test_id(node_id: str) -> str:
    path, _, rest = node_id.partition("::")
    return f"{path.removesuffix('.py').replace('/', '.')}::{rest}"


def node_id(test_id: str) -> str:
    module, _, rest = test_id.partition("::")
    return f"{module.replace('.', '/')}.py::{rest}"


def event(text: str) -> None:
    print(f"[stamp] {datetime.datetime.now().astimezone().isoformat(timespec='seconds')} {text}", flush=True)


def merge_results(results: list[dict]) -> dict:
    merged: dict = {key: sum(r[key] for r in results) for key in ("passed", "failed", "errors", "xfailed", "skipped")}
    tests = {}
    for r in results:
        tests.update(r["tests"])
    return {**merged, "tests": dict(sorted(tests.items()))}


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


def start_shard(isaac: Path, root: Path, scope: str, ids: list[str], gui: bool, env: dict, log_dir: Path, k, n: int = 1, gpu: int | None = None):
    cmd = [str(isaac / "python.sh"), "-m", "pytest", *(ids or ["tests/runtime"]), "--gripper", scope, "-v", "-rfEx"]
    cmd += ["--junitxml", str(log_dir / f"{scope}-{k}.xml")] + (["--gui"] if gui else [])
    if n > 1:
        cmd += ["-p", "stamp_shard", "--stamp-shard", f"{k}/{n}", "--stamp-collected", str(log_dir / f"{scope}-{k}.collected")]
    # Files left by an earlier run in a reused log dir must not stand in for this shard's.
    (log_dir / f"{scope}-{k}.xml").unlink(missing_ok=True)
    (log_dir / f"{scope}-{k}.collected").unlink(missing_ok=True)
    print("+", " ".join(cmd), flush=True)
    event(f"shard-start {scope}-{k}" + (f" gpu={gpu}" if gpu is not None else ""))
    with open(log_dir / f"{scope}-{k}.log", "w") as log:
        return subprocess.Popen(cmd, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)


def wait_shard(scope: str, k, proc) -> int:
    code = proc.wait()
    event(f"shard-end {scope}-{k} exit {code}")
    return code


def shards_collection(scope: str, n: int, log_dir: Path) -> tuple[list[str] | None, str | None]:
    """The test ids every shard of a scope collected (the same list in each), or why there is none."""
    lists = []
    for k in range(n):
        path = log_dir / f"{scope}-{k}.collected"
        if not path.exists():
            return None, f"shard {k} wrote no list of collected tests; see {log_dir / f'{scope}-{k}.log'}"
        # Not splitlines(): it would also split a test id on characters such as \x85 or \u2028.
        lists.append([line for line in path.read_text().split("\n") if line])
    if any(ids != lists[0] for ids in lists):
        return None, f"shards collected different tests; see {log_dir}/{scope}-*.collected"
    if not lists[0]:
        return None, "the shards collected no test"
    return lists[0], None


def run_scopes(isaac: Path, root: Path, scopes: list[str], shards: dict[str, int], gui: bool, log_dir: Path, gpus: int = 0):
    """Run every scope's tests, all shards of all scopes at once; returns {scope: (result or None, problems)}."""
    problems: dict[str, list[str]] = {s: [] for s in scopes}
    procs = []  # (scope, k, Popen)
    for scope in scopes:
        # A single shard runs the whole scope: no plugin, no collection check.
        n = shards.get(scope, 1)
        for k in range(n):
            env = shard_env(len(procs), gpus, plugin=n > 1)
            procs.append((scope, k, start_shard(isaac, root, scope, [], gui, env, log_dir, k, n, shard_gpu(len(procs), gpus))))
    reports: dict[str, list[dict]] = {s: [] for s in scopes}
    # Waited in parallel so each shard's end time is logged when it ends, not when the earlier ones are done.
    with ThreadPoolExecutor(max(len(procs), 1)) as pool:
        codes = list(pool.map(lambda p: wait_shard(*p), procs))
    for (scope, k, _), code in zip(procs, codes):
        junit = log_dir / f"{scope}-{k}.xml"
        if not junit.exists():
            problems[scope].append(f"shard {k} produced no report (exit code {code}); see {log_dir / f'{scope}-{k}.log'}")
            continue
        report = read_junit(junit)
        reports[scope].append(report)
        # pytest exits 5 when it runs nothing: a shard left without tests (more shards than tests). Any code but 0, 1
        # and that one (interrupted, internal or usage error) makes the report untrustworthy, failures or not.
        empty_shard = code == 5 and shards.get(scope, 1) > 1 and not report["tests"]
        if code and not empty_shard and (code != 1 or not (report["failed"] or report["errors"])):
            problems[scope].append(f"shard {k} exited with code {code}; see {log_dir / f'{scope}-{k}.log'}")
    out = {}
    for scope in scopes:
        if problems[scope] and not reports[scope]:
            out[scope] = (None, problems[scope])
            continue
        result = merge_results(reports[scope]) if reports[scope] else None
        if result and shards.get(scope, 1) > 1:
            collected, why = shards_collection(scope, shards[scope], log_dir)
            if why:
                problems[scope].append(why)
            else:
                expected = set(map(test_id, collected))
                missing = sorted(expected - set(result["tests"]))
                if missing:
                    problems[scope].append(f"{len(missing)} collected test(s) missing from the reports: {missing[:5]}")
                extra = sorted(set(result["tests"]) - expected)
                if extra:
                    problems[scope].append(f"{len(extra)} reported test(s) that no shard collected: {extra[:5]}")
        out[scope] = (result, problems[scope])
    return out


def write_stamp(root: Path, scope: str, stamp: dict) -> None:
    path = root / stamplib.stamp_path(scope, stamp["input_hash"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(stamp, indent=2) + "\n")
    print(f"{scope}: stamped {path.relative_to(root)} ({stamp['result']['passed']} passed, {stamp['result']['xfailed']} xfailed)")
    # Only the stamp of the current content is ever read; remove the scope's older ones so they don't pile up.
    old = [
        p for p in stamplib.git(root, "ls-files", "-z", "--", f"{stamplib.STAMPS_DIR}/{scope}-*.json").decode().split("\0")
        if p and p != str(path.relative_to(root))
    ]
    if old:
        stamplib.git(root, "rm", "-q", "--", *old)
        print(f"  removed {len(old)} older stamp(s) of {scope}")


def stamp_of(root: Path, cfg: dict, scope: str, head: str, author: str, isaac: Path, result: dict) -> dict:
    return {
        "version": 1,
        "scope": scope,
        "input_hash": stamplib.input_hash(stamplib.scope_files(root, cfg, scope, head)),
        "commit": head,
        "author": author,
        "date": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "environment": environment(isaac),
        "result": result,
    }


def complete(isaac: Path, root: Path, partial_file: Path, log_dir: Path) -> str | None:
    """Re-run the tests a partial result lists as failed, here, and stamp the scope if they pass.
    Returns why it did not, or None."""
    partial = json.loads(partial_file.read_text())
    if not partial.get("partial") or not partial.get("failed"):
        return "not a partial result with failed tests to re-run"
    scope, tests = partial["scope"], partial["result"]["tests"]
    cfg = stamplib.load_config((root / stamplib.CONFIG).read_bytes())
    if scope not in cfg["scopes"]:
        return f"unknown scope {scope!r}"
    head = stamplib.git(root, "rev-parse", "HEAD").decode().strip()
    if stamplib.input_hash(stamplib.scope_files(root, cfg, scope, head)) != partial["input_hash"]:
        return f"the partial describes other content than {scope} at HEAD of {root}"
    dirty = dirty_files(root, cfg, scope)
    if dirty:
        return f"uncommitted changes in files the tests depend on, commit them first: {dirty[:5]}"
    local = environment(isaac)
    if not stamplib.isaac_version_accepted(local["isaac_sim"], cfg["isaac_sim_versions"]):
        return f"Isaac Sim {local['isaac_sim']} here is not one of the accepted versions {cfg['isaac_sim_versions']}"
    failed = partial["failed"]
    proc = start_shard(isaac, root, scope, [node_id(t) for t in failed], False, shard_env(0), log_dir, "rerun")
    code = wait_shard(scope, "rerun", proc)
    junit = log_dir / f"{scope}-rerun.xml"
    if not junit.exists():
        return f"the re-run produced no report (exit code {code}); see {log_dir / f'{scope}-rerun.log'}"
    rerun = read_junit(junit)["tests"]
    bad = [t for t in failed if rerun.get(t) not in ("passed", "xfailed")]
    if bad:
        return f"still not passing here: {bad}"
    merged = {**tests, **{t: rerun[t] for t in failed}}
    counts = {k: sum(o == k for o in merged.values()) for k in ("passed", "failed", "errors", "xfailed", "skipped")}
    if partial.get("no_skips") and counts["skipped"]:
        return f"{counts['skipped']} test(s) skipped and the partial was made with --no-skips"
    stamp = {k: v for k, v in partial.items() if k not in ("partial", "failed", "no_skips")}
    stamp["result"] = {**counts, "tests": dict(sorted(merged.items()))}
    stamp["rerun"] = {"environment": local, "tests": failed}
    write_stamp(root, scope, stamp)
    return None


def require_python_sh(isaac: Path) -> None:
    if not (isaac / "python.sh").is_file():
        sys.exit(f"no python.sh in {isaac}: pass --isaacsim or set ISAACSIM_ROOT to an Isaac Sim install")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scope", action="append", help="scope to stamp (repeatable); default: those changed vs --base")
    parser.add_argument("--base", default="origin/main", help="branch the changes are measured against")
    parser.add_argument("--isaacsim", default=os.environ.get("ISAACSIM_ROOT", "~/isaacsim"), help="Isaac Sim install")
    parser.add_argument("--gui", action="store_true", help="run the tests with the Isaac Sim window")
    parser.add_argument("--root", default=str(DEFAULT_ROOT), help="checkout to test and stamp (default: this one)")
    parser.add_argument("--shards", action="append", default=[], metavar="SCOPE=N", help="parallel pytest processes for a scope (repeatable, default 1)")
    parser.add_argument("--gpus", type=int, default=0, metavar="N", help="pin shard i to GPU i %% N (CUDA_VISIBLE_DEVICES); default 0: leave it alone")
    parser.add_argument("--log-dir", help="where the per-shard reports and logs go (default: a temp dir)")
    parser.add_argument("--keep-partial", metavar="DIR", help="for a scope with failed tests, write its partial result here instead of nothing")
    parser.add_argument("--complete", action="append", default=[], metavar="FILE", help="re-run the failed tests of a partial result here and stamp the scope (repeatable)")
    parser.add_argument("--no-skips", action="store_true", help="refuse a stamp when any test was skipped")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    isaac = Path(args.isaacsim).expanduser()
    if args.complete:
        require_python_sh(isaac)
        log_dir = Path(args.log_dir) if args.log_dir else Path(tempfile.mkdtemp(prefix="runtime-stamp-"))
        log_dir.mkdir(parents=True, exist_ok=True)
        refused = False
        for item in args.complete:
            why = complete(isaac, root, Path(item), log_dir)
            if why:
                refused = True
                print(f"{item}: {why}; no stamp (logs in {log_dir})", file=sys.stderr)
        if not refused:
            print("\nCommit the stamp file(s) in the PR (git add -A ci/runtime/stamps, so the removals go with them).")
        return 1 if refused else 0
    shards: dict[str, int] = {}
    for item in args.shards:
        name, _, count = item.partition("=")
        if not re.fullmatch(r"[0-9]+", count) or int(count) < 1:
            sys.exit(f"--shards expects SCOPE=N with N >= 1, got {item!r}")
        shards[name] = int(count)

    cfg = stamplib.load_config((root / stamplib.CONFIG).read_bytes())
    head = stamplib.git(root, "rev-parse", "HEAD").decode().strip()
    if args.scope:
        unknown = sorted(set(args.scope) - set(cfg["scopes"]))
        if unknown:
            sys.exit(f"unknown scope(s) {unknown}; known: {list(cfg['scopes'])}")
        scopes = args.scope
    else:
        base = stamplib.git(root, "merge-base", args.base, "HEAD").decode().strip()
        scopes = stamplib.changed_scopes(root, cfg, base, head)
    if not scopes:
        print("Nothing to stamp: this branch changes no file the runtime tests depend on.")
        return 0
    unknown = sorted(set(shards) - set(scopes))
    if unknown:
        sys.exit(f"--shards names scope(s) not being stamped: {unknown}")
    print("Scopes to stamp:", ", ".join(scopes))

    author = stamplib.git(root, "config", "user.email", check=False).decode().strip() or os.environ.get("USER", "unknown")
    require_python_sh(isaac)

    for scope in scopes:
        dirty = dirty_files(root, cfg, scope)
        if dirty:
            sys.exit(f"{scope}: uncommitted changes in files the tests depend on, commit them first: {dirty[:5]}")

    log_dir = Path(args.log_dir) if args.log_dir else Path(tempfile.mkdtemp(prefix="runtime-stamp-"))
    log_dir.mkdir(parents=True, exist_ok=True)
    outcomes = run_scopes(isaac, root, scopes, shards, args.gui, log_dir, args.gpus)

    failed = []
    for scope in scopes:
        result, problems = outcomes[scope]
        if result is not None and not problems:
            if result["failed"] or result["errors"] or not result["passed"]:
                problems = [f"the tests did not pass ({result['failed']} failed, {result['errors']} errors)"]
            elif args.no_skips and result["skipped"]:
                problems = [f"{result['skipped']} test(s) skipped and --no-skips is set"]
        if problems:
            failed.append(scope)
            # A partial needs a clean run whose only fault is failures: --no-skips with skips refuses it outright.
            partial_ok = result and (result["failed"] or result["errors"]) and not outcomes[scope][1]
            if args.keep_partial and partial_ok and not (args.no_skips and result["skipped"]):
                keep = Path(args.keep_partial)
                keep.mkdir(parents=True, exist_ok=True)
                partial = stamp_of(root, cfg, scope, head, author, isaac, result)
                partial |= {"partial": True, "failed": [t for t, o in result["tests"].items() if o in ("failed", "errors")]}
                if args.no_skips:
                    partial["no_skips"] = True
                path = keep / f"{scope}-{partial['input_hash'][:16]}.partial.json"
                path.write_text(json.dumps(partial, indent=2) + "\n")
                print(f"{scope}: partial result kept in {path}")
            print(f"{scope}: {'; '.join(problems)}; no stamp (logs in {log_dir})", file=sys.stderr)
            continue
        stamp = stamp_of(root, cfg, scope, head, author, isaac, result)
        write_stamp(root, scope, stamp)

    if failed:
        return 1
    print("\nCommit the stamp file(s) in the PR (git add -A ci/runtime/stamps, so the removals go with them).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
