"""Tests of the stamp gate (stamplib.py, verify.py) in a throwaway git repo; no Isaac Sim needed.

    python3 -m pytest ci/runtime
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stamplib  # noqa: E402

CFG = {
    "isaac_sim_versions": ["6.2"],
    "shared": ["tests/runtime"],
    "per_gripper": ["tests/runtime/*/{scope}.json", "tests/runtime/*/{scope}_*.json"],
    "gripper_roots": ["grippers/*"],
    "ignore": ["*.md", "*/.thumbs/*"],
    "scopes": {"A": {"paths": ["grippers/A"]}, "AB": {"paths": ["grippers/AB"]}},
}


def run(repo: Path, *cmd: str) -> str:
    done = subprocess.run(cmd, cwd=repo, capture_output=True, text=True)
    assert done.returncode == 0, f"{cmd}: {done.stderr}"
    return done.stdout.strip()


def commit(repo: Path, message: str, files: dict[str, str]) -> str:
    for name, content in files.items():
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    run(repo, "git", "add", "-A")
    run(repo, "git", "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", message)
    return run(repo, "git", "rev-parse", "HEAD")


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / "repo"
    (root / "ci/runtime").mkdir(parents=True)
    run(root, "git", "init", "-q", "-b", "main")
    for script in ("stamplib.py", "verify.py"):
        shutil.copy(HERE / script, root / "ci/runtime" / script)
    commit(
        root,
        "base",
        {
            "ci/runtime/scopes.json": json.dumps(CFG),
            "grippers/A/a.usda": "a",
            "grippers/AB/b.usda": "b",
            "tests/runtime/t.py": "t",
        },
    )
    return root


def stamp(root: Path, scope: str, cfg: dict = CFG, **override) -> None:
    """What ci/runtime/stamp.py writes, without running the tests."""
    head = run(root, "git", "rev-parse", "HEAD")
    cfg = stamplib.load_config(json.dumps(cfg).encode())
    digest = stamplib.input_hash(stamplib.scope_files(root, cfg, scope, head))
    payload = {
        "scope": scope,
        "input_hash": digest,
        "author": "dev@x",
        "environment": {"isaac_sim": "6.2.0-rc.14"},
        "result": {"passed": 3, "failed": 0, "errors": 0, "xfailed": 0, "skipped": 0},
    }
    payload.update(override)
    path = root / stamplib.stamp_path(scope, digest)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def verify(root: Path, base: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "ci/runtime/verify.py", "--base", base, "--head", "HEAD"], cwd=root, capture_output=True, text=True
    )


def test_scopes_follow_the_changed_files(repo):
    root = repo
    cfg = stamplib.load_config(json.dumps(CFG).encode())
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "a", {"grippers/A/a.usda": "a2"})
    assert stamplib.changed_scopes(root, cfg, base, "HEAD") == ["A"]  # not AB: "grippers/A" is not a prefix of "grippers/AB"
    commit(root, "docs", {"grippers/AB/README.md": "docs", "grippers/AB/.thumbs/256x256/x.png": "png"})
    assert stamplib.changed_scopes(root, cfg, base, "HEAD") == ["A"]  # ignored files change nothing
    commit(root, "shared", {"tests/runtime/t.py": "t2"})
    assert stamplib.changed_scopes(root, cfg, base, "HEAD") == ["A", "AB"]


def test_a_per_gripper_file_counts_only_for_its_gripper(repo):
    root = repo
    cfg = stamplib.load_config(json.dumps(CFG).encode())
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "preset", {"tests/runtime/cubes/A.json": "1"})
    assert stamplib.changed_scopes(root, cfg, base, "HEAD") == ["A"]
    commit(root, "other preset", {"tests/runtime/fixed_cubes/AB_compliant.json": "1"})
    assert stamplib.changed_scopes(root, cfg, base, "HEAD") == ["A", "AB"]  # one file each: A.json, AB_compliant.json
    other = run(root, "git", "rev-parse", "HEAD")
    commit(root, "shared code", {"tests/runtime/_cube.py": "x"})
    assert stamplib.changed_scopes(root, cfg, other, "HEAD") == ["A", "AB"]  # anything else in tests/runtime is shared


def test_the_longest_scope_name_owns_a_per_gripper_file(repo):
    root = repo
    cfg = stamplib.load_config(json.dumps(CFG).encode())
    cfg["scopes"]["A_B"] = {"paths": ["grippers/A_B"]}  # "A_B.json" also matches A's "{scope}_*.json"
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "preset", {"tests/runtime/cubes/A_B.json": "1"})
    assert stamplib.changed_scopes(root, cfg, base, "HEAD") == ["A_B"]


def test_a_changed_folder_in_no_scope_fails(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "docs only", {"grippers/NEW/README.md": "docs"})
    assert verify(root, base).returncode == 0
    commit(root, "new gripper", {"grippers/NEW/n.usda": "n"})
    done = verify(root, base)
    assert done.returncode == 1 and "grippers/NEW is in no scope" in done.stdout


def test_a_removed_gripper_folder_does_not_fail(repo):
    root = repo
    commit(root, "gripper without a scope", {"grippers/OLD/o.usda": "o"})
    base = run(root, "git", "rev-parse", "HEAD")
    run(root, "git", "rm", "-rq", "grippers/OLD")
    commit(root, "remove it", {})
    assert verify(root, base).returncode == 0


def test_a_change_can_add_a_scope_and_then_needs_its_stamp(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    cfg = {**CFG, "scopes": {**CFG["scopes"], "NEW": {"paths": ["grippers/NEW"]}}}
    commit(root, "new gripper", {"grippers/NEW/n.usda": "n", "ci/runtime/scopes.json": json.dumps(cfg)})
    done = verify(root, base)
    assert done.returncode == 1 and "no stamp" in done.stdout  # covered now, but not yet tested
    stamp(root, "NEW", cfg=cfg)
    commit(root, "stamp", {})
    assert verify(root, base).returncode == 0


def test_a_change_cannot_remove_a_scope(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    cfg = {**CFG, "scopes": {"AB": CFG["scopes"]["AB"]}}  # drops A
    commit(root, "a without a stamp", {"grippers/A/a.usda": "a2", "ci/runtime/scopes.json": json.dumps(cfg)})
    assert verify(root, base).returncode == 1  # A is still required: the base's scopes stay


def test_hash_follows_content_not_commits(repo):
    root = repo
    cfg = stamplib.load_config(json.dumps(CFG).encode())
    first = stamplib.input_hash(stamplib.scope_files(root, cfg, "A", "HEAD"))
    commit(root, "unrelated", {"grippers/AB/b.usda": "b2"})
    assert stamplib.input_hash(stamplib.scope_files(root, cfg, "A", "HEAD")) == first
    commit(root, "related", {"grippers/A/a.usda": "a2"})
    assert stamplib.input_hash(stamplib.scope_files(root, cfg, "A", "HEAD")) != first


def test_no_stamp_needed_when_nothing_relevant_changed(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "docs", {"grippers/A/docs/README.md": "docs"})
    assert verify(root, base).returncode == 0


def test_missing_stamp_fails_and_valid_stamp_passes(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "a", {"grippers/A/a.usda": "a2"})
    missing = verify(root, base)
    assert missing.returncode == 1 and "no stamp" in missing.stdout
    stamp(root, "A")
    commit(root, "stamp", {})
    assert verify(root, base).returncode == 0


def test_only_the_changed_gripper_needs_a_stamp(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "ab", {"grippers/AB/b.usda": "b2"})
    stamp(root, "AB")
    commit(root, "stamp", {})
    assert verify(root, base).returncode == 0  # A did not change: no stamp for it


def test_a_stamp_is_void_once_the_content_changes(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "a", {"grippers/A/a.usda": "a2"})
    stamp(root, "A")
    commit(root, "stamp", {})
    commit(root, "more", {"grippers/A/a.usda": "a3"})
    assert verify(root, base).returncode == 1


@pytest.mark.parametrize(
    "override",
    [
        {"result": {"passed": 3, "failed": 1, "errors": 0}},
        {"result": {"passed": 0, "failed": 0, "errors": 0}},
        {"environment": {"isaac_sim": "6.0.1"}},
        {"environment": {"isaac_sim": "6.20.0"}},  # not an accepted "6.2"
    ],
)
def test_a_stamp_of_a_bad_run_is_refused(repo, override):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "a", {"grippers/A/a.usda": "a2"})
    stamp(root, "A", **override)
    commit(root, "stamp", {})
    assert verify(root, base).returncode == 1


def test_not_enforced_before_the_config_exists(tmp_path):
    root = tmp_path / "repo"
    (root / "ci/runtime").mkdir(parents=True)
    run(root, "git", "init", "-q", "-b", "main")
    for script in ("stamplib.py", "verify.py"):
        shutil.copy(HERE / script, root / "ci/runtime" / script)
    base = commit(root, "base", {"x": "x"})
    commit(root, "next", {"x": "y"})
    assert verify(root, base).returncode == 0


FAKE_PYTHON = """#!/bin/bash
# stands in for Isaac's python.sh: write the junit report pytest would
while [ $# -gt 0 ]; do [ "$1" = --junitxml ] && out="$2"; shift; done
cat > "$out" <<XML
<testsuites><testsuite>
<testcase classname="tests.runtime.test_x" name="test_ok[A]"/>
<testcase classname="tests.runtime.test_x" name="test_x[A]"><skipped type="pytest.xfail" message="known"/></testcase>
{extra}
</testsuite></testsuites>
XML
exit {code}
"""


def fake_isaac(path: Path, extra: str = "", code: int = 0) -> Path:
    path.mkdir()
    (path / "VERSION").write_text("6.2.0-rc.14+test\n")
    (path / "python.sh").write_text(FAKE_PYTHON.format(extra=extra, code=code))
    (path / "python.sh").chmod(0o755)
    return path


def run_stamp(root: Path, isaac: Path, *extra: str) -> subprocess.CompletedProcess:
    run(root, "git", "config", "user.email", "dev@x")
    return subprocess.run(
        [sys.executable, "ci/runtime/stamp.py", "--base", "base", "--isaacsim", str(isaac), *extra],
        cwd=root,
        capture_output=True,
        text=True,
    )


def test_stamp_script_writes_a_stamp_the_gate_accepts(repo, tmp_path):
    root = repo
    shutil.copy(HERE / "stamp.py", root / "ci/runtime/stamp.py")
    run(root, "git", "tag", "base")
    commit(root, "a", {"grippers/A/a.usda": "a2", "ci/runtime/stamp.py": (HERE / "stamp.py").read_text()})
    done = run_stamp(root, fake_isaac(tmp_path / "isaac"))
    assert done.returncode == 0, done.stdout + done.stderr
    stamp_file = next((root / "ci/runtime/stamps").glob("A-*.json"))
    written = json.loads(stamp_file.read_text())
    assert written["result"]["passed"] == 1 and written["result"]["xfailed"] == 1
    assert written["environment"]["isaac_sim"].startswith("6.2")
    assert list((root / "ci/runtime/stamps").glob("AB-*")) == []  # only the changed scope
    commit(root, "stamp", {})
    assert verify(root, "base").returncode == 0


def test_stamp_script_refuses_to_stamp_a_failing_run(repo, tmp_path):
    root = repo
    run(root, "git", "tag", "base")
    commit(root, "a", {"grippers/A/a.usda": "a2", "ci/runtime/stamp.py": (HERE / "stamp.py").read_text()})
    failing = '<testcase classname="tests.runtime.test_x" name="test_bad[A]"><failure message="boom"/></testcase>'
    done = run_stamp(root, fake_isaac(tmp_path / "isaac", extra=failing, code=1))
    assert done.returncode != 0 and "did not pass" in done.stderr
    assert not (root / "ci/runtime/stamps").exists()


def test_stamp_script_refuses_uncommitted_changes(repo, tmp_path):
    root = repo
    run(root, "git", "tag", "base")
    commit(root, "a", {"grippers/A/a.usda": "a2", "ci/runtime/stamp.py": (HERE / "stamp.py").read_text()})
    (root / "grippers/A/a.usda").write_text("dirty")
    done = run_stamp(root, fake_isaac(tmp_path / "isaac"))
    assert done.returncode != 0 and "uncommitted" in done.stderr


def test_a_stamp_that_describes_other_content_is_refused(repo):
    root = repo
    base = run(root, "git", "rev-parse", "HEAD")
    commit(root, "a", {"grippers/A/a.usda": "a2"})
    stamp(root, "A", input_hash="0" * 64)  # right file name, wrong recorded hash
    commit(root, "stamp", {})
    assert verify(root, base).returncode == 1


def test_stamp_script_removes_the_older_stamps_of_its_scope(repo, tmp_path):
    root = repo
    run(root, "git", "tag", "base")
    commit(root, "old stamps", {"ci/runtime/stamps/A-0123456789abcdef.json": "{}", "ci/runtime/stamps/AB-0123456789abcdef.json": "{}"})
    commit(root, "a", {"grippers/A/a.usda": "a2", "ci/runtime/stamp.py": (HERE / "stamp.py").read_text()})
    done = run_stamp(root, fake_isaac(tmp_path / "isaac"))
    assert done.returncode == 0, done.stdout + done.stderr
    names = sorted(p.name for p in (root / "ci/runtime/stamps").glob("*.json"))
    assert "AB-0123456789abcdef.json" in names  # another scope's stamp stays
    assert "A-0123456789abcdef.json" not in names and len([n for n in names if n.startswith("A-")]) == 1
    run(root, "git", "add", "-A", "ci/runtime/stamps")
    commit(root, "stamp", {})
    assert verify(root, "base").returncode == 0
