"""Tests of the sharded stamp.py with a fake python.sh; no Isaac Sim needed."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stamp as stamp_module  # noqa: E402
import stamp_shard  # noqa: E402
from test_runtime_stamp import commit, repo, run, verify  # noqa: E402,F401

FAKE_PYTHON = """#!/usr/bin/env python3
import os, sys

args = sys.argv[1:]
ids = sorted(os.environ.get("FAKE_IDS", "").split(","))
out = args[args.index("--junitxml") + 1]
if os.environ.get("FAKE_ARGS_LOG"):
    with open(os.environ["FAKE_ARGS_LOG"], "a") as f:
        f.write(" ".join(a for a in args if a.startswith("tests/")) + "\\n")
nodes = [a for a in args if a.startswith("tests/runtime/")] or ids
if "--stamp-shard" in args:
    # What stamp_shard.py does: write the whole sorted collection, keep this shard's share.
    k, n = map(int, args[args.index("--stamp-shard") + 1].split("/"))
    if str(k) == os.environ.get("FAKE_EXTRA_SHARD"):
        ids = sorted(ids + ["tests/runtime/test_x.py::test_extra[A]"])
    if not os.environ.get("FAKE_NO_COLLECTED"):
        with open(args[args.index("--stamp-collected") + 1], "w") as f:
            f.write("\\n".join(ids) + "\\n")
    nodes = ids[k::n]
with open(os.environ["FAKE_ENV_LOG"], "a") as log:
    log.write(os.environ["ROS_DOMAIN_ID"] + " " + os.environ.get("PYTHONPATH", "-") + " " + " ".join(args) + "\\n")
scope = args[args.index("--gripper") + 1]
fail = os.environ.get("FAKE_FAIL", "").split(",") if scope == os.environ.get("FAKE_FAIL_SCOPE", scope) else []
skip = os.environ.get("FAKE_SKIP", "").split(",")
drop = os.environ.get("FAKE_DROP", "").split(",")
if nodes and nodes[0] in os.environ.get("FAKE_NO_REPORT", "").split(","):
    sys.exit(0)
if nodes and nodes[0] in os.environ.get("FAKE_CRASH", "").split(","):
    sys.exit(139)
cases = []
for node in nodes:
    if node in drop:
        continue
    module, _, name = node.partition("::")
    cls = module.removesuffix(".py").replace("/", ".")
    body = '<failure message="boom"/>' if node in fail else '<skipped message="s"/>' if node in skip else ""
    cases.append(f'<testcase classname="{cls}" name="{name}">{body}</testcase>')
with open(out, "w") as f:
    f.write("<testsuites><testsuite>" + "".join(cases) + "</testsuite></testsuites>")
if os.environ.get("FAKE_EXIT"):
    sys.exit(int(os.environ["FAKE_EXIT"]))
sys.exit(1 if any(n in fail for n in nodes) else 0 if nodes else 5)
"""

IDS = [f"tests/runtime/test_x.py::test_{i}[A]" for i in range(5)]


def test_ids_round_trip():
    node = "tests/runtime/test_x.py::test_y[A-B]"
    assert stamp_module.node_id("tests.runtime.test_x::test_y[A-B]") == node
    assert stamp_module.test_id(node) == "tests.runtime.test_x::test_y[A-B]"


@pytest.fixture
def sharded(repo, tmp_path, monkeypatch):
    root = repo
    run(root, "git", "config", "user.email", "dev@x")
    run(root, "git", "tag", "base")
    commit(root, "a", {"grippers/A/a.usda": "a2", "ci/runtime/stamp.py": (HERE / "stamp.py").read_text()})
    isaac = tmp_path / "isaac"
    isaac.mkdir()
    (isaac / "VERSION").write_text("6.2.0-rc.14+test\n")
    (isaac / "python.sh").write_text(FAKE_PYTHON)
    (isaac / "python.sh").chmod(0o755)
    env_log = tmp_path / "env.log"
    monkeypatch.setenv("FAKE_IDS", ",".join(IDS))
    monkeypatch.setenv("FAKE_ENV_LOG", str(env_log))

    def go(*extra: str, stamp_py: Path | None = None, cwd: Path = root) -> subprocess.CompletedProcess:
        script = stamp_py or root / "ci/runtime/stamp.py"
        return subprocess.run(
            [sys.executable, str(script), "--root", str(root), "--base", "base", "--isaacsim", str(isaac),
             "--log-dir", str(tmp_path / "logs"), *extra],
            cwd=cwd, capture_output=True, text=True,
        )

    go.root, go.env_log = root, env_log
    return go


def stamps(root: Path, scope: str = "A") -> list[Path]:
    return sorted((root / "ci/runtime/stamps").glob(f"{scope}-*.json")) if (root / "ci/runtime/stamps").exists() else []


def test_sharded_run_writes_one_stamp_per_scope(sharded):
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode == 0, done.stdout + done.stderr
    (path,) = stamps(sharded.root)
    written = json.loads(path.read_text())
    assert written["result"]["passed"] == 5
    assert sorted(written["result"]["tests"]) == sorted(stamp_module.test_id(i) for i in IDS)
    commit(sharded.root, "stamp", {})
    assert verify(sharded.root, "base").returncode == 0


def test_shard_times_are_logged_and_nothing_collects_apart(sharded):
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode == 0, done.stdout + done.stderr
    pattern = re.compile(r"^\[stamp\] \d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\S* (.+)$")
    events = [m.group(1) for m in map(pattern.match, done.stdout.splitlines()) if m]
    for k in (0, 1):
        assert events.index(f"shard-start A-{k}") < events.index(f"shard-end A-{k} exit 0")
    assert len(events) == 4
    assert len(sharded.env_log.read_text().splitlines()) == 2


def test_each_shard_gets_its_own_ros_domain(sharded):
    sharded("--scope", "A", "--scope", "AB", "--shards", "A=2")
    domains = [line.split()[0] for line in sharded.env_log.read_text().splitlines()]
    assert len(domains) == 3 and len(set(domains)) == 3


def test_shard_env_pins_shard_i_to_gpu_i_mod_n(monkeypatch):
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    assert [stamp_module.shard_env(i, 4)["CUDA_VISIBLE_DEVICES"] for i in range(6)] == ["0", "1", "2", "3", "0", "1"]
    assert stamp_module.shard_env(3, 1)["CUDA_VISIBLE_DEVICES"] == "0"


def test_shard_env_leaves_the_gpu_alone_by_default(monkeypatch):
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    assert "CUDA_VISIBLE_DEVICES" not in stamp_module.shard_env(2)
    assert "CUDA_VISIBLE_DEVICES" not in stamp_module.shard_env(2, 0)


def test_gpu_index_runs_across_scopes_and_shows_in_the_event_line(sharded):
    done = sharded("--scope", "A", "--scope", "AB", "--shards", "A=2", "--gpus", "2")
    assert done.returncode == 0, done.stdout + done.stderr
    starts = [line.split(" ", 2)[2] for line in done.stdout.splitlines() if "shard-start" in line]
    assert sorted(starts) == ["shard-start A-0 gpu=0", "shard-start A-1 gpu=1", "shard-start AB-0 gpu=0"]


def test_shards_load_the_plugin_from_the_stamp_py_folder(sharded, monkeypatch, tmp_path):
    monkeypatch.setenv("PYTHONPATH", "/ros/site-packages")
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    done = sharded("--scope", "A", "--shards", "A=2", stamp_py=HERE / "stamp.py", cwd=elsewhere)
    assert done.returncode == 0, done.stdout + done.stderr
    logs = tmp_path / "logs"
    for k, line in enumerate(sorted(sharded.env_log.read_text().splitlines(), key=lambda l: "-0." not in l)):
        _, pythonpath, args = line.split(" ", 2)
        assert pythonpath == str(HERE)
        assert f"-p stamp_shard --stamp-shard {k}/2 --stamp-collected {logs / f'A-{k}.collected'}" in args
        assert " tests/runtime " in f" {args} "


def test_shards_that_collected_different_tests_mean_no_stamp(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_EXTRA_SHARD", "1")
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode != 0
    assert "shards collected different tests" in done.stderr
    assert stamps(sharded.root) == []


def test_a_shard_without_collected_list_means_no_stamp(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_NO_COLLECTED", "1")
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode != 0 and "shard 0 wrote no list of collected tests" in done.stderr
    assert stamps(sharded.root) == []


def test_more_shards_than_tests_still_stamps(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_IDS", IDS[0])
    done = sharded("--scope", "A", "--shards", "A=3")
    assert done.returncode == 0, done.stdout + done.stderr
    (path,) = stamps(sharded.root)
    assert json.loads(path.read_text())["result"]["passed"] == 1


def test_a_shard_without_report_means_no_stamp(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_CRASH", IDS[1])  # first id of shard 1
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode != 0
    assert "A:" in done.stderr and "A-1.log" in done.stderr
    assert stamps(sharded.root) == []


def test_merged_ids_must_match_collected_ids(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_DROP", IDS[3])
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode != 0
    assert "test_3[A]" in done.stderr
    assert stamps(sharded.root) == []


def test_no_skips_refuses_a_skipped_test(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_SKIP", IDS[0])
    done = sharded("--scope", "A", "--shards", "A=2", "--no-skips")
    assert done.returncode != 0 and "skipped" in done.stderr
    assert stamps(sharded.root) == []
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode == 0, done.stderr
    (path,) = stamps(sharded.root)
    assert json.loads(path.read_text())["result"]["skipped"] == 1


def test_a_failing_scope_does_not_stop_the_others_stamps(sharded, monkeypatch):
    monkeypatch.setenv("FAKE_FAIL", IDS[0])
    monkeypatch.setenv("FAKE_FAIL_SCOPE", "AB")
    done = sharded("--scope", "A", "--scope", "AB", "--shards", "A=2", "--shards", "AB=2")
    assert done.returncode == 1
    assert len(stamps(sharded.root, "A")) == 1 and stamps(sharded.root, "AB") == []
    assert "AB:" in done.stderr


def test_root_stamps_another_checkout(sharded, tmp_path):
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    done = sharded("--scope", "A", "--shards", "A=2", stamp_py=HERE / "stamp.py", cwd=elsewhere)
    assert done.returncode == 0, done.stdout + done.stderr
    assert len(stamps(sharded.root)) == 1


def test_a_single_shard_runs_the_runtime_tests_folder_without_the_plugin(sharded, monkeypatch, tmp_path):
    args_log = tmp_path / "args.log"
    monkeypatch.setenv("FAKE_ARGS_LOG", str(args_log))
    done = sharded("--scope", "A")
    assert done.returncode == 0, done.stdout + done.stderr
    assert args_log.read_text().split() == ["tests/runtime"]
    assert "stamp_shard" not in sharded.env_log.read_text()
    assert sharded.env_log.read_text().split()[1] == "-"


def test_an_old_report_in_a_reused_log_dir_is_not_a_result(sharded, monkeypatch, tmp_path):
    logs = tmp_path / "logs"
    logs.mkdir()
    cases = "".join(f'<testcase classname="tests.runtime.test_x" name="test_{i}[A]"/>' for i in (1, 3))
    (logs / "A-1.xml").write_text(f"<testsuites><testsuite>{cases}</testsuite></testsuites>")
    monkeypatch.setenv("FAKE_NO_REPORT", IDS[1])
    done = sharded("--scope", "A", "--shards", "A=2")
    assert done.returncode != 0 and "produced no report" in done.stderr
    assert stamps(sharded.root) == []


def partials(tmp_path: Path) -> list[Path]:
    return sorted((tmp_path / "out").glob("A-*.partial.json"))


def test_keep_partial_on_failures(sharded, monkeypatch, tmp_path):
    monkeypatch.setenv("FAKE_FAIL", IDS[2])
    done = sharded("--scope", "A", "--shards", "A=2", "--keep-partial", str(tmp_path / "out"))
    assert done.returncode == 1
    assert stamps(sharded.root) == []
    (path,) = partials(tmp_path)
    assert len(path.name) == len("A-") + 16 + len(".partial.json")
    written = json.loads(path.read_text())
    assert written["partial"] is True
    assert written["failed"] == [stamp_module.test_id(IDS[2])]
    assert written["result"]["failed"] == 1 and written["result"]["passed"] == 4


@pytest.fixture
def partial(sharded, monkeypatch, tmp_path):
    monkeypatch.setenv("FAKE_FAIL", IDS[2])
    sharded("--scope", "A", "--shards", "A=2", "--keep-partial", str(tmp_path / "out"))
    (path,) = partials(tmp_path)
    return path


def test_complete_turns_a_partial_into_a_stamp(sharded, partial, monkeypatch, tmp_path):
    monkeypatch.delenv("FAKE_FAIL")
    calls = tmp_path / "calls.log"
    monkeypatch.setenv("FAKE_ARGS_LOG", str(calls))
    done = sharded("--complete", str(partial))
    assert done.returncode == 0, done.stdout + done.stderr
    assert calls.read_text().splitlines() == [IDS[2]]
    (path,) = stamps(sharded.root)
    written = json.loads(path.read_text())
    assert "partial" not in written and "failed" not in written
    assert written["result"]["failed"] == 0 and written["result"]["passed"] == 5
    assert written["rerun"]["tests"] == [stamp_module.test_id(IDS[2])]
    assert written["rerun"]["environment"]["isaac_sim"] == "6.2.0-rc.14+test"
    commit(sharded.root, "stamp", {})
    assert verify(sharded.root, "base").returncode == 0


def test_complete_refuses_a_partial_without_failed_tests(sharded, partial, monkeypatch):
    monkeypatch.delenv("FAKE_FAIL")
    written = json.loads(partial.read_text())
    partial.write_text(json.dumps({**written, "failed": []}))
    done = sharded("--complete", str(partial))
    assert done.returncode == 1 and "not a partial result with failed tests" in done.stderr
    assert stamps(sharded.root) == []


def test_complete_refuses_an_isaac_sim_version_that_is_not_accepted(sharded, partial, monkeypatch, tmp_path):
    monkeypatch.delenv("FAKE_FAIL")
    (tmp_path / "isaac" / "VERSION").write_text("6.3.0\n")
    done = sharded("--complete", str(partial))
    assert done.returncode == 1 and "Isaac Sim 6.3.0 here is not one of the accepted versions" in done.stderr
    assert stamps(sharded.root) == []


@pytest.mark.parametrize("extra", [["--complete", "unused.json"], ["--scope", "A"]])
def test_a_missing_python_sh_is_a_clear_error(sharded, tmp_path, extra):
    (tmp_path / "isaac" / "python.sh").unlink()
    done = sharded(*extra)
    assert done.returncode != 0 and "no python.sh in" in done.stderr and "Traceback" not in done.stderr


def test_complete_still_failing_writes_nothing(sharded, partial):
    done = sharded("--complete", str(partial))
    assert done.returncode == 1
    assert stamps(sharded.root) == []


def test_complete_refuses_a_partial_of_other_content(sharded, partial, monkeypatch):
    monkeypatch.delenv("FAKE_FAIL")
    commit(sharded.root, "change", {"grippers/A/a.usda": "a3"})
    done = sharded("--complete", str(partial))
    assert done.returncode == 1 and "describes other content" in done.stderr
    assert stamps(sharded.root) == []



def test_an_interrupted_shard_with_failures_gives_no_partial_and_no_stamp(sharded, monkeypatch, tmp_path):
    monkeypatch.setenv("FAKE_FAIL", f"{IDS[1]},{IDS[2]}")  # one failure in each shard
    monkeypatch.setenv("FAKE_EXIT", "2")
    done = sharded("--scope", "A", "--shards", "A=2", "--keep-partial", str(tmp_path / "out"))
    assert done.returncode == 1 and "exited with code 2" in done.stderr
    assert partials(tmp_path) == [] and stamps(sharded.root) == []


def test_no_skips_with_skips_writes_no_partial(sharded, monkeypatch, tmp_path):
    monkeypatch.setenv("FAKE_FAIL", IDS[2])
    monkeypatch.setenv("FAKE_SKIP", IDS[0])
    done = sharded("--scope", "A", "--shards", "A=2", "--no-skips", "--keep-partial", str(tmp_path / "out"))
    assert done.returncode == 1
    assert partials(tmp_path) == [] and stamps(sharded.root) == []


def test_a_no_skips_partial_records_it_and_complete_refuses_skips(sharded, monkeypatch, tmp_path):
    monkeypatch.setenv("FAKE_FAIL", IDS[2])
    sharded("--scope", "A", "--shards", "A=2", "--no-skips", "--keep-partial", str(tmp_path / "out"))
    (path,) = partials(tmp_path)
    written = json.loads(path.read_text())
    assert written["no_skips"] is True
    monkeypatch.delenv("FAKE_FAIL")
    written["result"]["tests"][stamp_module.test_id(IDS[0])] = "skipped"
    path.write_text(json.dumps(written))
    done = sharded("--complete", str(path))
    assert done.returncode == 1 and "skipped" in done.stderr and "--no-skips" in done.stderr
    assert stamps(sharded.root) == []


def test_complete_refuses_an_unknown_scope(sharded, partial, monkeypatch):
    monkeypatch.delenv("FAKE_FAIL")
    partial.write_text(json.dumps({**json.loads(partial.read_text()), "scope": "Z"}))
    done = sharded("--complete", str(partial))
    assert done.returncode == 1 and "unknown scope 'Z'" in done.stderr
    assert stamps(sharded.root) == []


def test_complete_refuses_a_dirty_tree(sharded, partial, monkeypatch):
    monkeypatch.delenv("FAKE_FAIL")
    (sharded.root / "grippers/A/a.usda").write_text("dirty")
    done = sharded("--complete", str(partial))
    assert done.returncode == 1 and "uncommitted changes" in done.stderr
    assert stamps(sharded.root) == []


@pytest.mark.parametrize("spec", ["A=\u00b2", "A=\u0661", "A=x"])
def test_a_non_ascii_shard_count_is_a_clean_usage_error(sharded, spec):
    done = sharded("--scope", "A", "--shards", spec)
    assert done.returncode != 0 and "--shards expects SCOPE=N" in done.stderr and "Traceback" not in done.stderr

PLUGIN_TESTS = """
import pytest

@pytest.mark.parametrize("i", range(7))
def test_n(i):
    pass
"""

# Stands in for tests/runtime/conftest.py's --gripper filter, which must run before the shard split.
PLUGIN_CONFTEST = """
import pytest

@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config, items):
    items[:] = [i for i in items if not i.nodeid.endswith("[6]")]
"""


def run_plugin(folder: Path, *args: str) -> subprocess.CompletedProcess:
    env = {k: v for k, v in os.environ.items() if not k.startswith("PYTEST_")}
    return subprocess.run(
        [sys.executable, "-m", "pytest", "-p", "stamp_shard", "-p", "no:cacheprovider", "-q", "-rA", "tests", *args],
        cwd=folder, env={**env, "PYTHONPATH": str(HERE)}, capture_output=True, text=True,
    )


@pytest.fixture
def plugin_dir(tmp_path):
    (tmp_path / "tests").mkdir()
    (tmp_path / "tests/test_n.py").write_text(PLUGIN_TESTS)
    (tmp_path / "tests/conftest.py").write_text(PLUGIN_CONFTEST)
    return tmp_path


def passed_ids(done: subprocess.CompletedProcess) -> list[str]:
    return sorted(line.split()[1] for line in done.stdout.splitlines() if line.startswith("PASSED "))


def test_plugin_writes_the_collection_and_keeps_its_share(plugin_dir):
    everything = sorted(f"tests/test_n.py::test_n[{i}]" for i in range(6))
    shares = []
    for k in range(3):
        collected = plugin_dir / f"{k}.collected"
        done = run_plugin(plugin_dir, "--stamp-shard", f"{k}/3", "--stamp-collected", str(collected))
        assert done.returncode == 0, done.stdout + done.stderr
        assert collected.read_text().splitlines() == everything
        shares.append(passed_ids(done))
    assert shares == [everything[k::3] for k in range(3)]


def test_plugin_shard_with_nothing_left_runs_nothing(plugin_dir):
    done = run_plugin(plugin_dir, "--stamp-shard", "6/7", "--stamp-collected", str(plugin_dir / "c"))
    assert done.returncode == 5 and passed_ids(done) == []


def test_plugin_without_options_changes_nothing(plugin_dir):
    done = run_plugin(plugin_dir)
    assert done.returncode == 0 and len(passed_ids(done)) == 6


@pytest.mark.parametrize("spec", ["2/2", "x/2", "1", "0/0"])
def test_plugin_refuses_a_bad_shard(plugin_dir, spec):
    done = run_plugin(plugin_dir, "--stamp-shard", spec)
    assert done.returncode == 4 and "--stamp-shard" in done.stderr


@pytest.mark.parametrize("spec", ["\u0661/2", "1/\u0662", "+1/2", " 1/2", "1/2\n", "1 /2", ""])
def test_parse_shard_takes_ascii_digits_only(spec):
    with pytest.raises(pytest.UsageError):
        stamp_shard.parse_shard(spec)
    assert stamp_shard.parse_shard("1/2") == (1, 2)


def test_shards_collection_splits_on_newlines_only(tmp_path):
    ids = ["tests/runtime/test_x.py::test_a[\u2028]", "tests/runtime/test_x.py::test_b[\x85]"]
    for k in range(2):
        (tmp_path / f"A-{k}.collected").write_text("".join(f"{i}\n" for i in ids))
    assert stamp_module.shards_collection("A", 2, tmp_path) == (ids, None)
