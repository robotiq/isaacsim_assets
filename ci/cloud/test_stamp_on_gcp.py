"""Tests of the GCP launcher with a fake command runner; no gcloud, no ssh, no Isaac Sim."""

from __future__ import annotations

import io
import json
import re
import os
import shutil
import signal
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stamp_on_gcp as gcp  # noqa: E402


def git(root: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(root), *args], check=True, capture_output=True, text=True).stdout


@pytest.fixture
def target(tmp_path):
    root = tmp_path / "target"
    root.mkdir()
    git(root, "init", "-q", "-b", "main")
    git(root, "config", "user.email", "dev@example.com")
    git(root, "config", "user.name", "Dev")
    (root / "file.txt").write_text("x\n")
    config = root / "ci/runtime/scopes.json"
    config.parent.mkdir(parents=True)
    scopes = {n: {"paths": [f"grippers/{n}"]} for n in ("A", "B", "Robotiq_2F_85", "Robotiq_Hand_E_C10")}
    config.write_text(json.dumps({"scopes": scopes}))
    git(root, "add", "-A")
    git(root, "commit", "-qm", "init")
    return root


@pytest.fixture(autouse=True)
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("ISAAC_STAMP_GCP_PROJECT", "example-project")


class Fake:
    """Records every command; `fail(pred, exc)` makes matching commands raise, `on(pred, fn)` scripts a result."""

    def __init__(self):
        self.calls: list[list[str]] = []
        self.hooks: list[tuple] = []
        self.timeouts: list[float] = []
        self.inputs: list[str | None] = []
        self.on(lambda c: c[:4] == ["gcloud", "config", "get", "account"], lambda c: "jane.doe@example.com\n")
        self.on(lambda c: c[0] == "ssh-keygen", self.keygen)
        self.on(lambda c: c[0] == "ssh" and "clone -q /opt/repo" in c[-1], lambda c: "main=" + "0" * 40 + "\n")

    @staticmethod
    def keygen(cmd):
        key = Path(cmd[cmd.index("-f") + 1])
        key.write_text("PRIVATE\n")
        key.with_suffix(".pub").write_text("ssh-ed25519 AAAAfake stamp\n")
        return ""

    def on(self, pred, fn):
        self.hooks.insert(0, (pred, fn))

    def fail(self, pred, exc):
        def raiser(cmd):
            raise exc
        self.on(pred, raiser)

    def run(self, cmd, *, check=True, capture=False, stdout_only=False, timeout=180, input=None):
        """A hook returns stdout, or an (stdout, stderr) pair; stderr is only seen when stdout and stderr are merged."""
        self.calls.append(list(cmd))
        self.timeouts.append(timeout)
        self.inputs.append(input)
        for pred, fn in self.hooks:
            if pred(cmd):
                out = fn(cmd)
                if isinstance(out, tuple):
                    return out[0] if stdout_only else out[0] + out[1]
                return out
        return ""

    def matching(self, *words):
        return [c for c in self.calls if all(w in c for w in words)]


def is_(*words):
    return lambda c: all(w in c for w in words)


def ssh_with(text):
    return lambda c: c[0] == "ssh" and text in c[-1]


def vm_of(fake):
    return fake.matching("instances", "create")[-1][4]


def proxy_of(call):
    (proxy,) = [o[len("ProxyCommand="):] for o in call if o.startswith("ProxyCommand=")]
    return proxy


def iap_proxy(name, project, zone):
    return (f"gcloud compute start-iap-tunnel {name} 22 --listen-on-stdin --project={project} --zone={zone} "
            "--verbosity=warning")


def remote_commands(fake):
    return [c[-1] for c in fake.calls if c[0] == "ssh"]


def require_git_lfs():
    # Shipping the code lists the target's LFS files with git lfs.
    if shutil.which("git-lfs") is None:
        pytest.skip("git-lfs is not installed")


def launch(fake, target, *extra, scopes=("A",)):
    require_git_lfs()
    argv = ["--target", str(target), *(a for s in scopes for a in ("--scope", s)), *extra]
    return gcp.main(argv, runner=fake, sleep=lambda s: None)


def test_create_command_has_every_guard(target):
    fake = Fake()
    launch(fake, target)
    (create,) = fake.matching("instances", "create")
    for flag in [
        "--project=example-project", "--machine-type=n1-standard-64", "--accelerator=type=nvidia-tesla-t4,count=4",
        "--provisioning-model=STANDARD", "--instance-termination-action=DELETE", "--max-run-duration=2h",
        "--no-service-account", "--no-scopes", "--image-family=isaacsim-6-2", "--image-project=example-project",
        "--network=isaac-runtime-stamp", "--subnet=isaac-runtime-stamp-us-central1", "--tags=isaac-runtime-stamp",
        "--labels=purpose=isaac-runtime-stamp,owner=jane-doe", "--maintenance-policy=TERMINATE",
        "--zone=us-central1-b", "--boot-disk-type=pd-ssd",
    ]:
        assert flag in create
    assert create[create.index("create") + 1].startswith("isaac-stamp-jane-doe-")
    (metadata,) = [f for f in create if f.startswith("--metadata=")]
    assert metadata.startswith("--metadata=ssh-keys=stamp:ssh-ed25519 AAAAfake stamp")
    assert "enable-oslogin=FALSE" in metadata and "block-project-ssh-keys=TRUE" in metadata
    assert not [c for c in fake.calls if c[:3] in (["gcloud", "compute", "ssh"], ["gcloud", "compute", "scp"])]


def test_refuses_when_a_stamp_vm_exists(target):
    fake = Fake()
    fake.on(is_("instances", "list"), lambda c: "isaac-stamp-someone-20261111-101010\n")
    with pytest.raises(SystemExit) as stop:
        launch(fake, target)
    assert stop.value.code
    assert not fake.matching("create")


def test_a_gcloud_warning_is_not_an_existing_vm(target):
    fake = Fake()
    fake.on(is_("instances", "list"), lambda c: ("", "WARNING: The following filter keys were not present in any resource : labels.purpose\n"))
    launch(fake, target)
    assert fake.matching("create")


def test_refuses_a_dirty_target(target):
    (target / "file.txt").write_text("changed\n")
    fake = Fake()
    with pytest.raises(SystemExit) as stop:
        launch(fake, target)
    assert stop.value.code
    assert fake.calls == []


def test_dirty_stamps_do_not_block(target):
    stamps = target / "ci/runtime/stamps"
    stamps.mkdir(parents=True)
    (stamps / "A-old.json").write_text("{}")
    git(target, "add", "-A")
    git(target, "commit", "-qm", "stamp")
    (stamps / "A-old.json").write_text('{"a": 1}')
    fake = Fake()
    launch(fake, target)
    assert fake.matching("instances", "create")


def test_vm_is_deleted_when_a_step_fails(target):
    fake = Fake()
    fake.fail(ssh_with("stamp.py"), RuntimeError("boom"))
    with pytest.raises(RuntimeError):
        launch(fake, target)
    assert fake.calls[-1][:4] == ["gcloud", "compute", "instances", "delete"]
    for flag in ["--quiet", "--zone=us-central1-b"]:
        assert flag in fake.calls[-1]
    assert "--async" not in fake.calls[-1]


def test_a_failed_delete_is_loud_and_fails_the_launcher(target, capsys):
    fake = Fake()
    fake.fail(is_("instances", "delete"), gcp.CommandError(["delete"], "ERROR: permission denied", 1))
    assert launch(fake, target) != 0
    err = capsys.readouterr().err
    assert "ERROR:" in err
    assert "gcloud compute instances delete isaac-stamp-" in err
    assert "--zone=us-central1-b" in err


def test_summary_is_printed_before_the_delete(target, capsys):
    fake = Fake()
    seen = []
    fake.on(is_("instances", "delete"), lambda c: seen.append(capsys.readouterr().out) or "")
    launch(fake, target)
    assert "Stamps written" in seen[0]


def test_vm_is_deleted_on_keyboard_interrupt(target):
    fake = Fake()
    fake.fail(ssh_with("stamp.py"), KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        launch(fake, target)
    assert fake.calls[-1][:4] == ["gcloud", "compute", "instances", "delete"]


class Sentinel(Exception):
    pass


@pytest.mark.parametrize("signum", [signal.SIGTERM, signal.SIGHUP])
def test_vm_is_deleted_on_sigterm_or_sighup_and_handlers_are_restored(target, signum):
    def sentinel(*_):
        raise Sentinel
    before = signal.signal(signum, sentinel)
    try:
        fake = Fake()
        fake.on(ssh_with("stamp.py"), lambda c: os.kill(os.getpid(), signum) or "")
        with pytest.raises(KeyboardInterrupt):
            launch(fake, target)
        assert fake.calls[-1][:4] == ["gcloud", "compute", "instances", "delete"]
        assert signal.getsignal(signum) is sentinel
    finally:
        signal.signal(signum, before)


def test_unset_target_email_is_refused_before_any_gcloud_call(target, monkeypatch):
    git(target, "config", "--unset", "user.email")
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", "/dev/null")
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    fake = Fake()
    with pytest.raises(SystemExit, match="user.email"):
        launch(fake, target)
    assert fake.calls == []


def test_a_command_timing_out_raises_and_the_vm_is_still_deleted(target):
    fake = Fake()
    fake.fail(ssh_with("stamp.py"), gcp.CommandError(["ssh"], "timed out after 3600 s", -1))
    with pytest.raises(gcp.CommandError, match="timed out"):
        launch(fake, target)
    assert fake.calls[-1][:4] == ["gcloud", "compute", "instances", "delete"]


def test_runner_turns_a_subprocess_timeout_into_a_command_error():
    with pytest.raises(gcp.CommandError, match=r"timed out after 1 s"):
        gcp.Runner().run(["sleep", "5"], timeout=1)
    with pytest.raises(gcp.CommandError, match=r"timed out after 1 s"):
        gcp.Runner().run(["sleep", "5"], stdout_only=True, timeout=1)


def test_timeouts_per_command(target):
    fake = Fake()
    launch(fake, target)
    by = lambda *w: {t for c, t in zip(fake.calls, fake.timeouts) if all(x in c for x in w)}
    assert by("instances", "create") == {600}
    assert by("instances", "delete") == {300}
    assert by("instances", "list") == {180}
    assert {t for c, t in zip(fake.calls, fake.timeouts) if c[0] == "ssh" and "stamp.py" in c[-1]} == {3600}


def test_falls_back_through_the_zones_on_stockout(target):
    stockout = gcp.CommandError(["create"], "ZONE_RESOURCE_POOL_EXHAUSTED", 1)
    order = ["us-central1-b", "us-central1-a", "us-central1-c", "us-central1-f"]
    for stocked_out in range(1, 4):
        fake = Fake()
        for zone in order[:stocked_out]:
            fake.fail(is_("instances", "create", f"--zone={zone}"), stockout)
        launch(fake, target)
        creates = fake.matching("instances", "create")
        assert [c for z in order[:stocked_out + 1] for c in creates if f"--zone={z}" in c] == creates
        assert len(creates) == stocked_out + 1
        assert f"--zone={order[stocked_out]}" in fake.calls[-1] and fake.calls[-1][3] == "delete"
        assert {proxy_of(c) for c in fake.calls if c[0] in ("ssh", "scp")} == {
            iap_proxy(vm_of(fake), "example-project", order[stocked_out])}


def test_interrupted_fallback_create_deletes_in_every_tried_zone(target):
    fake = Fake()
    fake.fail(is_("instances", "create", "--zone=us-central1-b"),
              gcp.CommandError(["create"], "ZONE_RESOURCE_POOL_EXHAUSTED", 1))
    fake.fail(is_("instances", "create", "--zone=us-central1-a"),
              gcp.CommandError(["create"], "ZONE_RESOURCE_POOL_EXHAUSTED", 1))
    fake.fail(is_("instances", "create", "--zone=us-central1-c"), KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        launch(fake, target)
    deletes = fake.matching("instances", "delete")
    for z in ("a", "b", "c"):
        assert [d for d in deletes if f"--zone=us-central1-{z}" in d]
    assert fake.calls[-1][3] == "delete" and "--zone=us-central1-c" in fake.calls[-1]


def test_delete_not_found_is_tolerated_and_interrupt_retries(target, capsys):
    fake = Fake()
    fake.fail(is_("instances", "create", "--zone=us-central1-b"), RuntimeError("boom"))
    fake.fail(is_("instances", "delete"), gcp.CommandError(["delete"], "The resource was not found", 1))
    with pytest.raises(RuntimeError, match="boom"):
        launch(fake, target)
    assert "WARNING" not in capsys.readouterr().err

    fake = Fake()
    hits = []

    def once(cmd):
        hits.append(1)
        if len(hits) == 1:
            raise KeyboardInterrupt()
        return ""

    fake.on(is_("instances", "delete"), once)
    fake.fail(is_("instances", "create"), RuntimeError("boom"))
    with pytest.raises(RuntimeError):
        launch(fake, target)
    assert len(hits) == 2


def test_unknown_scope_is_refused_before_any_gcloud_call(target):
    fake = Fake()
    with pytest.raises(SystemExit):
        gcp.main(["--target", str(target), "--scope", "Nope"], runner=fake, sleep=lambda s: None)
    assert fake.calls == []


def test_project_is_required_before_any_gcloud_call(target, monkeypatch, capsys):
    monkeypatch.delenv("ISAAC_STAMP_GCP_PROJECT")
    fake = Fake()
    with pytest.raises(SystemExit, match="--project or set ISAAC_STAMP_GCP_PROJECT"):
        gcp.main(["--target", str(target), "--scope", "A"], runner=fake, sleep=lambda s: None)
    assert fake.calls == []


def test_project_option_overrides_the_environment(target):
    fake = Fake()
    launch(fake, target, "--project", "other-project")
    (create,) = fake.matching("instances", "create")
    assert "--project=other-project" in create and "--image-project=other-project" in create
    assert all("--project=other-project" in c for c in fake.calls if c[:2] == ["gcloud", "compute"])
    assert all("--project=other-project" in proxy_of(c) for c in fake.calls if c[0] in ("ssh", "scp"))


def test_zone_is_restricted():
    with pytest.raises(SystemExit):
        gcp.main(["--zone", "europe-west1-b"], runner=Fake())


def test_other_create_errors_are_not_retried(target):
    fake = Fake()
    fake.fail(is_("instances", "create"), gcp.CommandError(["create"], "QUOTA_EXCEEDED", 1))
    with pytest.raises(gcp.CommandError):
        launch(fake, target)
    assert len(fake.matching("instances", "create")) == 1
    assert fake.calls[-1][3] == "delete"


def test_a_preemption_during_create_is_retried_in_the_next_zone(target):
    fake = Fake()
    fake.fail(is_("instances", "create", "--zone=us-central1-b"),
              gcp.CommandError(["create"], "ERROR: Could not fetch resource:\n - Operation was canceled by user ''.", 1))
    launch(fake, target)
    creates = fake.matching("instances", "create")
    assert len(creates) == 2 and "--zone=us-central1-a" in creates[1]
    assert fake.calls[-1][3] == "delete" and "--zone=us-central1-a" in fake.calls[-1]
    assert [d for d in fake.matching("instances", "delete") if "--zone=us-central1-b" in d]


def test_gives_up_after_six_create_attempts_and_deletes_in_every_tried_zone(target, capsys):
    fake = Fake()
    fake.fail(is_("instances", "create"), gcp.CommandError(["create"], "Instance was PREEMPTED", 1))
    with pytest.raises(gcp.CommandError, match="gave up after 6 create attempts"):
        launch(fake, target)
    creates = fake.matching("instances", "create")
    zones = [next(a for a in c if a.startswith("--zone=")) for c in creates]
    assert zones == [f"--zone=us-central1-{z}" for z in "bacfba"]
    deletes = fake.matching("instances", "delete")
    for z in "bacf":
        assert [d for d in deletes if f"--zone=us-central1-{z}" in d]


def test_ssh_and_scp_go_through_iap_with_a_temporary_key(target):
    fake = Fake()
    launch(fake, target)
    (keygen,) = [c for c in fake.calls if c[0] == "ssh-keygen"]
    key = keygen[keygen.index("-f") + 1]
    name = vm_of(fake)
    calls = [c for c in fake.calls if c[0] in ("ssh", "scp")]
    assert {c[0] for c in calls} == {"ssh", "scp"}
    for call in calls:
        flat = " ".join(call)
        for part in [f"-i {key}", "-o StrictHostKeyChecking=no", "-o UserKnownHostsFile=/dev/null"]:
            assert part in flat
        assert proxy_of(call) == iap_proxy(name, "example-project", "us-central1-b")
        assert f"stamp@{name}" in flat
    assert not fake.matching("instances", "describe")
    assert not Path(key).exists() and not Path(key).parent.exists()


def test_iap_proxy_quotes_its_arguments():
    assert "'--project=p; rm -rf ~'" in gcp.iap_proxy("p; rm -rf ~", "vm", "us-central1-b")


def test_no_firewall_rule_is_ever_touched(target):
    fake = Fake()
    launch(fake, target)
    assert not [c for c in fake.calls if "firewall-rules" in " ".join(c)]
    assert not [c for c in fake.calls if c[0] == "curl"]


def test_remote_command_sets_home_only_for_stamp_py(target):
    fake = Fake()
    launch(fake, target, scopes=["Robotiq_Hand_E_C10"])
    remote = next(c[-1] for c in fake.calls if "ssh" in c and "stamp.py" in c[-1])
    assert "HOME=/opt/isaac-home python3 -u ~/tools/stamp.py" in remote
    assert remote.count("HOME=") == 1
    assert "git -C ~/repo config user.email" in remote


def test_remote_command_runs_stamp_py_with_scopes_and_shards(target):
    fake = Fake()
    launch(fake, target, scopes=["Robotiq_2F_85", "Robotiq_Hand_E_C10"])
    remote = next(c[-1] for c in fake.calls if "ssh" in c and "stamp.py" in c[-1])
    for part in ["user.email dev@example.com", "~/tools/stamp.py", "--root ~/repo", "--isaacsim /opt/isaacsim",
                 "--scope Robotiq_2F_85", "--scope Robotiq_Hand_E_C10", "--shards Robotiq_2F_85=3",
                 "--shards Robotiq_Hand_E_C10=1", "--no-skips", "--keep-partial ~/out/partial", "--log-dir ~/out/logs",
                 "tee ~/out/stamp.log"]:
        assert part in remote
    assert "Robotiq_Hand_E=" not in remote


def test_shard_map_fills_eight_processes():
    assert gcp.SHARDS == {"Robotiq_2F_85": 3, "Robotiq_2F_140": 3, "Robotiq_Hand_E": 1, "Robotiq_Hand_E_C10": 1}


def test_waits_for_ssh(target):
    fake = Fake()
    tries = []

    def flaky(cmd):
        tries.append(1)
        if len(tries) < 3:
            raise gcp.CommandError(cmd, "connection refused", 255)
        return ""

    fake.on(lambda c: c[0] == "ssh" and c[-1] == "true", flaky)
    launch(fake, target)
    assert len(tries) == 3


def test_apply_stamps_replaces_the_scope_stamps_only(tmp_path, target):
    stamps = target / "ci/runtime/stamps"
    stamps.mkdir(parents=True)
    (stamps / "A-old.json").write_text("{}")
    (stamps / "AB-old.json").write_text("{}")
    git(target, "add", "-A")
    git(target, "commit", "-qm", "stamps")
    fetched = tmp_path / "fetched"
    fetched.mkdir()
    (fetched / "A-new.json").write_text('{"new": 1}')
    (fetched / "AB-old.json").write_text("{}")

    assert gcp.apply_stamps(target, fetched, ["A"]) == ["A-new.json"]
    assert sorted(p.name for p in stamps.iterdir()) == ["A-new.json", "AB-old.json"]
    assert "D  ci/runtime/stamps/A-old.json" in git(target, "status", "--porcelain")
    assert "A  ci/runtime/stamps/A-new.json" in git(target, "status", "--porcelain")


def test_apply_stamps_keeps_the_old_stamp_when_nothing_new(tmp_path, target):
    stamps = target / "ci/runtime/stamps"
    stamps.mkdir(parents=True)
    (stamps / "A-old.json").write_text("{}")
    git(target, "add", "-A")
    git(target, "commit", "-qm", "stamps")
    fetched = tmp_path / "fetched"
    fetched.mkdir()
    (fetched / "A-old.json").write_text("{}")
    assert gcp.apply_stamps(target, fetched, ["A"]) == []
    assert (stamps / "A-old.json").exists()


def results_fake(exit_code="0", partial=True):
    fake = Fake()

    def fetch(cmd):
        cache = Path(cmd[-1])
        (cache / "stamps").mkdir(parents=True)
        (cache / "stamps" / "A-new.json").write_text("{}")
        (cache / "out").mkdir()
        (cache / "out" / "exit").write_text(exit_code + "\n")
        if partial:
            (cache / "out" / "partial").mkdir()
            (cache / "out" / "partial" / "B-0123456789abcdef.partial.json").write_text("{}")
        return ""

    fake.on(lambda c: c[0] == "scp" and "-r" in c, fetch)
    return fake


def test_partials_are_reported_with_the_laptop_command_and_exit_is_non_zero(target, capsys):
    code = launch(results_fake("1"), target)
    out = capsys.readouterr().out
    assert code == 1
    assert "A-new.json" in out
    assert "B-0123456789abcdef.partial.json" in out
    for part in ["systemd-run --user -P --unit=isaac-stamp-complete", "-p MemoryMax=16G", "-p MemorySwapMax=0",
                 "-p CPUWeight=20", "-p IOWeight=20", "-p Nice=10", "-E PYTHONPATH=",
                 f"--working-directory={target}", "ci/runtime/stamp.py", f"--root {target}", "--complete",
                 '--isaacsim "${ISAACSIM_ROOT:-$HOME/isaacsim}"']:
        assert part in out
    assert (target / "ci/runtime/stamps/A-new.json").exists()


def test_clean_run_exits_zero(target):
    assert launch(results_fake("0", partial=False), target) == 0


def test_a_run_without_exit_file_is_a_failure(target):
    assert launch(Fake(), target) == 1


def test_helpers():
    assert gcp.owner_label("jane.doe@example.com") == "jane-doe"
    assert gcp.owner_label("Jane_Doe+x@example.com") == "jane_doe-x"
    assert gcp.vm_name("jane_doe").startswith("isaac-stamp-jane-doe-20")


def commit_file(root: Path, name: str, text: str) -> str:
    (root / name).write_text(text)
    git(root, "add", "-A")
    git(root, "commit", "-qm", name)
    return git(root, "rev-parse", "HEAD").strip()


def shipped(fake):
    """Copies of the files sent by the shipping scp, read while the temp dir still exists."""
    seen = {}

    def keep(cmd):
        for arg in cmd[1:-1]:
            path = Path(arg)
            if path.name in ("repo.bundle", "lfs.tar"):
                seen[path.name] = path.read_bytes()
        return ""

    fake.on(lambda c: c[0] == "scp" and c[-1].endswith(":tools/"), keep)
    return seen


def test_spot_asks_for_a_spot_vm(target):
    fake = Fake()
    launch(fake, target, "--spot")
    (create,) = fake.matching("instances", "create")
    assert "--provisioning-model=SPOT" in create and "--provisioning-model=STANDARD" not in create
    assert "--instance-termination-action=DELETE" in create and "--max-run-duration=2h" in create


def test_the_shard_plugin_is_shipped_next_to_stamp_py(target):
    fake = Fake()
    launch(fake, target)
    (scp,) = [c for c in fake.calls if c[0] == "scp" and c[-1].endswith(":tools/")]
    assert {Path(a).name for a in scp[1:-1]} >= {"stamp.py", "stamplib.py", "stamp_shard.py"}


def bundle_prerequisites(data: bytes) -> list[str]:
    header = data.split(b"\n\n", 1)[0].decode().splitlines()
    return [line[1:].split()[0] for line in header if line.startswith("-")]


def test_incremental_bundle_when_the_image_main_is_an_ancestor(target):
    image_main = git(target, "rev-parse", "HEAD").strip()
    head = commit_file(target, "more.txt", "y\n")
    fake = Fake()
    fake.on(ssh_with("clone -q /opt/repo"), lambda c: f"main={image_main}\n")
    seen = shipped(fake)
    launch(fake, target)
    assert bundle_prerequisites(seen["repo.bundle"]) == [image_main]
    checkout = next(r for r in remote_commands(fake) if "git checkout" in r)
    assert "git fetch -q ~/tools/repo.bundle" in checkout and f"--detach {head}" in checkout


def test_full_bundle_when_the_image_main_is_unknown(target):
    fake = Fake()
    seen = shipped(fake)
    launch(fake, target)
    assert seen["repo.bundle"] and bundle_prerequisites(seen["repo.bundle"]) == []


def test_no_bundle_when_head_is_the_image_main(target):
    head = git(target, "rev-parse", "HEAD").strip()
    fake = Fake()
    fake.on(ssh_with("clone -q /opt/repo"), lambda c: f"main={head}\n")
    seen = shipped(fake)
    launch(fake, target)
    assert "repo.bundle" not in seen
    checkout = next(r for r in remote_commands(fake) if "git checkout" in r)
    assert "repo.bundle" not in checkout and f"--detach {head}" in checkout


def test_prepare_clones_the_image_repo_and_reads_its_main(target):
    fake = Fake()
    launch(fake, target)
    prepare = next(r for r in remote_commands(fake) if "clone -q /opt/repo" in r)
    trust = "git config --global --add safe.directory '*'"
    assert prepare.index(trust) < prepare.index("clone -q /opt/repo ~/repo")
    assert "HOME=/opt/isaac-home " + trust in prepare
    assert prepare.index(trust) < prepare.index("git -C /opt/repo rev-parse HEAD")
    assert "-c safe.directory" not in prepare


def lfs_target(target) -> dict[str, str]:
    require_git_lfs()
    git(target, "lfs", "install", "--local", "--skip-repo")
    (target / ".gitattributes").write_text("*.usd filter=lfs diff=lfs merge=lfs -text\n")
    (target / "a.usd").write_text("payload a\n")
    (target / "b.usd").write_text("payload b\n")
    git(target, "add", "-A")
    git(target, "commit", "-qm", "lfs")
    out = git(target, "lfs", "ls-files", "-l")
    return {line.split()[-1]: line.split()[0] for line in out.splitlines()}


def test_only_the_missing_lfs_objects_are_sent(target):
    oids = lfs_target(target)
    fake = Fake()
    fake.on(ssh_with("clone -q /opt/repo"), lambda c: "main=" + "0" * 40 + f"\nmissing={oids['b.usd']}\n")
    seen = shipped(fake)
    launch(fake, target)
    (stdin,) = [i for c, i in zip(fake.calls, fake.inputs) if c[0] == "ssh" and "clone -q /opt/repo" in c[-1]]
    assert sorted(stdin.splitlines()) == sorted(f"{o[:2]}/{o[2:4]}/{o}" for o in oids.values())
    prepare = next(r for r in remote_commands(fake) if "clone -q /opt/repo" in r)
    assert oids["a.usd"] not in prepare
    b = oids["b.usd"]
    with tarfile.open(fileobj=io.BytesIO(seen["lfs.tar"])) as tar:
        names = tar.getnames()
        assert names == [f".git/lfs/objects/{b[:2]}/{b[2:4]}/{b}"]
        assert tar.extractfile(names[0]).read() == b"payload b\n"
    checkout = next(r for r in remote_commands(fake) if "git checkout" in r)
    assert "tar -xf ~/tools/lfs.tar" in checkout and "git lfs checkout" in checkout


def test_nothing_sent_when_no_lfs_object_is_missing(target):
    lfs_target(target)
    fake = Fake()
    seen = shipped(fake)
    launch(fake, target)
    assert "lfs.tar" not in seen
    checkout = next(r for r in remote_commands(fake) if "git checkout" in r)
    assert "lfs.tar" not in checkout and "git lfs checkout" in checkout


def test_refuses_a_dirty_checkout_on_the_vm(target):
    fake = Fake()
    fake.on(ssh_with("git checkout"), lambda c: "dirty  M grippers/A/a.usd\n")
    with pytest.raises(SystemExit) as stop:
        launch(fake, target)
    assert "grippers/A/a.usd" in str(stop.value.code)
    assert not [r for r in remote_commands(fake) if "stamp.py" in r]
    assert fake.calls[-1][3] == "delete"


def test_an_lfs_pointer_left_after_checkout_stops_the_run(target):
    fake = Fake()
    fake.on(ssh_with("git lfs checkout"), lambda c: "pointer grippers/A/a.usd\n")
    with pytest.raises(SystemExit) as stop:
        launch(fake, target)
    assert "still pointers" in str(stop.value.code) and "grippers/A/a.usd" in str(stop.value.code)
    assert not [r for r in remote_commands(fake) if "stamp.py" in r]
    assert fake.calls[-1][3] == "delete"


def test_the_pointer_check_reads_the_lfs_marker():
    listing = "71307b5fa6 * grippers/A/done.usd\na2e7bf87bd - grippers/A/still pointer.usd\n"
    done = subprocess.run(["sh", "-c", gcp.LFS_POINTERS.replace("git lfs ls-files", "cat")], input=listing,
                          capture_output=True, text=True)
    assert done.stdout == "pointer grippers/A/still pointer.usd\n"


def test_prepare_lists_the_missing_objects_read_on_stdin(tmp_path):
    present, absent = "ab" * 32, "cd" * 32
    store = tmp_path / "objects"
    (store / "ab" / "ab").mkdir(parents=True)
    (store / "ab" / "ab" / present).write_text("x")
    loop = gcp.PREPARE[gcp.PREPARE.index("while read"):]
    done = subprocess.run(["sh", "-c", loop], cwd=store, input=gcp.lfs_paths([present, absent]),
                          capture_output=True, text=True)
    assert done.stdout == f"missing={absent}\n"


def test_ssh_wait_names_its_real_bound(target):
    fake = Fake()
    fake.fail(lambda c: c[0] == "ssh" and c[-1] == "true", gcp.CommandError(["ssh"], "refused", 255))
    with pytest.raises(SystemExit, match="after 30 attempts \\(up to 10 minutes\\)"):
        launch(fake, target)
    assert fake.calls[-1][3] == "delete"


def test_a_second_ctrl_c_during_cleanup_still_deletes_in_the_other_zones(target, capsys):
    fake = Fake()
    for z in "ba":
        fake.fail(is_("instances", "create", f"--zone=us-central1-{z}"),
                  gcp.CommandError(["create"], "ZONE_RESOURCE_POOL_EXHAUSTED", 1))
    fake.fail(is_("instances", "create", "--zone=us-central1-c"), RuntimeError("boom"))
    fake.fail(is_("instances", "delete", "--zone=us-central1-b"), KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        launch(fake, target)
    deletes = fake.matching("instances", "delete")
    assert [next(a for a in d if a.startswith("--zone=")) for d in deletes] == [
        "--zone=us-central1-b", "--zone=us-central1-b", "--zone=us-central1-a", "--zone=us-central1-c"]
    assert "--zone=us-central1-b" in capsys.readouterr().err


def test_phases_are_printed_in_order(target, capsys):
    launch(results_fake("0", partial=False), target)
    lines = [line for line in capsys.readouterr().out.splitlines() if line.startswith("[stamp-on-gcp]")]
    pattern = re.compile(r"^\[stamp-on-gcp\] \d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\S* \+\d+s (\S+)$")
    assert [pattern.match(line).group(1) for line in lines] == [
        "start", "vm-created", "ssh-ready", "code-shipped", "remote-run-done", "fetched", "deleted",
    ]


def test_machine_type_option(target):
    fake = Fake()
    launch(fake, target, "--machine-type", "n1-standard-16", "--gpus", "1")
    (create,) = fake.matching("instances", "create")
    assert "--machine-type=n1-standard-16" in create and "--accelerator=type=nvidia-tesla-t4,count=1" in create
    assert "--machine-type=n1-standard-64" not in create


def test_shards_override_reaches_the_remote_stamp(target):
    fake = Fake()
    launch(fake, target, "--shards", "Robotiq_2F_85=8", scopes=["Robotiq_2F_85"])
    remote = next(c[-1] for c in fake.calls if "ssh" in c and "stamp.py" in c[-1])
    assert "--shards Robotiq_2F_85=8" in remote
    assert "Robotiq_2F_85=3" not in remote


@pytest.mark.parametrize("bad", ["Robotiq_2F_85", "Robotiq_2F_85=0", "Robotiq_2F_85=33", "Robotiq_2F_85=x", "Nope=2", "=3"])
def test_bad_shards_are_refused_before_gcloud(target, bad):
    fake = Fake()
    with pytest.raises(SystemExit):
        launch(fake, target, "--shards", bad)
    assert fake.calls == []


def test_remote_command_samples_resources_and_keeps_the_exit_file(target):
    remote = gcp.remote_stamp_command("dev@example.com", ["Robotiq_2F_85"])
    assert remote.index("sampler=$!") < remote.index("stamp.py")
    assert "trap 'kill $sampler 2>/dev/null' EXIT" in remote
    assert "kill $sampler" in remote.split("echo $? > ~/out/exit")[1]
    assert ">> ~/out/resources.log" in remote and "sleep 10" in remote
    assert "nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits" in remote
    assert "free -m" in remote and "/proc/loadavg" in remote
    assert "tee ~/out/stamp.log; echo $? > ~/out/exit" in remote


def test_resources_summary(tmp_path):
    log = tmp_path / "resources.log"
    log.write_text(
        "2026-11-11T10:00:00+00:00 4000 50000 1.50 0:1200/15360/10 1:800/15360/0\n"
        "2026-11-11T10:00:10+00:00 9000 45000 12.25 0:9000/15360/99 1:7000/15360/50\n"
        "garbage\n"
        "2026-11-11T10:00:20+00:00 7000 47000 8.00\n"
    )
    assert gcp.resources_summary(log) == (
        "Resources on the VM (3 samples): peak RAM used 9000 MiB, min RAM available 45000 MiB, peak load 12.25\n"
        "  GPU 0: peak memory 9000/15360 MiB, average utilization 54 %\n"
        "  GPU 1: peak memory 7000/15360 MiB, average utilization 25 %")
    assert gcp.resources_summary(tmp_path / "missing.log") is None


def test_gpus_option_sets_the_accelerator_count_and_reaches_the_remote_stamp(target):
    fake = Fake()
    launch(fake, target, "--gpus", "2", "--machine-type", "n1-standard-32")
    (create,) = fake.matching("instances", "create")
    assert "--accelerator=type=nvidia-tesla-t4,count=2" in create
    assert "--machine-type=n1-standard-32" in create
    remote = next(c[-1] for c in fake.calls if "ssh" in c and "stamp.py" in c[-1])
    assert "--gpus 2" in remote


def test_default_is_the_measured_four_gpu_machine(target):
    fake = Fake()
    launch(fake, target)
    remote = next(c[-1] for c in fake.calls if "ssh" in c and "stamp.py" in c[-1])
    assert "--gpus 4" in remote


def test_bad_gpus_are_refused_before_gcloud(target):
    fake = Fake()
    with pytest.raises(SystemExit):
        launch(fake, target, "--gpus", "3")
    assert fake.calls == []
