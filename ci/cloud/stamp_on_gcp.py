#!/usr/bin/env python3
"""Stamp a checkout's HEAD on a GCP T4 machine that deletes itself (see ci/cloud/README.md).

    ci/cloud/stamp_on_gcp.py [--project P] [--target PATH] [--scope S ...] [--zone us-central1-b] [--base origin/main]
                                [--machine-type n1-standard-64] [--gpus 4] [--shards SCOPE=N ...]

Standard library only. Every gcloud/ssh/scp/ssh-keygen command goes through a Runner so the tests can fake it.
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNTIME = HERE.parent / "runtime"
sys.path.insert(0, str(RUNTIME))
import stamplib  # noqa: E402

PROJECT_ENV = "ISAAC_STAMP_GCP_PROJECT"
NETWORK = "isaac-runtime-stamp"
SUBNET = "isaac-runtime-stamp-us-central1"
TAG = "isaac-runtime-stamp"
ZONES = ["us-central1-b", "us-central1-a", "us-central1-c", "us-central1-f"]
SHARDS = {"Robotiq_2F_85": 3, "Robotiq_2F_140": 3, "Robotiq_Hand_E": 1, "Robotiq_Hand_E_C10": 1}
SSH_USER = "stamp"
SSH_ATTEMPTS = 30
SSH_RETRY_S = 10
SSH_CONNECT_TIMEOUT_S = 10
SSH_OPTIONS = ["-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR",
               "-o", "BatchMode=yes", "-o", "IdentitiesOnly=yes", "-o", f"ConnectTimeout={SSH_CONNECT_TIMEOUT_S}"]
STOCKOUT = ("ZONE_RESOURCE_POOL_EXHAUSTED", "does not have enough resources")
PREEMPTED = ("operation was canceled", "preempted")
MAX_CREATE_ATTEMPTS = 6
MACHINE_TYPES = ["n1-standard-16", "n1-standard-32", "n1-standard-64"]
GPU_COUNTS = [1, 2, 4]
DEFAULT_MACHINE_TYPE = "n1-standard-64"
DEFAULT_GPUS = 4
SAMPLE_INTERVAL_S = 10
DEFAULT_TIMEOUT_S = 180
CREATE_TIMEOUT_S = 600
DELETE_TIMEOUT_S = 300
COPY_TIMEOUT_S = 600
REMOTE_STAMP_TIMEOUT_S = 3600


class CommandError(RuntimeError):
    def __init__(self, cmd: list[str], output: str, code: int):
        super().__init__(f"{' '.join(cmd[:4])} failed ({code}): {output.strip()[-500:]}")
        self.cmd, self.output, self.code = cmd, output, code


class Runner:
    def run(self, cmd: list[str], *, check: bool = True, capture: bool = False, stdout_only: bool = False,
            timeout: float = DEFAULT_TIMEOUT_S, input: str | None = None) -> str:
        """``capture`` returns stdout and stderr merged; ``stdout_only`` returns stdout alone (for output that is parsed,
        since gcloud prints warnings on stderr). A failure still reports both. A command running longer than
        ``timeout`` seconds is killed and raises CommandError. ``input`` is fed to the command's stdin."""
        try:
            if stdout_only:
                done = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout,
                                      input=input)
                if check and done.returncode:
                    raise CommandError(cmd, (done.stdout or "") + (done.stderr or ""), done.returncode)
                return done.stdout or ""
            done = subprocess.run(
                cmd, text=True, stdout=subprocess.PIPE if capture else None,
                stderr=subprocess.STDOUT if capture else None, timeout=timeout, input=input,
            )
        except subprocess.TimeoutExpired:
            raise CommandError(cmd, f"timed out after {timeout:g} s", -1) from None
        if check and done.returncode:
            raise CommandError(cmd, done.stdout or "", done.returncode)
        return done.stdout or ""


def owner_label(account: str) -> str:
    return re.sub(r"[^a-z0-9_-]", "-", account.split("@")[0].lower())


def vm_name(user: str) -> str:
    return f"isaac-stamp-{user.replace('_', '-')}-{datetime.now():%Y%m%d-%H%M%S}"


def git(root: Path, *args: str) -> str:
    return stamplib.git(root, *args).decode()


def gcloud(*args: str) -> list[str]:
    return ["gcloud", *args]


def create_vm(runner, project: str, name: str, owner: str, zone: str, tried: list[str], pubkey: str, spot: bool = False,
              machine_type: str = DEFAULT_MACHINE_TYPE, gpus: int = DEFAULT_GPUS) -> None:
    """Create the VM, moving on to the next zone (cycling ZONES, MAX_CREATE_ATTEMPTS in all) on a stockout or when a
    Spot VM (--spot) is preempted during the create. Every zone is appended to ``tried`` before the attempt, so the caller can
    delete in each even if a create is interrupted."""
    def attempt(z: str) -> None:
        tried.append(z)
        runner.run(gcloud(
            "compute", "instances", "create", name, f"--project={project}", f"--zone={z}",
            f"--machine-type={machine_type}", f"--accelerator=type=nvidia-tesla-t4,count={gpus}", "--boot-disk-type=pd-ssd",
            f"--provisioning-model={'SPOT' if spot else 'STANDARD'}",
            "--instance-termination-action=DELETE", "--max-run-duration=2h",
            "--maintenance-policy=TERMINATE", "--no-service-account", "--no-scopes",
            "--image-family=isaacsim-6-2", f"--image-project={project}", f"--network={NETWORK}",
            f"--subnet={SUBNET}", f"--tags={TAG}", f"--labels=purpose={TAG},owner={owner}",
            f"--metadata=ssh-keys={SSH_USER}:{pubkey},enable-oslogin=FALSE,block-project-ssh-keys=TRUE",
        ), capture=True, timeout=CREATE_TIMEOUT_S)

    start = ZONES.index(zone)
    for n in range(MAX_CREATE_ATTEMPTS):
        z = ZONES[(start + n) % len(ZONES)]
        try:
            attempt(z)
            return
        except CommandError as err:
            out = err.output.lower()
            if not (any(s.lower() in out for s in STOCKOUT) or any(s in out for s in PREEMPTED)):
                raise
            last = err
            if n + 1 < MAX_CREATE_ATTEMPTS:
                print(f"Create failed in {z} (no capacity or preempted), trying {ZONES[(start + n + 1) % len(ZONES)]}")
    raise CommandError(last.cmd, f"gave up after {MAX_CREATE_ATTEMPTS} create attempts (stockout or Spot preemption); "
                       f"last error: {last.output}", last.code)


def apply_stamps(target: Path, fetched: Path, scopes: list[str]) -> list[str]:
    """Bring the fetched stamps of ``scopes`` into the target; a scope's older stamps are ``git rm``-ed."""
    dest = target / stamplib.STAMPS_DIR
    dest.mkdir(parents=True, exist_ok=True)
    written = []
    for scope in scopes:
        have = {p.name for p in fetched.glob(f"{scope}-*.json")}
        old = {p.name for p in dest.glob(f"{scope}-*.json")}
        new = sorted(have - old)
        if not new:
            continue
        for name in sorted(old - have):
            git(target, "rm", "-q", "-f", f"{stamplib.STAMPS_DIR}/{name}")
        for name in new:
            shutil.copy(fetched / name, dest / name)
            git(target, "add", f"{stamplib.STAMPS_DIR}/{name}")
        written += new
    return written


def complete_command(target: Path, partial: Path) -> str:
    return (
        "systemd-run --user -P --unit=isaac-stamp-complete -p MemoryMax=16G -p MemorySwapMax=0 -p CPUWeight=20 "
        "-p IOWeight=20 -p Nice=10 -E PYTHONPATH= "
        f"--working-directory={shlex.quote(str(target))} python3 {shlex.quote(str(RUNTIME / 'stamp.py'))} "
        f"--root {shlex.quote(str(target))} --isaacsim \"${{ISAACSIM_ROOT:-$HOME/isaacsim}}\" "
        f"--complete {shlex.quote(str(partial))}"
    )


def preflight(target: Path) -> None:
    dirty = git(target, "status", "--porcelain", "-uno", "--", ".", f":(exclude){stamplib.STAMPS_DIR}").strip()
    if dirty:
        sys.exit(f"{target} has uncommitted changes; the bundle would not contain them:\n{dirty}")


def refuse_if_a_vm_exists(runner, project: str) -> None:
    listing = runner.run(gcloud("compute", "instances", "list", f"--project={project}", f"--filter=labels.purpose={TAG}",
                                "--format=value(name,zone.basename())", "--verbosity=error"), stdout_only=True)
    running = "\n".join(line.strip() for line in listing.splitlines() if line.strip())
    if running:
        sys.exit(f"A runtime-stamp VM already exists (someone else's run, or a leftover to delete):\n{running}")


def interrupt(signum, frame):
    raise KeyboardInterrupt


def new_key(runner, tmp: Path) -> tuple[Path, str]:
    key = tmp / "key"
    runner.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-C", SSH_USER, "-f", str(key)], capture=True)
    return key, key.with_suffix(".pub").read_text().strip()


def iap_proxy(project: str, name: str, zone: str) -> str:
    """ssh's ProxyCommand: port 22 is only open to IAP's range, so ssh and scp reach the VM through an IAP tunnel."""
    return " ".join(shlex.quote(a) for a in gcloud(
        "compute", "start-iap-tunnel", name, "22", "--listen-on-stdin", f"--project={project}", f"--zone={zone}",
        "--verbosity=warning"))


class Remote:
    def __init__(self, runner, key: Path, project: str, name: str, zone: str):
        self.runner, self.host = runner, f"{SSH_USER}@{name}"
        self.options = ["-i", str(key), *SSH_OPTIONS, "-o", f"ProxyCommand={iap_proxy(project, name, zone)}"]

    def ssh(self, command: str, capture: bool = False, timeout: float = DEFAULT_TIMEOUT_S, input: str | None = None) -> str:
        return self.runner.run(["ssh", *self.options, self.host, command], capture=capture, timeout=timeout, input=input)

    def scp(self, *args: str) -> None:
        self.runner.run(["scp", *self.options, *args], capture=True, timeout=COPY_TIMEOUT_S)

    def at(self, path: str) -> str:
        return f"{self.host}:{path}"


def lfs_oids(target: Path) -> list[str]:
    out = git(target, "lfs", "ls-files", "-l", "HEAD")
    oids = sorted({line.split()[0] for line in out.splitlines() if line.strip()})
    bad = [o for o in oids if not re.fullmatch(r"[0-9a-f]{64}", o)]
    if bad:
        sys.exit(f"unexpected git lfs ls-files output: {bad[:3]}")
    return oids


# git lfs ls-files marks a file that is still a pointer with "-" instead of "*".
LFS_POINTERS = "git lfs ls-files | sed -n 's/^[0-9a-f]* - /pointer /p'"

# Clones the image's repo (with its LFS objects) to ~/repo, prints the image's main, then reads LFS object paths on
# stdin and prints the oids ~/repo lacks.
PREPARE = (
    "set -e; rm -rf ~/repo ~/out ~/tools; mkdir -p ~/tools ~/out/partial; "
    "git config --global --add safe.directory '*'; "
    "HOME=/opt/isaac-home git config --global --add safe.directory '*'; "
    "GIT_LFS_SKIP_SMUDGE=1 git clone -q /opt/repo ~/repo; "
    "mkdir -p ~/repo/.git/lfs/objects; cp -r /opt/repo/.git/lfs/objects/. ~/repo/.git/lfs/objects/; "
    "main=$(git -C /opt/repo rev-parse HEAD); echo main=$main; "
    'cd ~/repo/.git/lfs/objects; while read -r p; do [ -f "$p" ] || echo "missing=${p##*/}"; done'
)


def lfs_paths(oids: list[str]) -> str:
    """PREPARE's stdin: one object path per line, a list too long for one command-line argument."""
    return "".join(f"{o[:2]}/{o[2:4]}/{o}\n" for o in oids)


def ship_code(remote: Remote, target: Path, tmp: Path) -> None:
    """Check out the target's HEAD in ~/repo on the VM, sending only what the image's clone lacks."""
    head = git(target, "rev-parse", "HEAD").strip()
    oids = lfs_oids(target)
    out = remote.ssh(PREPARE, capture=True, input=lfs_paths(oids)).splitlines()
    main = next((line[5:] for line in out if line.startswith("main=")), "")
    if not re.fullmatch(r"[0-9a-f]{40}", main):
        sys.exit(f"could not read the image's main commit on the VM: {out[-3:]}")
    missing = sorted({line[8:] for line in out if line.startswith("missing=")} & set(oids))

    send = [str(RUNTIME / name) for name in ("stamp.py", "stamplib.py", "stamp_shard.py")]
    steps = ["set -eo pipefail", "cd ~/repo"]
    if main != head:
        bundle = tmp / "repo.bundle"
        known = subprocess.run(["git", "-C", str(target), "merge-base", "--is-ancestor", main, "HEAD"],
                               capture_output=True).returncode == 0
        git(target, "bundle", "create", "-q", str(bundle), f"{main}..HEAD" if known else "HEAD")
        send.append(str(bundle))
        steps.append("git fetch -q ~/tools/repo.bundle HEAD")
    steps.append(f"GIT_LFS_SKIP_SMUDGE=1 git checkout -q --detach {head}")
    if missing:
        store = target / git(target, "rev-parse", "--git-common-dir").strip() / "lfs" / "objects"
        tar_path = tmp / "lfs.tar"
        with tarfile.open(tar_path, "w") as tar:
            for oid in missing:
                local = store / oid[:2] / oid[2:4] / oid
                if not local.is_file():
                    sys.exit(f"LFS object {oid} is not on this laptop; run git lfs pull in {target}")
                tar.add(local, arcname=f".git/lfs/objects/{oid[:2]}/{oid[2:4]}/{oid}")
        send.append(str(tar_path))
        steps.append("tar -xf ~/tools/lfs.tar")
    steps += ["git lfs checkout", LFS_POINTERS, "git status --porcelain | sed 's/^/dirty /'"]
    remote.scp(*send, remote.at("tools/"))
    out = remote.ssh("; ".join(steps), capture=True).splitlines()
    pointers = [line[8:] for line in out if line.startswith("pointer ")]
    if pointers:
        sys.exit("LFS files are still pointers on the VM after git lfs checkout:\n" + "\n".join(pointers[:20]))
    dirty = [line[6:] for line in out if line.startswith("dirty ")]
    if dirty:
        sys.exit("the checkout on the VM is not clean, stamp.py would refuse it:\n" + "\n".join(dirty))


def phases(clock=time.monotonic):
    start = clock()

    def phase(name: str) -> None:
        now = datetime.now().astimezone().isoformat(timespec="seconds")
        print(f"[stamp-on-gcp] {now} +{clock() - start:.0f}s {name}", flush=True)
    return phase


def parse_shards(value: str) -> tuple[str, int]:
    scope, sep, n = value.partition("=")
    if not sep or not scope or not re.fullmatch(r"[0-9]+", n) or not 1 <= int(n) <= 32:
        raise argparse.ArgumentTypeError(f"{value!r}: expected SCOPE=N with N an integer from 1 to 32")
    return scope, int(n)


SAMPLER = (
    f"( while :; do echo \"$(date -Iseconds) $(free -m | awk '/^Mem:/{{print $3, $7}}') $(cut -d' ' -f1 /proc/loadavg)"
    "$(nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits 2>/dev/null "
    "| awk -F', *' '{printf \" %s:%s/%s/%s\", $1, $2, $3, $4}')\" >> ~/out/resources.log; "
    f"sleep {SAMPLE_INTERVAL_S}; done ) & sampler=$!; trap 'kill $sampler 2>/dev/null' EXIT; "
)


def remote_stamp_command(email: str, scopes: list[str], shards: dict[str, int] | None = None,
                         gpus: int = DEFAULT_GPUS) -> str:
    shards = {**SHARDS, **(shards or {})}
    stamp = ["python3", "-u", "~/tools/stamp.py", "--root", "~/repo", "--isaacsim", "/opt/isaacsim"]
    for scope in scopes:
        stamp += ["--scope", shlex.quote(scope)]
        if scope in shards:
            stamp += ["--shards", shlex.quote(f"{scope}={shards[scope]}")]
    stamp += ["--gpus", str(gpus), "--no-skips", "--keep-partial", "~/out/partial", "--log-dir", "~/out/logs"]
    return (
        f"set -o pipefail; git -C ~/repo config user.email {shlex.quote(email)}; {SAMPLER}"
        f"HOME=/opt/isaac-home {' '.join(stamp)} 2>&1 | tee ~/out/stamp.log; echo $? > ~/out/exit; "
        "kill $sampler 2>/dev/null; true"
    )


def resources_summary(log: Path) -> str | None:
    """Lines from the VM's resources.log: peak RAM used, min RAM available, peak load, then per GPU the peak memory
    and the average utilization over all samples. Sample: ``time used avail load idx:mem/total/util ...``."""
    used, avail, load = [], [], []
    gpus: dict[str, dict[str, list[int]]] = {}
    for line in log.read_text(errors="replace").splitlines() if log.is_file() else []:
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            u, a, ld = int(parts[1]), int(parts[2]), float(parts[3])
        except ValueError:
            continue
        used.append(u)
        avail.append(a)
        load.append(ld)
        for item in parts[4:]:
            m = re.fullmatch(r"(\d+):(\d+)/(\d+)/(\d+)", item)
            if m:
                g = gpus.setdefault(m[1], {"mem": [], "total": [], "util": []})
                g["mem"].append(int(m[2]))
                g["total"].append(int(m[3]))
                g["util"].append(int(m[4]))
    if not used:
        return None
    text = (f"Resources on the VM ({len(used)} samples): peak RAM used {max(used)} MiB, min RAM available "
            f"{min(avail)} MiB, peak load {max(load):.2f}")
    for idx in sorted(gpus, key=int):
        g = gpus[idx]
        text += (f"\n  GPU {idx}: peak memory {max(g['mem'])}/{max(g['total'])} MiB, "
                 f"average utilization {sum(g['util']) / len(g['util']):.0f} %")
    return text


def report(target: Path, cache: Path, scopes: list[str]) -> int:
    written = apply_stamps(target, cache / "stamps", scopes) if (cache / "stamps").is_dir() else []
    partials = sorted((cache / "out" / "partial").glob("*.partial.json"))
    exit_file = cache / "out" / "exit"
    stamp_exit = exit_file.read_text().strip() if exit_file.exists() else "missing"
    print(f"\nStamps written and staged in {target}: {', '.join(written) or 'none'}")
    for partial in partials:
        print(f"\nPartial result {partial}: finish it on this laptop (failures only, one Isaac process):")
        print(f"  {complete_command(target, partial)}")
    print(f"\nLogs and partials: {cache}")
    summary = resources_summary(cache / "out" / "resources.log")
    if summary:
        print(summary)
    if stamp_exit != "0" and not partials:
        print(f"stamp.py on the VM exited with {stamp_exit} and left no partial: see the logs.", file=sys.stderr)
    return 0 if stamp_exit == "0" and not partials else 1


def stamp_on_gcp(args, runner, sleep) -> int:
    shards = dict(args.shards or [])
    target = Path(args.target).resolve()
    preflight(target)
    cfg = stamplib.load_config((target / stamplib.CONFIG).read_bytes())
    unknown = sorted(set(shards) - set(cfg["scopes"]))
    if unknown:
        sys.exit(f"--shards: unknown scope(s) {unknown}; known: {list(cfg['scopes'])}")
    scopes = args.scope
    if scopes:
        unknown = sorted(set(scopes) - set(cfg["scopes"]))
        if unknown:
            sys.exit(f"unknown scope(s) {unknown}; known: {list(cfg['scopes'])}")
    else:
        base = git(target, "merge-base", args.base, "HEAD").strip()
        scopes = stamplib.changed_scopes(target, cfg, base, git(target, "rev-parse", "HEAD").strip())
    if not scopes:
        print("Nothing to stamp: this branch changes no file the runtime tests depend on.")
        return 0
    print("Scopes to stamp:", ", ".join(scopes))

    email = stamplib.git(target, "config", "user.email", check=False).decode().strip()
    if not email:
        sys.exit(f"git user.email is not set in {target}; the stamp records it as the author")

    phase = phases()
    phase("start")
    owner = owner_label(runner.run(gcloud("config", "get", "account"), stdout_only=True).strip())
    refuse_if_a_vm_exists(runner, args.project)

    name, zone, tried = vm_name(owner), args.zone, []
    cache = Path.home() / ".cache" / "isaac-stamp" / name
    with tempfile.TemporaryDirectory(prefix="isaac-stamp-") as tmp:
        key, pubkey = new_key(runner, Path(tmp))
        result = 1
        # A terminal closed or a kill must still reach the delete below.
        previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGTERM, signal.SIGHUP)}
        try:
            create_vm(runner, args.project, name, owner, zone, tried, pubkey, spot=args.spot, machine_type=args.machine_type, gpus=args.gpus)
            zone = tried[-1]
            phase("vm-created")
            remote = Remote(runner, key, args.project, name, zone)
            for _ in range(SSH_ATTEMPTS):
                try:
                    remote.ssh("true", capture=True)
                    break
                except CommandError:
                    sleep(SSH_RETRY_S)
            else:
                bound = SSH_ATTEMPTS * (SSH_RETRY_S + SSH_CONNECT_TIMEOUT_S) // 60
                sys.exit(f"{name} did not accept SSH after {SSH_ATTEMPTS} attempts (up to {bound} minutes)")
            phase("ssh-ready")
            ship_code(remote, target, Path(tmp))
            phase("code-shipped")
            remote.ssh(remote_stamp_command(email, scopes, shards, args.gpus), timeout=REMOTE_STAMP_TIMEOUT_S)
            phase("remote-run-done")
            cache.mkdir(parents=True, exist_ok=True)
            remote.scp("-r", remote.at(f"repo/{stamplib.STAMPS_DIR}"), remote.at("out"), str(cache))
            phase("fetched")
            result = report(target, cache, scopes)
        finally:
            try:
                undeleted = delete_everywhere(runner, args.project, name, tried)
                if tried:
                    phase("deleted")
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
        return 1 if undeleted else result


def manual_delete(project: str, name: str, zone: str) -> str:
    return " ".join(shlex.quote(a) for a in gcloud(
        "compute", "instances", "delete", name, f"--project={project}", f"--zone={zone}", "--quiet"))


def delete_vm(runner, project: str, name: str, zone: str) -> bool:
    """Delete synchronously; True when the VM is gone (or was never there). A first Ctrl-C retries the delete."""
    for attempt in range(2):
        try:
            runner.run(gcloud("compute", "instances", "delete", name, f"--project={project}", f"--zone={zone}",
                              "--quiet"), capture=True, timeout=DELETE_TIMEOUT_S)
            return True
        except KeyboardInterrupt:
            if attempt:
                raise
        except CommandError as err:
            if "was not found" in err.output or "notFound" in err.output:
                return True
            print(f"ERROR: could not delete {name} in {zone} ({err}). The VM is still running and billing "
                  f"(2 h at most); delete it by hand:\n  {manual_delete(project, name, zone)}", file=sys.stderr)
            return False
    return False


def delete_everywhere(runner, project: str, name: str, zones: list[str]) -> list[str]:
    """Delete in every zone and return those where it failed. A Ctrl-C that gives up on one zone still lets the
    others be tried; it is raised again afterwards."""
    undeleted, interrupted = [], False
    for zone in dict.fromkeys(zones):
        try:
            if delete_vm(runner, project, name, zone):
                continue
        except KeyboardInterrupt:
            interrupted = True
            print(f"Interrupted: {name} in {zone} may still be running; delete it by hand:\n"
                  f"  {manual_delete(project, name, zone)}", file=sys.stderr)
        undeleted.append(zone)
    if interrupted:
        raise KeyboardInterrupt
    return undeleted


def main(argv: list[str] | None = None, runner=None, sleep=time.sleep) -> int:
    argv = sys.argv[1:] if argv is None else argv
    runner = runner or Runner()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--project", default=os.environ.get(PROJECT_ENV),
                        help=f"GCP project of the VMs and the image (default: ${PROJECT_ENV})")
    parser.add_argument("--target", default=str(HERE.parent.parent), help="checkout to stamp (default: this one)")
    parser.add_argument("--scope", action="append", help="scope to stamp (repeatable); default: those changed vs --base")
    parser.add_argument("--zone", default=ZONES[0], choices=ZONES)
    parser.add_argument("--base", default="origin/main", help="branch the changes are measured against")
    parser.add_argument("--spot", action="store_true", help="a Spot VM (cheaper, can be preempted) instead of on-demand")
    parser.add_argument("--machine-type", default=DEFAULT_MACHINE_TYPE, choices=MACHINE_TYPES)
    parser.add_argument("--gpus", type=int, default=DEFAULT_GPUS, choices=GPU_COUNTS,
                        help="T4 GPUs on the VM; shard i is pinned to GPU i mod N")
    parser.add_argument("--shards", action="append", type=parse_shards, metavar="SCOPE=N",
                        help="shards for a scope (repeatable), overriding the default map")
    args = parser.parse_args(argv)
    if not args.project:
        sys.exit(f"no GCP project: pass --project or set {PROJECT_ENV}")
    return stamp_on_gcp(args, runner, sleep)


if __name__ == "__main__":
    sys.exit(main())
