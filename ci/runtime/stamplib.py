"""Shared logic of the runtime-test stamps (see README.md): scopes, input hashes, stamp checks.

Standard library only: verify.py runs on a plain ubuntu CI runner, stamp.py on a developer's machine.
"""

from __future__ import annotations

import fnmatch
import hashlib
import json
import subprocess
from pathlib import Path

CONFIG = "ci/runtime/scopes.json"
STAMPS_DIR = "ci/runtime/stamps"


def git(root: Path, *args: str, check: bool = True) -> bytes:
    done = subprocess.run(["git", "-C", str(root), *args], capture_output=True)
    if check and done.returncode:
        raise RuntimeError(f"git {' '.join(args)} failed: {done.stderr.decode().strip()}")
    return done.stdout


def show(root: Path, commit: str, path: str) -> bytes | None:
    """The content of ``path`` at ``commit``, or None if it does not exist there."""
    done = subprocess.run(["git", "-C", str(root), "show", f"{commit}:{path}"], capture_output=True)
    return done.stdout if done.returncode == 0 else None


def load_config(raw: bytes) -> dict:
    cfg = json.loads(raw)
    for scope, spec in cfg["scopes"].items():
        if not spec.get("paths"):
            raise ValueError(f"scope {scope!r} has no paths")
    return cfg


def _under(path: str, prefixes: list[str]) -> bool:
    return any(path == p or path.startswith(p.rstrip("/") + "/") for p in prefixes)


def _ignored(cfg: dict, path: str) -> bool:
    return any(fnmatch.fnmatch(path, pattern) for pattern in cfg["ignore"])


def _owner(cfg: dict, path: str) -> str | None:
    """The one scope a file of ``tests/runtime`` belongs to, or None for any other file (shared by every scope).
    A per-gripper data file matches the ``per_gripper`` patterns (``{scope}`` stands for a scope name); a test
    file that only exercises one gripper is listed in that scope's ``own`` patterns. ``Robotiq_Hand_E_C10.json``
    matches ``{scope}_*.json`` of ``Robotiq_Hand_E`` too, so the longest scope name wins."""
    owners = [
        scope
        for scope, spec in cfg["scopes"].items()
        for pattern in [p.replace("{scope}", scope) for p in cfg.get("per_gripper", [])] + spec.get("own", [])
        if fnmatch.fnmatch(path, pattern)
    ]
    return max(owners, key=len) if owners else None


def in_scope(cfg: dict, scope: str, path: str) -> bool:
    """Whether a change to ``path`` can change the result of the scope's tests."""
    if _ignored(cfg, path):
        return False
    owner = _owner(cfg, path)
    if owner:
        return owner == scope
    return _under(path, cfg["shared"] + cfg["scopes"][scope]["paths"])


def scope_files(root: Path, cfg: dict, scope: str, commit: str) -> list[tuple[str, str, str]]:
    """(mode, blob, path) of every file of the scope at ``commit``, sorted by path."""
    prefixes = cfg["shared"] + cfg["scopes"][scope]["paths"]
    out = git(root, "ls-tree", "-r", "-z", commit, "--", *prefixes)
    files = []
    for entry in out.split(b"\0"):
        if not entry:
            continue
        meta, path = entry.decode().split("\t", 1)
        mode, _type, blob = meta.split()
        if in_scope(cfg, scope, path):
            files.append((mode, blob, path))
    return sorted(files, key=lambda f: f[2])


def input_hash(files: list[tuple[str, str, str]]) -> str:
    """Hash of the scope's content. Blob ids make it independent of the commit it is read from, so a
    rebase or the merge queue's merge commit gives the same hash as long as the scope is unchanged."""
    digest = hashlib.sha256()
    for mode, blob, path in files:
        digest.update(f"{mode} {blob} {path}\n".encode())
    return digest.hexdigest()


def changed_paths(root: Path, base: str, head: str) -> list[str]:
    diff = git(root, "diff", "--name-only", "-z", base, head).decode()
    return [p for p in diff.split("\0") if p]


def changed_scopes(root: Path, cfg: dict, base: str, head: str) -> list[str]:
    """The scopes that changed between ``base`` and ``head``, in config order."""
    changed = changed_paths(root, base, head)
    return [s for s in cfg["scopes"] if any(in_scope(cfg, s, p) for p in changed)]


def uncovered_folders(root: Path, cfg: dict, head: str, paths: list[str]) -> list[str]:
    """Gripper folders (``gripper_roots`` patterns) with a changed file that no scope covers: a new gripper
    that nobody added to scopes.json. Ignored files (docs) and folders removed at ``head`` do not count."""
    folders = set()
    for path in paths:
        parts = path.split("/")
        folder = "/".join(parts[:2])
        if len(parts) < 3 or _ignored(cfg, path) or not any(fnmatch.fnmatch(folder, r) for r in cfg.get("gripper_roots", [])):
            continue
        if any(_under(path, spec["paths"]) for spec in cfg["scopes"].values()):
            continue
        if subprocess.run(["git", "-C", str(root), "cat-file", "-e", f"{head}:{folder}"], capture_output=True).returncode == 0:
            folders.add(folder)
    return sorted(folders)


def with_new_scopes(base_cfg: dict, head_cfg: dict) -> dict:
    """The base's rules plus the scopes a change adds. It can add a scope (the stamp of a new gripper) but
    not change or remove one, so it cannot loosen its own gate."""
    added = {name: spec for name, spec in head_cfg["scopes"].items() if name not in base_cfg["scopes"]}
    return {**base_cfg, "scopes": {**base_cfg["scopes"], **added}}


def stamp_path(scope: str, digest: str) -> str:
    return f"{STAMPS_DIR}/{scope}-{digest[:16]}.json"


def check_result(stamp: dict, cfg: dict) -> list[str]:
    """Why the recorded run does not count, if it does not."""
    problems = []
    result, env = stamp.get("result", {}), stamp.get("environment", {})
    if result.get("failed") or result.get("errors"):
        problems.append(f"the run had {result.get('failed', 0)} failure(s) and {result.get('errors', 0)} error(s)")
    if not result.get("passed"):
        problems.append("the run has no passing test")
    allowed = cfg["isaac_sim_versions"]
    version = str(env.get("isaac_sim", ""))
    if not any(version == prefix or version.startswith(prefix + ".") for prefix in allowed):
        problems.append(f"Isaac Sim {version or '?'} is not one of the accepted versions {allowed}")
    return problems
