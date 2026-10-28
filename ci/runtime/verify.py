#!/usr/bin/env python3
"""CI gate: every gripper scope a change touches must carry a runtime-test stamp for its current content.

    ci/runtime/verify.py --base <sha> --head <sha>

The scopes and the accepted Isaac Sim versions are read from BASE, not from HEAD, so a change
cannot loosen its own gate; a change may add a scope (a new gripper), not alter or remove one. The
scope hash and the stamp are read from HEAD.
Exits 1 if a touched scope has no valid stamp, or a changed gripper folder is in no scope.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stamplib  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]

HOWTO = "Run `ci/runtime/stamp.py` on the final commit of the branch (after the last rebase) and commit the files it writes."


def check_scope(cfg: dict, scope: str, head: str) -> str | None:
    """None if the scope has a valid stamp at ``head``, else why not."""
    digest = stamplib.input_hash(stamplib.scope_files(ROOT, cfg, scope, head))
    path = stamplib.stamp_path(scope, digest)
    payload = stamplib.show(ROOT, head, path)
    if payload is None:
        return f"no stamp for the current content ({path}). {HOWTO}"
    stamp = json.loads(payload)
    if stamp.get("scope") != scope or stamp.get("input_hash") != digest:
        return f"{path} does not describe this scope and content"
    problems = stamplib.check_result(stamp, cfg)
    return "; ".join(problems) if problems else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", required=True)
    parser.add_argument("--head", required=True)
    args = parser.parse_args()

    raw = stamplib.show(ROOT, args.base, stamplib.CONFIG)
    if raw is None:
        print(f"{stamplib.CONFIG} does not exist on the base yet: runtime stamps are not enforced.")
        return 0
    cfg = stamplib.load_config(raw)
    head_raw = stamplib.show(ROOT, args.head, stamplib.CONFIG)
    if head_raw is not None:
        cfg = stamplib.with_new_scopes(cfg, stamplib.load_config(head_raw))

    scopes = stamplib.changed_scopes(ROOT, cfg, args.base, args.head)
    print("Scopes touched:", scopes or "none")
    rows, failed = [], False
    for folder in stamplib.uncovered_folders(ROOT, cfg, args.head, stamplib.changed_paths(ROOT, args.base, args.head)):
        problem = f"{folder} is in no scope: add one to {stamplib.CONFIG}, then stamp it"
        failed = True
        rows.append((folder, problem))
        print(f"FAIL {folder}: {problem}")
        print(f"::error title=Gripper folder without a runtime-test scope::{problem}")
    for scope in scopes:
        problem = check_scope(cfg, scope, args.head)
        failed |= problem is not None
        rows.append((scope, problem))
        print(f"{'FAIL' if problem else 'ok  '} {scope}" + (f": {problem}" if problem else ""))
        if problem:
            print(f"::error title=Runtime stamp missing or invalid ({scope})::{problem}")

    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a") as out:
            out.write("### Runtime-test stamps\n\n")
            if not rows:
                out.write("No file the runtime tests depend on changed: no stamp required.\n")
            else:
                out.write("| Scope | Result |\n|---|---|\n")
                for scope, problem in rows:
                    out.write(f"| `{scope}` | {'❌ ' + problem if problem else '✅ stamp for this content'} |\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
