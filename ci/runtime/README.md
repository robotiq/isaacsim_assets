# Runtime-test stamps

The runtime tests (`tests/runtime`) need an RTX GPU and Isaac Sim, so they do not run in CI. Instead,
**CI requires a stamp saying the tests passed, on a developer's machine, on exactly the content
being merged.** It is scoped per gripper, like the SimReady gate: a PR that changes only
`grippers/Robotiq_2F_85/` needs only the 2F-85 stamp.

## For a PR author

```bash
ci/runtime/stamp.py            # stamps every gripper this branch changed vs origin/main
ci/runtime/stamp.py --scope Robotiq_2F_85 --gui      # or one scope, with the Isaac Sim window
git add -A ci/runtime/stamps && git commit -m "test: runtime stamp"
```

`stamp.py` refuses to run with uncommitted changes in files the tests depend on, runs that gripper's
tests (`pytest tests/runtime --gripper <scope>`), and on a pass writes
`ci/runtime/stamps/<scope>-<hash>.json`. It also removes the scope's older stamps (only the stamp of the current
content is ever read), so commit with `git add -A` to include those deletions.

- Use the Isaac Sim install in `--isaacsim` (default `$ISAACSIM_ROOT`, else `~/isaacsim`). Its version is recorded;
  the gate accepts the versions in `scopes.json`.
- **Stamp after your last rebase.** The stamp is valid for the content of the scope's files, not for a commit,
  so rebasing keeps it valid unless `main` changed those same files; then the hash differs and you re-stamp.
  Two PRs that change the same gripper can't both be stamped against the old `main`: the second one to enter
  the merge queue fails, rebases, and re-stamps (the combination was never tested).

## Expect to re-stamp after `main` moves

The hash covers the `shared` paths too, so a merge to `main` that touches `tests/runtime`, `grippers/scene` or
the other shared code changes the hash of every scope: every open PR that needs stamps must rebase and re-run
`stamp.py` before it can enter the queue. A merge that only touches one gripper changes only that gripper's hash.

## What the gate checks (`verify.py`, workflow `runtime-stamp`)

1. Which scopes the change touches: the files changed between the base and the head, matched against the
   scope's paths. Files under `shared` (the tests, the scene, the tactile and ROS 2 code) belong to every scope;
   `ignore` (docs, saved views, thumbnails) belongs to none. A per-gripper data file in `tests/runtime`
   (`cubes/Robotiq_2F_85.json`, `fixed_cubes/Robotiq_2F_85_compliant.json`, ...: the `per_gripper` patterns)
   belongs to that gripper only, so tuning one gripper's baseline needs one run, not four. So does a test file
   listed in a scope's `own` patterns (`test_gripper_c10_*.py` for the C10: it only exercises that gripper).
2. For each touched scope, the stamp named by the hash of that scope's files at the head exists, describes
   that scope and hash, records 0 failures and at least one pass, and was made with an accepted Isaac Sim version.

3. Every changed file under a gripper folder (`gripper_roots`) belongs to a scope: a new `grippers/Robotiq_X/`
   that nobody added to `scopes.json` fails the check instead of being skipped. Docs don't count.

A PR that touches none of this needs no stamp. The check runs on the PR and again in the merge queue.

## Configuration

| File | Content |
|---|---|
| `scopes.json` | the scopes and their paths, the `shared` paths, the `per_gripper` data patterns and each scope's `own` test files, the `gripper_roots`, the `ignore` patterns, the accepted Isaac Sim versions |
| `stamps/` | the committed stamps, one per scope (`stamp.py` removes the older ones) |

`verify.py` reads `scopes.json` from the **base** branch, so a PR can't change its own rules, with one exception:
a PR may **add** a scope. To add a gripper, add its scope to `scopes.json` in the same PR as its folder, then run
`stamp.py`; the PR needs the new scope's stamp. A PR can't change or remove an existing scope.

## What it does and doesn't prove

It catches the realistic mistakes: tests not run, files changed after the run, a combination of PRs that was
never tested. It does not prove the run happened: the stamp is a plain JSON file, so anyone who can commit can
write one. It records the author, Isaac Sim version, GPU and every test id so a reviewer can see what was run,
but nothing re-runs it or authenticates it. If that ever matters (outside contributors), the stamp can be signed with `ssh-keygen -Y sign` and checked against an
`allowed_signers` file read from the base branch. Protect `ci/runtime/` and `.github/workflows/runtime-stamp.yml` with CODEOWNERS: the gate runs the
head's copy of `verify.py`.

The tests of this tooling run without Isaac: `python3 -m pytest ci/runtime`.
