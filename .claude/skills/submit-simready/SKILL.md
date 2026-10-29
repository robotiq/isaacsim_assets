---
name: submit-simready
description: Submit or update Robotiq gripper assets (2F-85, 2F-140, Hand-E, Hand-E C10) in NVIDIA's SimReady Catalog (SimReady Central + the Robotiq-Official Hugging Face dataset). Runs all repo tests (unit + every Isaac Sim runtime test) first, then the preflight checks, then `simready-manage submit` in one run, and drafts the pull request's reviewer notes. Use whenever someone asks to submit, publish, resubmit, update or push a gripper to SimReady, SimReady Central, the SimReady Catalog or NVIDIA, or asks how to do it.
---

# Submit grippers to the SimReady Catalog

The procedure, the one-time setup and the reasons behind each rule are in
`ci/simready/README.md` (read it first). NVIDIA's guide is the source of truth
when it disagrees with us: re-read it at the start of every run, it changes.

## Hard rules

- **Never merge** the SimReady Central pull request (NVIDIA does) and never
  change the Hugging Face dataset's visibility.
- **One open pull request per dataset.** All grippers to submit go in ONE run;
  if a pull request is already open for `Robotiq-Official/simready-assets`, stop
  and tell the user.
- **All tests pass before any upload** (step 3). No exceptions without the
  user's explicit OK, and say so in the pull request notes.
- **Ask before every remote write** (Hugging Face upload, pull request, pull
  request edits), showing exactly what will be written.
- Never pass `--skip-benchmark`, `--skip-validate` or `--ignore-*` to
  `simready-manage`, and never use `submit.sh`'s `--allow-*` flags unless the
  user explicitly accepts that specific finding.
- Keep earlier versions in the catalog (the default). Removal is a separate,
  explicit request.
- Isaac Sim: one instance on the GPU at a time. Where `~/robotiq/devel/isaac-lock`
  is installed, launches queue by themselves; otherwise ask the user before
  launching and check nothing else is running.

## Steps

### 1. Read the current sources
- `ci/simready/README.md`.
- From `NVIDIA-Omniverse/simready-central` main (`gh api repos/NVIDIA-Omniverse/simready-central/contents/<path> --jq .content | base64 -d`):
  `docs/oem-asset-submission-guide.md`, `.agents/skills/submit-simready-assets/SKILL.md`,
  `docs/acceptance-criteria.md`. Note anything that changed since the README was
  written (new required steps, tool versions, rules) and tell the user before
  continuing; the README may need an update.
- Compare the guide's pinned tool versions with what is installed in
  `$SIMREADY_DIR/.venv-submit` (`pip list | grep -i simready`).

### 2. Settle the inputs (ask once, in one message)
- **Which grippers.** Propose the ones that changed since their latest listed
  version: the listed package IDs (`simready.hf.Robotiq-Official.simready-assets.<asset>.<YYYY.MM.DD>_NN`)
  are printed by `ci/simready/submit.sh --dry-run` (step 4); compare each date
  with `git log --since=<date> -- grippers/<asset>`. Grippers never submitted
  (e.g. a new folder under `grippers/`) need the user's explicit yes.
- **Version:** today's date `YYYY.MM.DD` (one version for the whole run).
- **`SIMREADY_DIR`** (NVIDIA's one-time setup; README §2) and the OEM contact
  (default: git `user.email`). If the setup is missing, walk the user through
  README §2 "One-time setup"; their credential steps (`hf auth login`, GitHub
  access to simready-central) are theirs to run.
- A one-sentence description of the asset set for the reviewer notes.

Check the working copy: submit from an up-to-date `origin/main` with no local
changes under `grippers/`.

### 3. Run all the tests
```bash
ci/simready/run_pre_submit_tests.sh          # add --gui if the user wants to watch
```
It runs `tests/unit`, every `tests/runtime` test on an Isaac Sim 6.2+ install
(all grippers, all physics variants), and the ROS 2 runtime test in Isaac's ROS
environment. It takes about 25 minutes (150 runtime + 10 ROS 2 cases): start it in the
background with its output in a log, and wait for it. It must end with
`ALL PRE-SUBMISSION TESTS PASSED`. On a failure or a skipped ROS 2 test: stop,
report the failing tests with their messages, and do not submit. Fixing an asset
is a separate change (pull request on this repo, merged, then start over).

### 4. Preflight
```bash
SIMREADY_DIR=<dir> ci/simready/submit.sh --dry-run <Gripper> [<Gripper> ...]
```
Checks committed state, no Kit/system MDL (`@OmniPBR.mdl@`), 256x256 thumbnail
on a black background, `simready-validate` 0 blocking, no open pull request,
version not already listed; then stages and strips non-package files. Fix every
FAIL at the source (repo change) rather than with a flag.

### 5. Confirm, then submit
Show the user: the grippers, the version, the dataset (`Robotiq-Official/simready-assets`),
the pull request target (`NVIDIA-Omniverse/simready-central`), and that earlier
versions stay listed. On their OK:
```bash
SIMREADY_DIR=<dir> ci/simready/submit.sh <Gripper> [<Gripper> ...]
```
Run it in the background with a log and wait (validation + Isaac benchmark +
upload: roughly 6-15 minutes per gripper, plus any wait for the Isaac lock).
Never stop it by process name mid-upload. It prints the pull request URL.

### 6. Reviewer notes
The generated pull request body has a "Notes for reviewers" placeholder. Draft:
- the one-sentence description of the asset set, and that earlier versions stay listed;
- per gripper, what changed since its previous listed version (from
  `git log <previous-date>.. -- grippers/<asset>`, in plain language);
- validation: profile + 0 blocking, and that no `--skip`/`--ignore` flag was used;
- benchmark: every non-passing test per gripper, with why it is acceptable.
  Compare with the previous accepted version's results
  (`https://huggingface.co/datasets/Robotiq-Official/simready-assets/resolve/<commit>/<asset>/.metadata/<asset>_benchmark.json`,
  commit from the listed URL) and call out anything new. Known and accepted so
  far: Hand-E `display_color_response` / `preview_surface_renders` (waived by
  NVIDIA in simready-central#38, see NVIDIA/simready-foundation#35).

Show the draft to the user. Once approved, replace only the placeholder:
```bash
gh pr view <n> --repo NVIDIA-Omniverse/simready-central --json body --jq .body > body.md
# replace the "<!-- ... -->" under "## Notes for reviewers" with the notes, change nothing else
gh api -X PATCH repos/NVIDIA-Omniverse/simready-central/pulls/<n> -f body="$(cat body.md)"
```
(`gh pr edit` fails on old `gh` versions.) Then wait for the pull request's
checks (package list, package scan) and report them.

### 7. Report and hand off
Report the pull request link, the upload URLs (`$SIMREADY_DIR/submission-<version>.txt`),
the test and benchmark results, and anything excluded. Reviewer comments arrive
on the pull request; reply there. After NVIDIA merges, assets go live with the
next Monday publication: in Isaac Sim under SimReady > OEMs > Robotiq-Official
(`omniverse-content-production…/Assets/Isaac/<isaac-version>/Isaac/SimReady/OEMs/Robotiq-Official/`)
and on https://huggingface.co/spaces/nvidia/simready-central.
