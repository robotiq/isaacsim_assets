---
name: submit-simready
description: Submit or update Robotiq gripper assets (2F-85, 2F-140, Hand-E, Hand-E C10) in NVIDIA's SimReady Catalog (SimReady Central + the Robotiq-Official Hugging Face dataset). Follows NVIDIA's submit-simready-assets skill and adds Robotiq's steps — all repo tests (unit + every Isaac Sim runtime test) first, then ci/simready/submit.sh. Use whenever someone asks to submit, publish, resubmit, update or push a gripper to SimReady, SimReady Central, the SimReady Catalog or NVIDIA, or asks how to do it.
---

# Submit grippers to the SimReady Catalog

NVIDIA's **`submit-simready-assets`** skill drives the submission; this skill
only adds what is specific to Robotiq. Load NVIDIA's skill first and follow it
throughout, with the answers and extra steps below:

```bash
gh api repos/NVIDIA-Omniverse/simready-central/contents/.agents/skills/submit-simready-assets/SKILL.md --jq .content | base64 -d
```

Its rules all apply (the OEM guide is the source of truth, ask before remote
writes, one open pull request per dataset, keep earlier versions, no
`--skip-*`/`--ignore-*` flags, never merge, never change the dataset's
visibility). Where it and this file disagree, NVIDIA's skill wins: tell the user
so `ci/simready/README.md` and this file get fixed. The human-readable version
of this procedure, and the reasons behind each extra check, are in
`ci/simready/README.md` §2.

## Robotiq answers to NVIDIA's inputs

- **Assets:** folders under `grippers/` (`Robotiq_2F_85`, `Robotiq_2F_140`,
  `Robotiq_Hand_E`, `Robotiq_Hand_E_C10`). They are flat in the repo:
  `ci/simready/submit.sh` stages them into the one intermediate folder NVIDIA
  requires, so never pass `grippers/<asset>` to `simready-manage` directly.
  Propose the grippers that changed since their latest listed version (the
  dry run prints the listed package IDs; compare each date with
  `git log --since=<date> -- grippers/<asset>`). A gripper never submitted
  before (the Hand-E C10 so far) needs the user's explicit yes.
- **Profile:** NVIDIA's skill says to ask: propose **`Robot-Gripper`** (what
  every Robotiq gripper has been accepted with) and let the user confirm.
- **Dataset:** `Robotiq-Official/simready-assets`; it is already public, under
  `cc-by-4.0`.
- **Organisation / contact:** `Robotiq` / the submitter's git `user.email`.
- **`<SimreadyDir>`:** ask; it is passed to the scripts as `SIMREADY_DIR`.
- Submit from an up-to-date `origin/main` with no local changes under `grippers/`.

## Extra steps

1. **All repo tests, before anything else touches the assets:**
   ```bash
   ci/simready/run_pre_submit_tests.sh          # add --gui if the user wants to watch
   ```
   Unit tests, every `tests/runtime` test on Isaac Sim 6.2+ and the ROS 2
   runtime test; about 25 minutes. Run it in the background with a log. It must
   end with `ALL PRE-SUBMISSION TESTS PASSED`; otherwise stop, report the failing
   tests, and do not submit (an asset fix is its own pull request on this repo).
   Isaac Sim runs one instance at a time: where `~/robotiq/devel/isaac-lock` is
   installed, launches queue by themselves; otherwise ask the user first.
2. **NVIDIA's "Check before uploading", automated:**
   ```bash
   SIMREADY_DIR=<dir> ci/simready/submit.sh --dry-run <Gripper> [<Gripper> ...]
   ```
   It runs NVIDIA's checks (open pull request, version already listed,
   Hugging Face write access, `simready-validate`), strips the file types
   SCAN.004 rejects from the staged copy, and adds two checks of ours: no Kit/system MDL (`@OmniPBR.mdl@`, rejected in simready-central#15)
   and a 256x256 thumbnail on a black background (REVIEW.004). Fix every FAIL
   at the source, not with an `--allow-*` flag, unless the user explicitly
   accepts that specific finding.
3. **Submit with the same script** (instead of calling `simready-manage`), after
   the confirmation NVIDIA's skill asks for:
   ```bash
   SIMREADY_DIR=<dir> ci/simready/submit.sh <Gripper> [<Gripper> ...]
   ```
   It stages, strips non-package files, queues for the Isaac lock, and runs
   `simready-manage submit` with `--central-index-repo … --open-pr`. Run it
   detached with a log, as NVIDIA's skill says.
4. **Reviewer notes**, in addition to what NVIDIA's skill lists: per gripper,
   what changed since its previous listed version (`git log <date>.. --
   grippers/<asset>`, in plain words), and the benchmark compared with the
   previous accepted version
   (`https://huggingface.co/datasets/Robotiq-Official/simready-assets/resolve/<commit>/<asset>/.metadata/<asset>_benchmark.json`,
   commit from the listed URL). Known and accepted: Hand-E
   `display_color_response` / `preview_surface_renders` (waived in
   simready-central#38, see NVIDIA/simready-foundation#35). `gh pr edit` fails
   on old `gh`: replace the placeholder with
   `gh api -X PATCH repos/NVIDIA-Omniverse/simready-central/pulls/<n> -f body="$(cat body.md)"`.
5. **After the hand-off:** NVIDIA publishes merged submissions on Mondays, in
   Isaac Sim under SimReady > OEMs > Robotiq-Official and on
   https://huggingface.co/spaces/nvidia/simready-central. Merged is not yet
   visible.
