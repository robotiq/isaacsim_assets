# SimReady: validate → submit

This folder holds the tooling that **validates** the Robotiq gripper assets and
submits them to the NVIDIA SimReady Catalog. The submission itself —
packaging, the Hugging Face upload, and opening the Catalog pull request — is done
by NVIDIA's **`simready-manage submit`** (see §2); there is no separate publish
step.

The authoritative source for the submission workflow is NVIDIA's OEM guide,
[`docs/oem-asset-submission-guide.md`](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/docs/oem-asset-submission-guide.md)
in `NVIDIA-Omniverse/simready-central`, and its
[`submit-simready-assets`](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/.agents/skills/submit-simready-assets/SKILL.md)
skill. Everything here automates or summarizes that guide — **if the two disagree,
the guide wins.** The acceptance checks are in
[`docs/acceptance-criteria.md`](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/docs/acceptance-criteria.md);
every rejection cites an id from it (`SUB.*`, `SCAN.*`, `REVIEW.*`).

| File | Role |
|---|---|
| `summarize.py` | Renders a `simready-validate` `results.json` as a pass/fail table and gates against the baseline (`expected/<asset>.json`, one per gripper). |
| `stage_package.sh` | Copies an asset into the one intermediate folder SimReady requires (`<asset>/simready_usd/…`, NP.005), keeping the organized subfolders. Used to stage an asset before `simready-manage submit`. |
| `run_pre_submit_tests.sh` | Runs every repo test (unit, all Isaac Sim runtime tests, the ROS 2 runtime test) before a submission; must pass. |
| `submit.sh` | Preflight checks + staging + `simready-manage submit` for several grippers in one run (one pull request). `--dry-run` uploads nothing. |
| `expected/author_mesh_extents_normals.py` | Dev helper: fills in `extent` + `normals` on CAD-imported meshes (the 2F-140 parts needed it for VG.002/VG.027). |

## Pipeline overview

```
.github/workflows/simready-validate-pr.yml   PR gate: validates only the gripper(s) whose folder changed
                    │  calls, per gripper
                    ▼
.github/workflows/simready-validate.yml      Robot-Gripper conformance (one asset; manual + reusable)

                    ─── then, locally, for a catalog submission: ───

simready-manage submit                        validate → benchmark → package (+ bill of materials)
  (NVIDIA's tool; see §2)                        → upload to our HF dataset → open the SimReady Central PR
```

`simready-manage submit` **uploads the package to our Hugging Face dataset itself**
and records the resulting immutable `resolve/<commit>/…` URL in the submission — we
do not push the package to Hugging Face separately.

## 1. Validate

`simready-validate` checks the root USD against a SimReady **profile**. We validate
against **`Robot-Gripper`** — the gripper-appropriate profile. (NVIDIA confirmed
this is correct for an end-effector in simready-central issue #7; the prop
`Prop-Robotics-*` profiles require a graspable line + semantic labels a gripper
isn't authored for.) Pinned to SimReady Foundation **v2026.08.0**
(`simready-validate` `2026.8.0`, profile **3.0.0**). Do **not** pass
`--profile-version`; the newest installed Robot-Gripper is used.

### PR validation is per gripper

`simready-validate-pr.yml` runs on every pull request. It validates the 2F-85 when
the PR changes `grippers/Robotiq_2F_85/**`, the 2F-140 when it changes
`grippers/Robotiq_2F_140/**`, the Hand-E when it changes
`grippers/Robotiq_Hand_E/**`, the Hand-E C10 when it changes `grippers/Robotiq_Hand_E_C10/**`, all of them when it changes the tooling (`ci/simready/**`,
the validate workflows), and nothing otherwise. Each gripper is gated against its
own baseline, `ci/simready/expected/<asset>.json`. Its `validate` job is the
required check. Refresh a baseline after an intended change with
`python3 ci/simready/summarize.py --update results.json` (the file is picked from
the validated asset's name) — generate it from an **in-repo (unstaged)**
validation, the same thing the gate runs, or folder-structure requirements (e.g.
`NP.005`) diverge and show up as false regressions. Manual dispatches of
`simready-validate.yml` still work and still post the `validate` commit status.

**What v2026.08.0 (profile 3.0.0) requires the asset to author** — all satisfied:
`FET_031` self-contained (AA.001), `FET_033` thumbnail + nested metadata
(SR.002-003), `FET_021_ISAAC` clean-folder (RC.001), `FET_006` UsdPreviewSurface
materials (VM.PS.002), and `RB.COL.004` (a rigid body's `principalAxes` and
`diagonalInertia` must be both-or-neither authored).

## 2. Submit to the SimReady Catalog (local)

This is the only accepted submission path: the catalog requires each package to
carry a **bill of materials** and the **validation + benchmark** sidecars
(`SUB.028`, `REVIEW.001`, `REVIEW.003`), which only NVIDIA's
`simready-manage submit` produces. It runs locally because the benchmark drives
**Isaac Sim** (RTX GPU).

> **With Claude Code:** ask it to *"submit the grippers to SimReady"* (or to update
> one). The project skill [`.claude/skills/submit-simready`](../../.claude/skills/submit-simready/SKILL.md)
> runs the whole procedure below — tests, preflight, submit, reviewer notes — and
> stops for your OK before anything is uploaded. Without Claude Code, follow the
> same steps by hand.

### Access and contacts

- **Hugging Face:** write access to the [`Robotiq-Official`](https://huggingface.co/Robotiq-Official)
  organisation (`hf auth login` with your account). The dataset
  `Robotiq-Official/simready-assets` is public and declares `license: cc-by-4.0`
  in its card (must match `--license`).
- **GitHub:** write access to `NVIDIA-Omniverse/simready-central` (NVIDIA OEM
  onboarding) and `gh auth login`.
- **NVIDIA contact** for the program: John Kosnik (`@jkosnik11` on the
  simready-central pull requests).

### One-time setup (`SIMREADY_DIR`)

Follow the OEM guide's *One-time setup* (Linux) in a directory of your choice,
exported as `SIMREADY_DIR`. It creates **two** venvs (several GB):

- `.venv-submit` (py3.12): the Foundation tier packages + `simready-manage` +
  `simready-validate`/`-package` + `simready-benchmark`/`-engine-kit` + HF CLI, at
  the versions the guide pins (2026.8.0 at the time of writing). **No Isaac Sim here.**
- `.venv-isaac` (py3.12): `isaacsim[all,extscache]` at the version the guide pins
  (6.1.0.0), from `https://pypi.nvidia.com`. This is the Isaac the **benchmark**
  uses; it is independent of the Isaac Sim 6.2+ the runtime tests need.
- `~/.simready-benchmark/engines.toml` pointing `isaac_sim` + `isaac_sim_newton`
  at `.venv-isaac`; `simready-benchmark --show-config` must list **both engines `[ok]`**.
- `$SIMREADY_DIR/workspace/project_config.toml` (empty `[validate]`, so the
  installed tier profiles are used).

### Procedure

1. **Read NVIDIA's current guide** (links at the top). If it changed since this
   README, follow the guide and update this file.
2. **Pick the grippers and the version.** All grippers go in **one** run: the
   catalog allows only one open pull request per dataset. Submit the ones that
   changed since their latest listed version. Version = today's date
   `YYYY.MM.DD` (`simready-manage` appends `_NN`); a version already listed for a
   gripper cannot be reused.
3. **Run every test** from an up-to-date `main`:
   ```bash
   ci/simready/run_pre_submit_tests.sh        # --gui to watch
   ```
   Unit tests, every runtime test on Isaac Sim 6.2+ (all grippers and physics
   variants, see [`tests/runtime`](../../tests/runtime/README.md)), and the ROS 2
   runtime test in Isaac's ROS environment. It must print
   `ALL PRE-SUBMISSION TESTS PASSED` (about 25 minutes on an RTX workstation:
   150 runtime cases + 10 ROS 2 cases). Do not submit otherwise.
4. **Preflight** (nothing is uploaded):
   ```bash
   SIMREADY_DIR=<dir> ci/simready/submit.sh --dry-run Robotiq_2F_85 Robotiq_2F_140 ...
   ```
   Fails on: uncommitted gripper changes, a Kit/system MDL reference
   (`@OmniPBR.mdl@`: NVIDIA's content audit rejected that in simready-central#15),
   a thumbnail that is not 256x256 on a **black** background (`REVIEW.004`;
   NVIDIA flagged our `#1a1a1a` ones), any blocking `simready-validate` failure,
   an already open pull request, or a version already listed. It then stages each
   gripper (`stage_package.sh`) and deletes every file the package scan rejects
   (`SCAN.004`: only USD, images, audio, `.glb`, `.mdl`, `.toml`, `.json`, `.txt`
   may ship; our `newton/` helpers, `docs/` and `LICENSE` files are dropped from
   the staged copy, the repo is untouched).
5. **Submit:**
   ```bash
   SIMREADY_DIR=<dir> ci/simready/submit.sh Robotiq_2F_85 Robotiq_2F_140 ...
   ```
   Runs `simready-manage submit` for all of them: validation, the Isaac benchmark
   (roughly 6-15 min per gripper), packaging, the Hugging Face upload and the
   SimReady Central pull request, which keeps every earlier version listed and
   adds the new ones. The pull request contact/author default to your git
   identity (`OEM_CONTACT`, `COMMIT_AUTHOR_NAME`, `COMMIT_AUTHOR_EMAIL` override).
   Where the Isaac lock (`~/robotiq/devel/isaac-lock`) is installed, the run
   queues for the GPU instead of starting next to another Isaac.
6. **Reviewer notes.** Replace the pull request's *"Notes for reviewers"*
   placeholder (and nothing else): what the asset set is, what changed per
   gripper since its previous version, that validation is 0 blocking with no
   `--skip`/`--ignore` flags, and every non-passing benchmark test with why it is
   acceptable. Failed benchmark tests do not block a submission; NVIDIA reads them.
   Known and accepted: Hand-E `display_color_response` / `preview_surface_renders`
   (waived in simready-central#38, NVIDIA/simready-foundation#35).
   `gh pr edit` fails on old `gh`; use
   `gh api -X PATCH repos/NVIDIA-Omniverse/simready-central/pulls/<n> -f body="$(cat body.md)"`.
7. **Never merge** the pull request: NVIDIA reviews it, comments there, and
   merges or closes it. A closed one is fixed and submitted again as a new pull
   request (the same version may be reused, nothing reached the catalog).

### After the merge

Merged is not yet visible. NVIDIA publishes merged submissions on **Mondays**:
then the assets appear in Isaac Sim under **SimReady > OEMs > Robotiq-Official**
(`omniverse-content-production…/Assets/Isaac/<version>/Isaac/SimReady/OEMs/Robotiq-Official/`,
a publicly listable bucket) and on the
[SimReady Central Space](https://huggingface.co/spaces/nvidia/simready-central).
The legacy NVIDIA-built `Isaac/Robots/Robotiq/2F-85` is hidden from Isaac Sim 6.3.

### Gotchas

- **Activate `.venv-submit`** when calling `simready-manage` yourself: it shells
  out to `simready-validate` by name (`submit.sh` does this for you).
- **`gh` < 2.7 has no `gh auth token`.** `submit.sh` falls back to
  `~/.config/gh/hosts.yml`.
- **A ROS sourced in the shell** breaks Isaac's pytest (`No module named 'lark'`);
  `run_pre_submit_tests.sh` unsets `PYTHONPATH`.
- **The runtime tests need Isaac Sim 6.2+**; the helper picks the newest
  `~/isaacsim*` at 6.2 or later (`ISAAC_SIM` overrides).

## Versioning model

- **Human version:** `--asset-version YYYY.MM.DD`; `submit` appends `_NN`, baked
  into the package ID
  `simready.hf.Robotiq-Official.simready-assets.<asset>.<version>`.
- **Immutable pin:** the Hugging Face commit SHA in the `resolve/<sha>/` URL. That
  URL is what SimReady Central records; a changed asset needs a new version + SHA
  once a PR is merged. While iterating (or after a rejected PR), the same version
  may be re-uploaded freely.
