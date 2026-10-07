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
`grippers/Robotiq_Hand_E/**`, all of them when it changes the tooling (`ci/simready/**`,
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

## 2. Submit to the SimReady Catalog — `simready-manage submit` (local)

This is the only accepted submission path: since the October 2026 release the
catalog requires each package to carry a **bill of materials** and the
**validation + benchmark** sidecars (`SUB.028`, `REVIEW.001`, `REVIEW.003`), which
only `simready-manage submit` produces. It runs locally because the benchmark
drives **Isaac Sim** (GPU, ~15 min per asset). Follow NVIDIA's OEM guide; the
summary below is what we run for the 2F-85.

### One-time setup

1. **Create the public HF dataset** `Robotiq-Official/simready-assets` with a
   `README.md` whose front-matter declares `license: cc-by-4.0` (must match the
   `--license` passed to `submit`). `simready-manage submit` uploads into it; it
   must be public and ungated by the time the PR is opened (SUB.018).
2. Complete NVIDIA **OEM onboarding** and get access to
   `NVIDIA-Omniverse/simready-central`.
3. **`<SimreadyDir>` toolchain** (per the guide's *One-time setup*, Linux — we use
   `~/robotiq/devel/nvidia/SimreadyDir`). Creates **two** venvs (several GB):
   - `.venv-submit` (py3.12): the Foundation tier packages + `simready-manage` +
     `simready-validate`/`-package` + `simready-benchmark`/`-engine-kit` + HF CLI,
     all `~=2026.8.0`. **Isaac Sim is NOT installed here.**
   - `.venv-isaac` (py3.12): `isaacsim[all,extscache]==6.1.0.0` from
     `https://pypi.nvidia.com`.
   - `~/.simready-benchmark/engines.toml` pointing `isaac_sim` + `isaac_sim_newton`
     at `.venv-isaac`; verify with `simready-benchmark --show-config` → **both
     engines `[ok]`**.
   - `<SimreadyDir>/workspace/project_config.toml` (empty `[validate]` so the
     installed tier profiles are used).
   - `hf auth login` (your HF credentials) and a `GITHUB_TOKEN` with write access
     to `simready-central`.

### Prepare the asset

`submit` takes an asset folder containing **one** intermediate folder with the
root USD named after the asset (`Robotiq_2F_85/simready_usd/Robotiq_2F_85.usda`)
plus `.thumbs/256x256/<root>.png`. Stage it with `stage_package.sh`, then **strip
files SCAN.004 forbids** — only `USD / image / audio / .glb / .mdl / .toml /
.json / .txt` are allowed inside a submitted folder. For the 2F-85 that means
removing the dev-only `newton/*.py`, `newton/README.md`, and the `*/LICENSE`
files from the staged copy (the repo is untouched).

### Run

```bash
source <SimreadyDir>/.venv-submit/bin/activate         # tools must be on PATH
GITHUB_TOKEN=<token> simready-manage \
  --project-config <SimreadyDir>/workspace/project_config.toml \
  submit <staged-asset-folder> \
  --hf-dataset Robotiq-Official/simready-assets \
  --profile Robot-Gripper \
  --asset-version 2026.10.07 \
  --license cc-by-4.0 \
  --central-index-repo https://github.com/NVIDIA-Omniverse/simready-central \
  --central-index-branch main --open-pr \
  --oem-org Robotiq --oem-contact "l-t.schreiber@robotiq.com" \
  --commit-author-name "Louis-Thomas" --commit-author-email "l-t.schreiber@robotiq.com" \
  --url-file <SimreadyDir>/submission-<version>.txt
```

It validates → benchmarks → packages (with BOM + validation + benchmark sidecars)
→ uploads version `<asset-version>_NN` to the HF dataset → opens a PR on SimReady
Central pointing at the new immutable commit URL. Then edit the PR's **"Notes for
reviewers"** placeholder (one sentence on the asset set, any `--skip-*`/`--ignore-*`
flags, and any failed benchmark tests + why they're acceptable), changing nothing
else in the generated body.

Failed **benchmark** tests do **not** block the submission — they're uploaded and
NVIDIA reads them (note them in the PR). Failed **validation** of a *required*
feature does block it. Do not pass `--skip-benchmark` / `--ignore-*` for a real
submission.

### Gotchas (local environment)

- **Activate `.venv-submit`.** `simready-manage` shells out to `simready-validate`
  by name; without the venv on `PATH` it fails with "simready-validate not found".
- **`gh` 2.4.0 on this host has no `gh auth token`.** Read the token from
  `~/.config/gh/hosts.yml` (`oauth_token`) instead. `gh pr edit` also fails here
  (deprecated Projects-classic GraphQL) — edit the PR body via
  `gh api -X PATCH repos/<owner>/<repo>/pulls/<n> -f body="$(cat body.md)"`.
- **SCAN.004 sweep** before uploading (see *Prepare the asset*) — `submit` uploads
  whatever is present and SCAN.004 only fails *after* publication.
- One **open PR per dataset**: close/merge an existing one before submitting again.

## Versioning model

- **Human version:** `--asset-version YYYY.MM.DD`; `submit` appends `_NN`, baked
  into the package ID
  `simready.hf.Robotiq-Official.simready-assets.Robotiq_2F_85.<version>`.
- **Immutable pin:** the Hugging Face commit SHA in the `resolve/<sha>/` URL. That
  URL is what SimReady Central records; a changed asset needs a new version + SHA
  once a PR is merged. While iterating (or after a rejected PR), the same version
  may be re-uploaded freely.
