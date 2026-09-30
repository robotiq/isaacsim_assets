# SimReady CI: validate → publish → submit

This folder holds the tooling that validates the gripper asset, publishes it to a
public Hugging Face **dataset**, and submits it to the NVIDIA SimReady Catalog.

The authoritative source for the submission workflow is NVIDIA's OEM guide,
[`docs/oem-asset-submission-guide.md`](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/docs/oem-asset-submission-guide.md)
in `NVIDIA-Omniverse/simready-central` (and its
[`submit-simready-assets`](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/.agents/skills/submit-simready-assets/SKILL.md)
skill). The scripts here are a thin, CI-friendly automation of that guide — if the
two ever disagree, the guide wins.

| File | Role |
|---|---|
| `summarize.py` | Renders a `simready-validate` `results.json` as a pass/fail table and gates against the baseline (`expected.json`). Strict for the validate gate; `--allow-improvements` for publish. |
| `stage_package.sh` | Copies the asset into the one intermediate folder `simready-package` requires (`<asset>/simready_usd/…`, NP.005), keeping the organized subfolders. |
| `submit_to_central.sh` | Stage 2 — opens the SimReady Central PR from an immutable HF URL. Run locally. |

## Pipeline overview

```
.github/workflows/simready-validate.yml   Robot-Gripper conformance (manual + reusable)
                    │  workflow_call (gate)
                    ▼
.github/workflows/hf-publish.yml          stage → Robot-Gripper stamp → package def → upload
                    │  emits immutable resolve URL
                    ▼
ci/simready/submit_to_central.sh          PR to NVIDIA-Omniverse/simready-central (local)
```

## How packaging works (and keeps the variants + organized layout)

The asset is a multi-layer, multi-variant USD: a root that composes
`configuration/ payloads/ parts/ materials/` sublayers, with `Physics` and
`Fingertip` variant sets. It is published **as authored** (nested), matching
NVIDIA's reference packages (`apple_a01_nobom` etc. are nested too):

- **`stage_package.sh`** — places the asset under one `simready_usd/` intermediate
  folder (NP.005), preserving the subfolders. Every reference stays within
  `simready_usd/`, so **root-only** validation resolves them and AA.001 passes —
  no flattening. (Flattening would satisfy AA.001 but violate Isaac RC.001
  "clean-folder"; it is not done.)
- **`simready-validate --profile Robot-Gripper --stamp-asset-validation`** on the
  interface USD — the guide's step 1 for a gripper. It writes the feature-level
  results into the root's `customLayerData` and, because Robot-Gripper 2.1.0 also
  requires `FET_031` (self-contained / AA.001) and `FET_033` (thumbnail + metadata
  / SR.002-003) **and** `FET_021_ISAAC` (RC.001 clean-folder), one root-only run
  is the asset-conformance stamp *and* the packaging gate. Gated with
  `summarize.py --allow-improvements` (blocking failures + regressions vs baseline
  fail; improvements are accepted). Robot-Gripper is the gripper-appropriate
  profile; the prop `Prop-Robotics-*` profiles the guide lists require
  graspable-line + semantic labels a gripper isn't authored for — see the open
  profile-clarification issue.
- **`simready-package --skip-*-validation`** writes the package definition
  (nobom form, like NVIDIA's `apple_a01_nobom`).

> We avoid `simready-package[publish]`'s WRAPP `--repo` flow: its freeze step
> (`ovpackage`/`omni.wrapp`) crashes on the CI runner with an asyncio "Semaphore
> bound to a different event loop" error. `simready-validate` (root-only) + a
> `--skip-*` package def gives the same result without `wrapp`.

The uploaded HF folder is the package: `com.nvidia.simready.packaging.json` +
the nested `simready_usd/` (root + subfolders + thumbnail).

## One-time setup

1. **Create the public HF dataset** `Robotiq-Official/simready-assets` with a
   `README.md` whose front-matter declares `license: cc-by-4.0` (must match the
   package license). The `hf-publish` workflow can also create it on first upload
   if the token allows.
2. **Add the `HF_TOKEN` secret** (a Hugging Face *write* token scoped to
   `Robotiq-Official`) to the GitHub repo: Settings → Secrets and variables →
   Actions.
3. For stage 2 only: complete NVIDIA **OEM onboarding**, get collaborator access
   to `NVIDIA-Omniverse/simready-central`, and authenticate `gh` locally.

## Stage 1 — publish to Hugging Face (CI)

Run the **`hf-publish`** workflow from the Actions tab (`workflow_dispatch`).

- `version` — **optional**. Leave blank and the job derives today's date plus the
  next `_NN` (from the dataset's existing version tags), e.g. `2026.09.28_00`.
  Provide one (`YYYY.MM.DD_NN`) only to override. Either way the job refuses to
  reuse an already-published version; NVIDIA pins by version + immutable commit SHA.
- Other inputs default to the 2F-85 / `Robotiq-Official/simready-assets`.
- Tick **`dry_run`** first: it builds + validates the package without uploading, so
  you can confirm the package definition before a real push.

The job:
1. **Gates** on `simready-validate` (Robot-Gripper). No pass → no publish.
2. Builds the package: `stage_package.sh` (nested `simready_usd/`, no flatten) →
   `simready-validate --profile Robot-Gripper --stamp-asset-validation` (root-only:
   asset conformance + packaging gate + writes the stamp) → `simready-package
   --skip-*-validation` (nobom package definition).
3. `hf upload … --repo-type dataset --delete` the unpacked package into the
   `Robotiq_2F_85/` folder, then prints the **immutable submission URL** to the job
   summary and saves it as the `submission-url.txt` artifact.

> First CI run: each step is proven locally with the real toolchain (Python 3.12),
> but the wired workflow runs in CI for the first time — use `dry_run` to shake out
> any environment specifics before uploading.

## Stage 2 — submit to SimReady Central (local)

With the immutable URL from stage 1:

```bash
ci/simready/submit_to_central.sh \
  --central /path/to/simready-central \
  --url @submission-url.txt \
  --description "Robotiq 2F-85 adaptive parallel gripper (PhysX + Newton variants)"
```

It appends the URL to `submissions/Robotiq-Official/simready-assets` (keeping
older versions), commits with a DCO sign-off (`git commit -s`), and opens the PR.
Use `--no-pr` to prepare the branch/commit without pushing.

This implements step 2 of NVIDIA's
[OEM asset submission guide](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/docs/oem-asset-submission-guide.md)
(the submission-file path/format, one immutable URL per line, DCO sign-off, PR
title/body). Read that guide — and the
[`submit-simready-assets`](https://github.com/NVIDIA-Omniverse/simready-central/blob/main/.agents/skills/submit-simready-assets/SKILL.md)
skill — before submitting; they are the source of truth and may change.

## Versioning model

- **Human version:** the `YYYY.MM.DD_NN` string, baked into the package ID
  `simready.hf.Robotiq-Official.simready-assets.Robotiq_2F_85.<version>`.
- **Immutable pin:** the Hugging Face dataset commit SHA in the `resolve/<sha>/`
  URL. That URL is what SimReady Central records; publishing a change means a new
  version + new SHA, never reusing an old one.
- **Version registry:** each publish creates a tag `<version>` on the dataset.
  That's how `hf-publish` auto-derives the next `_NN` and refuses to reuse a
  version. NVIDIA ignores these tags (it pins by SHA); they're just our bookkeeping.
