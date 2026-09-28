# SimReady CI: validate → publish → submit

This folder holds the tooling that validates the gripper asset, publishes it to a
public Hugging Face **dataset**, and submits it to the NVIDIA SimReady Catalog.

| File | Role |
|---|---|
| `summarize.py` | Renders a `simready-validate` `results.json` as a pass/fail table and gates on **blocking** failures only. |
| `stage_package.sh` | Reshapes the flat repo asset into the `simready-package` layout (`<asset>/simready_usd/…`). |
| `submit_to_central.sh` | Stage 2 — opens the SimReady Central PR from an immutable HF URL. Run locally. |

## Pipeline overview

```
.github/workflows/simready-validate.yml   Robot-Gripper conformance (manual + reusable)
                    │  workflow_call (gate)
                    ▼
.github/workflows/hf-publish.yml          stage → stamp → package → upload to HF dataset
                    │  emits immutable resolve URL
                    ▼
ci/simready/submit_to_central.sh          PR to NVIDIA-Omniverse/simready-central (local)
```

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

- `version` — **required**, format `YYYY.MM.DD_NN` (e.g. `2026.09.25_00`). Each
  publish must use a *new* version; NVIDIA pins by version + immutable commit SHA.
- Other inputs default to the 2F-85 / `Robotiq-Official/simready-assets`.
- Tick **`dry_run`** first: it stages, stamps, and packages without uploading, so
  you can confirm the toolchain and the package definition before a real push.

The job:
1. **Gates** on `simready-validate` (Robot-Gripper). No pass → no publish.
2. Stages the asset (`stage_package.sh`), **stamps** a Robot-Gripper validation
   result into the staged root USD, and builds `com.nvidia.simready.packaging.json`.
3. `hf upload … --repo-type dataset --delete` into the `Robotiq_2F_85/` folder,
   then prints the **immutable submission URL** to the job summary and saves it as
   the `submission-url.txt` artifact.

> First real run: the `simready-validate`/`simready-package` CLI is exercised in
> CI (the toolchain needs Python 3.11/3.12 and isn't run locally). Use `dry_run`
> to shake out any CLI/version specifics before uploading.

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

## Versioning model

- **Human version:** the `YYYY.MM.DD_NN` string, baked into the package ID
  `simready.hf.Robotiq-Official.simready-assets.Robotiq_2F_85.<version>`.
- **Immutable pin:** the Hugging Face dataset commit SHA in the `resolve/<sha>/`
  URL. That URL is what SimReady Central records; publishing a change means a new
  version + new SHA, never reusing an old one.
