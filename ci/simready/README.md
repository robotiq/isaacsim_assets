# SimReady CI: validate → publish → submit

This folder holds the tooling that validates the gripper asset, publishes it to a
public Hugging Face **dataset**, and submits it to the NVIDIA SimReady Catalog.

| File | Role |
|---|---|
| `summarize.py` | Renders a `simready-validate` `results.json` as a pass/fail table and gates on **blocking** failures (+ baseline ratchet). |
| `stage_package.sh` | Adds the one intermediate folder `simready-package` requires (`<asset>/simready_usd/…`, NP.005). |
| `colocate_package.py` | Flattens the *folder* layout to a self-contained flat `simready_usd/` with anchored `./` refs, **keeping all sublayers/payloads and both variant sets** (no `../`, which packaging's AA.001 rejects). Not a stage flatten. |
| `submit_to_central.sh` | Stage 2 — opens the SimReady Central PR from an immutable HF URL. Run locally. |

## Pipeline overview

```
.github/workflows/simready-validate.yml   Robot-Gripper conformance (manual + reusable)
                    │  workflow_call (gate)
                    ▼
.github/workflows/hf-publish.yml          stage → co-locate → package (--repo, WRAPP) → upload
                    │  emits immutable resolve URL
                    ▼
ci/simready/submit_to_central.sh          PR to NVIDIA-Omniverse/simready-central (local)
```

## How packaging preserves the variants

`simready-package`'s Package-Candidate validation rejects USD refs that use `../`
to escape a layer's own directory (AA.001), and — in the plain local flow — it
validates *every* USD file as a standalone asset (so multi-layer assets fail on
their sublayers). The fix is not to flatten the asset (that bakes a single
solver) but to match NVIDIA's reference package shape:

- **`simready-package[publish]`** — the `publish` extra pulls `ovpackage`, which
  provides the `wrapp` module. That unlocks the `--repo` flow, which validates
  only the declared **root** (Package-Candidate pre+post) and treats sublayers as
  dependencies. A **local folder** repo works — no Omniverse/Nucleus needed.
- **`colocate_package.py`** — co-locates every layer into one flat `simready_usd/`
  with `./` references, so nothing uses `../`. All variant payloads
  (PhysX/Newton/compliant/tactile) are copied and both `Physics` and `Fingertip`
  variant sets survive.

The uploaded HF folder is the unpacked package: `com.nvidia.simready.packaging.json`
+ `.metadata/` (BOM, conformance) + `simready_usd/`.

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
2. Builds the package: `stage_package.sh` → `colocate_package.py` →
   `simready-package --repo <local>` (Package-Candidate pre+post validation, BOM).
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

## Versioning model

- **Human version:** the `YYYY.MM.DD_NN` string, baked into the package ID
  `simready.hf.Robotiq-Official.simready-assets.Robotiq_2F_85.<version>`.
- **Immutable pin:** the Hugging Face dataset commit SHA in the `resolve/<sha>/`
  URL. That URL is what SimReady Central records; publishing a change means a new
  version + new SHA, never reusing an old one.
- **Version registry:** each publish creates a tag `<version>` on the dataset.
  That's how `hf-publish` auto-derives the next `_NN` and refuses to reuse a
  version. NVIDIA ignores these tags (it pins by SHA); they're just our bookkeeping.
