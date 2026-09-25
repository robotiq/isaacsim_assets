#!/usr/bin/env bash
# Stage 2: submit an already-published Hugging Face package to the NVIDIA
# SimReady Catalog by opening a pull request against NVIDIA-Omniverse/simready-central.
#
# This edits an EXTERNAL repo and opens a PR, so it is deliberately NOT part of
# CI: it needs your NVIDIA OEM onboarding, collaborator access to
# simready-central, and an authenticated `gh`. Run it locally.
#
# What it does:
#   - Appends the immutable resolve URL (from stage 1 / hf-publish) to
#     submissions/<namespace>/<dataset>, preserving all existing version lines.
#   - Commits with a DCO sign-off (git commit -s, required by the repo) and opens
#     a PR with `gh`.
#
# Usage:
#   submit_to_central.sh \
#     --central <path-to-simready-central-checkout> \
#     --url <immutable-resolve-url | @submission-url.txt> \
#     [--description "one sentence about the asset set"] \
#     [--profile Robot-Gripper] \
#     [--no-pr]     # prepare the branch + commit but don't open the PR
#
# The <namespace>/<dataset> and asset are parsed from the URL, so you don't pass
# them separately.
set -euo pipefail

CENTRAL="" URL="" DESCRIPTION="" PROFILE="Robot-Gripper" OPEN_PR=1
while [ "$#" -gt 0 ]; do
  case "$1" in
    --central) CENTRAL="$2"; shift 2 ;;
    --url) URL="$2"; shift 2 ;;
    --description) DESCRIPTION="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --no-pr) OPEN_PR=0; shift ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

[ -n "$CENTRAL" ] || { echo "::error::--central <path> is required" >&2; exit 2; }
[ -n "$URL" ] || { echo "::error::--url <url|@file> is required" >&2; exit 2; }

# --url @file reads the URL from a file (e.g. the hf-publish artifact).
if [ "${URL#@}" != "$URL" ]; then
  URL="$(tr -d '[:space:]' < "${URL#@}")"
fi

# Validate + parse the URL:
# https://huggingface.co/datasets/<ns>/<dataset>/resolve/<40-hex>/<asset>/com.nvidia.simready.packaging.json
re='^https://huggingface\.co/datasets/([^/.]+)/([^/.]+)/resolve/([0-9a-f]{40})/([^/]+)/com\.nvidia\.simready\.packaging\.json$'
if [[ ! "$URL" =~ $re ]]; then
  echo "::error::URL is not a valid fixed-version SimReady package URL:" >&2
  echo "  $URL" >&2
  echo "  expected .../datasets/<ns>/<dataset>/resolve/<40-hex-sha>/<asset>/com.nvidia.simready.packaging.json" >&2
  exit 1
fi
NAMESPACE="${BASH_REMATCH[1]}"
DATASET="${BASH_REMATCH[2]}"
ASSET="${BASH_REMATCH[4]}"

if [ ! -d "$CENTRAL/.git" ] || [ ! -f "$CENTRAL/docs/oem-asset-submission-guide.md" ]; then
  echo "::error::$CENTRAL does not look like a simready-central checkout" >&2
  exit 1
fi

SUB_REL="submissions/$NAMESPACE/$DATASET"
BRANCH="submit-$NAMESPACE-$DATASET"

echo "Namespace/dataset: $NAMESPACE/$DATASET"
echo "Asset:             $ASSET"
echo "Submission file:   $SUB_REL"
echo "Branch:            $BRANCH"
echo

cd "$CENTRAL"
git switch main
git pull --ff-only
git switch -c "$BRANCH" 2>/dev/null || git switch "$BRANCH"

mkdir -p "$(dirname "$SUB_REL")"
touch "$SUB_REL"

if grep -qxF "$URL" "$SUB_REL"; then
  echo "URL already present in $SUB_REL — nothing to add."
else
  # Keep older versions; append the new one at the bottom (preferred ordering).
  printf '%s\n' "$URL" >> "$SUB_REL"
  echo "Appended URL to $SUB_REL"
fi

echo
echo "=== $SUB_REL now contains ==="
cat "$SUB_REL"
echo "============================="
echo

N_LINES=$(grep -c . "$SUB_REL" || echo 0)
if [ -z "$DESCRIPTION" ]; then
  DESCRIPTION="the $ASSET asset from $NAMESPACE/$DATASET"
  echo "::warning::no --description given; using a generic one. Prefer a real sentence."
fi

git add "$SUB_REL"
git commit -s -m "Submit packages from $NAMESPACE/$DATASET" \
  -m "Adds $ASSET ($NAMESPACE/$DATASET) at an immutable Hugging Face commit."

if [ "$OPEN_PR" -eq 0 ]; then
  echo "Branch and commit prepared. Skipping PR (--no-pr). Push + open it yourself with:"
  echo "  git push -u origin $BRANCH"
  echo "  gh pr create ..."
  exit 0
fi

git push -u origin "$BRANCH"
gh pr create \
  --repo NVIDIA-Omniverse/simready-central \
  --title "Submit packages from $NAMESPACE/$DATASET" \
  --body "Submits $N_LINES packaged asset version(s) from $NAMESPACE/$DATASET for NVIDIA review. Assets: $DESCRIPTION. Validation profile: $PROFILE."

echo "PR opened against NVIDIA-Omniverse/simready-central."
