#!/usr/bin/env bash
# Submit Robotiq grippers to the NVIDIA SimReady Catalog in one run.
#
# Wraps NVIDIA's `simready-manage submit` with the checks we learned to run
# BEFORE anything is uploaded (an upload burns an immutable Hugging Face commit
# and the catalog's own checks only fail after publication):
#
#   1. the grippers exist and are committed (submissions come from a commit)
#   2. no Kit/system MDL reference (`@OmniPBR.mdl@`): NVIDIA's content audit
#      rejects a referenced file that is not in the package
#   3. the standard thumbnail exists, is 256x256 and sits on a black background
#      (REVIEW.004)
#   4. `simready-validate --profile Robot-Gripper` reports 0 blocking failures
#   5. no submission pull request is already open for the dataset (only one may
#      be open at a time, so all grippers go in ONE run / ONE pull request)
#   6. the version is not already listed for any of the grippers
#   7. the Hugging Face token can write to the dataset's organisation (a
#      read-only token would only fail at the first upload, after the benchmark)
#
# then stages each gripper (stage_package.sh), deletes every file type the
# catalog's package scan rejects (SCAN.004: anything but USD, images, audio,
# .glb, .mdl, .toml, .json, .txt), and runs `simready-manage submit` for all of
# them: validation, the Isaac Sim benchmark, packaging with a bill of materials,
# the Hugging Face upload and the SimReady Central pull request. It never merges.
#
# Usage:
#   ci/simready/submit.sh [options] GRIPPER [GRIPPER ...]
#     GRIPPER: folder name under grippers/, e.g. Robotiq_2F_85
#
# Options:
#   --version YYYY.MM.DD   asset version (default: today). simready-manage
#                          appends _NN. Use a new date for a changed asset.
#   --dry-run              run every check and stage, but do not submit
#   --no-pr                upload, but do not open the pull request
#   --allow-nonblack-thumbnail  downgrade check 3 to a warning
#
# Environment:
#   SIMREADY_DIR   the directory NVIDIA's one-time setup filled (.venv-submit,
#                  .venv-isaac, workspace/project_config.toml). Required.
#   OEM_CONTACT    contact written into the pull request
#                  (default: git config user.email)
#   COMMIT_AUTHOR_NAME / COMMIT_AUTHOR_EMAIL  Signed-off-by identity
#                  (default: git config user.name / user.email)
#   GITHUB_TOKEN   write access to NVIDIA-Omniverse/simready-central. If unset,
#                  taken from `gh auth token` or ~/.config/gh/hosts.yml.
#   ISAAC_LOCK     path to isaac_lock.sh; when present, the benchmark queues for
#                  the single Isaac Sim instead of starting next to another one.
#
# Hugging Face credentials come from `hf auth login` (~/.cache/huggingface).
# See ci/simready/README.md for the full procedure.
set -euo pipefail

DATASET="Robotiq-Official/simready-assets"
CENTRAL="https://github.com/NVIDIA-Omniverse/simready-central"
CENTRAL_REPO="NVIDIA-Omniverse/simready-central"
PROFILE="Robot-Gripper"
LICENSE_ID="cc-by-4.0"
OEM_ORG="Robotiq"
ALLOWED_EXT='usd|usda|usdc|usdz|png|jpg|jpeg|exr|hdr|tif|tiff|tga|bmp|webp|wav|ogg|mp3|flac|glb|mdl|toml|json|txt'

VERSION="$(date +%Y.%m.%d)"
DRY_RUN=0 OPEN_PR=1 ALLOW_THUMB=0
GRIPPERS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --version) VERSION="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --no-pr) OPEN_PR=0; shift ;;
    --allow-nonblack-thumbnail) ALLOW_THUMB=1; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *) GRIPPERS+=("$1"); shift ;;
  esac
done

REPO="$(git rev-parse --show-toplevel)"
cd "$REPO"
fail=0
err()  { echo "  FAIL: $*" >&2; fail=1; }
warn() { echo "  WARN: $*" >&2; }
step() { echo; echo "== $* =="; }

[ "${#GRIPPERS[@]}" -gt 0 ] || { echo "no gripper given (e.g. Robotiq_2F_85)" >&2; exit 2; }
[[ "$VERSION" =~ ^[0-9]{4}\.[0-9]{2}\.[0-9]{2}(_[0-9]{2})?$ ]] || { echo "bad --version '$VERSION' (YYYY.MM.DD)" >&2; exit 2; }
[ -n "${SIMREADY_DIR:-}" ] || { echo "SIMREADY_DIR is not set (see ci/simready/README.md, one-time setup)" >&2; exit 2; }
VENV="$SIMREADY_DIR/.venv-submit"
CONFIG="$SIMREADY_DIR/workspace/project_config.toml"
[ -x "$VENV/bin/simready-manage" ] || { echo "no simready-manage in $VENV (one-time setup not done?)" >&2; exit 2; }
[ -f "$CONFIG" ] || { echo "missing $CONFIG" >&2; exit 2; }
PY="$VENV/bin/python"

step "1. grippers present and committed"
for g in "${GRIPPERS[@]}"; do
  if [ ! -f "grippers/$g/$g.usda" ]; then err "grippers/$g/$g.usda not found"; continue; fi
  if [ -n "$(git status --porcelain -- "grippers/$g")" ]; then
    err "grippers/$g has uncommitted changes: submit from a commit (normally origin/main)"
  fi
done
echo "  HEAD $(git rev-parse --short HEAD) ($(git rev-parse --abbrev-ref HEAD))"
[ "$(git rev-parse HEAD)" = "$(git rev-parse origin/main 2>/dev/null || echo none)" ] \
  || warn "HEAD is not origin/main: the catalog will get exactly this commit's assets"

step "2. no Kit/system MDL references"
for g in "${GRIPPERS[@]}"; do
  hits=$(grep -rlE 'sourceAsset *= *@[A-Za-z]' "grippers/$g" --include='*.usda' --include='*.usd' 2>/dev/null || true)
  if [ -n "$hits" ]; then err "$g references an MDL from Kit's search path (e.g. @OmniPBR.mdl@): $hits"; else echo "  $g: ok"; fi
done

step "3. thumbnail (256x256, black background)"
for g in "${GRIPPERS[@]}"; do
  t="grippers/$g/.thumbs/256x256/$g.usda.png"
  if [ ! -f "$t" ]; then err "$g: $t missing"; continue; fi
  res=$("$PY" - "$t" <<'PY'
import sys
from collections import Counter
from PIL import Image
im = Image.open(sys.argv[1]).convert("RGB")
px = im.get_flattened_data() if hasattr(im, "get_flattened_data") else im.getdata()
bg, _ = Counter(px).most_common(1)[0]
print(f"{im.size[0]}x{im.size[1]} {bg[0]},{bg[1]},{bg[2]}")
PY
) || { err "$g: $t is not a readable image (Git LFS not pulled?)"; continue; }
  size=${res% *}; bg=${res#* }
  [ "$size" = "256x256" ] || err "$g: thumbnail is $size, expected 256x256"
  if [ "$bg" = "0,0,0" ]; then echo "  $g: ok"
  elif [ "$ALLOW_THUMB" = 1 ]; then warn "$g: thumbnail background is rgb($bg), not black (allowed by flag)"
  else err "$g: thumbnail background is rgb($bg), NVIDIA requires black (REVIEW.004)"; fi
done

step "4. simready-validate --profile $PROFILE (0 blocking)"
for g in "${GRIPPERS[@]}"; do
  out=$(mktemp)
  "$VENV/bin/simready-validate" --profile "$PROFILE" --output "$out" "grippers/$g/$g.usda" >/dev/null 2>&1 || true
  "$PY" - "$out" "$g" <<'PY' || fail=1
import json, sys
try:
    info = next(iter(json.load(open(sys.argv[1])).values()))
except Exception as e:
    print(f"  FAIL: {sys.argv[2]}: validation produced no results ({e})", file=sys.stderr); sys.exit(1)
fs = info["features_summary"]
blocking = [n for n, f in fs.items() if not f.get("passed") and f.get("blocking_failure")]
if blocking:
    print(f"  FAIL: {sys.argv[2]}: blocking features {blocking}", file=sys.stderr); sys.exit(1)
print(f"  {sys.argv[2]}: profile {info.get('profile_version')}, {len(fs)} features, 0 blocking")
PY
  rm -f "$out"
done

step "5. no open submission pull request for $DATASET"
open_prs=$(gh api "repos/$CENTRAL_REPO/pulls?state=open&per_page=100" \
  --jq ".[] | select(.title | contains(\"$DATASET\")) | \"#\(.number) \(.title)\"" 2>/dev/null || echo "?")
if [ "$open_prs" = "?" ]; then warn "could not query $CENTRAL_REPO (gh auth?)"
elif [ -n "$open_prs" ]; then err "already open: $open_prs (only one may be open; wait for it or add to it)"
else echo "  none"; fi

step "6. version $VERSION not already listed"
listed=$(gh api "repos/$CENTRAL_REPO/contents/submissions/$DATASET" --jq .content 2>/dev/null | base64 -d 2>/dev/null || true)
if [ -n "$listed" ]; then
  ids=$(for u in $listed; do curl -fsSL "$u" 2>/dev/null | "$PY" -c 'import json,sys; print(json.load(sys.stdin).get("package_id",""))' 2>/dev/null; done)
  for g in "${GRIPPERS[@]}"; do
    if grep -qiE "\.${g}\.${VERSION//./\\.}(_[0-9]{2})?$" <<<"$ids"; then err "$g $VERSION is already listed: use a new --version"; fi
  done
  echo "  listed now:"; sed 's/^/    /' <<<"$ids"
else
  echo "  no submission file yet (first submission)"
fi

step "7. Hugging Face token can write to ${DATASET%%/*}"
"$PY" - "${DATASET%%/*}" <<'PY' || fail=1
import sys
from huggingface_hub import HfApi
org = sys.argv[1]
try:
    who = HfApi().whoami()
except Exception as e:
    print(f"  FAIL: no usable Hugging Face login ({type(e).__name__}): run `hf auth login`", file=sys.stderr); sys.exit(1)
token = who.get("auth", {}).get("accessToken", {})
role = token.get("role")
in_org = {o.get("name"): o.get("roleInOrg") for o in who.get("orgs", [])}
if role == "fineGrained":
    ok = any(s.get("entity", {}).get("name") == org and "repo.write" in s.get("permissions", [])
             for s in (token.get("fineGrained") or {}).get("scoped", []))
else:  # a classic write or OAuth token writes wherever the account can
    ok = role != "read" and in_org.get(org) in ("admin", "write", "contributor")
if not ok:
    print(f"  FAIL: {who.get('name')}'s {role} token cannot write to {org} "
          f"(org role: {in_org.get(org)}); log in with a token that has repo.write there", file=sys.stderr); sys.exit(1)
print(f"  {who.get('name')}: {role} token, {org} role {in_org.get(org)}")
PY

[ "$fail" = 0 ] || { echo; echo "Preflight failed: nothing was staged or uploaded." >&2; exit 1; }

step "8. stage + strip (SCAN.004)"
STAGE="$SIMREADY_DIR/submit/$VERSION"
rm -rf "$STAGE"; mkdir -p "$STAGE"
DIRS=()
for g in "${GRIPPERS[@]}"; do
  bash ci/simready/stage_package.sh "grippers/$g" "$g.usda" "$STAGE" >/dev/null
  removed=$(find "$STAGE/$g" -type f | grep -viE "\.($ALLOWED_EXT)$" || true)
  [ -n "$removed" ] && xargs -d '\n' rm -f <<<"$removed"
  find "$STAGE/$g" -type d -empty -delete
  echo "  $g -> $STAGE/$g (removed $(grep -c . <<<"$removed" || true) non-package file(s))"
  [ -n "$removed" ] && sed "s#^$STAGE/$g/#      #" <<<"$removed"
  DIRS+=("$STAGE/$g")
done

if [ "$DRY_RUN" = 1 ]; then echo; echo "Dry run: all checks passed and $STAGE is staged. Nothing was uploaded."; exit 0; fi

step "9. simready-manage submit (validate, benchmark, package, upload$([ "$OPEN_PR" = 1 ] && echo ', pull request'))"
if [ -z "${GITHUB_TOKEN:-}" ]; then
  GITHUB_TOKEN=$(gh auth token 2>/dev/null) || GITHUB_TOKEN=""
  # gh < 2.7 has no `gh auth token`; it printed usage instead of a token
  [[ "$GITHUB_TOKEN" =~ ^(gh[opsu]_|github_pat_)[A-Za-z0-9_]+$ ]] || GITHUB_TOKEN=$("$PY" -c \
    "import os,yaml; print(yaml.safe_load(open(os.path.expanduser('~/.config/gh/hosts.yml')))['github.com'].get('oauth_token',''))" 2>/dev/null || true)
fi
[ -n "$GITHUB_TOKEN" ] || { echo "no GITHUB_TOKEN (set it, or log in with gh)" >&2; exit 2; }
export GITHUB_TOKEN
CONTACT="${OEM_CONTACT:-$(git config user.email)}"
AUTHOR_NAME="${COMMIT_AUTHOR_NAME:-$(git config user.name)}"
AUTHOR_EMAIL="${COMMIT_AUTHOR_EMAIL:-$(git config user.email)}"
URL_FILE="$SIMREADY_DIR/submission-$VERSION.txt"
pr_args=()
[ "$OPEN_PR" = 1 ] && pr_args=(--central-index-repo "$CENTRAL" --central-index-branch main --open-pr
                                --oem-org "$OEM_ORG" --oem-contact "$CONTACT")
cmd=(bash -c 'source "$0/bin/activate" && cd "$1" && shift 2 && exec simready-manage "$@"'
     "$VENV" "$SIMREADY_DIR/workspace" _
     --project-config "$CONFIG" submit "${DIRS[@]}"
     --hf-dataset "$DATASET" --profile "$PROFILE" --asset-version "$VERSION" --license "$LICENSE_ID"
     --commit-author-name "$AUTHOR_NAME" --commit-author-email "$AUTHOR_EMAIL"
     --url-file "$URL_FILE" "${pr_args[@]}")
LOCK="${ISAAC_LOCK:-$HOME/robotiq/devel/isaac-lock/isaac_lock.sh}"
if [ -x "$LOCK" ]; then
  echo "  queueing for Isaac via $LOCK (the benchmark starts its own Isaac Sim)"
  "$LOCK" run --wait --label "SimReady submit $VERSION: ${GRIPPERS[*]}" -- "${cmd[@]}"
else
  "${cmd[@]}"
fi
echo
echo "URLs: $URL_FILE"
echo "Next: fill the pull request's 'Notes for reviewers' (see ci/simready/README.md). Never merge it: NVIDIA does."
