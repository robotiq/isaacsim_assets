#!/usr/bin/env bash
# Stage a repo asset folder into the layout `simready-package` requires.
#
# NVIDIA's SimReady packaging profile (NP.005) mandates exactly ONE intermediate
# folder between the logical-asset folder and the interface USD:
#
#   <logical-asset>/
#   └── <intermediate>/            # conventionally `simready_usd`
#       ├── <root-usd>
#       ├── .thumbs/256x256/<root-usd>.png
#       ├── materials/ parts/ payloads/ configuration/ ...
#
# In this repo the asset is flat: the root `.usda` sits directly in
# grippers/Robotiq_2F_85/ with its sublayer folders as siblings. Every USD
# reference is relative (`@./...@` / `@../...@`) and only ever reaches a sibling
# folder, so copying the whole tree wholesale one level down into
# <intermediate>/ keeps every reference resolving inside the asset folder.
#
# Usage:
#   stage_package.sh <src-asset-dir> <root-usd-filename> <out-dir> [intermediate]
#
#   <src-asset-dir>     e.g. grippers/Robotiq_2F_85 (its basename is the logical
#                       asset name; must NOT contain a version)
#   <root-usd-filename> e.g. Robotiq_2F_85.usda (the interface USD, relative to
#                       <src-asset-dir>)
#   <out-dir>           staging root; the package is written to
#                       <out-dir>/<logical-asset>/
#   [intermediate]      intermediate folder name (default: simready_usd)
#
# Prints the path to the staged logical-asset folder on stdout (last line).
set -euo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
  echo "usage: $0 <src-asset-dir> <root-usd-filename> <out-dir> [intermediate]" >&2
  exit 2
fi

SRC="$1"
ROOT_USD="$2"
OUT_DIR="$3"
INTERMEDIATE="${4:-simready_usd}"

if [ ! -d "$SRC" ]; then
  echo "::error::source asset dir not found: $SRC" >&2
  exit 1
fi
if [ ! -f "$SRC/$ROOT_USD" ]; then
  echo "::error::root USD not found: $SRC/$ROOT_USD" >&2
  exit 1
fi

LOGICAL_NAME="$(basename "$SRC")"
# NP: the folder name must not carry a version. Reject the common YYYY.MM.DD_NN.
if printf '%s' "$LOGICAL_NAME" | grep -qE '[0-9]{4}\.[0-9]{2}\.[0-9]{2}'; then
  echo "::error::logical asset name '$LOGICAL_NAME' looks like it contains a version" >&2
  exit 1
fi

DEST="$OUT_DIR/$LOGICAL_NAME/$INTERMEDIATE"
rm -rf "$OUT_DIR/$LOGICAL_NAME"
mkdir -p "$DEST"

# Copy the entire asset tree (including dotfiles like .thumbs) into the
# intermediate folder, preserving structure so relative refs stay valid.
cp -a "$SRC/." "$DEST/"

# The thumbnail must sit beside the root USD as .thumbs/256x256/<root-usd>.png.
THUMB="$DEST/.thumbs/256x256/$ROOT_USD.png"
if [ ! -f "$THUMB" ]; then
  echo "::error::missing thumbnail expected at <intermediate>/.thumbs/256x256/$ROOT_USD.png" >&2
  echo "         (simready-package requires a representative PNG beside the root USD)" >&2
  exit 1
fi

echo "Staged '$LOGICAL_NAME' -> $DEST" >&2
echo "  root USD:  $INTERMEDIATE/$ROOT_USD" >&2
echo "  thumbnail: $INTERMEDIATE/.thumbs/256x256/$ROOT_USD.png" >&2

# Last line = the staged logical-asset folder (what simready-package/hf consume).
echo "$OUT_DIR/$LOGICAL_NAME"
