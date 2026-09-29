#!/usr/bin/env python3
"""Produce a self-contained, flat SimReady package source from a composed USD.

SimReady packaging (`simready-package`, Package-Candidate) rejects USD asset
references that use `../` to escape the referencing layer's own directory
(AA.001, "outside of the asset root"). Our gripper is authored as a layered
asset — the root sublayers `configuration/`, references `payloads/`, and those
sublayers cross-reference sibling `parts/` and `materials/` folders with `../`.
That fails packaging post-validation.

This tool rewrites the asset into the shape NVIDIA's reference packages use: a
single flat `simready_usd/` folder where every layer sits side by side and every
reference is an anchored `./<name>` (no `../`). It does NOT flatten composition —
all sublayers, payloads, and BOTH `Physics` and `Fingertip` variant sets are
preserved, so the packaged asset still switches PhysX / Newton / compliant /
tactile exactly like the source.

Standard NVIDIA MDL search paths (e.g. `@OmniPBR.mdl@`, no path separator) are
left untouched — they resolve at runtime and packaging only warns on them.

Usage:
  colocate_package.py --src-root <composed-root.usda> --out <dir>
                      --asset-name <name> --thumbnail <png>
                      [--root-usd-name sm_<name>_01.usda]

Writes <out>/<asset-name>/simready_usd/... and prints that logical-asset dir.
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

from pxr import Sdf, UsdUtils


def _alloc_name(path: str, used: set[str]) -> str:
    """Pick a flat basename for *path* not already in *used* (suffix on collision)."""
    base = os.path.basename(path)
    if base in used:
        stem, ext = os.path.splitext(base)
        i = 1
        while f"{stem}_{i}{ext}" in used:
            i += 1
        base = f"{stem}_{i}{ext}"
    used.add(base)
    return base


def _unique_basenames(layer_paths: list[str], used: set[str]) -> dict[str, str]:
    """Map each real layer path -> a unique flat basename, reserving names in *used*."""
    return {os.path.realpath(p): _alloc_name(p, used) for p in layer_paths}


def colocate(src_root: str, out: str, asset_name: str, thumbnail: str,
             root_usd_name: str) -> str:
    if not os.path.isfile(src_root):
        sys.exit(f"::error::composed root USD not found: {src_root}")
    if not os.path.isfile(thumbnail):
        sys.exit(f"::error::thumbnail not found: {thumbnail}")

    dst_dir = os.path.join(out, asset_name, "simready_usd")
    shutil.rmtree(os.path.join(out, asset_name), ignore_errors=True)
    os.makedirs(os.path.join(dst_dir, ".thumbs", "256x256"), exist_ok=True)

    # Every USD layer reachable from the root (root + sublayers + refs + payloads,
    # across all variant selections).
    layers, _assets, unresolved = UsdUtils.ComputeAllDependencies(src_root)
    layer_paths = [l.realPath for l in layers if l.realPath]
    used: set[str] = set()                 # every flat basename in simready_usd/
    names = _unique_basenames(layer_paths, used)
    sidecars: dict[str, str] = {}          # real sidecar path -> flat basename
    for u in unresolved:
        # MDL search paths are expected/unresolved; anything else is worth noting.
        # Must go to stderr — the last stdout line is the logical-asset dir the
        # caller captures.
        if not (("/" not in u) and u.lower().endswith(".mdl")):
            print(f"::warning::unresolved dependency (left as-authored): {u}", file=sys.stderr)

    # Copy every layer flat, then re-anchor its asset paths to ./<basename>.
    for src in layer_paths:
        shutil.copy2(src, os.path.join(dst_dir, names[os.path.realpath(src)]))

    for src in layer_paths:
        flat = os.path.join(dst_dir, names[os.path.realpath(src)])
        lyr = Sdf.Layer.FindOrOpen(flat)
        src_dir = os.path.dirname(src)

        def modify(ap: str, _src_dir: str = src_dir) -> str:
            if not ap:
                return ap
            # Leave standard MDL search paths (no separator) untouched.
            if "/" not in ap and ap.lower().endswith(".mdl"):
                return ap
            resolved = os.path.realpath(os.path.join(_src_dir, ap))
            if resolved in names:                      # a co-located USD layer
                return "./" + names[resolved]
            if os.path.isfile(resolved):               # a sidecar asset (texture, etc.)
                # Allocate a unique flat name (shared with layers + other sidecars)
                # so two same-basename assets from different folders don't collide.
                tb = sidecars.get(resolved)
                if tb is None:
                    tb = _alloc_name(resolved, used)
                    sidecars[resolved] = tb
                    shutil.copy2(resolved, os.path.join(dst_dir, tb))
                return "./" + tb
            return ap

        UsdUtils.ModifyAssetPaths(lyr, modify)
        lyr.Save()

    # Canonical root name + thumbnail beside it.
    root_flat = os.path.join(dst_dir, names[os.path.realpath(src_root)])
    os.replace(root_flat, os.path.join(dst_dir, root_usd_name))
    shutil.copy2(thumbnail, os.path.join(dst_dir, ".thumbs", "256x256", root_usd_name + ".png"))

    logical_dir = os.path.join(out, asset_name)
    print(f"Co-located {len(layer_paths)} layers -> {dst_dir}", file=sys.stderr)
    print(f"  root: simready_usd/{root_usd_name}", file=sys.stderr)
    print(logical_dir)   # last line = logical-asset dir for simready-package
    return logical_dir


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--src-root", required=True, help="composed root USD (.usda/.usd)")
    ap.add_argument("--out", required=True, help="output root; writes <out>/<asset-name>/")
    ap.add_argument("--asset-name", required=True, help="logical asset name (no version)")
    ap.add_argument("--thumbnail", required=True, help="representative PNG")
    ap.add_argument("--root-usd-name", default=None,
                    help="root USD filename in the package (default sm_<asset>_01.usd)")
    a = ap.parse_args()
    # .usda (ASCII) matches the co-located content and keeps
    # simready-validate --stamp-asset-validation's atomic-save happy (a generic
    # .usd extension makes it pass an invalid format="usd"). .usd/.usda are
    # interchangeable per the SimReady guide.
    root_usd_name = a.root_usd_name or f"sm_{a.asset_name}_01.usda"
    colocate(a.src_root, a.out, a.asset_name, a.thumbnail, root_usd_name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
