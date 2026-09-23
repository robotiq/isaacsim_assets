#!/usr/bin/env python3
"""Render a simready-validate results.json as a markdown pass/fail table.

Prints the table to stdout and, when running in GitHub Actions, appends it to
the job summary ($GITHUB_STEP_SUMMARY).

SimReady conformance is gated on *blocking* ("failing requirements") failures
only. A profile marks most features `optional`; requirements attached solely to
optional features are non-blocking and are surfaced by the validator as
"optional requirements". A feature that fails only optional requirements does
not block the asset, so this script exits non-zero only when a feature has
blocking failing requirements.

Usage: summarize.py [results.json]
"""
import ast
import json
import os
import sys


def _as_list(value):
    """The validator stores requirement lists as a stringified Python list."""
    value = value or []
    if isinstance(value, str):
        try:
            return ast.literal_eval(value)
        except Exception:
            return [value]
    return value


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "results.json"
    with open(path) as fh:
        data = json.load(fh)

    lines: list[str] = []
    overall_ok = True
    for asset, info in data.items():
        profile = info.get("profile_id", "?")
        version = info.get("profile_version", "?")
        feats = info.get("features_summary", {})

        blocking = {n: _as_list(f.get("failing requirements")) for n, f in feats.items()}
        blocking = {n: r for n, r in blocking.items() if r}
        optional = {n: _as_list(f.get("optional requirements")) for n, f in feats.items()}
        optional = {n: r for n, r in optional.items() if r}

        n_clean = sum(1 for n in feats if n not in blocking and n not in optional)
        ok = len(feats) > 0 and not blocking  # conformant iff no blocking failures
        overall_ok = overall_ok and ok

        lines += [
            f"## {'✅ PASSED' if ok else '❌ FAILED'} — `{profile}` v{version}",
            "",
            f"**Asset:** `{os.path.basename(asset)}`  ",
            f"**Blocking failures:** {len(blocking)}  ",
            f"**Optional (non-blocking) gaps:** {len(optional)}  ",
            f"**Features fully satisfied:** {n_clean} / {len(feats)}",
            "",
            "| Feature | Version | Result | Blocking | Optional (non-blocking) |",
            "|---|---|:---:|---|---|",
        ]
        for name, f in sorted(feats.items()):
            hard = sorted(blocking.get(name, []))
            soft = sorted(optional.get(name, []))
            if hard:
                mark = "❌"
            elif soft:
                mark = "⚠️"
            else:
                mark = "✅"
            hcell = ", ".join(f"`{r}`" for r in hard) if hard else "—"
            scell = ", ".join(f"`{r}`" for r in soft) if soft else "—"
            lines.append(f"| `{name}` | {f.get('version', '')} | {mark} | {hcell} | {scell} |")
        lines.append("")
        lines.append("_✅ satisfied · ⚠️ optional feature not met (non-blocking) · ❌ blocking failure_")
        lines.append("")

    md = "\n".join(lines)
    print(md)
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a") as fh:
            fh.write(md + "\n")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
