#!/usr/bin/env python3
"""Render a simready-validate results.json as a markdown pass/fail table.

Prints the table to stdout and, when running in GitHub Actions, appends it to
the job summary ($GITHUB_STEP_SUMMARY). Exits non-zero if any feature failed.

Usage: summarize.py [results.json]
"""
import ast
import json
import os
import sys


def _fails(value):
    """The validator stores 'failing requirements' as a stringified list."""
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
        n_pass = sum(1 for f in feats.values() if f.get("passed"))
        n_tot = len(feats)
        ok = n_tot > 0 and all(f.get("passed") for f in feats.values())
        overall_ok = overall_ok and ok
        lines += [
            f"## {'✅ PASSED' if ok else '❌ FAILED'} — `{profile}` v{version}",
            "",
            f"**Asset:** `{os.path.basename(asset)}`  ",
            f"**Features passed:** {n_pass} / {n_tot}",
            "",
            "| Feature | Version | Result | Failing requirements |",
            "|---|---|:---:|---|",
        ]
        for name, f in sorted(feats.items()):
            fails = _fails(f.get("failing requirements"))
            cell = ", ".join(f"`{r}`" for r in sorted(fails)) if fails else "—"
            mark = "✅" if f.get("passed") else "❌"
            lines.append(f"| `{name}` | {f.get('version', '')} | {mark} | {cell} |")
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
