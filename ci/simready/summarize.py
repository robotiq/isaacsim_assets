#!/usr/bin/env python3
"""Render a simready-validate results.json and gate the job against a baseline.

Prints a markdown table to stdout and, in GitHub Actions, appends it to the job
summary ($GITHUB_STEP_SUMMARY).

Gating (a "ratchet"):
  * A **blocking** failure (a "failing requirements" entry — a requirement of a
    mandatory feature) always fails the job.
  * An **optional** failure (an "optional requirements" entry — a requirement of
    an optional feature) does NOT fail on its own, BUT if it is a *regression*
    versus the asset's committed baseline (ci/simready/expected/<asset>.json) it fails the job.
    This keeps optional features from silently degrading.
  * An **improvement** (a requirement that the baseline records as failing but
    now passes) ALSO fails the job, until the baseline is refreshed. This is
    deliberate: if an unnoticed improvement left the baseline stale, the floor
    would never rise and a later regression back to the old state would go
    undetected. Failing forces the baseline to be updated in the same PR, so it
    always tracks reality and the ratchet can only move up.

    Pass --allow-improvements to accept improvements without failing (they are
    still reported). Use this when comparing a *different* asset against the
    baseline — e.g. validating a *staged* package (which gains NP.005 from the
    `simready_usd/` intermediate folder) against an in-repo baseline — so a gain
    there is expected and must not be treated as a stale baseline. Blocking
    failures and regressions still fail.

Regenerate the baseline after an intended change:
    python3 ci/simready/summarize.py --update results.json

Usage:
    summarize.py [results.json]                       # render + gate against baseline
    summarize.py --allow-improvements [results.json]  # ... but don't fail on improvements
    summarize.py --update [results.json]              # (re)write the baseline from results
"""
import ast
import json
import os
import sys

BASELINE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "expected")


def _baseline_path(asset):
    """One baseline per gripper, named after the validated USD (Robotiq_2F_85.usda ->
    expected/Robotiq_2F_85.json), so each gripper's ratchet moves independently.
    SIMREADY_BASELINE overrides."""
    override = os.environ.get("SIMREADY_BASELINE")
    if override:
        return override
    stem = os.path.splitext(os.path.basename(asset))[0]
    return os.path.join(BASELINE_DIR, stem + ".json")


def _as_list(value):
    """The validator stores requirement lists as a stringified Python list."""
    value = value or []
    if isinstance(value, str):
        try:
            return ast.literal_eval(value)
        except Exception:
            return [value]
    return value


def _feature_reqs(feats):
    """Return {feature: {"blocking": [...], "optional": [...]}} from a results feature map."""
    out = {}
    for name, f in feats.items():
        out[name] = {
            "blocking": sorted(_as_list(f.get("failing requirements"))),
            "optional": sorted(_as_list(f.get("optional requirements"))),
        }
    return out


def _load_results(path):
    with open(path) as fh:
        data = json.load(fh)
    # results.json is keyed by asset path; take the first (CI validates one asset).
    asset, info = next(iter(data.items()))
    return asset, info


def _write_baseline(path):
    asset, info = _load_results(path)
    baseline_path = _baseline_path(asset)
    baseline = {
        "profile_id": info.get("profile_id", "?"),
        "profile_version": info.get("profile_version", "?"),
        "_note": "SimReady feature baseline. Regenerate: python3 ci/simready/summarize.py --update results.json",
        "features": _feature_reqs(info.get("features_summary", {})),
    }
    with open(baseline_path, "w") as fh:
        json.dump(baseline, fh, indent=2, sort_keys=True)
        fh.write("\n")
    print(f"Wrote baseline for {baseline['profile_id']} v{baseline['profile_version']} "
          f"({len(baseline['features'])} features) -> {baseline_path}")
    return 0


def _emit(lines, gh_key=None):
    md = "\n".join(lines)
    print(md)
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary:
        with open(summary, "a") as fh:
            fh.write(md + "\n")


def main() -> int:
    flags = {"--update", "--allow-improvements"}
    args = [a for a in sys.argv[1:] if a not in flags]
    if "--update" in sys.argv:
        return _write_baseline(args[0] if args else "results.json")
    allow_improvements = "--allow-improvements" in sys.argv

    path = args[0] if args else "results.json"
    asset, info = _load_results(path)
    profile = info.get("profile_id", "?")
    version = info.get("profile_version", "?")
    cur = _feature_reqs(info.get("features_summary", {}))

    # Current blocking failures (always fatal).
    blocking = {n: r["blocking"] for n, r in cur.items() if r["blocking"]}

    # Load baseline and diff the optional/blocking sets per requirement.
    baseline = None
    baseline_path = _baseline_path(asset)
    if os.path.exists(baseline_path):
        with open(baseline_path) as fh:
            baseline = json.load(fh)

    regressions = {}   # feature -> [requirement codes newly failing vs baseline]
    improvements = {}  # feature -> [requirement codes now passing vs baseline]
    baseline_usable = bool(baseline) and baseline.get("profile_id") == profile \
        and baseline.get("profile_version") == version

    if baseline_usable:
        bfeats = baseline.get("features", {})
        for name, r in cur.items():
            cur_fail = set(r["blocking"]) | set(r["optional"])
            base = bfeats.get(name, {"blocking": [], "optional": []})
            base_fail = set(base.get("blocking", [])) | set(base.get("optional", []))
            new = sorted(cur_fail - base_fail)
            fixed = sorted(base_fail - cur_fail)
            if new:
                regressions[name] = new
            if fixed:
                improvements[name] = fixed

    # ---- verdict ----
    # Blocking failures and regressions always fail. Improvements also fail (until
    # the baseline is refreshed) so an un-recorded gain can't leave the floor stale
    # and let a later regression slip through undetected -- UNLESS --allow-improvements
    # is set (comparing a different asset, where a gain is expected).
    ok = not blocking and not regressions and (allow_improvements or not improvements)
    n_clean = sum(1 for n, r in cur.items() if not r["blocking"] and not r["optional"])

    lines = [
        f"## {'✅ PASSED' if ok else '❌ FAILED'} — `{profile}` v{version}",
        "",
        f"**Asset:** `{os.path.basename(asset)}`  ",
        f"**Blocking failures:** {len(blocking)}  ",
        f"**Regressions vs baseline:** {len(regressions)}  ",
        f"**Optional gaps:** {sum(1 for r in cur.values() if r['optional'])}  ",
        f"**Features fully satisfied:** {n_clean} / {len(cur)}",
        "",
        "| Feature | Version | Result | Blocking | Optional (non-blocking) |",
        "|---|---|:---:|---|---|",
    ]
    feats = info.get("features_summary", {})
    for name in sorted(cur):
        hard = cur[name]["blocking"]
        soft = cur[name]["optional"]
        if hard:
            mark = "❌"
        elif name in regressions:
            mark = "❌"          # optional regression is fatal
        elif soft:
            mark = "⚠️"
        else:
            mark = "✅"
        hcell = ", ".join(f"`{r}`" for r in hard) if hard else "—"
        scell = ", ".join(f"`{r}`" for r in soft) if soft else "—"
        lines.append(f"| `{name}` | {feats.get(name, {}).get('version', '')} | {mark} | {hcell} | {scell} |")
    lines.append("")
    lines.append("_✅ satisfied · ⚠️ known optional gap (in baseline) · ❌ blocking failure or optional regression_")
    lines.append("")

    if not baseline_usable:
        why = "no baseline file" if not baseline else \
            f"baseline is for {baseline.get('profile_id')} v{baseline.get('profile_version')}"
        lines += [
            f"> ⚠️ Regression check skipped ({why}); gating on blocking failures only. "
            f"Create/refresh it with `python3 ci/simready/summarize.py --update results.json`.",
            "",
        ]

    if regressions:
        lines += ["### ❌ Optional-feature regressions (new failures vs baseline)", ""]
        for name, reqs in sorted(regressions.items()):
            lines.append(f"- `{name}`: " + ", ".join(f"`{r}`" for r in reqs))
        lines += ["", "Fix these, or — if intentional — refresh the baseline "
                  "(`python3 ci/simready/summarize.py --update results.json`) in this PR.", ""]
        for name, reqs in sorted(regressions.items()):
            print(f"::error title=SimReady regression::{name}: {', '.join(reqs)} regressed vs baseline")

    if improvements and allow_improvements:
        lines += ["### ⬆️ Improvements vs baseline (accepted)", ""]
        for name, reqs in sorted(improvements.items()):
            lines.append(f"- `{name}`: now passing " + ", ".join(f"`{r}`" for r in reqs))
        lines += ["", "These pass beyond the baseline. Accepted (`--allow-improvements`) because "
                  "this compares a different asset than the baseline was captured from.", ""]
        for name, reqs in sorted(improvements.items()):
            print(f"::notice title=SimReady improvement::{name}: {', '.join(reqs)} now passes (accepted)")
    elif improvements:
        lines += ["### ⬆️ Improvements vs baseline — refresh the baseline to accept them", ""]
        for name, reqs in sorted(improvements.items()):
            lines.append(f"- `{name}`: now passing " + ", ".join(f"`{r}`" for r in reqs))
        lines += ["", "These pass now but the baseline still records them as failing. Refresh it in "
                  "this PR so the ratchet's floor rises and a later regression is caught:", "",
                  "```", "python3 ci/simready/summarize.py --update results.json", "```", ""]
        for name, reqs in sorted(improvements.items()):
            print(f"::error title=SimReady baseline out of date::{name}: {', '.join(reqs)} now passes; "
                  f"run `python3 ci/simready/summarize.py --update results.json` and commit {os.path.relpath(baseline_path)}")

    _emit(lines)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
