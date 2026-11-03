"""Train and cross-validate the piece recogniser on grasps recorded by ``pick_all.py --record_grasps`` (CPU only).

    python fit_recognizer.py grasps_policy.npz [more.npz ...] --out assets/shape_sorter/recognizer.joblib

Prints the 5-fold cross-validated accuracy and the confusion matrix, then fits on all the data and saves the model."""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import importlib.util  # noqa: E402

spec = importlib.util.spec_from_file_location("recognizer", Path(__file__).resolve().parents[1] / "robotiq_rl" / "shape_pick" / "recognizer.py")
rec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rec)

ap = argparse.ArgumentParser()
ap.add_argument("data", nargs="+")
ap.add_argument("--out", default=None)
ap.add_argument("--min_force", type=float, default=1.0, help="drop recordings whose pads carry less than this [N] in total (not a grasp)")
args = ap.parse_args()

parts = [np.load(f) for f in args.data]
maps = np.concatenate([p["maps"] for p in parts])
closure = np.concatenate([p["closure"] for p in parts])
y = np.concatenate([p["label"] for p in parts])
force = maps[:, -rec.K :].reshape(len(maps), rec.K, -1).sum(-1).mean(-1)
keep = force > args.min_force
maps, closure, y = maps[keep], closure[keep], y[keep]
print(f"{len(y)} grasps (dropped {int((~keep).sum())} with no load), per class {dict(zip(rec.CLASSES, np.bincount(y, minlength=6).tolist()))}")

from sklearn.model_selection import StratifiedKFold  # noqa: E402

pred = np.zeros(len(y), int)
for tr, te in StratifiedKFold(5, shuffle=True, random_state=0).split(maps, y):
    clf = rec.fit(maps[tr], closure[tr], y[tr])
    rng = np.random.default_rng(1)
    noisy = np.expm1(np.log1p(maps[te]) + rng.normal(0, 0.03, maps[te].shape) * (maps[te] > 0)).clip(min=0)
    pred[te] = rec.predict(clf, noisy, closure[te]).argmax(1)
print(f"cross-validated accuracy {np.mean(pred == y):.3f} (chance {1 / 6:.3f})")
cm = np.zeros((6, 6), int)
for a, b in zip(y, pred):
    cm[a, b] += 1
print(" " * 11 + " ".join(f"{n[:6]:>6s}" for n in rec.CLASSES))
for i, name in enumerate(rec.CLASSES):
    print(f"{name:10s} " + " ".join(f"{v:6d}" for v in cm[i]) + f"   recall {cm[i, i] / max(cm[i].sum(), 1):.2f}")
if args.out:
    import joblib

    joblib.dump(rec.fit(maps, closure, y), args.out, compress=3)
    print("saved", args.out, f"{Path(args.out).stat().st_size / 1e6:.1f} MB")
