"""How well can the shape sorter pieces be told apart from the tactile maps of a grasp? (CPU only, no Isaac)

    python train_recognizer.py grasps.npz [--noise 0.03]

``grasps.npz`` is the output of ``collect_grasps.py``. For several feature sets (gripper closure only, pad force only, the
taxel maps, maps + closure) a classifier is cross-validated (5 folds, stratified) and its accuracy and confusion matrix are
printed. The taxel maps are log(1 + N), as the policy sees them, with Gaussian noise of ``--noise`` added to every taxel
(the observation noise of the task), so the numbers are not flattered by a noise-free sensor. Also saved: the mean map of each
class (``maps_per_class.png``)."""

import argparse

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

NAMES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]

ap = argparse.ArgumentParser()
ap.add_argument("data")
ap.add_argument("--noise", type=float, default=0.03)
ap.add_argument("--last", type=int, default=3, help="index of the snapshot used (0, 1 while closing; 2, 3 while lifting)")
ap.add_argument("--plot", default="maps_per_class.png")
ap.add_argument("--sdf", action="store_true", help="also use the shape-aware maps (needs the geometry fields of collect_grasps.py)")
ap.add_argument("--deltas", type=float, nargs="+", default=[0.001, 0.002, 0.003, 0.005], help="skin depths [m] of the shape-aware model")
ap.add_argument("--powers", type=float, nargs="+", default=[1.0, 2.0])
args = ap.parse_args()

d = np.load(args.data)
y = d["label"]
rng = np.random.default_rng(0)
maps = np.log1p(d["maps"])  # (N, T, 2, 7, 4)
maps = maps + rng.normal(0, args.noise, maps.shape) * (d["maps"] > 0)  # the noise is on the sensor output of a taxel that reads
closure = d["closure"]
pad_force = np.log1p(d["pad_force"])
t = args.last
print(f"{len(y)} grasps, per class {dict(zip(NAMES, np.bincount(y, minlength=6).tolist()))}")

def sdf_maps(delta, power):
    """Shape-aware maps ``(N, 4, 2, 7, 4)`` [N] of the recorded grasps, from the geometry (pad force on the piece, piece pose, taxels)."""
    import importlib.util
    import sys
    from pathlib import Path

    import torch

    here = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(here.parents[1] / "tactile_sensors"))
    spec = importlib.util.spec_from_file_location("tactile_sdf", here / "robotiq_rl" / "shape_pick" / "tactile_sdf.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from robotiq_tactile_sensors.spread import spread_forces_by_distance

    n, T = d["label"].shape[0], d["taxel_w"].shape[1]
    sdfs = mod.PieceSdfs()
    types = torch.tensor(np.repeat(d["label"], T))
    pts = torch.tensor(d["taxel_w"]).reshape(n * T, 56, 3)
    dist = sdfs.distance(types, torch.tensor(d["piece_pos_t"]).reshape(-1, 3), torch.tensor(d["piece_quat_t"]).reshape(-1, 4), pts)
    dist = dist.reshape(n * T, 2, 1, 28)
    force = torch.tensor(d["force_piece"]).reshape(n * T, 2, 1)
    return spread_forces_by_distance(dist, force, delta, power).reshape(n, T, 2, 7, 4).numpy()


features = {
    "closure only": closure[:, t : t + 1],
    "pad force only": pad_force[:, t].reshape(len(y), -1),
    "closure + pad force": np.concatenate([closure[:, t : t + 1], pad_force[:, t].reshape(len(y), -1)], 1),
    "taxel maps": maps[:, t].reshape(len(y), -1),
    "maps + closure": np.concatenate([maps[:, t].reshape(len(y), -1), closure[:, t : t + 1]], 1),
    "maps of all 4 instants + closure": np.concatenate([maps.reshape(len(y), -1), closure], 1),
}
if args.sdf:
    for delta in args.deltas:
        for power in args.powers:
            m = np.log1p(sdf_maps(delta, power))
            m = m + rng.normal(0, args.noise, m.shape) * (m > 0)
            tag = f"shape-aware d={delta * 1000:g} mm p={power:g}"
            features[f"{tag}: maps"] = m[:, t].reshape(len(y), -1)
            features[f"{tag}: maps + closure"] = np.concatenate([m[:, t].reshape(len(y), -1), closure[:, t : t + 1]], 1)
models = {
    "logistic": lambda: make_pipeline(StandardScaler(), LogisticRegression(max_iter=2000, C=1.0)),
    "random forest": lambda: RandomForestClassifier(300, random_state=0, n_jobs=-1),
    "MLP": lambda: make_pipeline(StandardScaler(), MLPClassifier((128, 64), max_iter=600, random_state=0, alpha=1e-3)),
}
cv = StratifiedKFold(5, shuffle=True, random_state=0)
best = (0.0, None, None)
print(f"\n{'features':52s} " + " ".join(f"{m:>14s}" for m in models))
for fname, X in features.items():
    row = []
    for mname, make in models.items():
        pred = cross_val_predict(make(), X, y, cv=cv)
        acc = float((pred == y).mean())
        row.append(acc)
        if acc > best[0]:
            best = (acc, f"{fname} / {mname}", pred)
    print(f"{fname:52s} " + " ".join(f"{a:14.3f}" for a in row))
print(f"(chance: {1 / 6:.3f})\n\nbest: {best[1]}, accuracy {best[0]:.3f}; confusion matrix (rows: true, columns: predicted)")
cm = np.zeros((6, 6), int)
for a, b in zip(y, best[2]):
    cm[a, b] += 1
print(" " * 11 + " ".join(f"{n[:6]:>6s}" for n in NAMES))
for i, n in enumerate(NAMES):
    print(f"{n:10s} " + " ".join(f"{v:6d}" for v in cm[i]) + f"   recall {cm[i, i] / max(cm[i].sum(), 1):.2f}")

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 6, figsize=(13, 5.2))
    raw = d["maps"][:, t]
    for c in range(6):
        for side in range(2):
            ax = axs[side, c]
            ax.imshow(np.log1p(raw[y == c, side].mean(0)), cmap="inferno", vmin=0, vmax=np.log1p(raw[:, side].mean(0).max() * 3))
            ax.set_xticks([])
            ax.set_yticks([])
            if side == 0:
                ax.set_title(NAMES[c])
        axs[0, 0].set_ylabel("left pad")
        axs[1, 0].set_ylabel("right pad")
    plt.suptitle("mean taxel map of a lifted grasp, per piece (log scale, base of the pad at the top)")
    plt.tight_layout()
    plt.savefig(args.plot, dpi=80)
    print("saved", args.plot)
except Exception as e:  # noqa: BLE001
    print("no plot:", e)
