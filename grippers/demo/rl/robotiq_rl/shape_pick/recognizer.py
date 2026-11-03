"""Recognising the shape sorter pieces from the tactile maps of a grasp: features, training and prediction (numpy / scikit-learn, no Isaac).

The input is what the supervisor records when the policy holds a piece: the last ``H`` steps of the shape-aware taxel maps
(``(H, 56)`` [N]: left pad 7 x 4, then right pad 7 x 4) and of the gripper closure [rad]. The features are

* the log(1 + N) maps of the last ``K`` steps, averaged (the footprint of the piece on both pads),
* per pad: the total force, the centre of the footprint, its spread along and across the pad, their covariance, the number of taxels
  that carry load, the peak taxel, the entropy of the map and the rows / columns it spans (the shape of the footprint, whatever
  its place on the pad),
* the closure (the width of the piece between the pads) and the pad force ratio.

``fit`` trains a gradient boosting classifier; ``predict`` returns class probabilities."""

from __future__ import annotations

import numpy as np

CLASSES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]
K = 4  # steps averaged


def _invariants(m: np.ndarray) -> np.ndarray:
    """``m`` ``(N, 2, 7, 4)`` raw [N] -> ``(N, 22)`` footprint descriptors of both pads."""
    rows = np.arange(7)[None, None, :, None]
    cols = np.arange(4)[None, None, None, :]
    tot = m.sum((-1, -2)) + 1e-6
    cr = (m * rows).sum((-1, -2)) / tot
    cc = (m * cols).sum((-1, -2)) / tot
    vr = (m * (rows - cr[..., None, None]) ** 2).sum((-1, -2)) / tot
    vc = (m * (cols - cc[..., None, None]) ** 2).sum((-1, -2)) / tot
    cov = (m * (rows - cr[..., None, None]) * (cols - cc[..., None, None])).sum((-1, -2)) / tot
    area = (m > 0.05 * m.max((-1, -2), keepdims=True)).sum((-1, -2))
    peak = m.max((-1, -2))
    p = m / tot[..., None, None]
    ent = -(p * np.log(p + 1e-9)).sum((-1, -2))
    rowspan = (m.sum(-1) > 0.05 * tot[..., None]).sum(-1)
    colspan = (m.sum(-2) > 0.05 * tot[..., None]).sum(-1)
    feats = [np.log1p(tot), cr, cc, vr, vc, cov, area, np.log1p(peak), ent, rowspan, colspan]
    return np.concatenate(feats, axis=-1)  # each (N, 2) -> (N, 22)


def features(maps_hist: np.ndarray, closure_hist: np.ndarray) -> np.ndarray:
    """``maps_hist`` ``(N, H, 56)`` [N] raw, ``closure_hist`` ``(N, H)`` [rad] -> ``(N, F)``."""
    m = maps_hist[:, -K:].reshape(len(maps_hist), K, 2, 7, 4)
    mean = m.mean(1)  # (N, 2, 7, 4)
    inv = _invariants(mean)
    closure = closure_hist[:, -K:].mean(1, keepdims=True)
    tot = mean.sum((-1, -2))
    ratio = (tot[:, :1] + 1e-6) / (tot[:, 1:] + 1e-6)
    return np.concatenate([np.log1p(mean).reshape(len(mean), -1), inv, closure, np.log(ratio)], axis=1)


def fit(maps_hist, closure_hist, labels, noise: float = 0.03, seed: int = 0):
    """Train on recorded grasps; the maps get the tactile observation noise (Gaussian on log(1 + N) of the loaded taxels)."""
    from sklearn.ensemble import HistGradientBoostingClassifier

    rng = np.random.default_rng(seed)
    m = np.asarray(maps_hist, dtype=np.float64)
    noisy = np.expm1(np.log1p(m) + rng.normal(0, noise, m.shape) * (m > 0)).clip(min=0)
    clf = HistGradientBoostingClassifier(max_iter=300, learning_rate=0.08, random_state=seed)
    clf.fit(features(noisy, np.asarray(closure_hist, dtype=np.float64)), labels)
    return clf


def predict(clf, maps_hist, closure_hist) -> np.ndarray:
    """Class probabilities ``(N, 6)``."""
    return clf.predict_proba(features(np.asarray(maps_hist, dtype=np.float64), np.asarray(closure_hist, dtype=np.float64)))
