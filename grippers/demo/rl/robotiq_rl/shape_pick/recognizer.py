"""Recognising the shape sorter pieces from the tactile maps of a grasp: features, training and prediction (numpy / scikit-learn, no Isaac).

The input is what the supervisor records when the policy holds a piece: the last ``H`` steps of the shape-aware taxel maps
(``(H, 56)`` [N]: left pad 7 x 4, then right pad 7 x 4) and of the gripper closure [rad]. The features are

* the log(1 + N) maps of the last ``K`` steps, averaged (the footprint of the piece on both pads),
* per pad: the total force, the centre of the footprint, its spread along and across the pad, their covariance, the number of taxels
  that carry load, the peak taxel, the entropy of the map and the rows / columns it spans (the shape of the footprint, whatever
  its place on the pad),
* the closure (the width of the piece between the pads) and the pad force ratio.

``fit`` trains a gradient boosting classifier; ``predict`` returns class probabilities.

``fit_model`` adds what is needed to answer "unknown": the raw probabilities of a gradient boosting classifier are overconfident (it says
96 % and is right 73 % of the time), so the probabilities of cross-validated predictions are temperature-scaled, and the confidence
threshold below which the answer is "unknown" is the lowest one for which the cross-validated accuracy of the answered pieces is at
least ``target_accuracy``. The model is a plain dict (pickle-safe): ``decide`` gives the answer, ``-1`` for unknown, and the calibrated
probabilities."""

from __future__ import annotations

import numpy as np

CLASSES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]
K = 8  # steps averaged (all the history the supervisor keeps; measured: 4 -> 8 steps raises the coverage at 85 % accuracy from 48 to 51 %)


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


def _calibrate(proba: np.ndarray, temperature: float) -> np.ndarray:
    logits = np.log(np.clip(proba, 1e-9, 1.0)) / temperature
    logits -= logits.max(axis=1, keepdims=True)
    e = np.exp(logits)
    return e / e.sum(axis=1, keepdims=True)


def fit_model(maps_hist, closure_hist, labels, target_accuracy: float = 0.85, noise: float = 0.03, folds: int = 5, seed: int = 0,
              min_coverage: float = 0.05) -> dict:
    """Train the classifier, calibrate it and choose the "unknown" threshold from cross-validated predictions.

    Returns ``{"clf", "temperature", "threshold", "target_accuracy", "cv_accuracy", "cv_coverage", "cv_accuracy_answered"}``."""
    from scipy.optimize import minimize_scalar
    from sklearn.model_selection import StratifiedKFold

    maps = np.asarray(maps_hist, dtype=np.float64)
    closure = np.asarray(closure_hist, dtype=np.float64)
    y = np.asarray(labels)
    rng = np.random.default_rng(seed + 1)
    oof = np.zeros((len(y), 6))
    for tr, te in StratifiedKFold(folds, shuffle=True, random_state=seed).split(maps, y):
        clf = fit(maps[tr], closure[tr], y[tr], noise=noise, seed=seed)
        noisy = np.expm1(np.log1p(maps[te]) + rng.normal(0, noise, maps[te].shape) * (maps[te] > 0)).clip(min=0)
        oof[te] = predict(clf, noisy, closure[te])
    nll = lambda logt: -np.mean(np.log(np.clip(_calibrate(oof, np.exp(logt))[np.arange(len(y)), y], 1e-9, 1.0)))  # noqa: E731
    temperature = float(np.exp(minimize_scalar(nll, bounds=(np.log(0.3), np.log(30.0)), method="bounded").x))
    cal = _calibrate(oof, temperature)
    conf, ok = cal.max(axis=1), cal.argmax(axis=1) == y
    order = np.argsort(-conf)
    answered = np.arange(1, len(y) + 1)
    acc_at = np.cumsum(ok[order]) / answered
    good = np.where((acc_at >= target_accuracy) & (answered >= min_coverage * len(y)))[0]
    k = int(good.max()) if len(good) else int(min_coverage * len(y))  # the most answers that still meet the target
    threshold = float(conf[order][k])
    return {
        "clf": fit(maps, closure, y, noise=noise, seed=seed),
        "temperature": temperature,
        "threshold": threshold,
        "target_accuracy": target_accuracy,
        "cv_accuracy": float(ok.mean()),
        "cv_coverage": float((conf >= threshold).mean()),
        "cv_accuracy_answered": float(ok[conf >= threshold].mean()),
    }


def decide(model, maps_hist, closure_hist):
    """``(answer, proba)``: ``answer`` ``(N,)`` the class index, ``-1`` where the model is not confident enough; ``proba`` ``(N, 6)`` calibrated.

    ``model`` is the dict of ``fit_model`` (a bare classifier of ``fit`` is accepted, then there is no "unknown")."""
    if not isinstance(model, dict):
        proba = predict(model, maps_hist, closure_hist)
        return proba.argmax(1), proba
    proba = _calibrate(predict(model["clf"], maps_hist, closure_hist), model["temperature"])
    answer = np.where(proba.max(axis=1) >= model["threshold"], proba.argmax(axis=1), -1)
    return answer, proba
