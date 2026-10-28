"""Effective contact response of a taxel array (Shang et al., arXiv 2610.10510, Eq. 1).

From one pad's total normal force ``F`` and contact patch ``A`` (uniform pressure ``P = F / |A|``),
taxel ``j`` responds with

    r_j = [ 1/|A_j| * integral over A_j of (K_sigma * P^eta) dx ] ^ (gamma / eta)

``K_sigma`` is an anisotropic Gaussian, ``eta`` the pointwise nonlinearity, ``gamma`` the overall
force scaling; the paper identifies sigma = (1.5, 1.25) mm, eta = 0.7, gamma = 0.5. ``r`` is a
phenomenological response, not a depth, pressure or strain; a real sensor reads ``beta_j * r_j``
with a per-taxel gain that is not modelled here. ``P`` is in N / mm^2, which fixes the unit of ``r``;
over a uniform patch ``r = P ** gamma``.

Also here: ``patch_force`` (the force each taxel carries: the pressure over its share of the patch)
and ``response_from_taxel_pressure`` (Eq. 1 of a pressure known per taxel, for the raycast sensor).
Pure numpy / scipy, no Isaac. Points are in the pad plane, mm: u across the columns, v along the rows.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.sparse import csr_matrix
from scipy.spatial import ConvexHull

try:
    from scipy.spatial import QhullError
except ImportError:  # scipy < 1.8
    from scipy.spatial.qhull import QhullError

SIGMA_MM = (1.5, 1.25)  # (along u = columns, along v = rows)
ETA = 0.7
GAMMA = 0.5
RES_MM = 0.25  # raster resolution of the pressure field


def pad_plane(points: np.ndarray):
    """Pad-plane frame of a ``(ROWS, COLS, 3)`` taxel grid: ``(origin, u, v, taxel_uv_mm, pitch_mm)``.

    ``u`` (columns) and ``v`` (rows) are unit axes, ``taxel_uv_mm`` is ``(ROWS, COLS, 2)`` and
    ``pitch_mm`` the taxel spacing along (u, v)."""
    o = points.reshape(-1, 3).mean(axis=0)
    u = (points[:, -1] - points[:, 0]).mean(axis=0)
    v = (points[-1] - points[0]).mean(axis=0)
    u /= np.linalg.norm(u)
    v -= u * np.dot(u, v)
    v /= np.linalg.norm(v)
    uv = np.stack([(points - o) @ u, (points - o) @ v], axis=-1) * 1000.0
    pitch = (float(np.abs(np.diff(uv[..., 0], axis=1)).mean()), float(np.abs(np.diff(uv[..., 1], axis=0)).mean()))
    return o, u, v, uv, pitch


def convex_hull(points: np.ndarray) -> np.ndarray:
    """Counter-clockwise hull vertices of 2-D points; fewer than 3 distinct points are returned as they are."""
    pts = np.unique(np.round(np.asarray(points, dtype=float).reshape(-1, 2), 9), axis=0)
    if len(pts) < 3:
        return pts
    try:
        return pts[ConvexHull(pts).vertices]
    except QhullError:  # collinear
        return pts


def _raster(taxel_uv_mm, extra_uv_mm, pitch_mm, sigma_mm, res_mm):
    """Cell-centre coordinates ``(u, v)`` (1-D) of a raster covering the taxels, ``extra_uv_mm`` and the kernel's reach."""
    margin = 4.0 * max(sigma_mm) + max(pitch_mm)
    pts = np.concatenate([taxel_uv_mm.reshape(-1, 2), extra_uv_mm.reshape(-1, 2)])
    lo, hi = pts.min(axis=0) - margin, pts.max(axis=0) + margin
    nu, nv = (int(np.ceil(x)) + 1 for x in (hi - lo) / res_mm)
    return lo[0] + (np.arange(nu) + 0.5) * res_mm, lo[1] + (np.arange(nv) + 0.5) * res_mm


def _patch_mask(contact_uv_mm, u, v, res_mm):
    """Raster cells inside the hull of the contact points, plus the cell under each point (so a point or
    line patch still has an area)."""
    mask = np.zeros((len(u), len(v)), dtype=bool)
    pts = np.unique(np.round(contact_uv_mm, 9), axis=0)
    if len(pts) >= 3:
        try:
            hull = ConvexHull(pts)
        except QhullError:  # collinear
            hull = None
        if hull is not None:
            mask[:] = True
            for a, b, c in hull.equations:  # a*u + b*v + c <= 0 inside
                mask &= ((a * u)[:, None] + (b * v + c)[None, :]) <= 1e-9
    iu = np.clip(((contact_uv_mm[:, 0] - u[0]) / res_mm + 0.5).astype(int), 0, len(u) - 1)
    iv = np.clip(((contact_uv_mm[:, 1] - v[0]) / res_mm + 0.5).astype(int), 0, len(v) - 1)
    mask[iu, iv] = True
    return mask


def _span(center, pitch, axis, res_mm):
    """Slice of the raster cells whose centres lie in ``[center - pitch/2, center + pitch/2)`` (half-open, so
    neighbouring taxels tile without overlap)."""
    lo = int(np.ceil((center - pitch / 2.0 - axis[0]) / res_mm))
    hi = int(np.ceil((center + pitch / 2.0 - axis[0]) / res_mm))
    return slice(max(lo, 0), max(min(hi, len(axis)), 0))


def _taxel_slices(taxel_uv_mm, pitch_mm, u, v, res_mm):
    rows, cols = taxel_uv_mm.shape[:2]
    return [[(_span(taxel_uv_mm[r, c, 0], pitch_mm[0], u, res_mm), _span(taxel_uv_mm[r, c, 1], pitch_mm[1], v, res_mm))
             for c in range(cols)] for r in range(rows)]


class _Operator:
    """Eq. 1 for one taxel grid as a matrix: the raster, each taxel's cells and ``weights``, whose row ``j`` is the
    Gaussian blur followed by the average over taxel ``j`` (the blur kernel is symmetric, so it is the blur of the
    taxel's indicator over its cell count). A read is then one matrix-vector product."""

    def __init__(self, taxel_uv_mm, extra_uv_mm, pitch_mm, sigma_mm, res_mm):
        self.u, self.v = _raster(taxel_uv_mm, extra_uv_mm, pitch_mm, sigma_mm, res_mm)
        self.shape = taxel_uv_mm.shape[:2]
        self.slices = [[(_span(taxel_uv_mm[r, c, 0], pitch_mm[0], self.u, res_mm),
                         _span(taxel_uv_mm[r, c, 1], pitch_mm[1], self.v, res_mm))
                        for c in range(self.shape[1])] for r in range(self.shape[0])]
        grid = (len(self.u), len(self.v))
        weights = np.zeros((self.shape[0] * self.shape[1], grid[0] * grid[1]))
        for r, row in enumerate(self.slices):
            for c, (su, sv) in enumerate(row):
                ind = np.zeros(grid)
                ind[su, sv] = 1.0
                if ind.any():
                    blurred = gaussian_filter(ind, sigma=[s / res_mm for s in sigma_mm], mode="constant", truncate=4.0)
                    weights[r * self.shape[1] + c] = blurred.ravel() / ind.sum()
        self.weights = csr_matrix(weights)  # each row reaches only 4 sigma past its taxel

    def contains(self, uv_mm, res_mm) -> bool:
        return bool((uv_mm[:, 0] >= self.u[0] - res_mm / 2).all() and (uv_mm[:, 0] <= self.u[-1] + res_mm / 2).all()
                    and (uv_mm[:, 1] >= self.v[0] - res_mm / 2).all() and (uv_mm[:, 1] <= self.v[-1] + res_mm / 2).all())

    def response(self, field, eta, gamma):
        """Eq. 1 of a pressure field on the raster: ``(weights @ field ** eta) ** (gamma / eta)``, ``(R, C)``."""
        return ((self.weights @ (field ** eta).ravel()) ** (gamma / eta)).reshape(self.shape)


_operators: dict = {}


def _operator(taxel_uv_mm, contact_uv_mm, pitch_mm, sigma_mm, res_mm) -> _Operator:
    """The operator of the grid, built once; one for the contacts' own raster when they reach past the grid's."""
    uv = np.round(taxel_uv_mm, 4)  # constant to float precision, so the key survives frame to frame
    key = (uv.tobytes(), uv.shape, tuple(np.round(pitch_mm, 4)), tuple(sigma_mm), res_mm)
    if key not in _operators:
        _operators[key] = _Operator(uv, uv, pitch_mm, sigma_mm, res_mm)
    op = _operators[key]
    if contact_uv_mm is None or op.contains(contact_uv_mm, res_mm):
        return op
    return _Operator(uv, contact_uv_mm, pitch_mm, sigma_mm, res_mm)


def pad_maps(contact_uv_mm, force_n, taxel_uv_mm, pitch_mm, sigma_mm=SIGMA_MM, eta=ETA, gamma=GAMMA,
             res_mm=RES_MM, response=True):
    """``(force, response)`` of one pad, each ``(R, C)`` (``response`` is None when not asked for).

    ``contact_uv_mm``: ``(N, 2)`` contact points; their convex hull is the contact patch ``A``.
    ``force_n``: the pad's total normal force. ``taxel_uv_mm``: ``(R, C, 2)`` taxel centres, each taxel's
    area ``A_j`` a ``pitch_mm`` rectangle around its centre. ``force`` is the uniform pressure ``F / |A|``
    over the patch inside each taxel (N); it sums to ``F`` when the patch lies on the grid."""
    shape = taxel_uv_mm.shape[:2]
    contact_uv_mm = np.asarray(contact_uv_mm, dtype=float).reshape(-1, 2)
    if len(contact_uv_mm) == 0 or force_n <= 0.0:
        return np.zeros(shape), (np.zeros(shape) if response else None)
    op = _operator(taxel_uv_mm, contact_uv_mm, pitch_mm, sigma_mm, res_mm)
    mask = _patch_mask(contact_uv_mm, op.u, op.v, res_mm)
    pressure = force_n / (mask.sum() * res_mm ** 2) * mask  # N / mm^2
    force = np.array([[pressure[su, sv].sum() * res_mm ** 2 for su, sv in row] for row in op.slices])
    return force, (op.response(pressure, eta, gamma) if response else None)


def effective_response(contact_uv_mm, force_n, taxel_uv_mm, pitch_mm, sigma_mm=SIGMA_MM, eta=ETA, gamma=GAMMA,
                       res_mm=RES_MM) -> np.ndarray:
    """Per-taxel response ``r`` of one pad, ``(R, C)``; zeros without contact or force."""
    return pad_maps(contact_uv_mm, force_n, taxel_uv_mm, pitch_mm, sigma_mm, eta, gamma, res_mm)[1]


def patch_force(contact_uv_mm, force_n, taxel_uv_mm, pitch_mm, res_mm=RES_MM) -> np.ndarray:
    """Force (N) each taxel carries of the pad's contact patch, ``(R, C)``."""
    return pad_maps(contact_uv_mm, force_n, taxel_uv_mm, pitch_mm, res_mm=res_mm, response=False)[0]


def response_from_taxel_pressure(pressure_n_mm2, taxel_uv_mm, pitch_mm, sigma_mm=SIGMA_MM, eta=ETA, gamma=GAMMA,
                                 res_mm=RES_MM) -> np.ndarray:
    """Eq. 1 of a pressure known per taxel, ``(R, C)`` in N / mm^2, uniform over each taxel's rectangle."""
    if not np.any(pressure_n_mm2 > 0.0):
        return np.zeros(taxel_uv_mm.shape[:2])
    op = _operator(taxel_uv_mm, None, pitch_mm, sigma_mm, res_mm)
    field = np.zeros((len(op.u), len(op.v)))
    for r, row in enumerate(op.slices):
        for c, (su, sv) in enumerate(row):
            field[su, sv] = pressure_n_mm2[r, c]
    return op.response(field, eta, gamma)
