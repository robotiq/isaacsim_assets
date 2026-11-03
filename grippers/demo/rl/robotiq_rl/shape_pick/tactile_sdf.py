"""A shape-aware tactile map for the shape sorter pieces: the force of a pad on a piece goes to the taxels that touch the piece.

The contact report of a pad gives the normal force on a piece but one averaged contact point, which cannot tell a flat face
from a cylinder or a star. Here the force of every (pad, piece) pair is spread over the taxels by the distance of each taxel to the
surface of the piece (``robotiq_tactile_sensors.spread.spread_forces_by_distance``): the footprint of the piece on the pad.
The distances come from signed-distance grids of the six piece types (``scripts/make_piece_sdfs.py``) and the pose of the piece.

Pure torch (CPU or GPU); no Isaac Lab import, so it can also be used on recorded grasps."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from robotiq_tactile_sensors.sensing import quat_apply
from robotiq_tactile_sensors.spread import spread_forces_by_distance

ASSETS = Path(__file__).resolve().parents[2] / "assets" / "shape_sorter"
PIECES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]  # class order, as shape_pick.env_cfg.PIECES
DELTA = 0.002  # m: depth of the compliant skin
POWER = 1.0


class PieceSdfs:
    """The six signed-distance grids; ``distance`` samples them (trilinear) at points given in the world frame."""

    def __init__(self, device="cpu", assets: Path = ASSETS):
        self.device = device
        self.volumes, self.origins, self.extents = [], [], []
        for k, name in enumerate(PIECES):
            d = np.load(assets / "sdf" / f"piece_{k}_{name}.npz")
            sdf = torch.tensor(d["sdf"].astype(np.float32), device=device)  # [z, y, x]
            pitch = float(d["pitch"])
            self.volumes.append(sdf[None, None])
            self.origins.append(torch.tensor(d["origin"], dtype=torch.float32, device=device))
            self.extents.append(torch.tensor([(sdf.shape[2] - 1) * pitch, (sdf.shape[1] - 1) * pitch, (sdf.shape[0] - 1) * pitch],
                                             dtype=torch.float32, device=device))

    def distance(self, type_ids: torch.Tensor, pos_w: torch.Tensor, quat_w: torch.Tensor, points_w: torch.Tensor) -> torch.Tensor:
        """Distance [m] of points to the surface of a piece (0 on it, positive outside), ``(N, Q)``.

        ``type_ids`` ``(N,)`` piece type, ``pos_w`` ``(N, 3)`` and ``quat_w`` ``(N, 4)`` (x, y, z, w) the pose of the piece,
        ``points_w`` ``(N, Q, 3)`` the points in the world frame."""
        conj = torch.cat([-quat_w[:, :3], quat_w[:, 3:]], dim=-1)
        local = quat_apply(conj[:, None], points_w - pos_w[:, None])  # (N, Q, 3), in the piece frame
        out = torch.empty(points_w.shape[:2], device=points_w.device)
        for t in range(len(PIECES)):
            sel = type_ids == t
            if not bool(sel.any()):
                continue
            pts = local[sel]  # (n, Q, 3)
            norm = 2.0 * (pts - self.origins[t]) / self.extents[t] - 1.0  # (x, y, z) -> (W, H, D) of the grid
            grid = norm.reshape(1, -1, 1, 1, 3)
            val = F.grid_sample(self.volumes[t], grid, mode="bilinear", padding_mode="border", align_corners=True)
            out[sel] = val.reshape(pts.shape[:2])
        return out


def piece_taxel_force(
    sdfs: PieceSdfs,
    slot_types: list[int],
    taxel_w: torch.Tensor,
    force: torch.Tensor,
    pos_w: torch.Tensor,
    quat_w: torch.Tensor,
    delta: float = DELTA,
    power: float = POWER,
) -> torch.Tensor:
    """Force per taxel [N] from the pads' force on the pieces, ``(E, 2, T)``.

    ``taxel_w`` ``(E, 2, T, 3)`` taxel centres, ``force`` ``(E, 2, S)`` normal force of each pad on each piece slot,
    ``pos_w`` ``(E, S, 3)`` and ``quat_w`` ``(E, S, 4)`` the poses of the pieces, ``slot_types`` the type of each slot."""
    E, P, T, _ = taxel_w.shape
    pts = taxel_w.reshape(E, P * T, 3)
    dist = torch.stack(
        [sdfs.distance(torch.full((E,), slot_types[s], device=pts.device), pos_w[:, s], quat_w[:, s], pts).reshape(E, P, T) for s in range(len(slot_types))],
        dim=2,
    )  # (E, 2, S, T)
    return spread_forces_by_distance(dist, force, delta, power)
