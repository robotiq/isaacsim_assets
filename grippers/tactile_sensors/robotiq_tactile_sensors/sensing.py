"""The sensor logic of the Isaac Lab tactile sensor as pure torch functions (no Isaac Lab), so it can be unit-tested.

``taxel_force_map`` is what ``isaaclab.TactileSensor.taxel_force`` computes from the contact sensor's buffers:

1. ``pad_object_forces``: the normal force of each pad on each filtered object, the pads put in (left, right) order, small forces
   zeroed;
2. ``taxel_positions_world``: the taxel centres of the two pads in the world frame, from the poses of the pad bodies;
3. ``spread.spread_forces``: the forces spread over the taxels.
"""

from __future__ import annotations

import torch

from .spread import spread_forces


def quat_apply(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Rotate ``vec`` ``(..., 3)`` by the unit quaternion ``quat`` ``(..., 4)``, stored (x, y, z, w) as in Isaac Lab 3."""
    xyz, w = quat[..., :3], quat[..., 3:]
    t = 2.0 * torch.cross(xyz, vec, dim=-1)
    return vec + w * t + torch.cross(xyz, t, dim=-1)


def pad_object_forces(normal_force_matrix_w: torch.Tensor, pad_order: torch.Tensor, force_floor: float) -> torch.Tensor:
    """Normal force [N] of each pad on each filtered object, ``(E, 2, F)``.

    Args:
        normal_force_matrix_w: the contact sensor's force of each sensing body on each filter object, ``(E, S, F, 3)``.
        pad_order: ``(2,)`` index of the left and right pad among the S sensing bodies (the sensor lists them in its own order).
        force_floor: forces at or below this [N] read 0.
    """
    force = torch.linalg.norm(normal_force_matrix_w, dim=-1)[:, pad_order]
    return torch.where(force > force_floor, force, torch.zeros_like(force))


def taxel_positions_world(
    pos_w: torch.Tensor, quat_w: torch.Tensor, taxels_local: torch.Tensor, pad_order: torch.Tensor
) -> torch.Tensor:
    """Taxel centres in the world frame, ``(E, 2, T, 3)``.

    Args:
        pos_w, quat_w: pose of each sensing body, ``(E, S, 3)`` and ``(E, S, 4)`` (x, y, z, w).
        taxels_local: ``(2, T, 3)`` taxel centres in the frame of the (left, right) pad body.
        pad_order: as in ``pad_object_forces``.
    """
    pos, quat = pos_w[:, pad_order], quat_w[:, pad_order]  # (E, 2, 3), (E, 2, 4)
    return pos[:, :, None] + quat_apply(quat[:, :, None], taxels_local[None])


def taxel_force_map(
    pos_w: torch.Tensor,
    quat_w: torch.Tensor,
    normal_force_matrix_w: torch.Tensor,
    contact_pos_w: torch.Tensor,
    taxels_local: torch.Tensor,
    pad_order: torch.Tensor,
    force_floor: float = 0.05,
    sigma: float = 0.006,
    max_distance: float | None = 0.008,
) -> torch.Tensor:
    """Force per taxel [N], ``(E, 2, T)``, from the contact sensor's buffers.

    ``contact_pos_w``: average contact point of each sensing body with each filter object, ``(E, S, F, 3)``; it may hold NaN
    where there is no contact. The rest as in ``pad_object_forces`` / ``taxel_positions_world``."""
    force = pad_object_forces(normal_force_matrix_w, pad_order, force_floor)
    contact = torch.nan_to_num(contact_pos_w)[:, pad_order]
    taxel_w = taxel_positions_world(pos_w, quat_w, taxels_local, pad_order)
    return spread_forces(taxel_w, contact, force, sigma, max_distance)
