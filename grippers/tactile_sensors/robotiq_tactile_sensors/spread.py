"""Spreading the contact force of an object over the taxels of a pad (torch).

A contact report gives, per pad and touched object, the normal force and the average contact point. A taxel array reads
a force map. This turns the first into the second: each object's force is spread over the taxels around its contact
point with an isotropic Gaussian and normalised, so the taxels of a pad sum to the force the pad carries. Several
objects add up. A contact farther than ``max_distance`` from every taxel is not on the sensing area (the back of the finger, the
rest of the case): its force is dropped instead of being reported on the nearest taxels.

This is a model, not a calibration: the Gaussian width ``sigma`` is a free parameter (6 mm, about a taxel pitch), the pad
is rigid (no compliance, no shear) and one object gives one averaged contact point per pad. It is cheap enough for
thousands of parallel environments, which the repo's Isaac Sim reader (grippers/tactile/) is not.
"""

from __future__ import annotations

import torch


def spread_forces(
    taxel_w: torch.Tensor,
    contact_pos_w: torch.Tensor,
    force: torch.Tensor,
    sigma: float = 0.006,
    max_distance: float | None = None,
) -> torch.Tensor:
    """Force per taxel.

    Args:
        taxel_w: taxel centres in the world frame, ``(E, P, T, 3)`` (E envs, P pads, T taxels).
        contact_pos_w: average contact point per touched object, ``(E, P, F, 3)`` (F objects); ignored where force is 0.
        force: normal force [N] per touched object, ``(E, P, F)``.
        sigma: Gaussian width [m].
        max_distance: a contact farther than this [m] from every taxel centre (in 3D: off the array, or behind the
            sensing plane) adds nothing. ``None`` keeps every contact.

    Returns:
        ``(E, P, T)`` force [N] per taxel; the taxels of a pad sum to the sum of its objects' forces.
    """
    d2 = ((taxel_w[:, :, :, None] - contact_pos_w[:, :, None]) ** 2).sum(-1)  # (E, P, T, F)
    if max_distance is not None:
        on_array = d2.min(dim=2).values <= max_distance**2  # (E, P, F): the contact is near some taxel
        force = torch.where(on_array, force, torch.zeros_like(force))
    w = torch.exp(-d2 / (2 * sigma**2))
    w = w / (w.sum(dim=2, keepdim=True) + 1e-9)  # each object's force is shared out: the weights sum to 1 over taxels
    return (w * force[:, :, None]).sum(-1)


def spread_forces_by_distance(distance: torch.Tensor, force: torch.Tensor, delta: float = 0.002, power: float = 1.0) -> torch.Tensor:
    """Force per taxel when the shape of the touched object is known: the footprint of the object on the pad.

    A contact report gives one force per pad and object but not where on the pad it is carried. With the object's surface
    known, the force goes to the taxels that touch it: those within ``delta`` of the nearest taxel's distance to the surface.
    The pad is modelled as a thin compliant skin of that depth: a flat face loads every taxel it covers, a cylinder only a strip
    along its line of contact, the tip of a star a few taxels. (Distances are compared with the nearest taxel's, so a constant
    offset between the taxel plane and the surface of the pad does not matter.)

    Args:
        distance: distance [m] from each taxel centre to the surface of each object (0 on it, positive outside),
            ``(E, P, F, T)``.
        force: normal force [N] of each pad on each object, ``(E, P, F)``; objects with no force add nothing.
        delta: depth [m] of the compliant skin.
        power: exponent of the weight ``(1 - (d - d_min) / delta) ** power``.

    Returns:
        ``(E, P, T)`` force [N] per taxel; the taxels of a pad sum to the sum of its objects' forces.
    """
    nearest = distance.min(dim=-1, keepdim=True).values
    w = (1.0 - (distance - nearest) / delta).clamp(min=0.0) ** power  # (E, P, F, T), 1 at the nearest taxel
    w = w / (w.sum(dim=-1, keepdim=True) + 1e-9)
    return (w * force[..., None]).sum(dim=2)
