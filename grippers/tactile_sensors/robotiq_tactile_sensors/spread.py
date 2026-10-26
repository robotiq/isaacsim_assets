"""Spreading the contact force of an object over the taxels of a pad (torch).

A contact report gives, per pad and touched object, the normal force and the average contact point. A taxel array reads
a force map. This turns the first into the second: each object's force is spread over the taxels around its contact
point with an isotropic Gaussian and normalised, so the taxels of a pad sum to the force the pad carries. Several
objects add up.

This is a model, not a calibration: the Gaussian width ``sigma`` is a free parameter (6 mm, about a taxel pitch), the pad
is rigid (no compliance, no shear) and one object gives one averaged contact point per pad. It is cheap enough for
thousands of parallel environments, which the repo's Isaac Sim reader (``Robotiq_2F_85/tactile``) is not.
"""

from __future__ import annotations

import torch


def spread_forces(
    taxel_w: torch.Tensor, contact_pos_w: torch.Tensor, force: torch.Tensor, sigma: float = 0.006
) -> torch.Tensor:
    """Force per taxel.

    Args:
        taxel_w: taxel centres in the world frame, ``(E, P, T, 3)`` (E envs, P pads, T taxels).
        contact_pos_w: average contact point per touched object, ``(E, P, F, 3)`` (F objects); ignored where force is 0.
        force: normal force [N] per touched object, ``(E, P, F)``.
        sigma: Gaussian width [m].

    Returns:
        ``(E, P, T)`` force [N] per taxel; the taxels of a pad sum to the sum of its objects' forces.
    """
    d2 = ((taxel_w[:, :, :, None] - contact_pos_w[:, :, None]) ** 2).sum(-1)  # (E, P, T, F)
    w = torch.exp(-d2 / (2 * sigma**2))
    w = w / (w.sum(dim=2, keepdim=True) + 1e-9)  # each object's force is shared out: the weights sum to 1 over taxels
    return (w * force[:, :, None]).sum(-1)
