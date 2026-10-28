"""Observation helper for the Isaac Lab tactile sensor."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:  # importing isaaclab.envs pulls in pxr, which must not happen before Isaac Sim starts
    from isaaclab.envs import ManagerBasedEnv


def tactile_map(env: ManagerBasedEnv, sensor_cfg: SceneEntityCfg = SceneEntityCfg("tactile"), log_scale: bool = True) -> torch.Tensor:
    """Force per taxel of both pads, flattened: ``(num_envs, 2 * rows * cols)``.

    ``log_scale`` gives log(1 + N), which keeps a light touch visible next to a firm grip; pass ``False`` for newtons."""
    taxel = env.scene[sensor_cfg.name].taxel_force
    taxel = torch.log1p(taxel) if log_scale else taxel
    return taxel.reshape(taxel.shape[0], -1)
