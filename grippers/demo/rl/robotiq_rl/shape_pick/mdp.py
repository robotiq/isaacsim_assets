"""Reset of the shape pick: a random subset of the piece slots goes into the box, each dropped with a random orientation."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from ..tactile_pick import mdp as tp

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_pieces_in_box(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    slot_types: list[int],
    box_center: tuple[float, float],
    half_xy: tuple[float, float],
    floor_z: float,
    radius: float,
    zone_half_xy: tuple[float, float] | None = None,
    p_zone_only: float = 0.0,
    num_active_range: tuple[int, int] = (4, 8),
    min_gap: float = 0.058,
):
    """Put ``k`` pieces (``k`` drawn from ``num_active_range``) at random spots of the box, each with a random orientation.

    ``slot_types[i]`` is the piece type of slot ``i`` (the slots are the scene's ``sphere_i`` objects). The slots are drawn
    with weight 1 / (slots of their type), so every type is equally likely to be in the box. The other parameters go to
    ``tactile_pick.mdp.reset_spheres_in_box`` (box centre, half extents, floor height, ...)."""
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    n, dev = len(env_ids), env.device
    types = torch.tensor(slot_types, device=dev)
    counts = torch.stack([(types == t).sum() for t in types]).float()  # slots of each slot's type
    weights = (1.0 / counts).repeat(n, 1)
    order = torch.multinomial(weights, len(slot_types), replacement=False)  # (n, slots): a random order, light types first
    k = torch.randint(num_active_range[0], num_active_range[1] + 1, (n,), device=dev)
    mask = torch.zeros(n, len(slot_types), dtype=torch.bool, device=dev)
    for j in range(len(slot_types)):
        mask[torch.arange(n, device=dev), order[:, j]] = j < k
    tp.reset_spheres_in_box(
        env,
        env_ids,
        box_center=box_center,
        half_xy=half_xy,
        floor_z=floor_z,
        radius=radius,
        zone_half_xy=zone_half_xy,
        p_zone_only=p_zone_only,
        active_mask=mask,
        random_orientation=True,
        min_gap=min_gap,
        # the spare pieces wait on the ground (the ground plane is 1.05 m below the table top), out of view, at rest
        park_xy=(0.0, 1.15),
        park_z=-1.05 + 0.045,
        park_spacing=0.11,
    )


# --- the shape-aware tactile map -----------------------------------------------------------------------------------

_SDFS: dict = {}


def tactile_map_shape(env: ManagerBasedRLEnv, slot_types: list[int] | None = None, log_scale: bool = True) -> torch.Tensor:
    """Both pads' force on a 7 x 4 taxel grid each, ``(E, 56)`` (log(1 + N) if ``log_scale``), with the footprint of every touched
    piece from its shape (``tactile_sdf``) and the walls and floor of the box from the Gaussian model of the tactile sensor."""
    from .env_cfg import SLOT_TYPES
    from .tactile_sdf import PieceSdfs, piece_taxel_force

    slot_types = SLOT_TYPES if slot_types is None else slot_types
    sdfs = _SDFS.setdefault(str(env.device), PieceSdfs(str(env.device)))
    sensor = env.scene["tactile"]
    n = len(slot_types)
    pos = torch.stack([env.scene[f"sphere_{i}"].data.root_pos_w.torch for i in range(n)], dim=1)
    quat = torch.stack([env.scene[f"sphere_{i}"].data.root_quat_w.torch for i in range(n)], dim=1)
    taxel_w = sensor.taxel_positions_w  # (E, 2, T, 3)
    pieces = piece_taxel_force(sdfs, slot_types, taxel_w, sensor.object_force[..., :n], pos, quat)  # (E, 2, T)
    walls = sensor.taxel_force_of(slice(n, None)).reshape(env.num_envs, 2, -1)  # the box parts follow the pieces
    force = (pieces + walls).reshape(env.num_envs, -1)
    # a rolling history of the raw maps and of the closure (the last 8 steps): what the recogniser reads, for the recognition reward
    robot = env.scene["robot"]
    if not hasattr(env, "_finger_id"):
        env._finger_id = robot.find_joints("finger_joint")[0][0]
    closure = robot.data.joint_pos.torch[:, env._finger_id]
    if not hasattr(env, "_shape_hist_maps"):
        env._shape_hist_maps = torch.zeros(env.num_envs, 8, force.shape[1], device=env.device)
        env._shape_hist_closure = torch.zeros(env.num_envs, 8, device=env.device)
    env._shape_hist_maps = torch.cat([env._shape_hist_maps[:, 1:], force[:, None]], dim=1)
    env._shape_hist_closure = torch.cat([env._shape_hist_closure[:, 1:], closure[:, None]], dim=1)
    return torch.log1p(force) if log_scale else force


def tactile_map_gaussian(env: ManagerBasedRLEnv, log_scale: bool = True) -> torch.Tensor:
    """The Gaussian model of the tactile sensor for everything (the tactile pick's map), ``(E, 56)``."""
    return tp.tactile_maps(env, log_scale=log_scale)


_RECOGNIZERS: dict = {}


def r_recognise(env: ManagerBasedRLEnv, cfg: dict, model_path: str, every: int = 8) -> torch.Tensor:
    """Reward when a pick completes: the probability the (frozen, calibrated) recogniser gives to the true class of the held piece.

    The policy is paid for holding pieces in a way that can be recognised from the maps of the grasp. The classifier runs on the CPU and is
    slow to call (about 0.3 s whatever the batch), so the pick events are collected and scored together every ``every`` steps: the
    reward comes at most ``every`` steps (0.27 s) after the pick. ``(E,)``, 0 where nothing is scored."""
    import numpy as np

    from .env_cfg import SLOT_TYPES

    state = tp.get_state(env, cfg)
    out = torch.zeros(env.num_envs, device=env.device)
    pend = getattr(env, "_rec_pending", None)
    if pend is None:
        pend = env._rec_pending = {"ids": [], "maps": [], "closure": [], "truth": []}
    ids = state.success.nonzero().flatten()
    if len(ids) and hasattr(env, "_shape_hist_maps"):
        pend["ids"].append(ids)
        pend["maps"].append(env._shape_hist_maps[ids].clone())
        pend["closure"].append(env._shape_hist_closure[ids].clone())
        pend["truth"].append(torch.tensor(SLOT_TYPES, device=env.device)[state.near[ids]])
    if pend["ids"] and env.common_step_counter % every == 0:
        import joblib

        from . import recognizer

        model = _RECOGNIZERS.setdefault(model_path, joblib.load(model_path))
        ids, maps, closure, truth = (torch.cat(pend[k]) for k in ("ids", "maps", "closure", "truth"))
        for k in pend:
            pend[k].clear()
        _, proba = recognizer.decide(model, maps.cpu().numpy(), closure.cpu().numpy())
        p_true = torch.tensor(proba[np.arange(len(ids)), truth.cpu().numpy()], dtype=torch.float32, device=env.device)
        out.index_add_(0, ids, p_true)
    return out
