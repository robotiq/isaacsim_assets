"""Extra MDP terms for the UR5e + 2F-85 lift."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.utils.math import quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class tcp_object_distance(ManagerTermBase):
    """Tanh-kernel reward for bringing the tool centre point to the object.

    The TCP is ``tcp_offset`` away from the ``body_name`` link, in that link's frame. It is computed from the
    link pose rather than a FrameTransformer: the gripper base is welded to wrist_3_link and PhysX reports
    the two as one body, which the frame transformer cannot track."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        robot = env.scene[cfg.params.get("robot_cfg", SceneEntityCfg("robot")).name]
        body_ids, _ = robot.find_bodies(cfg.params["body_name"])
        self._body_id = body_ids[0]
        self._offset = torch.tensor(cfg.params["tcp_offset"], device=env.device).repeat(env.num_envs, 1)

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        std: float,
        body_name: str,
        tcp_offset: tuple[float, float, float],
        robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
        object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ) -> torch.Tensor:
        robot = env.scene[robot_cfg.name]
        cube_pos_w = env.scene[object_cfg.name].data.root_pos_w.torch
        link_pos_w = robot.data.body_pos_w.torch[:, self._body_id]
        link_quat_w = robot.data.body_quat_w.torch[:, self._body_id]
        tcp_w = link_pos_w + quat_apply(link_quat_w, self._offset)
        return 1 - torch.tanh(torch.linalg.norm(cube_pos_w - tcp_w, dim=1) / std)
