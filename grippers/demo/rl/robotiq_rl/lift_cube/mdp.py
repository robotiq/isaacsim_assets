"""Extra MDP terms for the UR5e + 2F-85 lift.

The stock lift terms only look at the cube's height, which a policy can satisfy without grasping: v2 wedged the
cube in the forearm/wrist crease and carried it up with the open gripper far away. The terms here gate the lift
on a grasp: the cube between the pads (TCP close to it) with the gripper at least partly closed."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.utils import index_fill_
from isaaclab.utils.math import combine_frame_transforms, quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class _TcpTerm(ManagerTermBase):
    """Base of the terms below: the tool centre point and the gripper closure, per environment.

    The TCP is ``tcp_offset`` away from the ``body_name`` link, in that link's frame. It is computed from the
    link pose rather than a FrameTransformer: the gripper base is welded to wrist_3_link and PhysX reports
    the two as one body, which the frame transformer cannot track.

    Params every subclass takes: ``body_name``, ``tcp_offset``, and ``joint_name`` / ``closed_pos`` for the
    gripper driver joint (closure = joint position over its closed value, in [0, 1])."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._robot_cfg = cfg.params.get("robot_cfg", SceneEntityCfg("robot"))
        self._object_cfg = cfg.params.get("object_cfg", SceneEntityCfg("object"))
        robot = env.scene[self._robot_cfg.name]
        self._body_id = robot.find_bodies(cfg.params["body_name"])[0][0]
        self._joint_id = robot.find_joints(cfg.params["joint_name"])[0][0]
        self._closed_pos = cfg.params["closed_pos"]
        self._offset = torch.tensor(cfg.params["tcp_offset"], device=env.device).repeat(env.num_envs, 1)

    def _distance(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """Distance in metres from the TCP to the object."""
        robot = env.scene[self._robot_cfg.name]
        cube_pos_w = env.scene[self._object_cfg.name].data.root_pos_w.torch
        link_pos_w = robot.data.body_pos_w.torch[:, self._body_id]
        link_quat_w = robot.data.body_quat_w.torch[:, self._body_id]
        return torch.linalg.norm(cube_pos_w - (link_pos_w + quat_apply(link_quat_w, self._offset)), dim=1)

    def _closure(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        joint_pos = env.scene[self._robot_cfg.name].data.joint_pos.torch[:, self._joint_id]
        return (joint_pos / self._closed_pos).clamp(0.0, 1.0)

    def _grasped(self, env: ManagerBasedRLEnv, reach: float, min_closure: float) -> torch.Tensor:
        return (self._distance(env) < reach) & (self._closure(env) > min_closure)


class tcp_object_distance(_TcpTerm):
    """Tanh-kernel reward for bringing the TCP to the object."""

    def __call__(self, env: ManagerBasedRLEnv, std: float, *, body_name=None, tcp_offset=None, joint_name=None, closed_pos=None) -> torch.Tensor:
        return 1 - torch.tanh(self._distance(env) / std)


class grasp_closure(_TcpTerm):
    """Reward closing the gripper while the TCP is within ``reach`` of the object (nothing for closing in the air)."""

    def __call__(self, env: ManagerBasedRLEnv, reach: float, *, body_name=None, tcp_offset=None, joint_name=None, closed_pos=None) -> torch.Tensor:
        return (self._distance(env) < reach).float() * self._closure(env)


class lifted_in_grasp(_TcpTerm):
    """1 while the object is above ``minimal_height`` and held: TCP within ``reach``, closure above ``min_closure``."""

    def __call__(self, env: ManagerBasedRLEnv, minimal_height: float, reach: float, min_closure: float, *, body_name=None, tcp_offset=None, joint_name=None, closed_pos=None):
        z = env.scene[self._object_cfg.name].data.root_pos_w.torch[:, 2]
        return ((z > minimal_height) & self._grasped(env, reach, min_closure)).float()


class goal_distance_in_grasp(_TcpTerm):
    """Tanh-kernel reward for the object's distance to the commanded goal, only while it is held and lifted.

    With ``success_threshold`` it also logs ``Metrics/success_rate``: the share of episodes in which the held
    object got within that distance of the goal at least once."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._track_success = cfg.params.get("success_threshold") is not None
        self._succeeded = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None):
        if self._track_success and env_ids is not None:
            self._env.extras.setdefault("log", {})["Metrics/success_rate"] = (
                self._succeeded[env_ids].float().mean().item()
            )
        index_fill_(self._succeeded, env_ids, False)

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        std: float,
        minimal_height: float,
        reach: float,
        min_closure: float,
        command_name: str,
        success_threshold: float | None = None,
        body_name=None,
        tcp_offset=None,
        joint_name=None,
        closed_pos=None,
    ) -> torch.Tensor:
        robot = env.scene[self._robot_cfg.name]
        object_pos_w = env.scene[self._object_cfg.name].data.root_pos_w.torch
        command = env.command_manager.get_command(command_name)
        goal_w, _ = combine_frame_transforms(
            robot.data.root_pos_w.torch, robot.data.root_quat_w.torch, command[:, :3]
        )
        distance = torch.linalg.norm(goal_w - object_pos_w, dim=1)
        held = (object_pos_w[:, 2] > minimal_height) & self._grasped(env, reach, min_closure)
        if self._track_success:
            self._succeeded |= held & (distance < success_threshold)
        return held.float() * (1 - torch.tanh(distance / std))
