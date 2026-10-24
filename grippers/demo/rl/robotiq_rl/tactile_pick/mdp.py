"""MDP terms of the blind sphere pick: tactile observations, sphere layout reset, and phase-based rewards.

The policy never sees the spheres. It gets joint state, the gripper opening and the two 7 x 4 tactile maps. The
rewards and the critic use the sphere positions (privileged) to tell a sphere from the box and to judge the grasp.

Phases, in order, each rewarded only once the previous one holds:
  1. enter    the tool tip is inside the box footprint and lowered towards the spheres' height;
  2. search   new floor area is swept (coverage), and a sphere is touched;
  3. grasp    both pads press a sphere with the gripper partly closed; "stable" after a few steps of it;
  4. lift     a stably held sphere is raised, and success is out of the box above the rim.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply

from .taxels import layout

if TYPE_CHECKING:  # importing isaaclab.envs pulls in pxr, which must not happen before Kit starts
    from isaaclab.envs import ManagerBasedRLEnv

NUM_SPHERES = 10


# ---------------------------------------------------------------------------------------------------------------
# tactile observation
# ---------------------------------------------------------------------------------------------------------------


class _Taxels:
    """Per env: the taxel grid in the world frame and the contact force spread over it."""

    def __init__(self, env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg, pad_bodies: tuple[str, str]):
        self.robot = env.scene[robot_cfg.name]
        ids = [self.robot.find_bodies(b)[0][0] for b in pad_bodies]
        self.body_ids = torch.tensor(ids, device=env.device)
        self.local = torch.stack(
            [torch.tensor(layout(side), dtype=torch.float32, device=env.device) for side in ("left", "right")]
        )  # (2, T, 3)

    def world(self) -> torch.Tensor:
        """Taxel centres in the world frame, ``(E, 2, T, 3)``."""
        pos = self.robot.data.body_pos_w.torch[:, self.body_ids]  # (E, 2, 3)
        quat = self.robot.data.body_quat_w.torch[:, self.body_ids]  # (E, 2, 4)
        e, _, t, _ = (pos.shape[0], 2, self.local.shape[1], 3)
        local = self.local[None].expand(e, -1, -1, -1)
        rot = quat_apply(quat[:, :, None].expand(-1, -1, t, -1).reshape(-1, 4), local.reshape(-1, 3))
        return pos[:, :, None] + rot.view(e, 2, t, 3)


def contact_forces(env: ManagerBasedRLEnv, sensor: str, floor_n: float = 0.05):
    """Per pad (left, right) and filtered object: the normal force [N] and the average contact point.

    Shapes ``(E, 2, F)`` and ``(E, 2, F, 3)``. The sensor lists its bodies in its own order: it is put in (left, right)."""
    perms = env.__dict__.setdefault("_tp_pad_perm", {})
    if sensor not in perms:
        names = env.scene[sensor].body_names
        perms[sensor] = [names.index("left_fingertip"), names.index("right_fingertip")]
    perm = perms[sensor]
    data = env.scene[sensor].data
    force = torch.linalg.norm(data.normal_force_matrix_w.torch, dim=-1)[:, perm]  # (E, 2, F)
    force = torch.where(force > floor_n, force, torch.zeros_like(force))
    pos = torch.nan_to_num(data.contact_pos_w.torch)[:, perm]
    return force, pos


def tactile_maps(
    env: ManagerBasedRLEnv,
    sensors: tuple[str, ...] = ("pad_spheres", "pad_box"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    pad_bodies: tuple[str, str] = ("left_fingertip", "right_fingertip"),
    sigma: float = 0.006,
    log_scale: bool = True,
) -> torch.Tensor:
    """Force per taxel for both pads, ``(E, 2 * ROWS * COLS)``.

    Each filtered object's normal force is spread over the taxels around its average contact point with a Gaussian
    (``sigma`` m), normalised so the taxels sum to the force. Contacts with the spheres and with the box both count:
    the sensor cannot tell what it touches. ``log_scale`` gives log(1 + N), which keeps light touches visible."""
    taxels = getattr(env, "_tp_taxels", None)
    if taxels is None:
        taxels = env._tp_taxels = _Taxels(env, robot_cfg, pad_bodies)
    centres = taxels.world()  # (E, 2, T, 3)
    out = torch.zeros(centres.shape[:3], device=env.device)  # (E, 2, T)
    for name in sensors:
        force, pos = contact_forces(env, name)  # (E, 2, F), (E, 2, F, 3)
        d2 = ((centres[:, :, :, None] - pos[:, :, None]) ** 2).sum(-1)  # (E, 2, T, F)
        w = torch.exp(-d2 / (2 * sigma**2))
        w = w / (w.sum(dim=2, keepdim=True) + 1e-9)
        out += (w * force[:, :, None]).sum(-1)
    out = torch.log1p(out) if log_scale else out
    return out.reshape(env.num_envs, -1)


def pad_force_total(env: ManagerBasedRLEnv, sensors: tuple[str, ...] = ("pad_spheres", "pad_box")) -> torch.Tensor:
    """Summed normal force per pad, ``(E, 2)`` in N (log-scaled for the observation)."""
    total = sum(contact_forces(env, s)[0].sum(-1) for s in sensors)
    return torch.log1p(total)


# ---------------------------------------------------------------------------------------------------------------
# resets
# ---------------------------------------------------------------------------------------------------------------


def reset_spheres_in_box(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    box_center: tuple[float, float],
    half_xy: tuple[float, float],
    floor_z: float,
    radius: float,
    min_gap: float = 0.046,
    candidates: int = 40,
):
    """Put the spheres at random, non-overlapping spots on the box floor (dart throwing, per env)."""
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    n = len(env_ids)
    placed = torch.zeros(n, NUM_SPHERES, 2, device=env.device)
    for i in range(NUM_SPHERES):
        cand = (torch.rand(n, candidates, 2, device=env.device) * 2 - 1) * torch.tensor(half_xy, device=env.device)
        if i == 0:
            placed[:, 0] = cand[:, 0]
            continue
        dist = torch.linalg.norm(cand[:, :, None] - placed[:, None, :i], dim=-1).min(-1).values  # (n, K)
        ok = dist > min_gap
        pick = torch.where(ok.any(-1), ok.float().argmax(-1), dist.argmax(-1))
        placed[:, i] = cand[torch.arange(n), pick]
    origins = env.scene.env_origins[env_ids]
    quat = torch.tensor([0.0, 0.0, 0.0, 1.0], device=env.device).repeat(n, 1)
    zero = torch.zeros(n, 6, device=env.device)
    for i in range(NUM_SPHERES):
        pos = torch.stack(
            [
                origins[:, 0] + box_center[0] + placed[:, i, 0],
                origins[:, 1] + box_center[1] + placed[:, i, 1],
                origins[:, 2] + floor_z + radius + 0.002 + torch.zeros(n, device=env.device),
            ],
            dim=-1,
        )
        sphere = env.scene[f"sphere_{i}"]
        sphere.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], dim=-1), env_ids=env_ids)
        sphere.write_root_velocity_to_sim_index(root_velocity=zero, env_ids=env_ids)


# ---------------------------------------------------------------------------------------------------------------
# task state: computed once per step, shared by the reward terms and the critic observations
# ---------------------------------------------------------------------------------------------------------------


class TaskState:
    """Per-env bookkeeping of the pick phases. Updated at most once per env step (``get_state``)."""

    COV_CELLS = 8  # coverage grid over the box floor: COV_CELLS x COV_CELLS

    def __init__(self, env: ManagerBasedRLEnv, cfg: dict):
        self.cfg = cfg
        self.env = env
        n, dev = env.num_envs, env.device
        robot = env.scene["robot"]
        self.body_id = robot.find_bodies(cfg["body_name"])[0][0]
        self.finger_id = robot.find_joints(cfg["joint_name"])[0][0]
        self.offset = torch.tensor(cfg["tcp_offset"], device=dev).repeat(n, 1)
        self.visited = torch.zeros(n, self.COV_CELLS**2, dtype=torch.bool, device=dev)
        self.held_count = torch.zeros(n, device=dev)
        self.was_stable = torch.zeros(n, dtype=torch.bool, device=dev)
        self.success_given = torch.zeros(n, dtype=torch.bool, device=dev)
        self.step = -1
        # derived each step
        zeros = torch.zeros(n, device=dev)
        self.new_cells = self.touch = self.held = self.stable = self.success = self.dropped = zeros
        self.lift_progress = self.sym = self.wall_force = self.enter = zeros
        self.tcp = torch.zeros(n, 3, device=dev)
        self.sphere_pos = torch.zeros(n, NUM_SPHERES, 3, device=dev)

    def reset(self, env_ids):
        self.visited[env_ids] = False
        self.held_count[env_ids] = 0
        self.was_stable[env_ids] = False
        self.success_given[env_ids] = False

    def update(self):
        env, c = self.env, self.cfg
        robot = env.scene["robot"]
        origins = env.scene.env_origins
        link_pos = robot.data.body_pos_w.torch[:, self.body_id]
        link_quat = robot.data.body_quat_w.torch[:, self.body_id]
        self.tcp = link_pos + quat_apply(link_quat, self.offset) - origins  # env frame
        self.sphere_pos = (
            torch.stack([env.scene[f"sphere_{i}"].data.root_pos_w.torch for i in range(NUM_SPHERES)], dim=1)
            - origins[:, None]
        )
        rel = self.tcp[:, :2] - torch.tensor(c["box_center"], device=env.device)
        half = torch.tensor(c["half_xy"], device=env.device)
        inside_xy = (rel.abs() < half).all(-1)
        # --- phase 1/2: enter and sweep
        lowered = self.tcp[:, 2] < c["search_z"] + 0.03
        low_enough = self.tcp[:, 2] < c["rim_z"] - 0.005
        self.enter = inside_xy.float() * (1 - torch.tanh((self.tcp[:, 2] - c["search_z"]).clamp(min=0) / 0.08))
        cell = ((rel + half) / (2 * half) * self.COV_CELLS).long().clamp(0, self.COV_CELLS - 1)
        idx = cell[:, 0] * self.COV_CELLS + cell[:, 1]
        sweeping = inside_xy & low_enough
        was = self.visited.gather(1, idx[:, None])[:, 0]
        new = sweeping & ~was
        self.visited[torch.arange(env.num_envs, device=env.device)[new], idx[new]] = True
        self.new_cells = new.float()
        # --- tactile ground truth (privileged: spheres vs box)
        f_sph, _ = contact_forces(env, "pad_spheres")
        f_box, _ = contact_forces(env, "pad_box")
        fl, fr = f_sph[:, 0].sum(-1), f_sph[:, 1].sum(-1)
        thr = c["touch_n"]
        self.wall_force = f_box.sum((-1, -2))
        self.touch = ((fl > thr) | (fr > thr)).float()
        both = (fl > thr) & (fr > thr)
        self.sym = torch.minimum(fl, fr) / (torch.maximum(fl, fr) + 1e-6)
        # --- phase 3: the sphere nearest the TCP is the one in the gripper
        d = torch.linalg.norm(self.sphere_pos - self.tcp[:, None], dim=-1)  # (E, S)
        dmin, near = d.min(-1)
        near_z = self.sphere_pos[torch.arange(env.num_envs, device=env.device), near, 2]
        closure = (robot.data.joint_pos.torch[:, self.finger_id] / c["closed_pos"]).clamp(0, 1)
        held = both & (dmin < c["hold_dist"]) & (closure > c["min_closure"])
        self.held = held.float()
        self.held_count = torch.where(held, self.held_count + 1, torch.zeros_like(self.held_count))
        stable = self.held_count >= c["stable_steps"]
        self.dropped = (self.was_stable & ~held).float()
        self.was_stable = stable | (self.was_stable & held)
        self.stable = stable.float()
        # --- phase 4: lift
        lift_h = c["rim_z"] + c["out_margin"] - c["rest_z"]
        self.lift_progress = stable.float() * ((near_z - c["rest_z"]) / lift_h).clamp(0, 1)
        out = stable & (near_z > c["rim_z"] + c["out_margin"])
        self.success = (out & ~self.success_given).float()
        self.success_given |= out
        self.step = env.common_step_counter


def get_state(env: ManagerBasedRLEnv, cfg: dict) -> TaskState:
    state = getattr(env, "_tp_state", None)
    if state is None:
        state = env._tp_state = TaskState(env, cfg)
    if state.step != env.common_step_counter:
        state.update()
    return state


def reset_task_state(env: ManagerBasedRLEnv, env_ids: torch.Tensor, cfg: dict):
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    get_state(env, cfg).reset(env_ids)


# --- reward terms (each returns a (E,) tensor; weights are set in the env cfg) -----------------------------------


def r_enter(env, cfg):
    return get_state(env, cfg).enter


def r_coverage(env, cfg):
    return get_state(env, cfg).new_cells


def r_touch(env, cfg):
    return get_state(env, cfg).touch


def r_grasp(env, cfg):
    s = get_state(env, cfg)
    return s.held * (0.5 + 0.5 * s.sym)


def r_stable(env, cfg):
    return get_state(env, cfg).stable


def r_lift(env, cfg):
    return get_state(env, cfg).lift_progress


def r_success(env, cfg):
    return get_state(env, cfg).success


def r_drop(env, cfg):
    return get_state(env, cfg).dropped


def r_wall(env, cfg):
    return get_state(env, cfg).wall_force.clamp(max=20.0)


def success_termination(env, cfg):
    return get_state(env, cfg).success.bool()


# --- actor observation ---------------------------------------------------------------------------------------


def tcp_in_box_frame(env, cfg):
    """Tool tip position relative to the box centre (from the joint angles: proprioception, not vision)."""
    s = get_state(env, cfg)
    return s.tcp - torch.tensor([*cfg["box_center"], 0.0], device=env.device)


# --- critic observations (privileged) -------------------------------------------------------------------------


def privileged_obs(env, cfg):
    """Sphere positions relative to the box centre, the TCP, and the phase flags, ``(E, 3 * S + 3 + 4)``."""
    s = get_state(env, cfg)
    centre = torch.tensor([*cfg["box_center"], 0.0], device=env.device)
    spheres = (s.sphere_pos - centre).reshape(env.num_envs, -1)
    flags = torch.stack([s.touch, s.held, s.stable, s.lift_progress], dim=-1)
    return torch.cat([spheres, s.tcp - centre, flags], dim=-1)
