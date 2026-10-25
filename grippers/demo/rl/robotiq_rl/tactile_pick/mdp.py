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
    zone_half_xy: tuple[float, float] | None = None,
    num_active_range: tuple[int, int] | None = None,
    p_zone_only: float = 0.0,
    park_xy: tuple[float, float] = (0.3, 0.95),
):
    """Put the spheres at random, non-overlapping spots on the box floor (dart throwing, per env).

    ``half_xy`` is the area the spheres are spread over (the whole floor). With ``zone_half_xy`` (the area the tool tip
    can reach) sphere 0 is placed inside it first, so there is always at least one sphere to pick. A share
    ``p_zone_only`` of the envs puts all spheres in that zone: every sphere can be picked, as when emptying the box.
    With ``num_active_range`` only a random number of spheres (sphere 0 first) stays in the box, the others are
    parked on the ground beside the table: the policy meets boxes with few spheres left, as when emptying one."""
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    n = len(env_ids)
    dev = env.device
    placed = torch.zeros(n, NUM_SPHERES, 2, device=dev)
    zone = torch.tensor(zone_half_xy if zone_half_xy is not None else half_xy, device=dev)
    whole = torch.tensor(half_xy, device=dev)
    zone_only = torch.rand(n, device=dev) < p_zone_only
    half_env = torch.where(zone_only[:, None], zone, whole)  # (n, 2)
    for i in range(NUM_SPHERES):
        half = zone[None] if (i == 0 and zone_half_xy is not None) else half_env
        cand = (torch.rand(n, candidates, 2, device=dev) * 2 - 1) * half[:, None, :]
        if i == 0:
            placed[:, 0] = cand[:, 0]
            continue
        dist = torch.linalg.norm(cand[:, :, None] - placed[:, None, :i], dim=-1).min(-1).values  # (n, K)
        ok = dist > min_gap
        pick = torch.where(ok.any(-1), ok.float().argmax(-1), dist.argmax(-1))
        placed[:, i] = cand[torch.arange(n), pick]
    origins = env.scene.env_origins[env_ids]
    quat = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).repeat(n, 1)
    zero = torch.zeros(n, 6, device=dev)
    if num_active_range is not None:
        k = torch.randint(num_active_range[0], num_active_range[1] + 1, (n,), device=dev)
    else:
        k = torch.full((n,), NUM_SPHERES, device=dev)
    for i in range(NUM_SPHERES):
        in_box = torch.stack(
            [
                origins[:, 0] + box_center[0] + placed[:, i, 0],
                origins[:, 1] + box_center[1] + placed[:, i, 1],
                origins[:, 2] + floor_z + radius + 0.002 + torch.zeros(n, device=dev),
            ],
            dim=-1,
        )
        parked = torch.stack(
            [origins[:, 0] + park_xy[0] + 0.05 * i, origins[:, 1] + park_xy[1], origins[:, 2] + 0.3 + torch.zeros(n, device=dev)],
            dim=-1,
        )
        pos = torch.where((i < k)[:, None], in_box, parked)
        sphere = env.scene[f"sphere_{i}"]
        sphere.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], dim=-1), env_ids=env_ids)
        sphere.write_root_velocity_to_sim_index(root_velocity=zero, env_ids=env_ids)


_START_TABLE = {}


def _start_table(device):
    """Joint angles of a UR5e whose tool tip is at (x, y, z) with the tool pointing down (``start_poses.npz``)."""
    if "q" not in _START_TABLE:
        import os

        import numpy as np

        data = np.load(os.path.join(os.path.dirname(__file__), "start_poses.npz"))
        _START_TABLE.update({k: torch.tensor(data[k], dtype=torch.float32, device=device) for k in ("x", "y", "z", "q")})
    return _START_TABLE


def reset_arm_start(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float],
    joint_names=("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"),
):
    """Start the arm with the tool tip at a random point (env frame), pointing down, gripper open."""
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    n = len(env_ids)
    table = _start_table(env.device)
    lo = torch.tensor([x_range[0], y_range[0], z_range[0]], device=env.device)
    hi = torch.tensor([x_range[1], y_range[1], z_range[1]], device=env.device)
    p = lo + (hi - lo) * torch.rand(n, 3, device=env.device)
    # trilinear interpolation of the joint table at p
    idx, frac = [], []
    for k, axis in enumerate(("x", "y", "z")):
        grid = table[axis]
        i0 = torch.bucketize(p[:, k], grid).clamp(1, len(grid) - 1) - 1
        t = ((p[:, k] - grid[i0]) / (grid[i0 + 1] - grid[i0])).clamp(0, 1)
        idx.append(i0)
        frac.append(t)
    q = torch.zeros(n, 6, device=env.device)
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                w = (frac[0] if dx else 1 - frac[0]) * (frac[1] if dy else 1 - frac[1]) * (frac[2] if dz else 1 - frac[2])
                q += w[:, None] * table["q"][idx[0] + dx, idx[1] + dy, idx[2] + dz]
    robot = env.scene["robot"]
    joint_ids = robot.find_joints(list(joint_names), preserve_order=True)[0]
    robot.write_joint_position_to_sim_index(position=q, joint_ids=joint_ids, env_ids=env_ids)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), joint_ids=joint_ids, env_ids=env_ids)


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
        self.out_count = torch.zeros(n, device=dev)
        self.prev_in_zone = torch.zeros(n, NUM_SPHERES, dtype=torch.bool, device=dev)
        self.fresh = torch.ones(n, dtype=torch.bool, device=dev)
        self.n_picked = torch.zeros(n, device=dev)
        self.disturb = torch.zeros(n, device=dev)
        self.speed_excess = torch.zeros(n, device=dev)
        self.step = -1
        # derived each step
        zeros = torch.zeros(n, device=dev)
        self.new_cells = self.touch = self.held = self.stable = self.success = self.dropped = zeros
        self.lift_progress = self.sym = self.wall_force = self.enter = self.approach = self.out_now = self.out_centred = zeros
        self.tcp = torch.zeros(n, 3, device=dev)
        self.sphere_pos = torch.zeros(n, NUM_SPHERES, 3, device=dev)

    def reset(self, env_ids):
        self.visited[env_ids] = False
        self.held_count[env_ids] = 0
        self.was_stable[env_ids] = False
        self.success_given[env_ids] = False
        self.out_count[env_ids] = 0
        self.fresh[env_ids] = True
        self.n_picked[env_ids] = 0

    def complete_pick(self, mask: torch.Tensor, near: torch.Tensor):
        """A sphere was held out of the box for a second: do what the scripted carry-and-drop does, instantly.

        The held sphere is parked outside, the arm goes to a random home pose above the box and the gripper opens.
        The episode goes on with the spheres that are left, so what a pick does to the others (shoving them against a
        wall, out of reach) costs the picks that follow."""
        env, c = self.env, self.cfg
        ids = mask.nonzero().flatten()
        origins = env.scene.env_origins[ids]
        quat = torch.tensor([0.0, 0.0, 0.0, 1.0], device=env.device).repeat(len(ids), 1)
        zero = torch.zeros(len(ids), 6, device=env.device)
        for i in range(NUM_SPHERES):
            sel = near[ids] == i
            if not sel.any():
                continue
            e = ids[sel]
            pos = origins[sel] + torch.tensor([0.3 + 0.05 * i, 0.95, 0.3], device=env.device)
            sphere = env.scene[f"sphere_{i}"]
            sphere.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat[sel]], dim=-1), env_ids=e)
            sphere.write_root_velocity_to_sim_index(root_velocity=zero[sel], env_ids=e)
        reset_arm_start(env, ids, **c["home_ranges"])
        robot = env.scene["robot"]
        gripper = [j for j in range(len(robot.joint_names)) if j >= 6]  # the 6 arm joints come first
        robot.write_joint_position_to_sim_index(
            position=robot.data.default_joint_pos.torch[ids][:, gripper], joint_ids=gripper, env_ids=ids
        )
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros(len(ids), len(gripper), device=env.device), joint_ids=gripper, env_ids=ids)
        self.held_count[ids] = 0
        self.was_stable[ids] = False
        self.out_count[ids] = 0
        self.success_given[ids] = False
        self.fresh[ids] = True
        self.n_picked[ids] += 1

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
        self.approach = 1 - torch.tanh(torch.linalg.norm(rel, dim=-1) / 0.15)
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
        # --- gentleness: spheres knocked out of the reachable zone (towards the walls), and fast-moving spheres
        on_floor = (self.sphere_pos[..., 2] > 0.0) & (self.sphere_pos[..., 2] < c["rim_z"] + 0.02)  # parked ones are lower
        rel_s = self.sphere_pos[..., :2] - torch.tensor(c["box_center"], device=env.device)
        in_zone = (rel_s.abs() < half).all(-1) & on_floor
        out_zone = (rel_s.abs() < c["box_inner_half"]).all(-1) & on_floor & ~in_zone
        is_held = held[:, None] & (torch.arange(NUM_SPHERES, device=env.device)[None] == near[:, None])
        event = self.prev_in_zone & out_zone & ~is_held & ~self.fresh[:, None]
        self.disturb = event.sum(-1).float()
        self.prev_in_zone = in_zone
        self.fresh[:] = False
        vel = torch.stack([env.scene[f"sphere_{i}"].data.root_lin_vel_w.torch for i in range(NUM_SPHERES)], dim=1)
        speed = torch.linalg.norm(vel, dim=-1)
        self.speed_excess = ((speed - c["gentle_speed"]).clamp(min=0) * (on_floor & ~is_held)).sum(-1)
        # --- phase 4: lift
        # the dense lift reward keeps rising ``lift_extra`` past the success height: no plateau to hover on
        lift_h = c["rim_z"] + c["out_margin"] + c["lift_extra"] - c["rest_z"]
        # rises to its maximum at the success height + lift_extra, stays there across the "out" band and falls off above
        # it: flying away is not rewarded, and there is a gradient back down into the band
        top = c["rim_z"] + c["out_margin"] + c["out_band"]
        too_high = ((near_z - top) / 0.5).clamp(0, 1)  # wide: a slope even for a sphere carried at 60 cm
        self.lift_progress = stable.float() * ((near_z - c["rest_z"]) / lift_h).clamp(0, 1) * (1 - too_high)
        # "out" is a height band above the rim (no reward for flying off) and, if configured, over the box
        near_xy = self.sphere_pos[torch.arange(env.num_envs, device=env.device), near, :2]
        out = stable & (near_z > c["rim_z"] + c["out_margin"]) & (near_z < c["rim_z"] + c["out_margin"] + c["out_band"])
        off_centre = torch.linalg.norm(near_xy - torch.tensor(c["box_center"], device=env.device), dim=-1)
        if c.get("out_half_xy") is not None:  # held above the box, not carried away
            out &= ((near_xy - torch.tensor(c["box_center"], device=env.device)).abs() < torch.tensor(c["out_half_xy"], device=env.device)).all(-1)
        self.out_now = out.float()
        self.out_centred = out.float() * (0.5 + 0.5 * (1 - torch.tanh(off_centre / 0.15)))  # more when over the box centre
        # steps in the band during one continuous hold: they add up (a policy that hunts around the band edge still
        # counts) and start over only when the sphere is dropped
        self.out_count = torch.where(out, self.out_count + 1, torch.where(stable, self.out_count, torch.zeros_like(self.out_count)))
        done = self.out_count >= c["success_steps"]
        self.success = (done & ~self.success_given).float()
        self.success_given |= done
        self.step = env.common_step_counter
        if c.get("home_ranges") is not None and bool(self.success.any()):
            self.complete_pick(self.success.bool(), near)


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


def r_approach(env, cfg):
    return get_state(env, cfg).approach


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


def r_out(env, cfg):
    return get_state(env, cfg).out_centred


def r_disturb(env, cfg):
    return get_state(env, cfg).disturb


def r_speed(env, cfg):
    return get_state(env, cfg).speed_excess


def r_success(env, cfg):
    return get_state(env, cfg).success


def r_drop(env, cfg):
    return get_state(env, cfg).dropped


def r_wall(env, cfg):
    return get_state(env, cfg).wall_force.clamp(max=20.0)


def success_termination(env, cfg):
    return get_state(env, cfg).success.bool()


def action_rate_used(env, dims: tuple[int, ...]):
    """Squared change of the actions that move the robot (the three rotation dims have scale 0 and are left out)."""
    a, prev = env.action_manager.action, env.action_manager.prev_action
    return ((a[:, list(dims)] - prev[:, list(dims)]) ** 2).sum(-1)


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
