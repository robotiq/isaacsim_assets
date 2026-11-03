"""Scripted grasps of shape sorter pieces, recording what the tactile fingertips feel: data to test how well the pieces can be recognised.

    python collect_grasps.py --out grasps.npz [--batches 8] [--num_envs 192]

One piece (random type, random orientation, dropped and settled) is in each env's box. The real controller, with the piece's true
position (this is not a policy): move over it, descend to its centre height, close the gripper, lift. At four instants (two while
closing, two while lifting) the 7 x 4 taxel forces of both pads, the gripper closure and the pad forces are recorded.

Saved: maps (N, 4, 2, 7, 4) [N] (the Gaussian model), taxel_w (N, 4, 2, 28, 3), piece_pos_t (N, 4, 3), piece_quat_t (N, 4, 4),
force_piece (N, 4, 2) (the geometry, to compute shape-aware maps offline), closure (N, 4) [rad], pad_force (N, 4, 2) [N], label (N,) (0 .. 5, the order of
``shape_pick.env_cfg.PIECES``), lifted (N,), piece_quat (N, 4) at the grasp (x y z w), piece_xy_in_tcp (N, 2) [m]."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Robotiq-Tactile-Shape-Pick-UR5e-2F85")
parser.add_argument("--num_envs", type=int, default=192)
parser.add_argument("--batches", type=int, default=8)
parser.add_argument("--out", default="grasps.npz")
parser.add_argument("--seed", type=int, default=0)
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import numpy as np  # noqa: E402
import robotiq_rl  # noqa: E402, F401  (registers the tasks)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.shape_pick.env_cfg import SLOT_TYPES  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import BOX_CENTER, BOX_INNER, TASK  # noqa: E402

RECORD = (19, 59)  # closing steps at which to record, then the lifting steps (below)
LIFT_RECORD = (49, 99)


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    env_cfg.episode_length_s = 3600.0
    env_cfg.seed = args_cli.seed
    env_cfg.observations.policy.enable_corruption = False
    params = env_cfg.events.reset_spheres.params
    params["num_active_range"] = (1, 1)  # one piece per box
    params["p_zone_only"] = 1.0  # within the tool tip's reach
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        n, dev = env.num_envs, env.device
        slot_type = torch.tensor(SLOT_TYPES, device=dev)
        centre = torch.tensor([*BOX_CENTER, 0.0], device=dev)
        keep = {k: [] for k in ("maps", "closure", "pad_force", "label", "lifted", "piece_quat", "piece_xy_in_tcp",
                                "taxel_w", "piece_pos_t", "piece_quat_t", "force_piece")}
        cur = {}  # the slot of each env's piece, set per batch

        def state():
            return tp.get_state(env, TASK)

        def snapshot():
            s = state()
            maps = tp.tactile_maps(env, log_scale=False).reshape(n, 2, 7, 4)
            closure = env.scene["robot"].data.joint_pos.torch[:, s.finger_id]
            sensor = env.scene["tactile"]
            ii, sl = torch.arange(n, device=dev), cur["slot"]
            pos = torch.stack([env.scene[f"sphere_{i}"].data.root_pos_w.torch for i in range(len(SLOT_TYPES))], 1)[ii, sl]
            quat = torch.stack([env.scene[f"sphere_{i}"].data.root_quat_w.torch for i in range(len(SLOT_TYPES))], 1)[ii, sl]
            geometry = (sensor.taxel_positions_w.clone(), pos.clone(), quat.clone(), sensor.object_force[ii, :, sl].clone())
            return maps.clone(), closure.clone(), maps.sum((-1, -2)).clone(), geometry

        for batch in range(args_cli.batches):
            env.reset()
            tp.get_state(env, TASK).cfg["home_ranges"] = None  # no pick completion (teleport) while the piece is lifted
            act = torch.zeros(n, env.action_manager.total_action_dim, device=dev)
            act[:, 6] = -1.0  # open
            for _ in range(90):  # let the dropped pieces settle
                env.step(act)
            s = state()
            rel = (s.sphere_pos - centre)[..., :2]
            in_box = (rel.abs() < BOX_INNER / 2).all(-1) & (s.sphere_pos[..., 2] > 0.0) & (s.sphere_pos[..., 2] < 0.1)
            slot = in_box.float().argmax(-1)  # the one piece in the box
            cur["slot"] = slot
            has_piece = in_box.any(-1)
            idx = torch.arange(n, device=dev)
            target = s.sphere_pos[idx, slot].clone()
            vel = torch.stack([env.scene[f"sphere_{i}"].data.root_lin_vel_w.torch for i in range(len(SLOT_TYPES))], 1)[idx, slot].norm(dim=-1)
            settled = has_piece & (vel < 0.02)

            def track(z_target):
                a = act.clone()
                a[:, 0:2] = ((target[:, :2] - state().tcp[:, :2]) * 60).clamp(-1, 1)
                a[:, 2] = ((z_target - state().tcp[:, 2]) * 60).clamp(-1, 1)
                return a

            for _ in range(150):  # over the piece
                act = track(torch.full((n,), 0.16, device=dev))
                act[:, 6] = -1.0
                env.step(act)
            for _ in range(150):  # down to its centre
                act = track(target[:, 2])
                act[:, 6] = -1.0
                env.step(act)
            piece_quat = torch.stack([env.scene[f"sphere_{i}"].data.root_quat_w.torch for i in range(len(SLOT_TYPES))], 1)[idx, slot].clone()
            z0 = torch.stack([env.scene[f"sphere_{i}"].data.root_pos_w.torch[:, 2] for i in range(len(SLOT_TYPES))], 1)[idx, slot].clone()
            act[:, 0:3] = 0.0
            act[:, 6] = 1.0  # close
            snaps = []
            for i in range(60):
                env.step(act)
                if i in RECORD:
                    snaps.append(snapshot())
            hold_xy = state().tcp[:, :2].clone()
            xy_in_tcp = (torch.stack([env.scene[f"sphere_{i}"].data.root_pos_w.torch for i in range(len(SLOT_TYPES))], 1)[idx, slot][:, :2]
                         - env.scene.env_origins[:, :2] - hold_xy)
            for i in range(100):  # lift
                act[:, 0:2] = ((hold_xy - state().tcp[:, :2]) * 60).clamp(-1, 1)
                act[:, 2] = 0.4
                env.step(act)
                if i in LIFT_RECORD:
                    snaps.append(snapshot())
            z1 = torch.stack([env.scene[f"sphere_{i}"].data.root_pos_w.torch[:, 2] for i in range(len(SLOT_TYPES))], 1)[idx, slot]
            lifted = (z1 - z0) > 0.04
            ok = settled & lifted
            keep["maps"].append(torch.stack([m for m, _, _, _ in snaps], 1)[ok].cpu())
            keep["closure"].append(torch.stack([c for _, c, _, _ in snaps], 1)[ok].cpu())
            keep["pad_force"].append(torch.stack([p for _, _, p, _ in snaps], 1)[ok].cpu())
            for j, name in enumerate(("taxel_w", "piece_pos_t", "piece_quat_t", "force_piece")):
                keep[name].append(torch.stack([g[j] for _, _, _, g in snaps], 1)[ok].cpu())
            keep["label"].append(slot_type[slot][ok].cpu())
            keep["lifted"].append(lifted[ok].cpu())
            keep["piece_quat"].append(piece_quat[ok].cpu())
            keep["piece_xy_in_tcp"].append(xy_in_tcp[ok].cpu())
            lab = slot_type[slot][ok].cpu().numpy()
            print(f"batch {batch}: {int(ok.sum())} of {n} grasps lifted the piece (settled {int(settled.sum())}); per class "
                  f"{np.bincount(lab, minlength=6).tolist()}", flush=True)
        out = {k: torch.cat(v).numpy() for k, v in keep.items()}
        np.savez_compressed(args_cli.out, **out)
        print("saved", args_cli.out, {k: v.shape for k, v in out.items()})
        env.close()


if __name__ == "__main__":
    main()
