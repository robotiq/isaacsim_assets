"""Where do the policy's grasp attempts of the pieces fail? (diagnostic, no training)

    python diag_shape.py --policy <policy.pt> [--num_envs 128] [--steps 900]

Counts, over all env-steps: a pad touching a piece; both pads pressing a piece (> touch_n); both pads pressing with the
gripper closed past min_closure; and the same with the piece's centre within hold_dist of the tool point (= the task's "held"). Also the
distance from the tool point to the nearest piece centre while both pads press, and the closure."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Robotiq-Tactile-Shape-Pick-UR5e-2F85")
parser.add_argument("--policy", required=True)
parser.add_argument("--num_envs", type=int, default=128)
parser.add_argument("--steps", type=int, default=900)
parser.add_argument("--seed", type=int, default=3)
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import TASK  # noqa: E402


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    env_cfg.seed = args_cli.seed
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        policy = torch.jit.load(args_cli.policy, map_location=env.device)
        obs, _ = env.reset()
        cnt = dict(steps=0, one=0, both=0, both_closed=0, held=0, success=0)
        dist_both, closure_both, pad_n = [], [], []
        for t in range(args_cli.steps):
            with torch.inference_mode():
                obs, *_ = env.step(policy(obs["policy"]))
            s = tp.get_state(env, TASK)
            f, _ = tp.contact_forces(env, "pad_spheres")
            fl, fr = f[:, 0].sum(-1), f[:, 1].sum(-1)
            thr = TASK["touch_n"]
            both = (fl > thr) & (fr > thr)
            closure = (env.scene["robot"].data.joint_pos.torch[:, s.finger_id] / TASK["closed_pos"]).clamp(0, 1)
            dmin = torch.linalg.norm(s.sphere_pos - s.tcp[:, None], dim=-1).min(-1).values
            cnt["steps"] += env.num_envs
            cnt["one"] += int(((fl > thr) | (fr > thr)).sum())
            cnt["both"] += int(both.sum())
            cnt["both_closed"] += int((both & (closure > TASK["min_closure"])).sum())
            cnt["held"] += int((s.held > 0).sum())
            cnt["success"] += int(s.success.sum())
            if both.any():
                dist_both.append(dmin[both].cpu())
                closure_both.append(closure[both].cpu())
        d, c = torch.cat(dist_both), torch.cat(closure_both)
        tot = cnt["steps"]
        print(f"DIAG env-steps {tot}: a pad on a piece {cnt['one'] / tot:.3%}, both pads {cnt['both'] / tot:.3%}, both + closed > {TASK['min_closure']} "
              f"{cnt['both_closed'] / tot:.3%}, held (also within {TASK['hold_dist'] * 100:.0f} cm) {cnt['held'] / tot:.3%}, successes {cnt['success']}")
        print(f"DIAG while both pads press: distance tool point - nearest piece centre cm p10/p50/p90 = "
              f"{torch.quantile(d, torch.tensor([0.1, 0.5, 0.9])).mul(100).tolist()}, closure p10/p50/p90 = {torch.quantile(c, torch.tensor([0.1, 0.5, 0.9])).tolist()}")
        env.close()


if __name__ == "__main__":
    main()
