"""Roll out an exported policy on the tactile pick: per-phase statistics over many envs and a timeline of env 0.

    play.py (any run) first exports <run>/exported/policy.pt; then:
    uv run python <this file> --policy <run>/exported/policy.pt --num_envs 64 [--video dir]"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.envs.utils.video_recorder_cfg import VideoRecorderCfg
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--steps", type=int, default=600)
parser.add_argument("--policy", required=True)
parser.add_argument("--task", default="Robotiq-Tactile-Sphere-Pick-UR5e-2F85")
parser.add_argument("--video", default=None, help="record env 0 into this directory")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import RIM_Z, TASK  # noqa: E402


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    if args_cli.video:
        env_cfg.video_recorders = [VideoRecorderCfg(source="viz", output_dir=args_cli.video, video_length=args_cli.steps)]
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        policy = torch.jit.load(args_cli.policy, map_location=env.device)
        obs, _ = env.reset()
        n = env.num_envs
        dev = env.device
        ever = {k: torch.zeros(n, dtype=torch.bool, device=dev) for k in ("inside", "touch", "held", "stable", "out")}
        frac = {k: torch.zeros(n, device=dev) for k in ("inside", "touch", "held", "stable")}
        successes = torch.zeros(n, device=dev)  # success events per env (the env resets on success, so count them)
        max_z = torch.zeros(n, device=dev)
        first = {k: torch.full((n,), -1, device=dev, dtype=torch.long) for k in ever}
        for t in range(args_cli.steps):
            with torch.inference_mode():
                obs, *_ = env.step(policy(obs["policy"]))
            s = tp.get_state(env, TASK)
            successes += s.success
            inside = (s.enter > 0.0) & (s.tcp[:, 2] < RIM_Z)
            for k, v in (("inside", inside), ("touch", s.touch > 0), ("held", s.held > 0), ("stable", s.stable > 0), ("out", s.out_now > 0)):
                new = v & ~ever[k]
                first[k][new] = t
                ever[k] |= v
                if k in frac:
                    frac[k] += v.float()
            if t % 25 == 0:
                fs, _ = tp.contact_forces(env, "pad_spheres")
                print(
                    f"t={t:3d} env0 tcp {[round(v, 3) for v in s.tcp[0].tolist()]} F_sph {[round(v, 1) for v in fs[0].sum(-1).tolist()]} "
                    f"touch/held/stable/lift {[round(float(x[0]), 2) for x in (s.touch, s.held, s.stable, s.lift_progress)]} "
                    f"cells {int(s.visited[0].sum())}"
                )
        print(f"RESULT envs={n} steps={args_cli.steps}")
        for k in ("inside", "touch", "held", "stable", "out"):
            f = first[k][ever[k]].float()
            print(f"RESULT {k:7s}: ever {ever[k].float().mean().item():.2f}  first at step {f.median().item() if len(f) else float('nan'):.0f}"
                  + (f"  share of steps {frac[k].mean().item() / args_cli.steps:.2f}" if k in frac else ""))
        print(f"RESULT successes (sphere held out of the box for 1 s): {successes.sum().item():.0f} in {n} envs x {args_cli.steps} steps; envs with one: {(successes > 0).float().mean().item():.2f}")
        print(f"RESULT coverage cells visited (mean of 64): {tp.get_state(env, TASK).visited.float().sum(-1).mean().item():.1f}")
        env.close()


if __name__ == "__main__":
    main()
