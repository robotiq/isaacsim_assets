"""Record a clip of a task's scene (zero actions): the robot at its start pose, the spheres settling in the box.

    uv run python <this file> --num_envs 1 --steps 90 --eye 1.15 -0.75 0.75 --lookat 0.5 0 0.1 --out clip_dir
"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.envs.utils.video_recorder_cfg import VideoRecorderCfg
from isaaclab.utils import instantiate
from isaaclab.visualizers import VisualizerCfg

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--task", default="Robotiq-Tactile-Sphere-Pick-UR5e-2F85")
parser.add_argument("--steps", type=int, default=90)
parser.add_argument("--eye", type=float, nargs=3, default=None)
parser.add_argument("--lookat", type=float, nargs=3, default=None)
parser.add_argument("--out", required=True)
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    if args_cli.eye:
        env_cfg.sim.visualizer_cfgs = VisualizerCfg(eye=tuple(args_cli.eye), lookat=tuple(args_cli.lookat))
    env_cfg.video_recorders = [VideoRecorderCfg(source="viz", output_dir=args_cli.out, video_length=args_cli.steps)]
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        env.reset()
        action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
        action[:, -1] = 1.0  # gripper open
        for _ in range(args_cli.steps):
            env.step(action)
        env.close()


if __name__ == "__main__":
    main()
