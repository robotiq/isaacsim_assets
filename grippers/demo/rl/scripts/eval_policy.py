"""Roll out an exported policy and print grasp statistics (what the policy does when the cube is up).

    play.py (any run) first exports <run>/exported/policy.pt; then:
    uv run python <this file> --policy <run>/exported/policy.pt --num_envs 64
"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--steps", type=int, default=300)
parser.add_argument("--policy", required=True)
parser.add_argument("--task", default="Robotiq-Lift-Cube-UR5e-2F85")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab.utils.math import quat_apply  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.lift_cube.env_cfg import TCP_OFFSET  # noqa: E402


def main():
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides
    )
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        policy = torch.jit.load(args_cli.policy, map_location=env.device)
        obs, _ = env.reset()
        robot, cube = env.scene["robot"], env.scene["object"]
        wrist = robot.body_names.index("wrist_3_link")
        finger = robot.joint_names.index("finger_joint")
        off = torch.tensor(TCP_OFFSET, device=env.device)
        n = env.num_envs
        max_z = torch.zeros(n, device=env.device)
        lifted_steps = torch.zeros(n, device=env.device)
        stats = {"closure": [], "tcp_dist": []}
        for _ in range(args_cli.steps):
            with torch.inference_mode():
                action = policy(obs["policy"])
                obs, *_ = env.step(action)
            z = cube.data.root_pos_w.torch[:, 2]
            max_z = torch.maximum(max_z, z)
            up = z > 0.06
            lifted_steps += up
            tcp = robot.data.body_pos_w.torch[:, wrist] + quat_apply(robot.data.body_quat_w.torch[:, wrist], off.repeat(n, 1))
            dist = torch.linalg.norm(cube.data.root_pos_w.torch - tcp, dim=1)
            if up.any():
                stats["closure"].append((robot.data.joint_pos.torch[up, finger] / 0.8).mean().item())
                stats["tcp_dist"].append(dist[up].mean().item())
        print(f"RESULT envs={n} steps={args_cli.steps}")
        print(f"RESULT envs that ever lifted >6cm: {(max_z > 0.06).float().mean().item():.2f}")
        print(f"RESULT mean fraction of steps lifted: {(lifted_steps / args_cli.steps).mean().item():.2f}")
        print(f"RESULT max cube z (median over envs): {max_z.median().item():.3f}")
        if stats["closure"]:
            print(f"RESULT while lifted: finger closure {sum(stats['closure']) / len(stats['closure']):.2f}, "
                  f"TCP-cube distance {sum(stats['tcp_dist']) / len(stats['tcp_dist']):.3f} m")
        env.close()


if __name__ == "__main__":
    main()
