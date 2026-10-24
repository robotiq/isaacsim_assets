"""Print the geometry of the lift scene: bodies, joints, start pose, TCP, cube, gripper opening/closing.

    uv run python <this file> --num_envs 2
"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=2)
parser.add_argument("--task", default="Robotiq-Lift-Cube-UR5e-2F85")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
from isaaclab.utils.math import quat_apply  # noqa: E402
from robotiq_rl.lift_cube.env_cfg import TCP_OFFSET  # noqa: E402
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        env.reset()
        robot = env.scene["robot"]
        cube = env.scene["object"] if "object" in env.scene.keys() else env.scene["sphere_0"]  # the object to measure against
        print("ROOT", robot.data.root_pos_w.torch[0].tolist(), "ENV_ORIGIN", env.scene.env_origins[0].tolist())
        print("BODIES", robot.body_names)
        print("JOINTS", robot.joint_names)

        def report(tag):
            wrist = robot.body_names.index("wrist_3_link")
            pos = robot.data.body_pos_w.torch[0]
            print(f"--- {tag}")
            print("wrist_3_link", pos[wrist].tolist())
            print("cube", cube.data.root_pos_w.torch[0].tolist())
            quat = robot.data.body_quat_w.torch[0, wrist]
            tcp = pos[wrist] + quat_apply(quat, torch.tensor(TCP_OFFSET, device=pos.device))
            axis = quat_apply(quat, torch.tensor([0.0, 0.0, 1.0], device=pos.device))
            print("tool axis (world)", [round(v, 3) for v in axis.tolist()])
            print("tcp", tcp.tolist(), "tcp-cube", (tcp - cube.data.root_pos_w.torch[0]).tolist())
            print("joint_pos", dict(zip(robot.joint_names, [round(v, 3) for v in robot.data.joint_pos.torch[0].tolist()])))
            for i, n in enumerate(robot.body_names):
                if any(k in n for k in ("finger", "pad", "knuckle")):
                    print("  ", n, [round(v, 4) for v in (pos[i] - pos[wrist]).tolist()])

        action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
        action[:, -1] = 1.0  # open
        for _ in range(60):
            env.step(action)
        report("start pose, gripper open, 60 steps")
        action[:, -1] = -1.0  # close
        for _ in range(90):
            env.step(action)
        report("gripper closed on nothing, 90 steps")
        env.close()


if __name__ == "__main__":
    main()
