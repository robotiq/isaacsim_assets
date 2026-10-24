"""Debug the pad contact sensors: pin a sphere against the left pad's centre taxel and compare the readings.

    uv run python <this file>"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Robotiq-Tactile-Sphere-Pick-UR5e-2F85")
parser.add_argument("--inset", type=float, default=0.0, help="m the sphere surface is pushed into the pad")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import SPHERE_RADIUS, TASK  # noqa: E402


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1, overrides=hydra_overrides)
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        env.reset()
        dev = env.device
        sphere = env.scene["sphere_0"]
        ids = torch.arange(1, device=dev)
        quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]], device=dev)
        act = torch.zeros(1, env.action_manager.total_action_dim, device=dev)
        act[:, 6] = -0.3  # gripper about a third closed
        for _ in range(30):
            env.step(act)
        print("sphere_0 after reset (env frame)", [round(v, 3) for v in (sphere.data.root_pos_w.torch[0] - env.scene.env_origins[0]).tolist()])
        taxels = tp._Taxels(env, env.scene["robot"] and __import__("isaaclab.managers", fromlist=["SceneEntityCfg"]).SceneEntityCfg("robot"), ("left_fingertip", "right_fingertip"))
        centres = taxels.world()[0]  # (2, T, 3)
        mid = 3 * 4 + 1  # row 3, col 1
        left, right = centres[0, mid], centres[1, mid]
        toward_right = torch.nn.functional.normalize(right - left, dim=0)
        target = left + toward_right * (SPHERE_RADIUS - args_cli.inset)
        print("left taxel", [round(v, 4) for v in left.tolist()], "right taxel", [round(v, 4) for v in right.tolist()], "pad gap", round(float((right - left).norm()), 4))
        for i in range(60):
            sphere.write_root_pose_to_sim_index(root_pose=torch.cat([target[None], quat], dim=-1), env_ids=ids)
            sphere.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=ids)
            env.step(act)
            if i in (5, 59):
                print(f"--- step {i}: sphere at", [round(v, 4) for v in sphere.data.root_pos_w.torch[0].tolist()], "target", [round(v, 4) for v in target.tolist()])
                for name in ("pad_spheres", "pad_box"):
                    d = env.scene[name].data
                    print(name, "unfiltered |F| per pad", [round(float(v), 3) for v in torch.linalg.norm(d.net_normal_forces_w.torch[0], dim=-1).tolist()])
                    print(name, "filtered |F| per pad/filter", [[round(float(v), 3) for v in row] for row in torch.linalg.norm(d.normal_force_matrix_w.torch[0], dim=-1).tolist()])
                    cp = d.contact_pos_w.torch[0]
                    print(name, "contact pos pad0 filter0", [round(float(v), 4) for v in cp[0, 0].tolist()], "nan?", bool(torch.isnan(cp).any()))
        env.close()


if __name__ == "__main__":
    main()
