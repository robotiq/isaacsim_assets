"""Scripted check of the tactile pipeline: close the gripper on a sphere, then press the tool tip on the box floor.

    uv run python <this file> --num_envs 2
Prints the sensors' body order, the per-pad forces, the taxel maps and the phase flags."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=2)
parser.add_argument("--task", default="Robotiq-Tactile-Sphere-Pick-UR5e-2F85")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import TASK  # noqa: E402


def show(env, tag):
    s = tp.get_state(env, TASK)
    maps = tp.tactile_maps(env, log_scale=False)[0].reshape(2, 7, 4)
    fs, _ = tp.contact_forces(env, "pad_spheres")
    fb, _ = tp.contact_forces(env, "pad_box")
    print(f"--- {tag}")
    print("TCP (env frame)", [round(v, 3) for v in s.tcp[0].tolist()], "finger", round(env.scene["robot"].data.joint_pos.torch[0, s.finger_id].item(), 3))
    for name in ("pad_spheres", "pad_box"):
        net = env.scene[name].data.net_normal_forces_w.torch[0]
        print(f"{name}: unfiltered net force per pad [N]", [round(float(v), 2) for v in torch.linalg.norm(net, dim=-1).tolist()])
    print("sphere force per pad [N]", [round(v, 2) for v in fs[0].sum(-1).tolist()], "box force per pad [N]", [round(v, 2) for v in fb[0].sum(-1).tolist()])
    print("flags touch/held/stable/lift", [round(float(x[0]), 2) for x in (s.touch, s.held, s.stable, s.lift_progress)], "sym", round(float(s.sym[0]), 2))
    for side, name in enumerate(("left", "right")):
        print(f"taxel force {name} (rows x cols, N):")
        for row in maps[side].tolist():
            print("   ", " ".join(f"{v:5.2f}" for v in row))


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, overrides=hydra_overrides)
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        env.reset()
        for name in ("pad_spheres", "pad_box"):
            sensor = env.scene[name]
            print(name, "bodies", sensor.body_names, "force matrix", tuple(sensor.data.normal_force_matrix_w.torch.shape))
        n = env.num_envs
        act = torch.zeros(n, env.action_manager.total_action_dim, device=env.device)
        act[:, -1] = -1.0  # gripper open
        for _ in range(20):
            env.step(act)
        s = tp.get_state(env, TASK)
        ids = torch.arange(n, device=env.device)
        quat = torch.tensor([0.0, 0.0, 0.0, 1.0], device=env.device).repeat(n, 1)
        sphere = env.scene["sphere_0"]

        def pin_sphere(at):
            sphere.write_root_pose_to_sim_index(root_pose=torch.cat([at, quat], dim=-1), env_ids=ids)
            sphere.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=env.device), env_ids=ids)

        # 1. sphere held between the open pads (re-pinned every step: gravity must not take it), gripper closes
        hold = tp.get_state(env, TASK).tcp.clone() + env.scene.env_origins
        act[:, 6] = 1.0  # gripper closes (target = 0.4 + 0.4 * a)
        for i in range(40):
            pin_sphere(hold)
            env.step(act)
        show(env, "sphere pinned between the pads, gripper closed 40 steps")
        for i in range(30):  # released: it is now squeezed (or falls)
            env.step(act)
        show(env, "released, still squeezed?")
        # 2. open, move over the box centre, press the tool tip down
        act[:] = 0.0
        act[:, 6] = -1.0
        for _ in range(30):
            env.step(act)
        c = torch.tensor([*TASK["box_center"]], device=env.device)
        for _ in range(60):  # centre xy over the box
            rel = tp.get_state(env, TASK).tcp[:, :2] - c
            act[:, 0:2] = (-rel * 50).clamp(-1, 1)
            env.step(act)
        act[:, 0:2] = 0.0
        act[:, 2] = -1.0
        for _ in range(80):
            env.step(act)
        show(env, "tool tip pressed down in the box centre")
        env.close()


if __name__ == "__main__":
    main()
