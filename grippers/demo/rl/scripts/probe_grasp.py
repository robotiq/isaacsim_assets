"""Scripted pick with the real controller: drive over a sphere, descend, close, lift. Logs the tactile signals.

    uv run python <this file>
Not a policy: it uses the sphere's true position, to check the sensors and the phase logic end to end."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.envs.utils.video_recorder_cfg import VideoRecorderCfg
from isaaclab.utils import instantiate
from isaaclab.visualizers import VisualizerCfg

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Robotiq-Tactile-Sphere-Pick-UR5e-2F85")
parser.add_argument("--sphere", type=int, default=-1, help="sphere index; -1 = the one nearest the box centre")
parser.add_argument("--video", default=None, help="record the whole run into this directory")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401  (registers our tasks; entry points are only loaded by the isaaclab CLI)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from robotiq_rl.tactile_pick import mdp as tp  # noqa: E402
from robotiq_rl.tactile_pick.env_cfg import TASK  # noqa: E402


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1, overrides=hydra_overrides)
    if args_cli.video:
        env_cfg.sim.visualizer_cfgs = VisualizerCfg(eye=(0.95, -0.45, 0.4), lookat=(0.45, 0.03, 0.12))
        env_cfg.video_recorders = [VideoRecorderCfg(source="viz", output_dir=args_cli.video, video_length=520)]
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        env.reset()
        dev = env.device
        act = torch.zeros(1, env.action_manager.total_action_dim, device=dev)
        act[:, 6] = -1.0  # open

        def state():
            return tp.get_state(env, TASK)

        def log(tag):
            s = state()
            fs, _ = tp.contact_forces(env, "pad_spheres")
            fb, _ = tp.contact_forces(env, "pad_box")
            net = env.scene["pad_spheres"].data.net_normal_forces_w.torch[0]
            print(
                f"{tag:28s} tcp {[round(v, 3) for v in s.tcp[0].tolist()]} sphere z {s.sphere_pos[0, args_cli.sphere, 2].item():.3f} "
                f"F_sph {[round(v, 2) for v in fs[0].sum(-1).tolist()]} F_box {[round(v, 2) for v in fb[0].sum(-1).tolist()]} "
                f"F_net {[round(float(v), 2) for v in torch.linalg.norm(net, dim=-1).tolist()]} "
                f"finger {env.scene['robot'].data.joint_pos.torch[0, s.finger_id].item():.2f} "
                f"touch/held/stable/lift {[round(float(x[0]), 2) for x in (s.touch, s.held, s.stable, s.lift_progress)]}"
            )

        for _ in range(20):
            env.step(act)
        sp = state().sphere_pos[0]
        centre = torch.tensor([*TASK['box_center'], 0.0], device=dev)
        which = args_cli.sphere if args_cli.sphere >= 0 else int(torch.linalg.norm((sp - centre)[:, :2], dim=-1).argmin())
        args_cli.sphere = which
        target = sp[which].clone()
        print("sphere", [round(v, 3) for v in target.tolist()])
        # 1. over the sphere, at a safe height
        for i in range(150):
            rel = target[:2] - state().tcp[0, :2]
            act[0, 0:2] = (rel * 60).clamp(-1, 1)
            act[0, 2] = ((0.16 - state().tcp[0, 2]) * 60).clamp(-1, 1)
            env.step(act)
        log("over the sphere")
        # 2. descend to the sphere's centre height
        for i in range(150):
            rel = target[:2] - state().tcp[0, :2]
            act[0, 0:2] = (rel * 60).clamp(-1, 1)
            act[0, 2] = ((target[2] - state().tcp[0, 2]) * 60).clamp(-1, 1)
            env.step(act)
        log("descended")
        # 3. close
        act[:, 0:3] = 0.0
        act[:, 6] = 1.0
        def show_maps(tag):
            maps = tp.tactile_maps(env, log_scale=False)[0].reshape(2, 7, 4)
            for side, name in enumerate(("left", "right")):
                print(f"[{tag}] taxel force {name} (N):")
                for row in maps[side].tolist():
                    print("   ", " ".join(f"{v:5.2f}" for v in row))

        for i in range(60):
            env.step(act)
            if i in (10, 20, 59):
                log(f"closing, step {i}")
        show_maps("holding")
        # 4. lift, holding xy
        hold_xy = state().tcp[0, :2].clone()
        for i in range(160):
            act[0, 0:2] = ((hold_xy - state().tcp[0, :2]) * 60).clamp(-1, 1)
            act[0, 2] = 0.4
            env.step(act)
            if i < 70 and i % 4 == 0 or i in (100, 159):
                log(f"lifting, step {i}")
        s = state()
        print("success flag so far:", float(s.success_given[0]))
        env.close()


if __name__ == "__main__":
    main()
