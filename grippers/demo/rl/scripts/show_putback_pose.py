"""Render the arm at the put-back poses of pick_all.py in the scene: the waypoint above the box and the release pose, from the front and from the side.

    python show_putback_pose.py --out putback_pose.png"""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.utils import instantiate

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Robotiq-Tactile-Shape-Pick-UR5e-2F85")
parser.add_argument("--out", default="putback_pose.png")
add_launcher_args(parser)
args_cli, hydra_overrides = parser.parse_known_args()

import isaaclab_tasks  # noqa: E402, F401
import robotiq_rl  # noqa: E402, F401
import torch  # noqa: E402
from isaaclab.sensors.camera import CameraCfg  # noqa: E402
from isaaclab.sim import PinholeCameraCfg  # noqa: E402
from isaaclab.utils.math import quat_from_matrix  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

PUT_BACK_ABOVE = (-0.2534, -1.6529, 1.9059, -2.0038, -1.6055, -0.8434)
PUT_BACK_RELEASE = (-0.2534, -1.5247, 2.1176, -2.3437, -1.6055, -0.8434)
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
VIEWS = {"front": ((1.0, -0.55, 0.62), (0.5, 0.0, 0.12)), "side": ((0.5, 1.1, 0.45), (0.5, 0.0, 0.12))}


def camera(eye, target):
    eye, target = torch.tensor(eye), torch.tensor(target)
    fwd = (target - eye) / (target - eye).norm()
    right = torch.linalg.cross(fwd, torch.tensor([0.0, 0.0, 1.0]))
    right = right / right.norm()
    up = torch.linalg.cross(right, fwd)
    quat = quat_from_matrix(torch.stack([right, up, -fwd], dim=-1)).tolist()
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/Cam" + str(abs(hash(tuple(eye.tolist()))) % 9999), update_period=0.0, height=720, width=1280, data_types=["rgb"],
        spawn=PinholeCameraCfg(focal_length=18.0, horizontal_aperture=20.955, clipping_range=(0.05, 20.0)),
        offset=CameraCfg.OffsetCfg(pos=tuple(eye.tolist()), rot=tuple(quat), convention="opengl"),
    )


def main():
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1, overrides=hydra_overrides)
    for name, (eye, target) in VIEWS.items():
        setattr(env_cfg.scene, f"cam_{name}", camera(eye, target))
    with launch_simulation(env_cfg, args_cli):
        env = instantiate(env_cfg)
        env.reset()
        robot = env.scene["robot"]
        arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        act = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
        act[:, 6] = 1.0  # gripper closed, as when a piece is held
        images = {}
        for pose_name, q in (("above the box (waypoint)", PUT_BACK_ABOVE), ("release pose", PUT_BACK_RELEASE)):
            qt = torch.tensor([q], device=env.device)
            for _ in range(40):  # drive the joints there and hold them
                robot.write_joint_position_to_sim_index(position=qt, joint_ids=arm_ids, env_ids=torch.tensor([0], device=env.device))
                robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qt), joint_ids=arm_ids, env_ids=torch.tensor([0], device=env.device))
                robot.set_joint_position_target_index(target=qt, joint_ids=arm_ids)
                env.scene.write_data_to_sim()
                env.sim.step()
                env.scene.update(env.physics_dt)
            env.sim.render()
            for name in VIEWS:
                cam = env.scene[f"cam_{name}"]
                cam.update(1 / 120, force_recompute=True)
                images[(pose_name, name)] = Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy())
            from robotiq_rl.tactile_pick import mdp as tp
            from robotiq_rl.tactile_pick.env_cfg import TASK

            print(f"POSE {pose_name}: tool tip at {[round(v, 3) for v in tp.get_state(env, TASK).tcp[0].tolist()]}, joints {[round(v, 3) for v in robot.data.joint_pos.torch[0, arm_ids].tolist()]}")
        sheet = Image.new("RGB", (1280 * 2, 720 * 2))
        for r, pose_name in enumerate(("above the box (waypoint)", "release pose")):
            for c, name in enumerate(VIEWS):
                im = images[(pose_name, name)]
                ImageDraw.Draw(im).text((20, 20), f"{pose_name} - {name} view", fill=(255, 230, 120))
                sheet.paste(im, (c * 1280, r * 720))
        sheet.resize((1920, 1080)).save(args_cli.out)
        print("saved", args_cli.out)
        env.close()


if __name__ == "__main__":
    main()
