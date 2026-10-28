"""Check of the Isaac Lab tactile sensor on a bare gripper: close it on a sphere, then open it.

    python examples/check_tactile_sensor.py [--gripper 2f85|2f140] [--num_envs 2]

The gripper points up, its base is fixed. It closes until its pads are nearly a sphere's diameter apart, a zero-gravity
sphere is put between them at the middle of the taxel arrays, and the gripper closes on it and opens again. (The pads slide
along the fingers as the gripper closes, so the sphere is placed once they have almost closed: it then touches the arrays
where it was put.) The check asserts what a correct sensor reads:
  - nothing while the gripper is open,
  - once closed, both pads carry the same force (the sphere is squeezed, so action = reaction), a few newtons,
  - the force sits where the sphere touches the pad: the force-weighted taxel position, in the world frame, matches the
    sphere's position along the pad,
  - nothing again after the gripper opens.
It needs Isaac Sim with Isaac Lab 3 (a GPU), unlike tests/unit, and prints the readings so they can be looked at."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument("--gripper", choices=("2f85", "2f140"), default="2f85")
parser.add_argument("--num_envs", type=int, default=2)
add_launcher_args(parser)
args_cli = parser.parse_args()

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import configclass, instantiate, replace  # noqa: E402
from robotiq_tactile_sensors.isaaclab import ROBOTIQ_2F85_TSF85_CFG, ROBOTIQ_2F140_TSF140_CFG, TactileSensorCfg  # noqa: E402
from robotiq_tactile_sensors.mounts import ROBOTIQ_2F_85, ROBOTIQ_2F_140  # noqa: E402

SPHERE_RADIUS = 0.02
GRIPPER_Z = 0.5
GRIPPERS = {
    # robot config, mount, name of the prim that holds the pad bodies, joint target that closes the gripper [rad]
    "2f85": (ROBOTIQ_2F85_TSF85_CFG, ROBOTIQ_2F_85, "Robotiq_2F_85", 0.8),
    "2f140": (ROBOTIQ_2F140_TSF140_CFG, ROBOTIQ_2F_140, "Robotiq_2F_140", 0.785),
}


def make_scene_cfg(gripper: str):
    robot_cfg, mount, prim, _ = GRIPPERS[gripper]
    # the middle of the taxel array, in the gripper frame (the gripper's own frame is the world frame shifted up by GRIPPER_Z)
    mid = mount.left.origin[2] - 0.5 * (mount.spec.rows - 1) * mount.spec.pitch_row

    @configclass
    class CheckSceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
        gripper = replace(
            robot_cfg,
            prim_path="{ENV_REGEX_NS}/Gripper",
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, GRIPPER_Z), joint_pos={"finger_joint": 0.0}),
        )
        sphere = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Sphere",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, GRIPPER_Z + mid)),  # aside until it is put between the pads
            spawn=sim_utils.SphereCfg(
                radius=SPHERE_RADIUS,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                mass_props=sim_utils.MassCfg(mass=0.03),
                collision_props=sim_utils.CollisionPropertiesCfg(),
            ),
        )
        tactile = TactileSensorCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Gripper/{prim}",
            mount=mount,
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Sphere"],
        )

    return CheckSceneCfg


def main():
    closed = GRIPPERS[args_cli.gripper][3]
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1 / 120)
    with launch_simulation(sim_cfg, args_cli):
        sim = SimulationContext(sim_cfg)
        scene = instantiate(make_scene_cfg(args_cli.gripper)(num_envs=args_cli.num_envs, env_spacing=1.0))
        sim.reset()
        gripper, tactile = scene["gripper"], scene["tactile"]
        dt = sim.get_physics_dt()
        joint = gripper.find_joints("finger_joint")[0]

        def run(target: float, steps: int, tag: str):
            for i in range(steps):
                gripper.set_joint_position_target_index(
                    target=torch.full((scene.num_envs, 1), target, device=sim.device), joint_ids=joint
                )
                scene.write_data_to_sim()
                sim.step()
                scene.update(dt)
            force = tactile.taxel_force  # (E, 2, rows, cols)
            pad = tactile.pad_force
            rows = torch.arange(force.shape[2], device=force.device, dtype=force.dtype)
            cols = torch.arange(force.shape[3], device=force.device, dtype=force.dtype)
            total = force.sum(dim=(-1, -2)).clamp(min=1e-9)
            centroid_row = (force.sum(-1) * rows).sum(-1) / total  # (E, 2)
            centroid_col = (force.sum(-2) * cols).sum(-1) / total
            print(f"[{tag}] pad force L/R env0: {pad[0].tolist()}  centroid row/col L env0: "
                  f"{centroid_row[0, 0].item():.2f} / {centroid_col[0, 0].item():.2f}")
            return pad, centroid_row, centroid_col, force

        pad, _, _, _ = run(0.0, 60, "open")
        assert pad.max() < 1e-3, f"force with the gripper open: {pad}"

        # close until the middles of the two taxel arrays are a sphere's diameter + 2 cm apart, then put the sphere between them
        target = 0.0
        for _ in range(2000):
            mids = tactile.taxel_positions_w.mean(2)  # (E, 2, 3) centre of each pad's array
            if ((mids[:, 1] - mids[:, 0]).norm(dim=-1) <= 2 * SPHERE_RADIUS + 0.02).all() or target >= closed:
                break
            target = min(target + 0.002, closed)
            gripper.set_joint_position_target_index(
                target=torch.full((scene.num_envs, 1), target, device=sim.device), joint_ids=joint
            )
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
        centre = mids.mean(1)  # (E, 3)
        pose = torch.cat([centre, torch.tensor([[0.0, 0.0, 0.0, 1.0]], device=sim.device).repeat(scene.num_envs, 1)], dim=-1)
        scene["sphere"].write_root_pose_to_sim_index(root_pose=pose)
        scene["sphere"].write_root_velocity_to_sim_index(root_velocity=torch.zeros(scene.num_envs, 6, device=sim.device))
        print(f"sphere put between the pads at joint target {target:.3f} rad, jaws {(mids[0, 1] - mids[0, 0]).norm().item() * 1000:.0f} mm apart")

        pad, row, col, force = run(closed, 240, "closed on the sphere")
        left, right = pad[:, 0], pad[:, 1]
        assert (pad > 0.5).all(), f"no force on a pad while squeezing the sphere: {pad}"
        assert ((left - right).abs() / (left + right) < 0.15).all(), f"the two pads should carry the same force: {pad}"
        # where on the pad the force is, in the world frame, against where the sphere is (x and z: along the pad)
        taxel_w = tactile.taxel_positions_w  # (E, 2, T, 3)
        flat = force.reshape(force.shape[0], 2, -1)
        centroid_w = (flat[..., None] * taxel_w).sum(2) / flat.sum(-1, keepdim=True).clamp(min=1e-9)  # (E, 2, 3)
        sphere_w = scene["sphere"].data.root_pos_w.torch  # (E, 3)
        err = (centroid_w - sphere_w[:, None])[..., [0, 2]].norm(dim=-1)  # (E, 2) m
        print(f"distance from the force centroid to the sphere centre along the pad [mm]: {(err * 1000).tolist()[0]}")
        assert (err < 0.006).all(), f"the force is not where the sphere touches the pad: {err}"
        print("taxel force, left pad, env 0 [N]:")
        for r in force[0, 0].tolist():
            print("   ", " ".join(f"{v:5.2f}" for v in r))

        pad, _, _, _ = run(0.0, 240, "open again")
        assert pad.max() < 1e-3, f"force after opening: {pad}"
        print(f"TACTILE SENSOR CHECK PASSED ({args_cli.gripper})")


if __name__ == "__main__":
    main()
