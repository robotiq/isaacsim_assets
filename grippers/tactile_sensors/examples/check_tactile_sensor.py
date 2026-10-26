"""Check of the Isaac Lab tactile sensor on the bare 2F-85: close the gripper on a sphere, then open it.

    python examples/check_tactile_sensor.py [--num_envs 2]

A zero-gravity sphere sits between the open pads (the gripper points up, its base is fixed). The gripper closes on it
and opens again. The check asserts what a correct sensor reads:
  - nothing while the gripper is open,
  - once closed, both pads carry the same force (the sphere is squeezed, so action = reaction), a few newtons,
  - the force sits where the sphere touches the pad: the force-weighted taxel position, in the world frame, matches the
    sphere's position along the pad (the pads slide along the fingers as the gripper closes, so this is not the middle
    row of the array),
  - nothing again after the gripper opens.
It needs Isaac Sim with Isaac Lab 3 (a GPU), unlike tests/unit, and prints the readings so they can be looked at."""

import argparse

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=2)
add_launcher_args(parser)
args_cli = parser.parse_args()

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import configclass, instantiate, replace  # noqa: E402
from robotiq_tactile_sensors.isaaclab import ROBOTIQ_2F85_TSF85_CFG, TactileSensorCfg  # noqa: E402
from robotiq_tactile_sensors.mounts import ROBOTIQ_2F_85  # noqa: E402

SPHERE_RADIUS = 0.02
GRIPPER_Z = 0.5
# the middle of the taxel array, in the gripper frame (the gripper's own frame is the world frame shifted up by GRIPPER_Z)
MID = ROBOTIQ_2F_85.left.origin[2] - 0.5 * (ROBOTIQ_2F_85.spec.rows - 1) * ROBOTIQ_2F_85.spec.pitch_row


@configclass
class CheckSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    gripper = replace(
        ROBOTIQ_2F85_TSF85_CFG,
        prim_path="{ENV_REGEX_NS}/Gripper",
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, GRIPPER_Z), joint_pos={"finger_joint": 0.0}),
    )
    sphere = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Sphere",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, GRIPPER_Z + MID)),
        spawn=sim_utils.SphereCfg(
            radius=SPHERE_RADIUS,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            mass_props=sim_utils.MassCfg(mass=0.03),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )
    tactile = TactileSensorCfg(
        prim_path="{ENV_REGEX_NS}/Gripper/Robotiq_2F_85",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Sphere"],
    )


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1 / 120)
    with launch_simulation(sim_cfg, args_cli):
        sim = SimulationContext(sim_cfg)
        scene = instantiate(CheckSceneCfg(num_envs=args_cli.num_envs, env_spacing=1.0))
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

        pad, row, col, force = run(0.8, 240, "closed on the sphere")
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
        print("TACTILE SENSOR CHECK PASSED")


if __name__ == "__main__":
    main()
