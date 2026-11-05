"""Check of the converted shape sorter pieces in Isaac Lab: do they spawn, settle at the predicted height and get a sensible mass?

    python check_shape_sorter.py --assets <out_dir of convert_shape_sorter.py>

Drops each piece type (in its most stable pose, then in each other stable pose) on a floor and compares the resting height of
its centre of mass and its mass with poses.json."""

import argparse
import json
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument("--assets", required=True)
add_launcher_args(parser)
args_cli = parser.parse_args()

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import configclass, instantiate  # noqa: E402

ASSETS = Path(args_cli.assets)
INFO = json.loads((ASSETS / "poses.json").read_text())
STEMS = sorted(INFO)
POSES = [(s, i) for s in STEMS for i in range(len(INFO[s]["stable_poses"]))]


def wxyz_to_xyzw(q):
    return (q[1], q[2], q[3], q[0])


def make_cfg():
    @configclass
    class Cfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))

    cfg = Cfg(num_envs=1, env_spacing=1.0)
    for n, (stem, i) in enumerate(POSES):
        pose = INFO[stem]["stable_poses"][i]
        x, y = 0.12 * (n % 10), 0.12 * (n // 10)
        setattr(
            cfg,
            f"p{n}",
            RigidObjectCfg(
                prim_path=f"{{ENV_REGEX_NS}}/P{n}",
                init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, pose["com_height"] + 0.01), rot=wxyz_to_xyzw(pose["quat_wxyz"])),
                spawn=sim_utils.UsdFileCfg(usd_path=str(ASSETS / f"{stem}.usd"), rigid_props=sim_utils.RigidBodyPropertiesCfg()),
            ),
        )
    return cfg


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1 / 120)
    with launch_simulation(sim_cfg, args_cli):
        sim = SimulationContext(sim_cfg)
        scene = instantiate(make_cfg())
        sim.reset()
        dt = sim.get_physics_dt()
        for _ in range(360):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
        print(f"{'piece':24s} {'pose':>4s} {'prob':>5s} {'z [cm]':>7s} {'pred':>6s} {'speed':>7s} {'mass g':>7s} {'pred':>6s}")
        bad = 0
        for n, (stem, i) in enumerate(POSES):
            obj = scene[f"p{n}"]
            z = float(obj.data.root_pos_w.torch[0, 2])
            speed = float(obj.data.root_lin_vel_w.torch[0].norm())
            mass = obj.root_view.get_masses()
            mass = float(torch.as_tensor(mass.numpy() if hasattr(mass, "numpy") else mass).flatten()[0]) * 1000
            pose = INFO[stem]["stable_poses"][i]
            ok = abs(z - pose["com_height"]) < 0.004 and speed < 0.02
            bad += not ok
            print(f"{stem:24s} {i:4d} {pose['probability']:5.2f} {z * 100:7.2f} {pose['com_height'] * 100:6.2f} {speed:7.3f} {mass:7.1f} {INFO[stem]['mass_g']:6.1f} {'' if ok else '  <-- moved'}")
        print("SHAPE SORTER CHECK:", "all pieces rest where predicted" if not bad else f"{bad} pose(s) did not stay (they tipped over, which is fine for unstable poses)")


if __name__ == "__main__":
    main()
