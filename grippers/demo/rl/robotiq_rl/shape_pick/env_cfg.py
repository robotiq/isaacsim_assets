"""UR5e + 2F-85 (TSF-85 fingertips) with shape-sorter pieces in the box instead of spheres.

Everything else (box, arm, sensors, rewards, PPO settings) is the tactile pick's; the objects keep the names ``sphere_i``
so that its code works unchanged. There are 10 piece slots of fixed types (``SLOT_TYPES``); every reset puts a random subset
of 4 - 8 of them in the box (types equally likely), each dropped with a random orientation so it settles on a face of its
own. The class of the piece in slot ``i`` is ``SLOT_TYPES[i]``. The rewards still use the sphere radius for the pick test,
so this cfg is for the recognition data (scripts/collect_grasps.py) until the shape rewards are adapted."""

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils import configclass, replace
from isaaclab.utils.noise import GaussianNoiseCfg as Gnoise
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg

from ..tactile_pick import env_cfg as base
from . import mdp as sp

ASSETS = Path(__file__).resolve().parents[2] / "assets" / "shape_sorter"
PIECES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]  # the class names, in class order
SLOT_TYPES = [0, 1, 2, 3, 4, 5, 0, 1, 2, 3]  # piece type of each of the 10 slots


def _piece(i: int):
    name = f"piece_{SLOT_TYPES[i]}_{PIECES[SLOT_TYPES[i]]}"
    return replace(
        base._sphere(i),
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(ASSETS / f"{name}.usd"),
            rigid_props=PhysxRigidBodyCfg(
                solver_position_iteration_count=16, solver_velocity_iteration_count=1, max_depenetration_velocity=2.0
            ),
        ),
    )


@configclass
class ShapePickSceneCfg(base.TactilePickSceneCfg):
    sphere_0 = _piece(0)
    sphere_1 = _piece(1)
    sphere_2 = _piece(2)
    sphere_3 = _piece(3)
    sphere_4 = _piece(4)
    sphere_5 = _piece(5)
    sphere_6 = _piece(6)
    sphere_7 = _piece(7)
    sphere_8 = _piece(8)
    sphere_9 = _piece(9)


# SHAPE_TACTILE=gaussian: the tactile pick's map (one averaged contact point per pad and object) instead of the shape-aware one
TACTILE_MAP = sp.tactile_map_gaussian if os.environ.get("SHAPE_TACTILE", "sdf") == "gaussian" else sp.tactile_map_shape


@configclass
class ObservationsCfg(base.ObservationsCfg):
    @configclass
    class PolicyCfg(base.ObservationsCfg.PolicyCfg):
        tactile = ObsTerm(func=TACTILE_MAP, noise=Gnoise(mean=0.0, std=0.03))

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg(base.EventCfg):
    reset_spheres = EventTerm(
        func=sp.reset_pieces_in_box,
        mode="reset",
        params={
            "slot_types": SLOT_TYPES,
            "num_active_range": (4, 8),
            **{k: v for k, v in base.EventCfg().reset_spheres.params.items() if k not in ("num_active_range",)},
        },
    )


@configclass
class TactileShapePickEnvCfg(base.TactileSpherePickEnvCfg):
    scene: ShapePickSceneCfg = ShapePickSceneCfg(num_envs=512, env_spacing=2.0)
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
