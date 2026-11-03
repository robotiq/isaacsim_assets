"""UR5e + 2F-85 (TSF-85 fingertips) with shape-sorter pieces in the box instead of spheres.

Everything else (box, arm, sensors, rewards, PPO settings) is the tactile pick's; the objects keep the names ``sphere_i``
so that its code works unchanged. There are 18 piece slots of fixed types (``SLOT_TYPES``); every reset puts a random subset
of 8 - 16 of them in the box (types equally likely), each dropped with a random orientation so it settles on a face of its
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

# The tactile pick's task constants with one change: a grasped piece's centre is farther from the tool point than a sphere's (the pieces are
# up to 6.6 cm wide and held anywhere along the pads; measured while both pads press a piece: median 3.5 cm, p90 4.1 cm), so the "held" test
# of the rewards needs a larger distance than the 3 cm of the spheres. With 3 cm the policy was rewarded for only 6 % of its grasps.
SHAPE_TASK = {
    **base.TASK,
    "hold_dist": 0.055,
    # and a reward for holding the piece near the middle of the pads: a centred grasp is a better one and gives a cleaner tactile map to recognise it from
    "shaping_weights": {**base.TASK["shaping_weights"], "centre": 2.0},
}

ASSETS = Path(__file__).resolve().parents[2] / "assets" / "shape_sorter"
PIECES = ["rectangle", "pentagon", "cross", "cylinder", "star", "square"]  # the class names, in class order
NUM_SLOTS = 18  # three pieces of each type; every reset puts 8 to 16 of them in the box
SLOT_TYPES = [i % len(PIECES) for i in range(NUM_SLOTS)]  # piece type of each slot


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
    sphere_10 = _piece(10)
    sphere_11 = _piece(11)
    sphere_12 = _piece(12)
    sphere_13 = _piece(13)
    sphere_14 = _piece(14)
    sphere_15 = _piece(15)
    sphere_16 = _piece(16)
    sphere_17 = _piece(17)

    # the tactile sensor filters on every piece, then on the box parts (the order ``num_objects`` relies on)
    tactile = replace(
        base.TactilePickSceneCfg(num_envs=1, env_spacing=2.0).tactile,
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Sphere_{i}" for i in range(NUM_SLOTS)]
        + [f"{{ENV_REGEX_NS}}/Box/{part}" for part in ("floor", "wall_n", "wall_s", "wall_e", "wall_w")],
    )


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
            "num_active_range": (8, 16),
            **{k: v for k, v in base.EventCfg().reset_spheres.params.items() if k not in ("num_active_range", "p_zone_only")},
            "p_zone_only": 0.0,  # 16 pieces do not fit in the tool tip's reach: they are spread over the whole floor
        },
    )


@configclass
class TactileShapePickEnvCfg(base.TactileSpherePickEnvCfg):
    scene: ShapePickSceneCfg = ShapePickSceneCfg(num_envs=512, env_spacing=2.0)
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        super().__post_init__()
        # every term that shares the task constants gets the shape task's
        for group in (self.rewards, self.observations.policy, self.observations.critic, self.events):
            for term in vars(group).values():
                params = getattr(term, "params", None)
                if isinstance(params, dict) and isinstance(params.get("cfg"), dict) and params["cfg"].get("body_name") == base.TASK["body_name"]:
                    params["cfg"] = SHAPE_TASK
