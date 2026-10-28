"""Isaac Lab (PhysX) integration of the simulated TSF tactile sensors.

    from robotiq_tactile_sensors.isaaclab import (
        ROBOTIQ_2F_85, ROBOTIQ_2F_140, ROBOTIQ_2F85_TSF85_CFG, TactileSensorCfg, tactile_map,
    )

    @configclass
    class MySceneCfg(InteractiveSceneCfg):
        robot = ...                                       # a robot with a 2F-85 / 2F-140 on TSF fingertips
        tactile = TactileSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.../Robotiq_2F_85",      # the prim that holds left_fingertip, right_fingertip
            mount=ROBOTIQ_2F_140,                                    # only for another gripper than the 2F-85 (mounts.py)
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Object"],        # what the pads can feel
        )

    obs = ObsTerm(func=tactile_map, params={"sensor_cfg": SceneEntityCfg("tactile")})     # (num_envs, 56)

Only the light config classes are imported here, so this stays importable before Isaac Sim starts; the sensor itself
(``sensor.py``) is imported by Isaac Lab when the scene is built.
"""

from ..mounts import ROBOTIQ_2F_85, ROBOTIQ_2F_140
from .cfg import TactileSensorCfg
from .grippers import grippers_dir, robotiq_2f85_tsf85_cfg, robotiq_2f140_tsf140_cfg
from .mdp import tactile_map

__all__ = [
    "TactileSensorCfg",
    "tactile_map",
    "ROBOTIQ_2F_85",
    "ROBOTIQ_2F_140",
    "ROBOTIQ_2F85_TSF85_CFG",
    "ROBOTIQ_2F140_TSF140_CFG",
    "robotiq_2f85_tsf85_cfg",
    "robotiq_2f140_tsf140_cfg",
    "grippers_dir",
]


def __getattr__(name):
    # the robot configs are built on first use: they need the gripper USD files (see grippers.py)
    if name in ("ROBOTIQ_2F85_TSF85_CFG", "ROBOTIQ_2F140_TSF140_CFG"):
        from . import grippers

        return getattr(grippers, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
