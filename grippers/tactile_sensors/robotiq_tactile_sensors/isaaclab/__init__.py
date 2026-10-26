"""Isaac Lab (PhysX) integration of the simulated TSF tactile sensors.

    from robotiq_tactile_sensors.isaaclab import TactileSensorCfg, tactile_map, ROBOTIQ_2F85_TSF85_CFG

    @configclass
    class MySceneCfg(InteractiveSceneCfg):
        robot = ...                                       # a robot with a 2F-85 on Tactile_TSF85 fingertips
        tactile = TactileSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.../Robotiq_2F_85",      # the prim that holds left_fingertip, right_fingertip
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Object"],        # what the pads can feel
        )

    obs = ObsTerm(func=tactile_map, params={"sensor_cfg": SceneEntityCfg("tactile")})     # (num_envs, 56)

Only the light config classes are imported here, so this stays importable before Isaac Sim starts; the sensor itself
(``sensor.py``) is imported by Isaac Lab when the scene is built.
"""

from .cfg import TactileSensorCfg
from .grippers import ROBOTIQ_2F85_TSF85_CFG
from .mdp import tactile_map

__all__ = ["TactileSensorCfg", "tactile_map", "ROBOTIQ_2F85_TSF85_CFG"]
