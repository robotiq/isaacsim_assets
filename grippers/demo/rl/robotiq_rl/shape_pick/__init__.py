"""Finding and recognising shape-sorter pieces in a box with the TSF-85 tactile fingertips."""

import gymnasium as gym

from . import agents

gym.register(
    id="Robotiq-Tactile-Shape-Pick-UR5e-2F85",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:TactileShapePickEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:ShapePickPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
