"""Blind sphere picking from a box with the TSF-85 tactile fingertips."""

import gymnasium as gym

from . import agents

gym.register(
    id="Robotiq-Tactile-Sphere-Pick-UR5e-2F85",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:TactileSpherePickEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:TactilePickPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
