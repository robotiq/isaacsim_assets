"""UR5e + Robotiq 2F-85 cube lift."""

import gymnasium as gym

from . import agents

gym.register(
    id="Robotiq-Lift-Cube-UR5e-2F85",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:UR5e2F85CubeLiftEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}:UR5e2F85LiftPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
