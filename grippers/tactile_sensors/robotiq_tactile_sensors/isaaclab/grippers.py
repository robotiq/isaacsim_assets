"""Ready-made Isaac Lab robot configs of the Robotiq grippers with TSF fingertips (the gripper alone)."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

GRIPPERS_DIR = Path(__file__).resolve().parents[3]  # .../grippers, the folder that holds the gripper assets

ROBOTIQ_2F85_TSF85_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(GRIPPERS_DIR / "Robotiq_2F_85" / "Robotiq_2F_85.usda"),
        # the tactile fingertips, the TSF pad collider and the pad contact reports of the asset
        variants={"Fingertip": "Tactile_TSF85", "SensorSim": "EffectiveResponse"},
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(joint_pos={"finger_joint": 0.0}),
    actuators={
        # the other gripper joints follow finger_joint through PhysX mimic joints
        "gripper": ImplicitActuatorCfg(joint_names_expr=["finger_joint"], effort_limit_sim=26.0, stiffness=3.0, damping=0.05),
    },
)
"""The 2F-85 on TSF-85 fingertips, alone (its own articulation root, fixed base). Position it with ``init_state.pos/rot``;
to mount it on an arm, weld its ``base_link`` to the flange as in grippers/demo/rl/assets/ur5e_2F-85.usda. The pad
bodies are then at ``<prim>/Robotiq_2F_85/{left,right}_fingertip``: give that prim as ``TactileSensorCfg.prim_path``.
The gripper joint goes 0 (open) to about 0.8 rad (closed)."""
