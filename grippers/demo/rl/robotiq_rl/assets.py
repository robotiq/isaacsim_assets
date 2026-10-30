"""Isaac Lab articulation configs for the Robotiq demo robots."""

import math
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import replace
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

ASSETS_DIR = Path(__file__).resolve().parents[1] / "assets"

UR5E_2F85_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(ASSETS_DIR / "ur5e_2F-85.usda"),
        rigid_props=PhysxRigidBodyCfg(disable_gravity=False, max_depenetration_velocity=5.0),
        articulation_props=PhysxArticulationCfg(
            enabled_self_collisions=False, solver_position_iteration_count=16, solver_velocity_iteration_count=1
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        # the teleop scene's start pose (shoulder_lift -75, elbow 110), wrist pointing down
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": math.radians(-75.0),
            "elbow_joint": math.radians(110.0),
            "wrist_1_joint": math.radians(-134.5),
            "wrist_2_joint": math.radians(-94.5),
            "wrist_3_joint": math.radians(-34.0),
            "finger_joint": 0.0,
        },
    ),
    actuators={
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*"], stiffness=1320.0, damping=72.66, friction=0.0, armature=0.0
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow_joint"], stiffness=600.0, damping=34.64, friction=0.0, armature=0.0
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_.*"], stiffness=216.0, damping=29.39, friction=0.0, armature=0.0
        ),
        # the other gripper joints are passive and follow finger_joint through PhysX mimic joints
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["finger_joint"], effort_limit_sim=26.0, stiffness=3.0, damping=0.05
        ),
    },
)
"""UR5e with a Robotiq 2F-85 welded to the flange (PhysX)."""

UR5E_2F85_TSF85_CFG = replace(
    UR5E_2F85_CFG, spawn=replace(UR5E_2F85_CFG.spawn, usd_path=str(ASSETS_DIR / "ur5e_2F-85_tsf85.usda"))
)
"""UR5e with a Robotiq 2F-85 on TSF-85 tactile fingertips (EffectiveResponse sensor model)."""
