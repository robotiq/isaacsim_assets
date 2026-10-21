"""UR5e + Robotiq 2F-85 cube lift (PhysX), on Isaac Lab's stock lift MDP."""

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass, replace
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg
from isaaclab_tasks.contrib.lift import mdp
from isaaclab_tasks.contrib.lift.lift_env_cfg import LiftEnvCfg

from ..assets import UR5E_2F85_CFG

# Distance from wrist_3_link to the point between the fingertips, along the tool axis.
# ESTIMATE: measure it in the composed stage (pad centre in the wrist_3_link frame) before training.
TCP_OFFSET = (0.0, 0.0, 0.17)

# 2F-85 driver joint: 0 = open, ~0.8 rad = closed
FINGER_OPEN, FINGER_CLOSED = 0.0, 0.8


@configclass
class UR5e2F85CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = replace(UR5E_2F85_CFG, prim_path="{ENV_REGEX_NS}/Robot")

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*_joint"], scale=0.5, use_default_offset=True
        )
        # `.*_joint` also matches finger_joint: restrict the arm term to the six UR joints
        self.actions.arm_action.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger_joint"],
            open_command_expr={"finger_joint": FINGER_OPEN},
            close_command_expr={"finger_joint": FINGER_CLOSED},
        )
        self.commands.object_pose.body_name = "wrist_3_link"

        # 5 cm cube on the table; the teleop scene's props are of the same size
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.025], rot=[0, 0, 0, 1]),
            spawn=sim_utils.CuboidCfg(
                size=(0.05, 0.05, 0.05),
                rigid_props=PhysxRigidBodyCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_depenetration_velocity=5.0,
                ),
                mass_props=sim_utils.MassCfg(mass=0.1),
                collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                physics_material=sim_utils.RigidBodyMaterialBaseCfg(static_friction=1.0, dynamic_friction=1.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1)),
            ),
        )

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/(Geometry/)?base_link",
            debug_vis=False,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/(Geometry/.*/)?wrist_3_link",
                    name="end_effector",
                    offset=OffsetCfg(pos=TCP_OFFSET),
                ),
            ],
        )
