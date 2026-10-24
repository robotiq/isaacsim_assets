"""UR5e + 2F-85 with TSF-85 tactile fingertips picking spheres out of an open box, blind.

Scene only for now: the MDP is a placeholder (joint-position actions, joint observations, no task reward)."""

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass, replace
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.visualizers import VisualizerCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg

from ..assets import UR5E_2F85_TSF85_CFG

NUM_SPHERES = 10
SPHERE_RADIUS = 0.02  # m: 4 cm balls, well inside the 2F-85's 85 mm stroke
SPHERE_MASS = 0.03  # kg

# Open-top box on the table (table top at z = 0): inner footprint BOX_INNER x BOX_INNER, walls BOX_WALL thick.
BOX_CENTER = (0.5, 0.0)
BOX_INNER = 0.26
BOX_HEIGHT = 0.10
BOX_WALL = 0.012
BOX_FLOOR = 0.012
_CX, _CY = BOX_CENTER
_OUTER = BOX_INNER + 2 * BOX_WALL
_WALL_Z = BOX_FLOOR + BOX_HEIGHT / 2
_WALL_OFFSET = (BOX_INNER + BOX_WALL) / 2


def _box_part(name: str, size, pos) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Box/{name}",
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos),
        spawn=sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            physics_material=sim_utils.RigidBodyMaterialBaseCfg(static_friction=0.6, dynamic_friction=0.6),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.45, 0.28), roughness=0.7),
        ),
    )


def _sphere(i: int) -> RigidObjectCfg:
    # a jittered 4-3-3 grid dropped from just above the box floor; reset events re-jitter it
    row, col = (0, i) if i < 4 else (1, i - 4) if i < 7 else (2, i - 7)
    pitch = (BOX_INNER - 2 * SPHERE_RADIUS) / 3
    x = BOX_CENTER[0] - (BOX_INNER / 2 - SPHERE_RADIUS) + row * pitch * 1.3
    y = BOX_CENTER[1] - (BOX_INNER / 2 - SPHERE_RADIUS) + col * pitch
    z = BOX_FLOOR + SPHERE_RADIUS + 0.002 + 0.01 * (i % 3)
    hue = i / NUM_SPHERES
    colour = tuple(0.15 + 0.75 * abs(((hue * 6 + k) % 6) - 3) / 3 for k in (0, 4, 2))
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Sphere_{i}",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, z)),
        spawn=sim_utils.SphereCfg(
            radius=SPHERE_RADIUS,
            rigid_props=PhysxRigidBodyCfg(
                solver_position_iteration_count=16, solver_velocity_iteration_count=1, max_depenetration_velocity=2.0
            ),
            mass_props=sim_utils.MassCfg(mass=SPHERE_MASS),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            physics_material=sim_utils.RigidBodyMaterialBaseCfg(static_friction=0.9, dynamic_friction=0.8),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=colour),
        ),
    )


@configclass
class TactilePickSceneCfg(InteractiveSceneCfg):
    robot = replace(UR5E_2F85_TSF85_CFG, prim_path="{ENV_REGEX_NS}/Robot")

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0, 0, 0.707, 0.707]),
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/light", spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0)
    )

    # box: floor + four walls
    box_floor = _box_part("floor", (_OUTER, _OUTER, BOX_FLOOR), (_CX, _CY, BOX_FLOOR / 2))
    box_wall_n = _box_part("wall_n", (BOX_WALL, _OUTER, BOX_HEIGHT), (_CX + _WALL_OFFSET, _CY, _WALL_Z))
    box_wall_s = _box_part("wall_s", (BOX_WALL, _OUTER, BOX_HEIGHT), (_CX - _WALL_OFFSET, _CY, _WALL_Z))
    box_wall_e = _box_part("wall_e", (BOX_INNER, BOX_WALL, BOX_HEIGHT), (_CX, _CY + _WALL_OFFSET, _WALL_Z))
    box_wall_w = _box_part("wall_w", (BOX_INNER, BOX_WALL, BOX_HEIGHT), (_CX, _CY - _WALL_OFFSET, _WALL_Z))

    # spheres: sphere_0 .. sphere_9 (attributes set below, the config class needs them named)
    sphere_0 = _sphere(0)
    sphere_1 = _sphere(1)
    sphere_2 = _sphere(2)
    sphere_3 = _sphere(3)
    sphere_4 = _sphere(4)
    sphere_5 = _sphere(5)
    sphere_6 = _sphere(6)
    sphere_7 = _sphere(7)
    sphere_8 = _sphere(8)
    sphere_9 = _sphere(9)


@configclass
class ActionsCfg:
    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        scale=0.5,
        use_default_offset=True,
    )
    gripper_action = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["finger_joint"],
        open_command_expr={"finger_joint": 0.0},
        close_command_expr={"finger_joint": 0.8},
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class RewardsCfg:
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class TactileSpherePickEnvCfg(ManagerBasedRLEnvCfg):
    scene: TactilePickSceneCfg = TactilePickSceneCfg(num_envs=64, env_spacing=2.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysxCfg(bounce_threshold_velocity=0.01)
        self.sim.visualizer_cfgs = VisualizerCfg(eye=(1.15, -0.75, 0.75), lookat=(0.5, 0.0, 0.1))
