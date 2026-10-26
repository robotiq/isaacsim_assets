"""UR5e + 2F-85 with TSF-85 tactile fingertips picking spheres out of an open box, blind (see mdp.py)."""

import math

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
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
from isaaclab.utils.noise import GaussianNoiseCfg as Gnoise
from isaaclab.visualizers import VisualizerCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg

from robotiq_tactile_sensors.isaaclab import TactileSensorCfg, tactile_map

from ..assets import UR5E_2F85_TSF85_CFG
from . import mdp as tp

# The lift task's start pose with the tool tip 15 cm higher and the same orientation (UR5e inverse kinematics:
# only shoulder_lift, elbow and wrist_1 change).
START_JOINT_POS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": math.radians(-83.41),
    "elbow_joint": math.radians(96.17),
    "wrist_1_joint": math.radians(-112.26),
    "wrist_2_joint": math.radians(-94.5),
    "wrist_3_joint": math.radians(-34.0),
    "finger_joint": 0.0,
}

NUM_SPHERES = 10
SPHERE_RADIUS = 0.02  # m: 4 cm balls, well inside the 2F-85's 85 mm stroke
SPHERE_MASS = 0.03  # kg

# Open-top box on the table (table top at z = 0): inner footprint BOX_INNER x BOX_INNER, walls BOX_WALL thick.
BOX_CENTER = (0.5, 0.0)
BOX_INNER = 0.36
BOX_HEIGHT = 0.10
BOX_WALL = 0.012
BOX_FLOOR = 0.012
# Where the tool tip can go with the box walls in the way: the gripper base (75 x 85 mm) is inside the box at pad
# height and the open fingers span ~125 mm across the jaws (y). The spheres are placed inside this zone only.
REACH_HALF = (BOX_INNER / 2 - 0.0475, BOX_INNER / 2 - 0.075)  # (x, y) half extents around the box centre
_CX, _CY = BOX_CENTER
_OUTER = BOX_INNER + 2 * BOX_WALL
_WALL_Z = BOX_FLOOR + BOX_HEIGHT / 2
_WALL_OFFSET = (BOX_INNER + BOX_WALL) / 2


def _box_part(name: str, size, pos) -> RigidObjectCfg:
    # kinematic rigid bodies, not static colliders: the pads' contact sensors can only filter on rigid bodies
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Box/{name}",
        init_state=RigidObjectCfg.InitialStateCfg(pos=pos),
        spawn=sim_utils.CuboidCfg(
            size=size,
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
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
    robot = replace(
        UR5E_2F85_TSF85_CFG,
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=replace(UR5E_2F85_TSF85_CFG.init_state, joint_pos=START_JOINT_POS),
        spawn=replace(
            UR5E_2F85_TSF85_CFG.spawn,
            activate_contact_sensors=True,  # the pads' contact reports feed the tactile maps
            rigid_props=replace(UR5E_2F85_TSF85_CFG.spawn.rigid_props, disable_gravity=True),  # IK-driven arm
        ),
    )

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

    # tactile: both pads' force on the spheres and on the box, as 7 x 4 taxel maps (grippers/tactile_sensors)
    tactile = TactileSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/wrist_3_link/Robotiq_2F_85_edit/Robotiq_2F_85",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Sphere_{i}" for i in range(NUM_SPHERES)]
        + [f"{{ENV_REGEX_NS}}/Box/{part}" for part in ("floor", "wall_n", "wall_s", "wall_e", "wall_w")],
    )




# ---------------------------------------------------------------------------------------------------------------
# task constants shared by the reward terms (all in the env frame, metres)
# ---------------------------------------------------------------------------------------------------------------

TCP_OFFSET = (0.0, 0.0, 0.13)  # point between the pads, in the wrist_3_link frame
RIM_Z = BOX_FLOOR + BOX_HEIGHT
TASK = {
    "body_name": "wrist_3_link",
    "joint_name": "finger_joint",
    "tcp_offset": TCP_OFFSET,
    "closed_pos": 0.8,
    "box_center": BOX_CENTER,
    "half_xy": REACH_HALF,  # the zone the tool tip can reach: swept for coverage, "inside" for the entry reward
    "rim_z": RIM_Z,
    "search_z": BOX_FLOOR + SPHERE_RADIUS + 0.012,  # tool tip height while searching: pads clear of the floor (tips 19 mm below)
    "rest_z": BOX_FLOOR + SPHERE_RADIUS,
    "out_margin": 0.07,  # success: the sphere centre this far above the rim (the supervisor waits for the tool tip 6 cm above it)
    "touch_n": 0.2,  # N: pad force that counts as a touch
    "hold_dist": 0.03,  # m: sphere centre within this of the TCP, with both pads pressing
    "min_closure": 0.25,
    "out_half_xy": None,  # no limit on where the sphere is held: the supervisor only needs it above the rim
    "box_inner_half": BOX_INNER / 2,
    # after a successful pick the sphere is parked outside and the arm is put at a random point like this, gripper open
    "home_ranges": {"x_range": (0.34, 0.68), "y_range": (-0.26, 0.26), "z_range": (0.17, 0.33)},
    "gentle_speed": 0.15,  # m/s: spheres moving faster than this (other than the one held) are penalised
    "out_band": 1.0,  # m: the sphere counts as out at any height above the success height (a pick completes after 10 steps)
    "gamma": 0.995,
    "shaping_weights": {"approach": 0.2, "enter": 1.0, "touch": 1.0, "grasp": 1.0, "stable": 1.0, "lift": 5.0},
    "lift_extra": 0.06,  # m: the lift reward reaches its maximum this far above the success height
    "success_steps": 10,  # steps with the held sphere out of the box = a pick: what the supervisor (scripts/pick_all.py) waits for
    "stable_steps": 8,  # held this many policy steps in a row = a stable grasp
}


@configclass
class ActionsCfg:
    # tool tip moves in Cartesian space (relative, 3 translations + 3 rotations whose scale is 0: orientation is held)
    arm_action = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        body_name="wrist_3_link",
        controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
        scale=(0.01, 0.01, 0.01, 0.0, 0.0, 0.0),  # m per policy step at full action (0.3 m/s)
        body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=TCP_OFFSET),
    )
    # continuous gripper target: action -1 .. 1 -> finger_joint 0 (open) .. 0.8 rad (closed)
    gripper_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["finger_joint"],
        scale=0.4,
        offset=0.4,
        use_default_offset=False,
        clip={"finger_joint": (0.0, 0.8)},
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """What the robot can feel: proprioception, the gripper opening and the two tactile maps."""

        # noise: a simulated pad is cleaner than a real one, and the policy must not lean on exact values
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Gnoise(mean=0.0, std=0.003))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, scale=0.1, noise=Gnoise(mean=0.0, std=0.002))
        tcp = ObsTerm(func=tp.tcp_in_box_frame, params={"cfg": TASK}, noise=Gnoise(mean=0.0, std=0.002))
        tactile = ObsTerm(func=tactile_map, noise=Gnoise(mean=0.0, std=0.03))  # on log(1 + N): ~3 % of a 1 N touch
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 3

    @configclass
    class CriticCfg(ObsGroup):
        """Privileged: where the spheres are, and the phase flags."""

        privileged = ObsTerm(func=tp.privileged_obs, params={"cfg": TASK})

        def __post_init__(self):
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")
    # start with the tool tip anywhere above and around the box (above the rim), pointing down: it has to find the way in
    reset_arm = EventTerm(
        func=tp.reset_arm_start,
        mode="reset",
        params={"x_range": (0.34, 0.68), "y_range": (-0.26, 0.26), "z_range": (0.17, 0.33)},
    )
    reset_spheres = EventTerm(
        func=tp.reset_spheres_in_box,
        mode="reset",
        params={
            "box_center": BOX_CENTER,
            "half_xy": (BOX_INNER / 2 - SPHERE_RADIUS - 0.004,) * 2,  # spread over the whole floor ...
            "zone_half_xy": REACH_HALF,  # ... with sphere 0 where the tool tip can reach
            "num_active_range": (3, NUM_SPHERES),  # a random number of spheres is in the box, the others are parked
            "p_zone_only": 0.5,  # half of the boxes have every sphere within reach
            "floor_z": BOX_FLOOR,
            "radius": SPHERE_RADIUS,
        },
    )
    reset_state = EventTerm(func=tp.reset_task_state, mode="reset", params={"cfg": TASK})

    def __post_init__(self):
        # sphere mass 15 .. 60 g (per sphere and env, once at start-up)
        for i in range(NUM_SPHERES):
            setattr(
                self,
                f"mass_sphere_{i}",
                EventTerm(
                    func=mdp.randomize_rigid_body_mass,
                    mode="startup",
                    params={
                        "asset_cfg": SceneEntityCfg(f"sphere_{i}"),
                        "mass_distribution_params": (0.5, 2.0),
                        "operation": "scale",
                    },
                ),
            )


@configclass
class RewardsCfg:
    # Progress through the phases (approach, enter, touch, grasp, stable, lift) as ONE potential-based term: it pays for
    # getting further but not for staying put. Per-step rewards for being in a phase were farmed by hovering just under
    # the success height: finishing a pick resets the hold, and 5 per step beats 100 per pick cycle.
    shaping = RewTerm(func=tp.r_shaping, weight=1.0, params={"cfg": TASK})
    coverage = RewTerm(func=tp.r_coverage, weight=1.0, params={"cfg": TASK})  # new floor cells swept (an event)
    success = RewTerm(func=tp.r_success, weight=100.0, params={"cfg": TASK})  # per sphere picked
    drop = RewTerm(func=tp.r_drop, weight=-2.0, params={"cfg": TASK})
    # gentleness: knocking a sphere out of the reachable zone (to a wall, where it cannot be picked) costs more than the
    # touch reward earns, and so does shoving spheres around fast
    disturb = RewTerm(func=tp.r_disturb, weight=-15.0, params={"cfg": TASK})
    speed = RewTerm(func=tp.r_speed, weight=-1.0, params={"cfg": TASK})
    # regularisation
    wall = RewTerm(func=tp.r_wall, weight=-0.005, params={"cfg": TASK})
    action_rate = RewTerm(func=tp.action_rate_used, weight=-0.002, params={"dims": (0, 1, 2, 6)})
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-1e-3, params={"asset_cfg": SceneEntityCfg("robot")})


@configclass
class TerminationsCfg:
    # No termination on success: it would forfeit the per-step reward of holding the sphere out (the policy then avoids
    # finishing). Holding out until the time-out is the goal; success is counted by the one-time bonus and the eval.
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class TactileSpherePickEnvCfg(ManagerBasedRLEnvCfg):
    scene: TactilePickSceneCfg = TactilePickSceneCfg(num_envs=512, env_spacing=2.0)
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 4  # 30 Hz policy
        self.episode_length_s = 90.0  # several picks: each success parks the sphere and the episode goes on
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysxCfg(bounce_threshold_velocity=0.01, gpu_found_lost_pairs_capacity=2**23)
        self.sim.visualizer_cfgs = VisualizerCfg(eye=(1.15, -0.75, 0.75), lookat=(0.5, 0.0, 0.1))
