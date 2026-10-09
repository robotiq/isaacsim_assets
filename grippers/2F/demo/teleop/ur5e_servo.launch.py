"""
Bring up MoveIt Servo for a UR5e whose joint states come from Isaac Sim.

Starts: robot_state_publisher (UR5e URDF), servo_node, and the
Servo->Isaac /joint_command bridge.

Does NOT start move_group, RViz, or ros2_control — Servo only needs
URDF + SRDF + kinematics + /joint_states, which Isaac already provides.

Runs on both ROS 2 Humble and Jazzy. Everywhere the two diverge — the servo
executable name, the kinematics.yaml layout, the planning-group name, which
parameters Servo declares, and how idle output is suppressed — is resolved at
runtime by _servo_compat, which interrogates the installed packages rather than
trusting ROS_DISTRO. See that module for why.
"""
import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# `ros2 launch <path>` does not put the launch file's own directory on sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _servo_compat as compat  # noqa: E402


def _load_yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


TELEOP_SCENE = "ur5robot_with_2F-85.usda"


def _scene_tick_hz(default=60.0):
    """The scene's timeCodesPerSecond -- the rate Isaac applies commands at.

    Isaac uses fixed time stepping: one time code per rendered frame. So this
    is the ceiling on how often a /joint_command can actually take effect, and
    the rate Servo should be paced to.

    Read from the scene rather than hardcoded, so the two cannot drift apart
    and nobody has to remember to override anything. Falls back to 60 if the
    scene moves or the metadata is missing -- a wrong-but-sane rate is much
    better than failing to launch.
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), TELEOP_SCENE)
    try:
        with open(path) as fh:
            for _, line in zip(range(200), fh):      # it lives in the header
                if "timeCodesPerSecond" in line:
                    return float(line.split("=", 1)[1].strip())
    except Exception:
        pass
    return default


# Isaac applies /joint_command once per tick, so pacing Servo there rather than
# at upstream's 250 Hz removes ~4.5x of redundant work per tick.
DEFAULT_PUBLISH_PERIOD = float(
    os.environ.get("SERVO_PUBLISH_PERIOD", 1.0 / _scene_tick_hz()))


def build_servo_context(ur_type="ur5e", publish_period=DEFAULT_PUBLISH_PERIOD):
    """Assemble every parameter servo_node needs, with no launch machinery.

    Split out from generate_launch_description so smoke_launch_check.py can
    inspect the resolved configuration directly — launch's Node action keeps
    its parameters in private state, and a test that reached into that would
    be checking an implementation detail rather than the configuration.
    """
    robot_description = {
        "robot_description": compat.run_xacro(
            "ur_description",
            "urdf/ur.urdf.xacro",
            name=ur_type,
            ur_type=ur_type,
            tf_prefix="",
            safety_limits="true",
            safety_pos_margin="0.15",
            safety_k_position="20",
        )
    }

    srdf_xml = compat.run_xacro(
        "ur_moveit_config", "srdf/ur.srdf.xacro", name=ur_type, prefix=""
    )
    robot_description_semantic = {"robot_description_semantic": srdf_xml}

    # Read the planning-group name out of the SRDF we just generated instead of
    # assuming it. Jazzy's ur_macro.srdf.xacro hardcodes "ur_manipulator" and
    # takes no `name` parameter at all; Humble's interpolated it into
    # "ur5e_manipulator". Asking the SRDF works on both and on anything built
    # from source.
    move_group = compat.srdf_group_name(srdf_xml)

    # kinematics.yaml is keyed "ur_manipulator" on both distros, even on Humble
    # where the SRDF group is "ur5e_manipulator" — so re-key its sole entry onto
    # whatever the SRDF actually calls the group.
    raw_kin = compat.unwrap_kinematics(_load_yaml("ur_moveit_config", "config/kinematics.yaml"))
    kin_entry = raw_kin[move_group] if move_group in raw_kin else next(iter(raw_kin.values()))
    kinematics_yaml = {"robot_description_kinematics": {move_group: kin_entry}}

    joint_limits_yaml = {
        "robot_description_planning": _load_yaml("ur_moveit_config", "config/joint_limits.yaml")
    }

    servo_yaml = _load_yaml("ur_moveit_config", "config/ur_servo.yaml")

    servo_yaml["move_group_name"] = move_group
    servo_yaml["is_primary_planning_scene_monitor"] = True
    # Isaac publishes 14 joints on /joint_states (6 UR5e arm + 8 2F-85 finger); the
    # gripper joints aren't in the UR URDF and break the CurrentStateMonitor.
    # The arm filter republishes only the 6 arm joints on /joint_states_arm.
    servo_yaml["joint_topic"] = "/joint_states_arm"
    # The ur_servo.yaml ships without this key, and the ROS param default is
    # 0.0 — every outgoing command gets multiplied by zero. Has to be 1.0.
    servo_yaml["override_velocity_scaling_factor"] = 1.0
    # Default ur_servo.yaml caps angular at 0.3 rad/s (~17°/s) — too slow for
    # interactive teleop. 3.0 rad/s ≈ 170°/s feels right paired with the
    # gamepad's SPEED_ANGULAR=3.0, so full stick deflection actually reaches
    # the cap instead of being clipped well below it.
    servo_yaml["scale"]["rotational"] = 3.0
    # Pace Servo to the simulator instead of to the wall clock.
    #
    # Upstream is 0.004 (250 Hz). Isaac applies /joint_command once per tick,
    # ~55 Hz on a healthy session, so 250 Hz produced ~4.5 commands per tick of
    # which only the last had any effect. The surplus is not free: each one
    # costs a Servo solve and smoothing-filter pass, a DDS round trip, a
    # conversion in the Python servo_to_isaac_bridge, and Action Graph work
    # inside Isaac -- all of it while the sim is trying to render.
    #
    # This is NOT fixed by use_sim_time. That governs message STAMPS, which is
    # what keeps Servo's stale-message filter from dropping input (gotcha #1).
    # Servo's control loop itself is paced by a wall-clock rate, so it keeps
    # emitting at 250 Hz however slowly the sim runs -- at RTF 0.45 that was
    # ~550 commands per simulated second against a configured 250.
    #
    # Arm speed is unaffected: Servo integrates velocity over publish_period,
    # so a longer period yields proportionally larger steps along the same
    # trajectory -- fewer, bigger increments, not slower motion.
    #
    # Override by passing publish_period= to build_servo_context(), or set
    # SERVO_PUBLISH_PERIOD in the environment.
    servo_yaml["publish_period"] = publish_period
    # MoveIt Servo's collision monitor segfaults inside FCL on some
    # self-collision states during teleop (CollisionCheck::run → FCL
    # registerObjects → SIGSEGV), which kills the whole servo_node and the
    # arm stops responding. This is a sim, so disable the monitor.
    servo_yaml["check_collisions"] = False

    # Stops Servo republishing a lagging filtered state while idle, which
    # otherwise drifts the arm into a singularity and emits NaN (README.md,
    # "Servo drift loop"). The two distros spell this differently; compat picks
    # the one the installed Servo declares and refuses to launch without either.
    servo_yaml = compat.apply_idle_protection(servo_yaml)
    # ur_moveit_config still ships a Humble-era ur_servo.yaml whose older keys
    # the rewritten Servo no longer declares. Undeclared overrides are silently
    # ignored, so leaving them is harmless but misleading — drop them so this
    # config reflects what Servo actually reads. No-op on Humble.
    servo_yaml = compat.filter_servo_config(servo_yaml)

    return {
        "move_group": move_group,
        "srdf_xml": srdf_xml,
        "servo_yaml": servo_yaml,
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "kinematics_yaml": kinematics_yaml,
        "joint_limits_yaml": joint_limits_yaml,
    }


def generate_launch_description():
    ctx = build_servo_context()

    servo_params = {"moveit_servo": ctx["servo_yaml"]}

    # Isaac Sim publishes /clock; without use_sim_time, header stamps coming
    # from any wall-time publisher are billions of seconds ahead of Servo's
    # internal clock and the stale-message filter drops them silently.
    sim_time = {"use_sim_time": True}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[ctx["robot_description"], sim_time],
    )

    servo_node = Node(
        package="moveit_servo",
        executable=compat.servo_executable(),
        output="screen",
        parameters=[
            servo_params,
            ctx["robot_description"],
            ctx["robot_description_semantic"],
            ctx["kinematics_yaml"],
            ctx["joint_limits_yaml"],
            sim_time,
        ],
    )

    here = os.path.dirname(os.path.abspath(__file__))

    bridge_proc = ExecuteProcess(
        cmd=[
            "python3", os.path.join(here, "servo_to_isaac_bridge.py"),
            "--ros-args", "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    arm_filter_proc = ExecuteProcess(
        cmd=[
            "python3", os.path.join(here, "joint_states_arm_filter.py"),
            "--ros-args", "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    # Reseeds Servo whenever the Isaac sim is stopped and replayed (detected via
    # a /clock jump-back), so teleop resumes from the reset pose instead of
    # snapping the arm back toward its pre-stop setpoint. Uses wall clock (no
    # use_sim_time) so it can observe the sim clock jumping backwards.
    reset_watchdog_proc = ExecuteProcess(
        cmd=[
            "python3", os.path.join(here, "sim_reset_watchdog.py"),
        ],
        output="screen",
    )

    # NOTE: nothing here selects Servo's command type. On Jazzy that selection
    # is required, but it belongs to whichever frontend is publishing — each one
    # knows whether it is sending twists or joint jogs, and a launch-time choice
    # would fight the keyboard frontend's mode switching. See gamepad_teleop.py
    # and servo_keyboard_teleop.py, both of which call
    # compat.switch_command_type().
    return LaunchDescription(
        [rsp_node, arm_filter_proc, servo_node, bridge_proc, reset_watchdog_proc]
    )
