"""
Compatibility shims for MoveIt Servo across ROS 2 Humble and Jazzy.

Every place where the two distros diverge lives here, so the launch file and
the teleop frontends stay readable and carry no version checks of their own.

Every check interrogates the installed `moveit_servo` rather than reading
`ROS_DISTRO`: the thing that actually differs is the package version, and a
from-source build on either distro makes the distro string a lie. See
README.md, "Servo drift loop when idle output isn't suppressed", for why the
obvious signals (the shipped `ur_servo.yaml`, `incoming_command_timeout`) are
no guide.
"""
import os
import re
import subprocess
from functools import lru_cache

from ament_index_python.packages import get_package_prefix, get_package_share_directory

# Humble's moveit_servo installs this; Jazzy's installs "servo_node".
_OLD_EXECUTABLE = "servo_node_main"
_NEW_EXECUTABLE = "servo_node"

# Declared only by the pre-rewrite (Humble) Servo.
_OLD_IDLE_KEY = "low_latency_mode"
# Declared only by the rewritten (Jazzy / MoveIt 2.12+) Servo.
_NEW_IDLE_KEYS = ("halt_all_joints_in_cartesian_mode", "halt_all_joints_in_joint_mode")

# How long Servo waits for input before halting. gamepad.launch.py's
# autorepeat_rate has to stay comfortably inside this.
INCOMING_COMMAND_TIMEOUT = 0.1


@lru_cache(maxsize=1)
def declared_servo_params():
    """Names of the parameters moveit_servo declares, or None if unknowable.

    Jazzy's moveit_servo uses generate_parameter_library, so its declarations
    are readable from share/moveit_servo/config/servo_parameters.yaml. Humble
    declared its parameters in C++ and ships no such file — the absence is a
    signal in its own right, so return None and let callers fall back.

    Nested namespace names ("scale") are included alongside their leaves
    ("scale.linear" -> "linear"), because a config maps a whole `scale:` dict
    onto that namespace and a leaves-only set would drop it. That makes the set
    slightly permissive — a stray top-level `linear:` would survive — which is
    the right way round: keeping an undeclared key is inert (Servo ignores it),
    dropping a declared one silently changes behaviour.
    """
    import yaml

    path = os.path.join(
        get_package_share_directory("moveit_servo"), "config", "servo_parameters.yaml"
    )
    if not os.path.exists(path):
        return None

    with open(path) as f:
        spec = yaml.safe_load(f) or {}

    names = set()

    def walk(node):
        for key, value in node.items():
            if not isinstance(value, dict):
                continue
            # A leaf declaration is a mapping carrying a scalar "type" key;
            # anything else is a nested namespace (e.g. "scale", "pose_tracking").
            names.add(key)
            if "type" not in value or isinstance(value["type"], dict):
                walk(value)

    walk(spec.get("servo", spec))
    return frozenset(names) or None


@lru_cache(maxsize=1)
def new_servo():
    """True for the rewritten (2.12+) Servo, False for the Humble-era one.

    Tried in order of directness; refuses to guess if nothing is conclusive,
    because every caller makes a safety-relevant decision from this. Resolved
    lazily rather than at import, so a frontend that only needs
    switch_command_type() still imports on a machine without moveit_servo.
    """
    declared = declared_servo_params()
    if declared:
        if _NEW_IDLE_KEYS[0] in declared:
            return True
        if _OLD_IDLE_KEY in declared:
            return False

    lib = os.path.join(get_package_prefix("moveit_servo"), "lib", "moveit_servo")
    if os.path.exists(os.path.join(lib, _NEW_EXECUTABLE)):
        return True
    if os.path.exists(os.path.join(lib, _OLD_EXECUTABLE)):
        return False

    raise RuntimeError(
        f"Cannot determine which moveit_servo generation is installed: no "
        f"declared-parameter manifest and neither {_NEW_EXECUTABLE!r} nor "
        f"{_OLD_EXECUTABLE!r} found in {lib}. Refusing to guess."
    )


def servo_executable():
    return _NEW_EXECUTABLE if new_servo() else _OLD_EXECUTABLE


def filter_servo_config(cfg):
    """Drop keys the installed Servo does not declare.

    Only filters when the declared set is knowable (Jazzy). On Humble the
    config is passed through untouched, which reproduces the pre-port behaviour
    exactly instead of guessing at a hardcoded strip list that would rot
    against the next ur_moveit_config release.
    """
    declared = declared_servo_params()
    if declared is None:
        return dict(cfg)
    return {k: v for k, v in cfg.items() if k in declared}


def apply_idle_protection(cfg):
    """Set whichever mechanism stops Servo republishing when input is idle.

    Without this the arm drifts into a singularity and Servo emits NaN — see
    "Servo drift loop" in README.md. Humble solves it with low_latency_mode;
    the rewritten Servo halts on a command timeout instead. Exactly one of the
    two must end up set.
    """
    cfg = dict(cfg)
    declared = declared_servo_params()

    if new_servo():
        cfg.pop(_OLD_IDLE_KEY, None)
        cfg["incoming_command_timeout"] = INCOMING_COMMAND_TIMEOUT
        for key in _NEW_IDLE_KEYS:
            cfg[key] = True
        chosen = _NEW_IDLE_KEYS[0]
    else:
        for key in _NEW_IDLE_KEYS:
            cfg.pop(key, None)
        cfg[_OLD_IDLE_KEY] = True
        chosen = _OLD_IDLE_KEY

    # The failure this guards against is silent — an unprotected Servo launches
    # happily and only misbehaves once left idle — so refuse to launch rather
    # than fall through to a default.
    if declared is not None and chosen not in declared:
        raise RuntimeError(
            f"Servo idle protection unavailable: {chosen!r} is not declared by "
            f"the installed moveit_servo. Refusing to launch an unprotected "
            f"Servo (drift/NaN runaway, see README.md)."
        )
    return cfg


def unwrap_kinematics(raw):
    """Return the group -> kinematics mapping from a kinematics.yaml.

    Self-describing, so no version flag is needed: Humble's ur_moveit_config
    wraps it in "/**" -> ros__parameters -> robot_description_kinematics, Jazzy
    ships the mapping flat.
    """
    if "/**" in raw:
        return raw["/**"]["ros__parameters"]["robot_description_kinematics"]
    return raw


def srdf_group_name(srdf_xml):
    """The manipulator planning-group name, read from the generated SRDF.

    Jazzy's ur_macro.srdf.xacro takes no `name` parameter and hardcodes
    <group name="ur_manipulator">; Humble's interpolated it into
    "<ur_type>_manipulator". Rather than encode that difference, ask the SRDF
    we just generated what its group is actually called.
    """
    groups = re.findall(r'<group\s+name="([^"]+)"', srdf_xml)
    manipulators = [g for g in groups if "manipulator" in g]
    if len(manipulators) != 1:
        raise RuntimeError(
            f"Expected exactly one manipulator group in the generated SRDF, "
            f"found {manipulators or groups}."
        )
    return manipulators[0]


@lru_cache(maxsize=1)
def supports_command_type_switch():
    """Whether Servo requires (and offers) explicit command-type selection.

    Gated on new_servo() and not on the message alone: moveit_msgs is a
    separate package, so a Humble-era servo_node can sit alongside a moveit_msgs
    that defines ServoCommandType. Calling the service in that case would block
    for the full timeout waiting on something that will never be advertised.
    """
    if not new_servo():
        return False
    try:
        from moveit_msgs.srv import ServoCommandType  # noqa: F401
    except ImportError:
        return False
    return True


def switch_command_type(node, want, spinning=False, timeout_sec=60.0):
    """Select Servo's active command type. The single implementation.

    The rewritten Servo boots with NO command type selected and rejects every
    message with "Command type has not been set, cannot accept input" until
    /servo_node/switch_command_type is called, and it accepts exactly one type
    at a time. Humble's Servo took TwistStamped and JointJog at any time and
    offers no such service, so this is a no-op there and callers never branch.

    Pass spinning=True if something else already spins `node` (a background
    rclpy.spin thread); the call then waits by polling instead of spinning the
    node a second time, which rclpy refuses. Callers must never call this from
    inside a callback of their own spin loop — the node cannot be spun
    reentrantly, and nothing would service the response.

    The generous default timeout is deliberate: servo_node serves no services
    while its CurrentStateMonitor sits in "Waiting to receive robot state
    update", which lasts until a joint actually moves.
    """
    if not supports_command_type_switch():
        return True

    from moveit_msgs.srv import ServoCommandType

    types = {
        "joint_jog": ServoCommandType.Request.JOINT_JOG,
        "twist": ServoCommandType.Request.TWIST,
        "pose": ServoCommandType.Request.POSE,
    }
    want = want.lower()
    if want not in types:
        raise ValueError(f"unknown command type {want!r}; expected one of {list(types)}")

    client = node.create_client(ServoCommandType, "/servo_node/switch_command_type")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error(
            f"/servo_node/switch_command_type unavailable after {timeout_sec:g}s — "
            "servo_node is probably still waiting for a robot state update "
            "(see https://github.com/moveit/moveit2/issues/3040)"
        )
        return False

    request = ServoCommandType.Request()
    request.command_type = types[want]
    future = client.call_async(request)
    _wait_for(node, future, spinning=spinning, timeout_sec=20.0)

    result = future.result()
    if result is not None and result.success:
        node.get_logger().info(f"servo command type set to {want.upper()}")
        return True
    node.get_logger().error(f"failed to set servo command type to {want.upper()}")
    return False


def _wait_for(node, future, spinning, timeout_sec):
    """Wait on a future, spinning the node ourselves unless someone else is."""
    import time

    import rclpy

    if not spinning:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        return

    deadline = time.time() + timeout_sec
    while time.time() < deadline and not future.done():
        time.sleep(0.02)


def run_xacro(package, relative_path, **args):
    """Expand a xacro from an installed package and return the XML.

    Eager rather than a lazy launch Command substitution: the SRDF has to be
    parsed at launch-description build time to learn the planning-group name.
    A xacro failure surfaces immediately instead of as an opaque parameter
    error after the nodes have started.
    """
    path = os.path.join(get_package_share_directory(package), relative_path)
    cmd = ["xacro", path] + [f"{k}:={v}" for k, v in args.items()]
    return subprocess.check_output(cmd, text=True)
