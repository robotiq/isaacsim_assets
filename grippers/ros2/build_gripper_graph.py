"""
Build the ROS 2 Action Graph that lets a Robotiq 2F gripper in Isaac Sim be driven
by the `robotiq/ros` stack like the real gripper.

    ros2 launch robotiq_description robotiq_control.launch.py sim_topic_based:=true

The ros2_control plugin that launch selects (topic_based_ros2_control) exchanges two
sensor_msgs/JointState topics with the simulator. This script builds the simulator end
of them, for the PhysX and the Newton physics variants alike.

What it exposes is what the real gripper reports, one position: ONE joint, the driven
one, under its URDF name (#88, option 2: the asset keeps `finger_joint`, the graph remaps).
  * commands: the driven joint is picked out of every JointState the plugin sends and
    handed to the articulation controller under the asset's own name.
  * state:    the driven joint's position, velocity and effort are published under its
    URDF name.

This one-joint contract is the intended one, and it depends on robotiq/ros#118 being fixed
on the ros side (declare the URDF mimic joints for Gazebo only). Until a ros release carries
that fix (ros <= 1.3), under sim_topic_based:
  * the plugin's command also names the five mimic joints, with a position for the driven
    one only. Isaac's ROS2SubscribeJointState discards such a message (its arrays differ in
    length), so this graph reads it with the generic ROS2Subscriber and filters it. Once
    ros#118 is fixed the command carries the driven joint alone and the filter can go.
  * /joint_states shows the mimic joints at 0, since nothing publishes them; RViz shows
    the fingers frozen while the knuckle moves (#89). A known limitation of those ros
    releases, not of this graph.

Run it from the Script Editor (Window > Script Editor > Open > Run) in a stage that
references the gripper: that builds the graph for a 2F-85. For anything else, call
`build_graph` with keyword arguments:

    import runpy
    runpy.run_path(".../grippers/ros2/build_gripper_graph.py")["build_graph"](model="2f_140")

Running it again replaces the graph, so it is safe to re-run after switching the
Physics variant.
"""

# model -> (driven joint in the URDF of robotiq/ros, driven joint in the Isaac asset).
# The 2F-85 URDF names its driven joint robotiq_85_left_knuckle_joint; the asset follows
# the 2F-140 naming. `prefix` is the URDF's `prefix` xacro argument.
DRIVEN_JOINTS = {
    "2f_85": ("robotiq_85_left_knuckle_joint", "finger_joint"),
    "2f_140": ("finger_joint", "finger_joint"),
}

# robotiq_control.launch.py defaults; the plugin's topic names are relative to the root.
COMMAND_TOPIC = "/sim/joint_commands"
STATE_TOPIC = "/sim/joint_states"

GRAPH_PATH = "/World/Robotiq_ROS2"

# Script-node bodies, filled in when the graph is built. Their inputs and outputs are
# dynamic attributes connected to the neighbouring nodes (see build_graph).
_COMMAND_SCRIPT = '''
import carb
import omni.graph.core as og

def compute(db):
    names = [str(n) for n in db.inputs.name]
    if "{urdf}" not in names:
        # Once, not every tick: a --model or --prefix that does not match the launch's
        # gripper_model / prefix otherwise leaves the gripper motionless with no hint.
        if not getattr(db.per_instance_state, "warned", False):
            db.per_instance_state.warned = True
            carb.log_warn(
                "[robotiq-ros2] commands on {topic} name " + repr(names) + ", not '{urdf}': "
                "check the graph's model and prefix against the launch's gripper_model and prefix"
            )
        return False
    i = names.index("{urdf}")
    positions = db.inputs.position
    # ros <= 1.3 (robotiq/ros#118): the plugin names the mimic joints too but fills in a
    # position for the driven joint only, which comes first.
    if i >= len(positions):
        return False
    db.outputs.jointNames = ["{asset}"]
    db.outputs.positionCommand = [float(positions[i])]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''

_STATE_SCRIPT = '''
import omni.graph.core as og

def compute(db):
    names = [str(n) for n in db.inputs.jointNames]
    if "{asset}" not in names:
        return False
    i = names.index("{asset}")
    def pick(values):
        return [float(values[i])] if len(values) > i else []
    t = float(db.inputs.sensorTime)
    db.outputs.sec = int(t)
    db.outputs.nanosec = int((t - int(t)) * 1e9)
    db.outputs.name = ["{urdf}"]
    db.outputs.position = pick(db.inputs.jointPositions)
    db.outputs.velocity = pick(db.inputs.jointVelocities)
    db.outputs.effort = pick(db.inputs.jointEfforts)
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''

# Dynamic attributes of the two script nodes: (name, type).
_COMMAND_ATTRIBUTES = [
    ("inputs:name", "token[]"),
    ("inputs:position", "double[]"),
    ("outputs:jointNames", "token[]"),
    ("outputs:positionCommand", "double[]"),
]
_STATE_ATTRIBUTES = [
    ("inputs:jointNames", "token[]"),
    ("inputs:jointPositions", "double[]"),
    ("inputs:jointVelocities", "double[]"),
    ("inputs:jointEfforts", "double[]"),
    ("inputs:sensorTime", "float"),
    ("outputs:sec", "int"),
    ("outputs:nanosec", "uint"),
    ("outputs:name", "token[]"),
    ("outputs:position", "double[]"),
    ("outputs:velocity", "double[]"),
    ("outputs:effort", "double[]"),
]


def find_articulation_root(stage, asset_joint):
    """The one articulation root whose subtree holds `asset_joint`, or None."""
    from pxr import Usd, UsdPhysics

    found = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        # A root can sit on a joint (a PhysicsFixedJoint such as /World/ur5e/root_joint) and
        # then own its parent's subtree.
        owner = prim.GetParent() if prim.IsA(UsdPhysics.Joint) else prim
        if any(p.GetName() == asset_joint and p.IsA(UsdPhysics.Joint) for p in Usd.PrimRange(owner)):
            found.append(prim.GetPath())
    if len(found) > 1:
        raise RuntimeError(
            f"several articulations have a joint named {asset_joint!r}: "
            f"{', '.join(map(str, found))}; pass articulation_root"
        )
    return found[0] if found else None


class Ros2BridgeUnavailable(RuntimeError):
    """Isaac was started without the ROS 2 libraries its bridge needs."""


def check_ros2_libraries():
    """Fail cleanly when Isaac's ROS 2 bridge cannot load.

    The bridge dlopens librosidl_runtime_c.so from the library path Isaac was started with. When
    it is missing the bridge only logs "ROS2 Bridge startup failed", and the first ROS2Subscriber
    / ROS2Publisher whose message type is then set dereferences the missing plugin: a native
    segfault that takes the whole Isaac process down. The path cannot be fixed from inside the
    running process, so say what to do before touching a node.
    """
    import ctypes

    try:
        ctypes.CDLL("librosidl_runtime_c.so")
    except OSError as error:
        raise Ros2BridgeUnavailable(
            "Isaac's ROS 2 bridge cannot load (librosidl_runtime_c.so is not on LD_LIBRARY_PATH), "
            "and building the graph would crash Isaac. Start Isaac with its ROS environment: "
            "`source <isaac-sim>/setup_ros_env.sh` (or source a system ROS 2) before launching it. "
            f"({error})"
        ) from error


def build_graph(
    model="2f_85",
    articulation_root=None,
    prefix="",
    command_topic=COMMAND_TOPIC,
    state_topic=STATE_TOPIC,
    graph_path=GRAPH_PATH,
):
    """Build (or rebuild) the gripper graph in the open stage and return its path."""
    import omni.graph.core as og
    import omni.kit.app
    import omni.usd
    from pxr import Sdf

    if model not in DRIVEN_JOINTS:
        raise ValueError(f"unknown model {model!r}, expected one of {sorted(DRIVEN_JOINTS)}")
    urdf_joint, asset_joint = DRIVEN_JOINTS[model]
    urdf_joint = prefix + urdf_joint

    check_ros2_libraries()

    extensions = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.physics.nodes"):
        extensions.set_extension_enabled_immediate(ext, True)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no stage is open")

    if articulation_root is None:
        root = find_articulation_root(stage, asset_joint)
        if root is None:
            raise RuntimeError(
                f"no articulation with a joint named {asset_joint!r} in the stage; "
                "pass articulation_root (and check model)"
            )
    else:
        root = Sdf.Path(articulation_root)
        if not stage.GetPrimAtPath(root):
            raise RuntimeError(f"articulation root {root} does not exist in the stage")

    if stage.GetPrimAtPath(graph_path):
        stage.RemovePrim(graph_path)

    fill = {"urdf": urdf_joint, "asset": asset_joint, "topic": command_topic}
    keys = og.Controller.Keys
    # The ROS2Subscriber / ROS2Publisher attributes for the message fields only appear once
    # the message type is set, so the nodes are configured first and wired in a second edit.
    graph, _, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                # command path: ROS -> pick the driven joint -> articulation
                ("Sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("PickCommand", "omni.graph.scriptnode.ScriptNode"),
                ("Art", "isaacsim.core.nodes.IsaacArticulationController"),
                # state path: articulation -> pick the driven joint -> ROS
                ("ReadState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PickState", "omni.graph.scriptnode.ScriptNode"),
                ("Pub", "isaacsim.ros2.bridge.ROS2Publisher"),
            ],
            keys.CREATE_ATTRIBUTES: [(f"PickCommand.{n}", t) for n, t in _COMMAND_ATTRIBUTES]
            + [(f"PickState.{n}", t) for n, t in _STATE_ATTRIBUTES],
            keys.SET_VALUES: [
                ("Sub.inputs:topicName", command_topic),
                ("Sub.inputs:messagePackage", "sensor_msgs"),
                ("Sub.inputs:messageSubfolder", "msg"),
                ("Sub.inputs:messageName", "JointState"),
                ("PickCommand.inputs:script", _COMMAND_SCRIPT.format(**fill)),
                ("Art.inputs:targetPrim", [root]),
                ("ReadState.inputs:prim", [root]),
                ("PickState.inputs:script", _STATE_SCRIPT.format(**fill)),
                ("Pub.inputs:topicName", state_topic),
                ("Pub.inputs:messagePackage", "sensor_msgs"),
                ("Pub.inputs:messageSubfolder", "msg"),
                ("Pub.inputs:messageName", "JointState"),
            ],
        },
    )
    connections = [
        ("Tick.outputs:tick", "Sub.inputs:execIn"),
        ("Sub.outputs:execOut", "PickCommand.inputs:execIn"),
        ("Sub.outputs:name", "PickCommand.inputs:name"),
        ("Sub.outputs:position", "PickCommand.inputs:position"),
        ("PickCommand.outputs:execOut", "Art.inputs:execIn"),
        ("PickCommand.outputs:jointNames", "Art.inputs:jointNames"),
        ("PickCommand.outputs:positionCommand", "Art.inputs:positionCommand"),
        ("Tick.outputs:tick", "ReadState.inputs:execIn"),
        ("ReadState.outputs:execOut", "PickState.inputs:execIn"),
        ("ReadState.outputs:jointNames", "PickState.inputs:jointNames"),
        ("ReadState.outputs:jointPositions", "PickState.inputs:jointPositions"),
        ("ReadState.outputs:jointVelocities", "PickState.inputs:jointVelocities"),
        ("ReadState.outputs:jointEfforts", "PickState.inputs:jointEfforts"),
        ("ReadState.outputs:sensorTime", "PickState.inputs:sensorTime"),
        ("PickState.outputs:execOut", "Pub.inputs:execIn"),
        ("PickState.outputs:sec", "Pub.inputs:header:stamp:sec"),
        ("PickState.outputs:nanosec", "Pub.inputs:header:stamp:nanosec"),
        ("PickState.outputs:name", "Pub.inputs:name"),
        ("PickState.outputs:position", "Pub.inputs:position"),
        ("PickState.outputs:velocity", "Pub.inputs:velocity"),
        ("PickState.outputs:effort", "Pub.inputs:effort"),
    ]
    # A second edit resolves "Node.attr" against the stage root, not the graph: use full paths.
    og.Controller.edit(
        graph,
        {keys.CONNECT: [(f"{graph_path}/{src}", f"{graph_path}/{dst}") for src, dst in connections]},
    )

    # The silent failures this graph exists to rule out: a target that is not the prim we
    # meant, or values Isaac reads differently from what we typed. The panel shows a wrong
    # value exactly like a right one, so compare the stored values, not the display.
    expect_root = [str(root)]
    for node, attr in (("Art", "inputs:targetPrim"), ("ReadState", "inputs:prim")):
        stored = [str(p) for p in og.Controller.attribute(f"{graph_path}/{node}.{attr}").get()]
        if stored != expect_root:
            raise RuntimeError(f"{node}.{attr} is {stored!r}, expected {expect_root!r}")
    for node, expected in (("Sub", command_topic), ("Pub", state_topic)):
        stored = og.Controller.attribute(f"{graph_path}/{node}.inputs:topicName").get()
        if stored != expected:
            raise RuntimeError(f"{node}.inputs:topicName is {stored!r}, expected {expected!r}")

    print(
        f"[robotiq-ros2] {graph_path}: {model} on {root}; ROS joint {urdf_joint!r} "
        f"<-> asset joint {asset_joint!r}; commands {command_topic}, state {state_topic}"
    )
    return graph_path


# The Script Editor runs a file as __main__; runpy.run_path does not, so importing the
# function from another script builds nothing by itself.
if __name__ == "__main__":
    build_graph()
