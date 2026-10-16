"""
Build the ROS 2 Action Graph that lets a Robotiq 2F gripper in Isaac Sim be driven
by the `robotiq/ros` stack exactly like the real gripper.

    ros2 launch robotiq_description robotiq_control.launch.py sim_topic_based:=true

The ros2_control plugin that launch selects (topic_based_ros2_control) exchanges two
sensor_msgs/JointState topics with the simulator. This script builds the simulator end
of them, for the PhysX and the Newton physics variants alike.

What it exposes mimics what the real gripper gives: ONE joint, the driven one, under its
URDF name.
  * commands: the driven joint is picked out of every JointState the plugin sends (it
    also sends the mimic joints, which the asset does not have under those names; Isaac
    drops a whole command that names an unknown joint) and handed to the articulation
    controller under the asset's own name.
  * state:    the driven joint's position, velocity and effort are published under its
    URDF name.

Run it from the Script Editor (Window > Script Editor > Open > Run) with the gripper
stage open, or standalone/headless:

    python build_gripper_graph.py --model 2f_85 [--articulation-root PRIM] [--prefix P]
                                  [--command-topic T] [--state-topic T]

Running it again replaces the graph, so it is safe to re-run after switching the
Physics variant. Defaults (the model below, the topics of robotiq_control.launch.py, the
articulation root found in the stage) work for the stock assets with no arguments.
"""
import argparse
import sys

# model -> (driven joint in the URDF of robotiq/ros, driven joint in the Isaac asset).
# The 2F-85 URDF names its driven joint robotiq_85_left_knuckle_joint; the asset follows
# the 2F-140 naming. `--prefix` is the URDF's `prefix` xacro argument.
DRIVEN_JOINTS = {
    "2f_85": ("robotiq_85_left_knuckle_joint", "finger_joint"),
    "2f_140": ("finger_joint", "finger_joint"),
}

# robotiq_control.launch.py defaults; the plugin's topic names are relative to the root.
COMMAND_TOPIC = "/sim/joint_commands"
STATE_TOPIC = "/sim/joint_states"

GRAPH_PATH = "/World/Robotiq_ROS2"

# Script-node bodies. {graph}, {urdf} and {asset} are filled in when the graph is built.
# They run inside Kit, once per tick, and talk to their neighbours through attributes.
_COMMAND_SCRIPT = '''
import omni.graph.core as og

def compute(db):
    sub = "{graph}/Sub"
    names = [str(n) for n in og.Controller.attribute(sub + ".outputs:jointNames").get()]
    positions = og.Controller.attribute(sub + ".outputs:positionCommand").get()
    if "{urdf}" not in names or positions is None or len(positions) != len(names):
        return False
    i = names.index("{urdf}")
    art = "{graph}/Art"
    og.Controller.attribute(art + ".inputs:jointNames").set(["{asset}"])
    og.Controller.attribute(art + ".inputs:positionCommand").set([float(positions[i])])
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''

_STATE_SCRIPT = '''
import omni.graph.core as og

def compute(db):
    read = "{graph}/ReadState"
    names = [str(n) for n in og.Controller.attribute(read + ".outputs:jointNames").get()]
    if "{asset}" not in names:
        return False
    i = names.index("{asset}")
    def pick(attr):
        values = og.Controller.attribute(read + ".outputs:" + attr).get()
        return [float(values[i])] if values is not None and len(values) > i else []
    pub = "{graph}/Pub"
    t = float(og.Controller.attribute("{graph}/SimTime.outputs:simulationTime").get())
    sec = int(t)
    og.Controller.attribute(pub + ".inputs:header:stamp:sec").set(sec)
    og.Controller.attribute(pub + ".inputs:header:stamp:nanosec").set(int((t - sec) * 1e9))
    og.Controller.attribute(pub + ".inputs:name").set(["{urdf}"])
    og.Controller.attribute(pub + ".inputs:position").set(pick("jointPositions"))
    og.Controller.attribute(pub + ".inputs:velocity").set(pick("jointVelocities"))
    og.Controller.attribute(pub + ".inputs:effort").set(pick("jointEfforts"))
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''


def find_articulation_root(stage, asset_joint):
    """The articulation root whose subtree holds `asset_joint`."""
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        # A root can sit on a joint (a PhysicsFixedJoint such as /World/ur5e/root_joint) and
        # then own its parent's subtree.
        owner = prim.GetParent() if prim.IsA(UsdPhysics.Joint) else prim
        for child in _descendants(owner):
            if child.GetName() == asset_joint and child.IsA(UsdPhysics.Joint):
                return prim.GetPath()
    return None


def _descendants(prim):
    from pxr import Usd

    return Usd.PrimRange(prim)


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

    extensions = omni.kit.app.get_app().get_extension_manager()
    for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.physics.nodes"):
        extensions.set_extension_enabled_immediate(ext, True)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no stage is open")

    if articulation_root is None:
        found = find_articulation_root(stage, asset_joint)
        if found is None:
            raise RuntimeError(
                f"no articulation with a joint named {asset_joint!r} in the stage; "
                "pass --articulation-root (and check --model)"
            )
        root = found
    else:
        root = Sdf.Path(articulation_root)
        if not stage.GetPrimAtPath(root):
            raise RuntimeError(f"articulation root {root} does not exist in the stage")

    if stage.GetPrimAtPath(graph_path):
        stage.RemovePrim(graph_path)

    command_script = _COMMAND_SCRIPT.format(graph=graph_path, urdf=urdf_joint, asset=asset_joint)
    state_script = _STATE_SCRIPT.format(graph=graph_path, urdf=urdf_joint, asset=asset_joint)

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # command path: ROS -> filter -> articulation
                ("Sub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("PickCommand", "omni.graph.scriptnode.ScriptNode"),
                ("Art", "isaacsim.core.nodes.IsaacArticulationController"),
                # state path: articulation -> pick the driven joint -> ROS
                ("ReadState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PickState", "omni.graph.scriptnode.ScriptNode"),
                ("Pub", "isaacsim.ros2.bridge.ROS2Publisher"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "Sub.inputs:execIn"),
                ("Sub.outputs:execOut", "PickCommand.inputs:execIn"),
                ("PickCommand.outputs:execOut", "Art.inputs:execIn"),
                ("Tick.outputs:tick", "ReadState.inputs:execIn"),
                ("ReadState.outputs:execOut", "PickState.inputs:execIn"),
                ("PickState.outputs:execOut", "Pub.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("Sub.inputs:topicName", command_topic),
                ("PickCommand.inputs:script", command_script),
                ("Art.inputs:targetPrim", [root]),
                ("ReadState.inputs:prim", [root]),
                ("PickState.inputs:script", state_script),
                ("Pub.inputs:topicName", state_topic),
                ("Pub.inputs:messagePackage", "sensor_msgs"),
                ("Pub.inputs:messageSubfolder", "msg"),
                ("Pub.inputs:messageName", "JointState"),
            ],
        },
    )

    # The silent failures this graph exists to rule out: a target that is not the prim we
    # meant, or joint names Isaac reads differently from what we typed. The panel shows a
    # wrong value exactly like a right one, so compare the stored values, not the display.
    expect_root = [str(root)]
    for node in ("Art", "ReadState"):
        attr = "inputs:targetPrim" if node == "Art" else "inputs:prim"
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--model", choices=sorted(DRIVEN_JOINTS), default="2f_85")
    parser.add_argument("--articulation-root", help="default: the root holding the driven joint")
    parser.add_argument("--prefix", default="", help="the URDF's `prefix` xacro argument")
    parser.add_argument("--command-topic", default=COMMAND_TOPIC)
    parser.add_argument("--state-topic", default=STATE_TOPIC)
    # Kit's own arguments are in sys.argv when this runs from the Script Editor.
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])
    return build_graph(
        model=args.model,
        articulation_root=args.articulation_root,
        prefix=args.prefix,
        command_topic=args.command_topic,
        state_topic=args.state_topic,
    )


if __name__ == "__main__":
    main()
