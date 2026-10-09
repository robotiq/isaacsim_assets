"""Make the teleop scene run on the Newton engine (Isaac Sim 6.1).

Runs INSIDE Isaac, after the scene is open and BEFORE the first Play. Either
open_scene.py calls it (it does so on Newton runs), or run it from the Script
Editor:

    exec(open(r".../grippers/2F/demo/teleop/newton_runtime.py").read())

It does two things the scene cannot do on its own under Newton:

1. Joint states for ROS. The scene's ReadJointState node
   (isaacsim.sensors.physics.IsaacReadJointState) reads joint efforts, and
   Newton has none to give: it does not expose projected joint forces. On
   Isaac Sim 6.1 the node does not fail cleanly -- it crashes Isaac at Play.
   This swaps it for a Script Node that reads positions and velocities through
   isaacsim.core.experimental and publishes zero efforts (ROS2PublishJointState
   requires every array to be the same length). The swap is authored in the
   SESSION layer, so it is never saved into the scene file and the PhysX path
   keeps the stock node.

2. Gripper tuning. Robotiq_2F_85/newton/apply_gripper_tuning.py has to run after
   every Stop -> Play or the fingers go floppy. This runs it automatically a few
   frames after each Play.

Idempotent: running it twice does not stack anything.
"""
import os

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Sdf, Usd

GRAPH = "/World/ROS_JointControl"
STOCK_READER = GRAPH + "/ReadJointState"
NEWTON_READER = GRAPH + "/ReadJointStateNewton"
PUBLISHER = GRAPH + "/PubJointState"
TICK = GRAPH + "/OnTick.outputs:tick"
ROBOT = "/World/ur5e"

# The mujoco_warp model is built during the first steps after Play; tuning it
# earlier finds nothing to tune.
TUNING_DELAY_FRAMES = 10

# Script Node body. Module globals are per node instance, so they hold the
# articulation between ticks. The articulation re-initialises itself on every
# Play; it is recreated only if that ever leaves it invalid.
READER_SCRIPT = '''
import omni.graph.core as og
import omni.physics.tensors
import omni.usd
from pxr import UsdGeom
from isaacsim.core.experimental.prims import Articulation

ROBOT = "%s"
_art = None
_types = None


def setup(db):
    global _art, _types
    _art = None
    _types = None


def compute(db):
    global _art, _types
    if _art is None or not _art.is_physics_tensor_entity_valid():
        _art = Articulation(ROBOT)
        _types = None
        if not _art.is_physics_tensor_entity_valid():
            return False
    if _types is None:
        prismatic = omni.physics.tensors.DofType.Translation
        _types = [1 if t == prismatic else 0 for t in _art.dof_types]
    n = len(_types)
    db.outputs.jointNames = list(_art.dof_names)
    db.outputs.jointPositions = _art.get_dof_positions().numpy()[0].tolist()
    db.outputs.jointVelocities = _art.get_dof_velocities().numpy()[0].tolist()
    db.outputs.jointEfforts = [0.0] * n   # Newton has no projected joint forces
    db.outputs.jointDofTypes = _types
    db.outputs.stageMetersPerUnit = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
''' % ROBOT

# Script Node output -> PubJointState input. sensorTime stays unconnected: at 0
# the publisher stamps messages with its timeStamp input (simulation time).
OUTPUTS = {
    "jointNames": Sdf.ValueTypeNames.TokenArray,
    "jointPositions": Sdf.ValueTypeNames.DoubleArray,
    "jointVelocities": Sdf.ValueTypeNames.DoubleArray,
    "jointEfforts": Sdf.ValueTypeNames.DoubleArray,
    "jointDofTypes": Sdf.ValueTypeNames.UCharArray,
    "stageMetersPerUnit": Sdf.ValueTypeNames.Float,
}


def _tuning_path():
    # exec() from the Script Editor has no __file__; the open scene sits in the
    # same teleop/ directory as this file, so its URL works as well.
    if "__file__" in globals():
        here = os.path.dirname(os.path.abspath(__file__))
    else:
        here = os.path.dirname(omni.usd.get_context().get_stage().GetRootLayer().realPath)
    return os.path.normpath(os.path.join(
        here, "..", "..", "Robotiq_2F_85", "newton", "apply_gripper_tuning.py"))


def swap_joint_state_reader(stage):
    """Returns a note about what it did."""
    if not stage.GetPrimAtPath(PUBLISHER):
        return "no %s; joint-state graph left alone" % PUBLISHER
    if stage.GetPrimAtPath(NEWTON_READER):
        return "joint-state reader already swapped"

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        old = stage.GetPrimAtPath(STOCK_READER)
        if old:
            old.SetActive(False)

        node = stage.DefinePrim(NEWTON_READER, "OmniGraphNode")
        node.CreateAttribute("node:type", Sdf.ValueTypeNames.Token).Set(
            "omni.graph.scriptnode.ScriptNode")
        node.CreateAttribute("node:typeVersion", Sdf.ValueTypeNames.Int).Set(2)
        node.CreateAttribute("inputs:script", Sdf.ValueTypeNames.String).Set(READER_SCRIPT)
        node.CreateAttribute("inputs:usePath", Sdf.ValueTypeNames.Bool).Set(False)
        node.CreateAttribute("inputs:execIn", Sdf.ValueTypeNames.UInt).SetConnections(
            [Sdf.Path(TICK)])
        node.CreateAttribute("outputs:execOut", Sdf.ValueTypeNames.UInt)
        for name, vtype in OUTPUTS.items():
            node.CreateAttribute("outputs:" + name, vtype)

        pub = stage.GetPrimAtPath(PUBLISHER)
        pub.GetAttribute("inputs:execIn").SetConnections(
            [Sdf.Path(NEWTON_READER + ".outputs:execOut")])
        pub.GetAttribute("inputs:sensorTime").SetConnections([])
        for name in OUTPUTS:
            pub.GetAttribute("inputs:" + name).SetConnections(
                [Sdf.Path(NEWTON_READER + ".outputs:" + name)])
    return "joint-state reader swapped for the Newton-safe Script Node (session layer)"


def install_auto_tuning():
    """Run apply_gripper_tuning.py a few frames after every Play."""
    path = _tuning_path()
    if not os.path.isfile(path):
        return "apply_gripper_tuning.py not found at %s; tune by hand" % path

    old = getattr(carb, "_newton_teleop_tuning", None)
    if old:
        old["timeline"] = None
        old["update"] = None

    state = {"frames": -1, "timeline": None, "update": None}

    def _on_timeline(event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            state["frames"] = 0

    def _on_update(_event):
        if state["frames"] < 0:
            return
        state["frames"] += 1
        if state["frames"] >= TUNING_DELAY_FRAMES:
            state["frames"] = -1
            with open(path) as fh:
                exec(compile(fh.read(), path, "exec"), {"__name__": "apply_gripper_tuning"})  # noqa: S102

    tl = omni.timeline.get_timeline_interface()
    state["timeline"] = tl.get_timeline_event_stream().create_subscription_to_pop(
        _on_timeline, name="newton_teleop_tuning_play")
    state["update"] = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _on_update, name="newton_teleop_tuning_update")
    carb._newton_teleop_tuning = state
    if tl.is_playing():
        state["frames"] = 0
    return "gripper tuning will run %d frames after every Play" % TUNING_DELAY_FRAMES


def install(stage=None):
    stage = stage or omni.usd.get_context().get_stage()
    return [swap_joint_state_reader(stage), install_auto_tuning()]


if __name__ == "__main__":
    for _note in install():
        print("[newton-runtime] " + _note)
