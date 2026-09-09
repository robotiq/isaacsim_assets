"""
Build the ROS 2 joint-control Action Graph for a UR5e + 2F-85 in Isaac Sim 6.

Two ways to run this:
  1. From Script Editor (Window → Script Editor → Open → Run) — the module-
     level call at the bottom builds the graph immediately in the open stage.
  2. Attached to a prim via a Python Scripting Component (omni.kit.scripting) —
     saving this file hot-reloads the script -> on_init rebuilds the graph.

Topics: /joint_command (in), /joint_states (out), /clock (out)
"""
import omni.graph.core as og
import omni.usd
from pxr import Sdf

# --- EDIT THIS to match your articulation root prim path ---------------------
ARTICULATION_PRIM = "/World/ur5e/root_joint"
# -----------------------------------------------------------------------------

# Keep the graph under /World so the delete-then-rebuild here is idempotent
# against the graph shipped inside ur5robot_with_2F-85.usda (also /World/…).
GRAPH_PATH = "/World/ROS_JointControl"


def _delete_existing_graph():
    stage = omni.usd.get_context().get_stage()
    if stage and stage.GetPrimAtPath(GRAPH_PATH):
        stage.RemovePrim(GRAPH_PATH)


def _build_graph():
    _delete_existing_graph()

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick",        "omni.graph.action.OnPlaybackTick"),
                ("SimTime",       "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PubClock",      "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("SubJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PubJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
                ("OnTick.outputs:tick", "SubJointState.inputs:execIn"),
                ("OnTick.outputs:tick", "ArtController.inputs:execIn"),
                ("OnTick.outputs:tick", "PubJointState.inputs:execIn"),

                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "PubJointState.inputs:timeStamp"),

                ("SubJointState.outputs:jointNames",       "ArtController.inputs:jointNames"),
                ("SubJointState.outputs:positionCommand",  "ArtController.inputs:positionCommand"),
                ("SubJointState.outputs:velocityCommand",  "ArtController.inputs:velocityCommand"),
                ("SubJointState.outputs:effortCommand",    "ArtController.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("SubJointState.inputs:topicName", "joint_command"),
                ("PubJointState.inputs:topicName", "joint_states"),
                ("PubClock.inputs:topicName",      "clock"),
                ("ArtController.inputs:targetPrim", [Sdf.Path(ARTICULATION_PRIM)]),
                ("PubJointState.inputs:targetPrim", [Sdf.Path(ARTICULATION_PRIM)]),
            ],
        },
    )
    print(f"[ROS_JointControl] built, driving {ARTICULATION_PRIM}")


# Python Scripting Component path — the class only fires when this file is
# attached to a prim via omni.kit.scripting. We import BehaviorScript inside
# a try so plain Script Editor execution (which usually has kit-scripting
# available anyway) doesn't error out on unusual setups.
try:
    from omni.kit.scripting import BehaviorScript

    class BuildRosActionGraph(BehaviorScript):
        def on_init(self):
            try:
                _build_graph()
            except Exception as e:
                print(f"[ROS_JointControl] build failed: {e}")

        def on_destroy(self):
            _delete_existing_graph()
            print("[ROS_JointControl] removed")
except Exception:
    pass


# Script Editor entry point — running this file with F5 or the Run button
# executes here and builds the graph in the current stage. Idempotent:
# _build_graph() deletes any existing graph at GRAPH_PATH first, so re-running
# never stacks graphs and rebuilds cleanly against the graph the shipped
# ur5robot_with_2F-85.usda scene already has at /World/ROS_JointControl.
try:
    _build_graph()
except Exception as e:
    print(f"[ROS_JointControl] build failed: {e}")
