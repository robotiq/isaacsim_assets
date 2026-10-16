# Driving the simulated 2F gripper from ROS 2

The simulated 2F-85 and 2F-140 answer the [`robotiq/ros`](https://github.com/robotiq/ros/tree/main/grippers)
stack exactly like the real gripper: same launch file, same `gripper_cmd` action, same
`/joint_states`. Switching between the real gripper and the simulation is one launch argument.

| | Real gripper | Simulation |
|---|---|---|
| Launch | `ros2 launch robotiq_description robotiq_control.launch.py` | `ros2 launch robotiq_description robotiq_control.launch.py sim_topic_based:=true` |
| Command | `gripper_cmd` action on `/robotiq_gripper_controller` | same |
| State | `/joint_states` | same |

With `sim_topic_based:=true` the description loads `topic_based_ros2_control`, which exchanges
two `sensor_msgs/JointState` topics with the simulator: `/sim/joint_commands` and
`/sim/joint_states` (change them with `sim_joint_commands_topic` / `sim_joint_states_topic`).
[`build_gripper_graph.py`](build_gripper_graph.py) builds the Isaac Sim end of those topics. It
works on every Physics variant, PhysX and Newton alike.

What the graph exposes is what a real gripper gives: **one joint, the driven one, under its URDF name.**

| Model | Joint in ROS (URDF) | Joint in the asset |
|---|---|---|
| `2f_85` | `robotiq_85_left_knuckle_joint` | `finger_joint` |
| `2f_140` | `finger_joint` | `finger_joint` |

The rest of the four-bar linkage is the simulator's business. The plugin's command names the
mimic joints of the URDF too, with a position only for the driven joint, so its arrays differ in
length. The graph reads it with Isaac's generic `ROS2Subscriber` (the dedicated
`ROS2SubscribeJointState` node discards such a message) and keeps only the driven joint (the
articulation controller drops a *whole* command that names a joint the asset does not have).

## Walkthrough

1. **Open the gripper** in Isaac Sim (`grippers/Robotiq_2F_85/Robotiq_2F_85.usda`, or a scene that
   contains it) and pick a `Physics` variant. For `Newton_*` variants start Isaac with
   `isaac-sim.newton.sh`, and apply the gripper's `newton/apply_gripper_tuning.py` after each
   Stop → Play, as for any Newton run.
2. **Build the graph.** In the Script Editor (Window → Script Editor) open `build_gripper_graph.py`
   and run it: that builds the graph for a 2F-85. For a 2F-140 call the function instead, in the
   Script Editor (its keyword arguments are the options below, with `_` for `-`):

   ```python
   import runpy
   runpy.run_path("/path/to/grippers/ros2/build_gripper_graph.py")["build_graph"](model="2f_140")
   ```

   It finds the articulation root by itself. If the gripper is merged into an arm's articulation, or
   the stage holds several grippers, pass `--articulation-root /World/...`. Re-running replaces the
   graph, so run it again after switching the Physics variant.
3. **Press Play.**
4. **Start the ROS side**, in a shell where ROS 2 is sourced (and not the one that started Isaac,
   see [the Gripper Simulation Guide](../GRIPPER_SIMULATION_GUIDE.md)):

   ```bash
   ros2 launch robotiq_description robotiq_control.launch.py gripper_model:=2f_85 sim_topic_based:=true
   ```

   No `robotiq/ros` on your machine? [`tests/ros2/Dockerfile`](../../tests/ros2/Dockerfile) builds
   the stack (Humble) in a container. Run it with the UDP-only FastDDS profile next to it, or the
   container discovers Isaac Sim but its messages never arrive (shared memory / data sharing
   between a container and the host):

   ```bash
   docker build -t robotiq-ros-humble tests/ros2
   docker run --rm -it --network host \
     -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml -v $PWD/tests/ros2/fastdds_udp.xml:/fastdds_udp.xml \
     robotiq-ros-humble bash -c "source /ws/install/setup.bash && \
     ros2 launch robotiq_description robotiq_control.launch.py gripper_model:=2f_85 sim_topic_based:=true"
   ```
5. **Check it.**

   ```bash
   ros2 topic echo /joint_states --once                # the finger joint, plus the mimic joints at 0
   ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
     control_msgs/action/GripperCommand "{command: {position: 0.5, max_effort: 50.0}}"
   ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
     control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"
   ```

   The gripper closes then opens in the viewport; `position` is the driven joint in radians
   (0 open, about 0.8 closed). On Jazzy the action type is `control_msgs/action/ParallelGripperCommand`,
   see the `robotiq/ros` README.

## Options

`build_gripper_graph.py --help`:

| Option | Default | |
|---|---|---|
| `--model` | `2f_85` | `2f_85` or `2f_140` |
| `--articulation-root` | found in the stage | the prim carrying the articulation root |
| `--prefix` | none | the URDF's `prefix` xacro argument, for several grippers on one robot |
| `--command-topic` | `/sim/joint_commands` | match `sim_joint_commands_topic` |
| `--state-topic` | `/sim/joint_states` | match `sim_joint_states_topic` |

## Troubleshooting

Isaac Sim reports most graph mistakes only in the log, if at all.

- **Nothing moves and nothing is published.** The timeline must be playing. Check the topics with
  `ros2 topic list` and compare their names with the launch arguments.
- **The script fails with "no articulation with a joint named ...".** Wrong `--model`, or the
  gripper's articulation root was removed (a gripper merged into an arm keeps its joints but the
  root moves): pass `--articulation-root`.
- **Checking a value by hand.** The property panel shows a wrong value exactly like a right one,
  for example `jointNames` stored as one literal string token instead of a list. Read it back in
  the Script Editor with `repr()`:

  ```python
  import omni.graph.core as og
  print(repr(og.Controller.attribute("/World/Robotiq_ROS2/Art.inputs:jointNames").get()))
  ```

  The script checks the targets and topic names it stores for you and raises on a mismatch.
- **The goal is aborted instead of succeeding when the gripper grasps an object.** By design:
  `robotiq_controllers.topic_based.yaml` sets `allow_stalling: false`, because the simulation
  has no object-detection status. Your client decides whether a stalled goal means an object is held.

## Tests

`tests/runtime/test_gripper_ros2.py` builds the graph for every 2F gripper and Physics variant,
sends the command the plugin sends from a separate ROS 2 process, and checks the driven joint
reaches the target and is reported under its URDF name. It needs a system ROS 2 and, like the
other runtime tests, a local Isaac Sim: it is not part of the GitHub CI.
