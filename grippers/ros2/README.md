# Driving the simulated 2F gripper from ROS 2

The simulated 2F-85 and 2F-140 answer the [`robotiq/robotiq-ros`](https://github.com/robotiq/robotiq-ros/tree/main/grippers)
stack like the real gripper: same launch file, same `gripper_cmd` action, same `/joint_states`.
Switching between the real gripper and the simulation is one launch argument. Until a ros release
carries the fix of [robotiq/robotiq-ros#118](https://github.com/robotiq/robotiq-ros/issues/118) there is one
difference: see [Known limitation](#known-limitation-ros--13).

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

What the graph exposes is what the real gripper reports, one position: **one joint, the driven
one, under its URDF name.** The asset keeps its own joint name and the graph remaps it (option 2 of
[#88](https://github.com/robotiq/isaacsim_assets/issues/88)):

| Model | Joint in ROS (URDF) | Joint in the asset | Closed: URDF `gripper_closed_position` | Asset upper limit |
|---|---|---|---|---|
| `2f_85` | `robotiq_85_left_knuckle_joint` | `finger_joint` | 0.7929 rad | 47° (0.820 rad) |
| `2f_140` | `finger_joint` | `finger_joint` | 0.695 rad | 45° (0.785 rad) |

The asset closes a little further than the position the driver calls closed, so every position
the controller commands is reachable; a goal beyond the asset's limit stops at the limit. The
rest of the four-bar linkage is the simulator's business.

### Known limitation (ros ≤ 1.3)

The one-joint contract is the intended one, and it depends on
[robotiq/robotiq-ros#118](https://github.com/robotiq/robotiq-ros/issues/118) being fixed on the ros side (declare the
URDF mimic joints for Gazebo only). Up to ros 1.3, under `sim_topic_based`:

- **`/joint_states` shows the five mimic joints at 0.** The description declares them as state
  interfaces, nothing publishes them, and `robot_state_publisher` takes those zeros instead of
  deriving them from `<mimic>`: RViz, MoveIt and TF show the fingers frozen while the knuckle moves
  ([#89](https://github.com/robotiq/isaacsim_assets/issues/89)). On the real gripper `/joint_states`
  carries the knuckle alone. Once ros#118 is fixed, the simulation's `/joint_states` is the
  hardware's, with nothing to change here.
- **The command names the mimic joints too, with a position for the driven joint only**, so its
  arrays differ in length and Isaac's `ROS2SubscribeJointState` node discards it. The graph reads
  it with the generic `ROS2Subscriber` and keeps only the driven joint (the articulation controller
  would also drop a *whole* command that names a joint the asset does not have). That filter can go
  once ros#118 is fixed; it keeps working either way.

## Walkthrough

1. **Put the gripper in a stage** and pick its `Physics` variant: create a new stage (File → New)
   and drag `grippers/Robotiq_2F_85/Robotiq_2F_85.usda` into it, or open a scene that references
   it. Do not open the asset file itself: the graph would be authored into the shipped asset and
   saved with it on Ctrl+S. For `Newton_*` variants start Isaac with `isaac-sim.newton.sh`, on
   Isaac Sim 6.1+ sublayer [`../scene/Robotiq_Newton_scene_physics.usda`](../scene/Robotiq_Newton_scene_physics.usda)
   into the stage (the elliptic friction cone, see the simulation guide §5.3), and apply the gripper's `newton/apply_gripper_tuning.py` after each Stop → Play, as for any Newton
   run.
2. **Build the graph.** In the Script Editor (Window → Script Editor) open `build_gripper_graph.py`
   and run it: that builds the graph for a 2F-85. For anything else call `build_graph` with the
   [options](#options) as keyword arguments, in the Script Editor:

   ```python
   import runpy
   runpy.run_path("/path/to/grippers/ros2/build_gripper_graph.py")["build_graph"](model="2f_140")
   ```

   It finds the articulation root by itself. If the gripper is merged into an arm's articulation,
   or the stage holds several grippers, pass `articulation_root="/World/..."`. Re-running replaces
   the graph, so run it again after switching the Physics variant.
3. **Press Play.**
4. **Start the ROS side**, in a shell where ROS 2 is sourced (and not the one that started Isaac,
   see [the Gripper Simulation Guide](../GRIPPER_SIMULATION_GUIDE.md)):

   ```bash
   ros2 launch robotiq_description robotiq_control.launch.py gripper_model:=2f_85 sim_topic_based:=true
   ```

   No `robotiq/robotiq-ros` on your machine? [`tests/ros2/Dockerfile`](../../tests/ros2/Dockerfile) builds
   the stack in a container (Jazzy by default, `--build-arg ROS_DISTRO=humble` for Humble). Run it
   with the UDP-only FastDDS profile next to it, or the container discovers Isaac Sim but its
   messages never arrive (shared memory and data sharing between a container and the host):

   ```bash
   docker build -t robotiq-ros tests/ros2
   docker run --rm -it --network host \
     -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml -v $PWD/tests/ros2/fastdds_udp.xml:/fastdds_udp.xml \
     robotiq-ros bash -c "source /ws/install/setup.bash && \
     ros2 launch robotiq_description robotiq_control.launch.py gripper_model:=2f_85 sim_topic_based:=true"
   ```
5. **Check it.** Close, then open:

   ```bash
   # Jazzy / Lyrical
   ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand \
     "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [0.5]}}"
   ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand \
     "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [0.0]}}"

   # Humble
   ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
     "{command: {position: 0.5, max_effort: 50.0}}"

   ros2 topic echo /joint_states --once
   ```

   The gripper moves in the viewport and both goals succeed. `position` is the driven joint in
   radians, from 0 (open) to the closed position in the table above; on a 2F-140 the joint is
   `finger_joint`. See "Commanding the gripper" in the `robotiq/robotiq-ros` README for the goal fields.
   `/joint_states` carries the knuckle (and, up to ros 1.3, the mimic joints at 0: see
   [Known limitation](#known-limitation-ros--13)).

## Options

`build_graph` keyword arguments:

| Argument | Default | |
|---|---|---|
| `model` | `"2f_85"` | `"2f_85"` or `"2f_140"`, as the launch's `gripper_model` |
| `articulation_root` | found in the stage | the prim carrying the articulation root |
| `prefix` | `""` | the URDF's `prefix` xacro argument, for several grippers on one robot |
| `command_topic` | `"/sim/joint_commands"` | match `sim_joint_commands_topic` |
| `state_topic` | `"/sim/joint_states"` | match `sim_joint_states_topic` |

## Troubleshooting

Isaac Sim reports most graph mistakes only in the log, if at all.

- **Nothing moves and nothing is published.** The timeline must be playing. Check the topics with
  `ros2 topic list` and compare their names with the launch arguments.
- **Nothing moves, and the log says `[robotiq-ros2] commands on ... name [...], not '...'`.** The
  commands name another joint than the graph expects: `model` or `prefix` does not match the
  launch's `gripper_model` / `prefix`. Rebuild the graph with the right ones.
- **The script fails with "no articulation with a joint named ...".** Wrong `model`, or the
  gripper's articulation root was removed (a gripper merged into an arm keeps its joints but the
  root moves): pass `articulation_root`.
- **The script fails with "several articulations have a joint named ...".** The stage holds more
  than one gripper: pass `articulation_root`, and a graph path of its own per gripper
  (`graph_path`).
- **Checking a value by hand.** The property panel shows a wrong value exactly like a right one,
  for example `jointNames` stored as one literal string token instead of a list. Read it back in
  the Script Editor with `repr()`:

  ```python
  import omni.graph.core as og
  print(repr(og.Controller.attribute("/World/Robotiq_ROS2/Art.inputs:targetPrim").get()))
  ```

  The script checks the targets and topic names it stores for you and raises on a mismatch.
- **The goal is aborted instead of succeeding when the gripper grasps an object.** By design:
  `robotiq_controllers.topic_based.yaml` sets `allow_stalling: false`, because the simulation
  has no object-detection status (the real gripper's gOBJ). Your client decides whether a stalled
  goal means an object is held.

## Tests

`tests/runtime/test_gripper_ros2.py` builds the graph for every 2F gripper and Physics variant,
sends the command the plugin sends from a separate ROS 2 process, and checks the driven joint
reaches the target and is reported under its URDF name. It needs a system ROS 2 and, like the
other runtime tests, a local Isaac Sim: it is not part of the GitHub CI.
