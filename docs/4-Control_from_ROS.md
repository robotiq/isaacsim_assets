# Control the robot and the gripper from ROS 2

This guide explains how to set up Isaac Sim so that ROS 2 can read the
robot's joint states and send it joint commands, for the arm and the gripper.

The steps are for **Linux**, with Isaac Sim and ROS 2 on the same machine.
On **Windows**, run ROS 2 in WSL and open the *Windows* sections along the way.

## How it fits together

```text
 ROS 2                                     Isaac Sim
 ┌──────────────────┐                      ┌──────────────────────┐
 │ any ROS 2 node   │◀────────────────────▶│ Joint States graph   │
 │ or package       │                      │ (ROS 2 bridge)       │
 └──────────────────┘                      └──────────────────────┘

 /isaac_joint_commands : ROS 2 ──▶ Isaac Sim   (joint commands)
 /isaac_joint_states   : Isaac Sim ──▶ ROS 2   (joint states)
```

Isaac Sim publishes the robot's joint states on `/isaac_joint_states` and
applies the joint commands it receives on `/isaac_joint_commands`. An action
graph does both. Both topics carry `sensor_msgs/msg/JointState` messages.

<details>
<summary>Windows: Isaac Sim on Windows, ROS 2 in WSL</summary>

ROS 2 runs in WSL, and the messages go through a **Zenoh router** that runs
in WSL:

```text
 ROS 2 (WSL)                                     Isaac Sim (Windows)
 ┌──────────────────┐     ┌────────────────┐     ┌──────────────────────┐
 │ any ROS 2 node   │◀───▶│  Zenoh router  │◀───▶│ Joint States graph   │
 │ or package       │     │  (rmw_zenohd)  │     │ (ROS 2 bridge)       │
 └──────────────────┘     └────────────────┘     └──────────────────────┘
```

**Why Zenoh and not DDS.** With the default DDS middleware, ROS 2 finds the
other side by multicast, and multicast does not cross between Windows and
WSL. Zenoh uses one TCP connection to the router instead. It is also Isaac
Sim 6.1's default on Windows.

</details>

## 1. Install ROS 2

On Ubuntu 24.04, install ROS 2 Jazzy (see the
[ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).
Every ROS 2 terminal in this guide needs this line first:

```bash
source /opt/ros/jazzy/setup.bash
```

<details>
<summary>Windows: set up WSL and Zenoh</summary>

1. **Use mirrored networking in WSL.** In `%UserProfile%\.wslconfig`:

   ```ini
   [wsl2]
   networkingMode=mirrored
   ```

   then `wsl --shutdown` and reopen WSL. Mirrored mode shares `localhost`
   between Windows and WSL, which is what the router address
   `127.0.0.1:7447` relies on.

2. **Install ROS 2 Jazzy and Zenoh in WSL** (Ubuntu 24.04):

   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-rmw-zenoh-cpp
   ```

3. **Every ROS 2 terminal** in WSL needs these two lines first:

   ```bash
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

</details>

## 2. Start Isaac Sim and enable the ROS 2 bridge

Start Isaac Sim (`isaac-sim.sh`). Isaac Sim uses its own ROS 2 Jazzy
libraries and the same default middleware as ROS 2, so the two sides find
each other without extra settings. If you set `ROS_DOMAIN_ID` in your ROS 2
terminals, set the same value in the terminal you start Isaac Sim from.

Then **enable the ROS 2 bridge**: **Window → Extensions**, search
`isaacsim.ros2.bridge`, turn it on and tick **Autoload**, so it is already
running when you open scenes.

<details>
<summary>Windows: start the Zenoh router, then Isaac Sim</summary>

1. **Start the Zenoh router** in a WSL terminal (set up as in step 1):

   ```bash
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ```

   Leave it running. Start the router **before** Isaac Sim: Isaac Sim
   connects to it when it starts. If you restart the router, restart Isaac
   Sim too.

2. **Start Isaac Sim** from PowerShell, with this variable set in the
   **same window**:

   ```powershell
   $env:ZENOH_CONFIG_OVERRIDE = 'connect/endpoints=["tcp/127.0.0.1:7447"];listen/endpoints=["tcp/127.0.0.1:0"]'
   C:\isaacsim\isaac-sim.bat
   ```

3. Enable the ROS 2 bridge as above.

The order matters: router, then Isaac Sim, then **Play**, then your ROS 2
nodes.

</details>

## 3. Create the joint states graph

**Tools → Robotics → ROS 2 OmniGraphs → Joint States**, then fill in:

| Field | Value |
|---|---|
| **Articulation Root** | `/World/ur5e/root_joint` |
| **Publisher** | ticked |
| **Publisher Topic** | `/isaac_joint_states` |
| **Subscriber** | ticked |
| **Subscriber Topic** | `/isaac_joint_commands` |
| **Move Robot?** | ticked |

Leave **Graph Path** and **Node Namespace** at their defaults, click **OK**,
and save the scene. Press **Play**: the graph only runs while the simulation
plays.

The articulation root covers the arm and the gripper: once the gripper is
attached ([Get started](2-Get_started.md)), its joints are part of the arm's
articulation, so one graph handles both.

## 4. Test the connection

With Isaac Sim playing, in a ROS 2 terminal:

1. **See the topics.**

   ```bash
   ros2 topic list
   ```

   The list includes `/isaac_joint_states` and `/isaac_joint_commands`.

2. **Read the joint states.**

   ```bash
   ros2 topic echo /isaac_joint_states --once
   ```

   You get the names and positions of the six arm joints and of the
   gripper joints (`finger_joint` and the joints that follow it).

3. **Close the gripper** by sending a command:

   ```bash
   ros2 topic pub --once -w 1 /isaac_joint_commands sensor_msgs/msg/JointState \
     "{name: ['finger_joint'], position: [0.7]}"
   ```

   and open it again with `position: [0.0]`. `-w 1` waits until Isaac Sim's
   subscriber is connected before publishing.

## 5. Connect your own ROS 2 code

Any node that publishes on `/isaac_joint_commands` and reads
`/isaac_joint_states` can now drive the robot. What it must respect:

- **Joint names.** Commands use Isaac Sim's joint names, the same ones
  `/isaac_joint_states` reports:

  | Part | Joints to command |
  |---|---|
  | UR5e arm | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
  | 2F-85 | `finger_joint` only |

  Command only `finger_joint` for the gripper: the other finger joints follow
  it, and commanding them too fights the gripper's mechanism. A command
  for a name the robot does not have does nothing, and nothing reports an
  error. If your ROS package names the joints differently, for example
  a gripper description that calls the driven joint
  `robotiq_85_left_knuckle_joint`, add a small node that renames them.
- **Units.** Positions are in radians for revolute joints: `finger_joint`
  goes from `0.0` (open) to about `0.8` (closed).
- **One command can address any subset of joints.** For example, send only
  `finger_joint` to move the gripper without touching the arm.
- **Matching array lengths.** In each command, `position` (and `velocity`
  or `effort` if you fill them) must have one value per name in `name`.
  Isaac Sim silently drops a message whose arrays do not match.
- **QoS.** Isaac Sim publishes and subscribes with the default reliable QoS.
  If your node subscribes with a reliable profile and receives nothing,
  use a best-effort profile (`qos_profile_sensor_data` in Python): it
  accepts both.
- **Run each controller once.** Two nodes commanding the same joints at the
  same time (for example an old launch you forgot to stop) fight over them:
  the gripper twitches between two positions.

## Troubleshooting

- **ROS 2 does not see Isaac Sim's topics** (`ros2 topic list` shows no
  `/isaac_joint_states`): check that the ROS 2 bridge is enabled (step 2),
  that Isaac Sim is playing, and that the scene contains the graph of
  step 3. Check that both sides use the same `ROS_DOMAIN_ID`, and that your
  ROS 2 terminal doesn't set a different `RMW_IMPLEMENTATION`.
- **States arrive but commands do nothing:** check the joint names and array
  lengths (step 5).
- **Isaac Sim crashes when you open a scene with ROS 2 graphs:** enable the
  ROS 2 bridge first (step 2) and let it start before you open the scene.
  Enabling the bridge from a script and opening the scene right after
  can crash Isaac Sim 6.1.
- **Isaac Sim crashes when you press Play with the Newton engine:** the
  Joint States graph of step 3 reads joint efforts, which Newton does not
  provide, and on Isaac Sim 6.1 this crashes Isaac Sim at Play. Use PhysX
  for ROS 2 control.

<details>
<summary>Windows troubleshooting</summary>

- **`Address already in use` when starting the router** (`Unable to open
  listener tcp/[::]:7447`): a router is already running, for example in
  another terminal or left over from an earlier session. Use that one, or
  stop it (`pkill rmw_zenohd`) and start yours. If you restart the router,
  restart Isaac Sim too.
- **ROS 2 does not see Isaac Sim's topics:** check that the router runs and
  that Isaac Sim was started with `ZENOH_CONFIG_OVERRIDE` set (step 2). Check
  that Windows Firewall does not block Isaac Sim: an **NVIDIA Omniverse Kit**
  inbound rule set to **Block** (created if you refused the firewall prompt
  the first time Isaac Sim ran) blocks its network traffic.
- **States arrive but commands do nothing:** check that
  `ZENOH_CONFIG_OVERRIDE` contains **both** `connect` and `listen`, and that
  Isaac Sim was restarted after you set it.

</details>

<!-- docs-site:exclude -->

---

**Next:** [Connect AI agent to Isaac Sim](5-Connect_AI_agent_to_Isaac_Sim.md): let an AI agent build and inspect the scene.

<!-- docs-site:exclude -->
