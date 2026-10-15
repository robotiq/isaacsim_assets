# Setup your scene

Here are some instructions to build a basic Isaac Sim scene, with a robot arm and a cube to pick,
ready for a Robotiq gripper. Once it is ready, attach the gripper with
[Get started](2-Get_started.md).

For more information about Isaac Sim, check related documentation:

https://docs.isaacsim.omniverse.nvidia.com/latest/index.html

- Start a new Isaac Sim file and create an environment
**Create → Environments → Simple Room**
   
![Select a Robot](img/2-add%20environment.png)

The Simple Room comes with a low table at the centre of the room, its top at
about height `0`.

## Add a cube to manipulate

Let's add a 5 cm cube on the table, in reach of the robot so that we can play
with it.

1. **Create the cube.** **Create → Shape → Cube**. It appears as
   `/World/Cube`, at the centre of the room. Its width is the **Size** shown
   in **Property → Geometry**: `1` (1 m) by default.
2. **Make it 5 cm.** Select `/World/Cube` and, in **Property → Transform**,
   set:
   - **Translate** to `0.5, 0.2, 0.05`: just above the table, in front of
     the robot. It drops onto the table when you press **Play**;
   - **Scale** to `0.05, 0.05, 0.05`: the 1 m cube becomes 5 cm. If
     **Size** is not `1`, use `0.05 / Size` instead.

   Change **Scale**, not **Size**: changing **Size** leaves the cube's
   bounding box at its old width, and the pick and place script uses that
   bounding box to find the cube.
3. **Give it physics.** Right-click `/World/Cube` and choose
   **Add → Physics → Rigid Body with Colliders Preset**. The cube now falls,
   collides and can be grasped. Its mass comes from the default density:
   about 0.125 kg.

When you press **Play**, the cube settles on the table. It must be smaller
than the 85 mm opening of the 2F-85.

## Setup the robot

In this documentation, we present 2 simple robot setup with UR5e and Franka
Panda. Refere to [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/index.html) to learn more about how to setup a
robot.

### Universal Robots UR5e

1. Open the asset browser: **Create → Robots → Asset Browser**.
2. Go to **Robots → UniversalRobots → ur5e** and drag `ur5e.usd` onto `/World`
   in the **Stage** panel. It appears as `/World/ur5e`; the paths below assume
   that name.

   ![Select a Robot](img/Add_ur5e.png)

3. Place select `/World/ur5e`, then selcet UR5e in the stage tree and set the
   `Transfom → Translate` property inside the property panel to `0, 0, 0`

   ![Select a Robot](img/UR5e%20positioning.png)

4. **Give the arm a starting pose above the ground.** The default pose of the
   UR5e robot makes it collide with the ground scene. Give it the UR *home*
   pose instead:

   | Joint | `shoulder_pan` | `shoulder_lift` | `elbow` | `wrist_1` | `wrist_2` | `wrist_3` |
   |---|---|---|---|---|---|---|
   | Angle (deg) | `0` | `-90` | `90` | `-90` | `-90` | `0` |

   With the timeline stopped, open **Tools → Robotics → Joint Inspector** and
   select `/World/ur5e`. For each joint in the table, set both columns to the
   angle above:

   - **Target Position**: the pose the joint drive holds.
   - **State Position**: the pose the simulation starts from. Without it, the
     arm starts inside the ground and swings up when you press **Play**.

   <details>
   <summary>Alternative: set the pose with a script</summary>

   Open **Window → Script Editor**, paste the script and click **Run**:

   ```python
   import omni.usd
   from pxr import UsdPhysics

   ROBOT = "/World/ur5e"  # path of the robot in the Stage panel, e.g. /World/ur5

   stage = omni.usd.get_context().get_stage()
   home = {  # degrees
       "shoulder_pan_joint": 0, "shoulder_lift_joint": -90, "elbow_joint": 90,
       "wrist_1_joint": -90, "wrist_2_joint": -90, "wrist_3_joint": 0,
   }
   for name, angle in home.items():
       joint = stage.GetPrimAtPath(f"{ROBOT}/joints/{name}")
       if not joint:
           raise RuntimeError(f"No joint at {ROBOT}/joints/{name}: check ROBOT")
       UsdPhysics.DriveAPI.Get(joint, "angular").GetTargetPositionAttr().Set(float(angle))
       joint.GetAttribute("state:angular:physics:position").Set(float(angle))
       joint.GetAttribute("state:angular:physics:velocity").Set(0.0)
   ```

   </details>

   The viewport keeps showing the old pose until you press **Play**.

   ![Robot home position](img/UR5e%20home.png)

5. **Only with a Newton gripper variant: prepare the arm for Newton.** Skip
   this step with PhysX. Under Newton, the whole robot runs on Newton, not
   only the gripper.

   > **Warning: robot control under Newton is limited for now.** NVIDIA
   > describes the Newton integration in Isaac Sim 6.1 as experimental (see
   > the [Isaac Sim 6.1 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/overview/release_notes.html),
   > *PhysX and Newton*). Isaac Sim robot assets, the UR5e included, are tuned for PhysX: their joint
   > drives are very stiff and have almost no damping, which MuJoCo, the
   > solver behind Newton, doesn't handle well. Out of the box, the arm
   > vibrates when it should hold still. The settings below are a rough
   > adjustment, not a proper tuning: they stop the vibration, but the arm
   > still follows its targets loosely and can swing or drift. For precise
   > arm control, use PhysX.

   The rough adjustment:

   | Setting, on every arm joint | Value |
   |---|---|
   | Drive damping (`drive:angular:physics:damping`) | The joint's drive **stiffness ÷ 10**, for example 9400.5 → **940.05** on `shoulder_pan_joint` |
   | `mjc:armature` | **1** on `shoulder_pan_joint`, `shoulder_lift_joint` and `elbow_joint`; **0.3** on the three wrist joints |

   `mjc:armature` adds inertia at the joint, the way a real motor's rotor
   does, which keeps MuJoCo's solver stable. The UR5e joints don't have
   this setting yet, so use the script: open **Window → Script Editor**,
   paste it and click **Run**.

   ```python
   import omni.usd
   from pxr import Sdf, UsdPhysics

   ROBOT = "/World/ur5e"  # path of the robot in the Stage panel

   stage = omni.usd.get_context().get_stage()
   armature = {
       "shoulder_pan_joint": 1.0, "shoulder_lift_joint": 1.0, "elbow_joint": 1.0,
       "wrist_1_joint": 0.3, "wrist_2_joint": 0.3, "wrist_3_joint": 0.3,
   }
   for name, value in armature.items():
       joint = stage.GetPrimAtPath(f"{ROBOT}/joints/{name}")
       if not joint:
           raise RuntimeError(f"No joint at {ROBOT}/joints/{name}: check ROBOT")
       drive = UsdPhysics.DriveAPI.Get(joint, "angular")
       drive.GetDampingAttr().Set(drive.GetStiffnessAttr().Get() / 10.0)
       joint.AddAppliedSchema("MjcJointAPI")
       joint.CreateAttribute("mjc:armature", Sdf.ValueTypeNames.Double, False,
                             Sdf.VariabilityUniform).Set(value)
   ```

   Run it once, then save the scene. These settings are for Newton only:
   PhysX runs the arm well with the stock values, and these haven't been
   tested with PhysX. To keep one scene for both engines, the teleop demo
   puts them in a variant (`Gripper = newton` on
   `/World` in `grippers/demo/teleop/ur5robot_with_2F-85.usda`).

### Franka Panda

Use this section instead of [the UR5e](#universal-robots-ur5e) if you want a Franka Panda. Unlike the UR5e,
the Franka ships with its own two-finger hand, which you must remove before
mounting a Robotiq gripper. Setting the asset's **Gripper** variant to `None`
does **not** remove it.

#### Add the Franka to the scene

1. Open the asset browser: **Create → Robots → Asset Browser**.
2. Go to **Robots → FrankaRobotics → FrankaPanda** and drag `franka.usd` onto
   `/World` in the **Stage** panel. It appears as `/World/franka`; the scripts
   below assume that name.
3. Select `/World/franka`, then in **Property → Transform** set **Translate**
   to `0, 0, 0`.
4. In **Property → Variants**, set **Gripper** to `None`. Do not pick
   `Robotiq_2F_85` there: that is the Isaac Sim gripper asset, not the
   Robotiq one from this repository.

   ![Remove Franka Panda Defaut gripper](img/Franka_panda_None_gripper.png)

#### Remove the Franka hand and set the home pose

With the timeline stopped:

1. **Remove the Franka hand.** In the **Stage** panel, expand `/World/franka`
   and `/World/franka/panda_hand`. Select these five prims (**Ctrl+click** to
   select several), right-click one of them and choose **Deactivate**:

   - `panda_leftfinger`
   - `panda_rightfinger`
   - `panda_hand/panda_finger_joint1`
   - `panda_hand/panda_finger_joint2`
   - `panda_hand/geometry`

   Keep `panda_hand` itself active: it is the Franka's flange, and the Robotiq
   gripper is mounted on it in [Get started](2-Get_started.md). The fingers disappear from the viewport
   right away.

2. **Set the home pose.** The default pose of the Franka (every joint at `0°`)
   is outside the limits of `panda_joint4`, so give the arm its standard
   *ready* pose:

   | Joint | `panda_joint1` | `2` | `3` | `4` | `5` | `6` | `7` |
   |---|---|---|---|---|---|---|---|
   | Angle (deg) | `0` | `-45` | `0` | `-135` | `0` | `90` | `45` |

   Open **Tools → Robotics → Joint Inspector** and select `/World/franka`. For
   each joint in the table, set both **Target Position** and
   **State Position** to the angle above, as for the UR5e (step 4).
   Leave the `rootJoint` row unchanged: it fixes the Franka base to the world.
   The arm keeps showing its old pose until you press **Play**.

<details>
<summary>Alternative: do both steps with a script</summary>

Open **Window → Script Editor**, paste the script and click **Run**:

```python
import omni.usd
from pxr import UsdPhysics

ROBOT = "/World/franka"  # path of the robot in the Stage panel

stage = omni.usd.get_context().get_stage()

# Remove the Franka's own fingers and hand geometry. Keep the panda_hand body:
# it is the flange the Robotiq gripper is mounted on.
for path in ["panda_leftfinger", "panda_rightfinger",
             "panda_hand/panda_finger_joint1", "panda_hand/panda_finger_joint2",
             "panda_hand/geometry"]:
    prim = stage.GetPrimAtPath(f"{ROBOT}/{path}")
    if not prim:
        raise RuntimeError(f"No prim at {ROBOT}/{path}: check ROBOT")
    prim.SetActive(False)

# Ready pose, in degrees, for panda_joint1 ... panda_joint7.
home = [0, -45, 0, -135, 0, 90, 45]
for i, angle in enumerate(home, start=1):
    joint = stage.GetPrimAtPath(f"{ROBOT}/panda_link{i - 1}/panda_joint{i}")
    UsdPhysics.DriveAPI.Get(joint, "angular").GetTargetPositionAttr().Set(float(angle))
    joint.GetAttribute("state:angular:physics:position").Set(float(angle))
    joint.GetAttribute("state:angular:physics:velocity").Set(0.0)
```

</details>

![Franka Panda Home Position](img/franka%20panda%20home.png)
