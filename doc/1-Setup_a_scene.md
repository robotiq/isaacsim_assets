# Setup a scene

Here are some basic instructions about how to setup a basic isaac sim scene to
start playing with robotiq Isaac Sim assets.

Start a new Isaac Sim file and create an environment
**Create → Environments → Simple Room**
   
![Select a Robot](../static/img/2-add%20environment.png)

The Simple Room comes with a low table at the centre of the room, its top at
about height `0`. The robot stands on it (§1.1).

## Add a cube to manipulate

The pick and place examples (see *Control the robot*) need an object to pick:
a 5 cm cube on the table, in reach of the robot.

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

### Universal Robot URe5

1. Open the asset browser: **Create → Robots → Asset Browser**.
2. Go to **Robots → UniversalRobots → ur5e** and drag `ur5e.usd` onto `/World`
   in the **Stage** panel. It appears as `/World/ur5e`; the paths below assume
   that name.

   ![Select a Robot](../static/img/Add_ur5e.png)

3. Place select `/World/ur5e`, then selcet UR5e in the stage tree and set the
   `Transfom → Translate` property inside the property panel to `0, 0, 0`

   ![Select a Robot](../static/img/UR5e%20positioning.png)

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

   ![Robot home position](../static/img/UR5e%20home.png)

5. **Only with a Newton gripper variant: prepare the arm for Newton.** Skip
   this step with PhysX. Under Newton, the whole robot runs on Newton, not
   only the gripper. The UR5e asset is tuned for PhysX: its joint drives
   are very stiff and have almost no damping, and Newton makes the arm buzz
   in place. In our tests, joint speeds reached 8.2 rad/s with the arm
   commanded to hold still, and dropped to 0.4 rad/s with these settings:

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
   PhysX runs the arm well with the stock values, and the Newton values
   haven't been tested with it. To keep one scene for both engines, the
   teleop demo puts them in a variant (`Gripper = newton` on
   `/World` in `grippers/2F/demo/teleop/ur5robot_with_2F-85.usda`).

## 1.2 Scene with Franka Panda arm

Use this section instead of §1.1 if you want a Franka Panda. Unlike the UR5e,
the Franka ships with its own two-finger hand, which you must remove before
mounting a Robotiq gripper. Setting the asset's **Gripper** variant to `None`
does **not** remove it.

### Add the Franka to the scene

1. Open the asset browser: **Create → Robots → Asset Browser**.
2. Go to **Robots → FrankaRobotics → FrankaPanda** and drag `franka.usd` onto
   `/World` in the **Stage** panel. It appears as `/World/franka`; the scripts
   below assume that name.
3. Select `/World/franka`, then in **Property → Transform** set **Translate**
   to `0, 0, 0`.
4. In **Property → Variants**, set **Gripper** to `None`. Do not pick
   `Robotiq_2F_85` there: that is the Isaac Sim gripper asset, not the
   Robotiq one from this repository.

   ![Remove Franka Panda Defaut gripper](../static/img/Franka_panda_None_gripper.png)

### Remove the Franka hand and set the home pose

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
   gripper is mounted on it in §1.3. The fingers disappear from the viewport
   right away.

2. **Set the home pose.** The default pose of the Franka (every joint at `0°`)
   is outside the limits of `panda_joint4`, so give the arm its standard
   *ready* pose:

   | Joint | `panda_joint1` | `2` | `3` | `4` | `5` | `6` | `7` |
   |---|---|---|---|---|---|---|---|
   | Angle (deg) | `0` | `-45` | `0` | `-135` | `0` | `90` | `45` |

   Open **Tools → Robotics → Joint Inspector** and select `/World/franka`. For
   each joint in the table, set both **Target Position** and
   **State Position** to the angle above, as for the UR5e (§1.1 step 4).
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

![Franka Panda Home Position](../static/img/franka%20panda%20home.png)

## 1.3 Attach Robotiq gripper to the robot

> **Warning:** Use this method rather than Isaac Sim's **Robot Assembler**
> (**Tools → Robotics → Asset Editors → Robot Assembler**).

1. **Get the gripper asset from Robotiq Isaac Sim Asset repository.** Clone
   the repository in your project folder. The `.usd` files are stored with
   [Git LFS](https://git-lfs.com), so install it *before* cloning. Open a
   terminal in the folder where you want the project (for example
   `Documents/GitHub`) and run:

   ```bash
   git lfs install
   git clone https://github.com/robotiq/isaacsim_assets.git
   ```

   This creates an copy of the Robotiq Isaac Sim Asset repository in your project folder.

   Open the **Content** browser (**Window → Browsers → Content**)
   and navigate inside the previously cloned Robotiq Isaac Sim Asset repository
   to the folder of the Robotiq gripper asset you want to import.

2. **Add the gripper to the robot.** In the **Content** browser, drag the usda
   file of the gripper you want to import (`Robotiq_2F_85.usda` in this example)
   onto the robot's flange body in the **Stage** panel.

   | Robot | `ROBOT` | Flange body (`FLANGE`) |
   |---|---|---|
   | UR5e (§1.1) | `/World/ur5e` | `/World/ur5e/wrist_3_link` |
   | Franka Panda (§1.2) | `/World/franka` | `/World/franka/panda_hand` |

   The Robotiq gripper lines up with both flange bodies as is: you do not need to
   move or rotate it.

   ![Drag and drop the gripper on the flange body](../static/img/UR5e%20attach%20gripper.png)

   > **Warning** This needs **Isaac Sim 6.x**. Isaac Sim 5.0 does not simulate
   > rigid bodies nested inside another rigid body: every gripper joint then
   > fails with `CreateJoint - no bodies defined at body0 and body1`.

   > **Warning**
   > Do not use the default gripper asset available in Isaac Sim. This asset is
   > not developped by robotiq and got some important limitation.

3. **Choose gripper variant.**
   Robotiq Isaac Sim asset have some variant
   which have the effect of changing the configuration of the asset.
   As an example, variant will let you change the finger of the 2F85, select
   the physics egine and grip mechanism.
   Select the gripper prism in the stage tree (exampel: `Robotiq_2F_85` prim)
   and set the variant of your choose in the `property` panel.

   ![Select gripper variant](../static/img/Ur5e%202F85%20variant.png)

4. **Connect the gripper to the robot.**
   By default, the gripper asset is set up as a standalone robot, to be
   simulated without an arm. Two things make it standalone, and both must be
   undone on an arm:

   - Its `root_joint` fixes it at a fixed position in the world. On an arm,
     it would hold the gripper in place while the arm tries to move it.
   - Its articulation root makes it a separate robot for the physics engine.
     On an arm, its joints must belong to the arm's articulation instead.

   A fixed joint must then weld the gripper to the flange. Being a
   child of the flange body only
   places the gripper there; it does not connect them physically. With the
   timeline stopped, in the **Stage** panel expand the gripper down to its
   inner prim, `<FLANGE>/Robotiq_2F_85/Robotiq_2F_85` (two levels named
   `Robotiq_2F_85`), then:

   1. **Remove the gripper's articulation root.** Select the inner
      `Robotiq_2F_85` prim. In the **Property** panel, find the
      **Articulation Root** section under **Physics** and click its remove
      button (**✕**). The gripper joints then become part of the robot's
      articulation.

      ![Delete gripper root articulation 1](../static/img/UR5e%20delete%202F85%20root%20articulation.png)

      ![Delete gripper root articulation 2](../static/img/UR5e%20delete%202F85%20root%20articulation%202.png)

   2. **Disable the gripper's world joint.** Expand the inner `Robotiq_2F_85`
      prim, right-click `root_joint` and choose **Deactivate**.
   
      ![Desactivate gripper root joint](../static/img/UR5e%20desactivate%202F85%20root%20joint.png)

   3. **Weld the gripper to the flange.** Click the flange body (`FLANGE` in
      the table above), then **Ctrl+click** the gripper's `base_link`, inside
      the inner `Robotiq_2F_85` prim. The order matters: flange first. Then
      choose **Create → Physics → Joint → Fixed Joint**. Isaac Sim creates
      `base_link/FixedJoint` between the two bodies.

      ![Weld the gripper to the flange](../static/img/UR5e%20attach%202F85%20to%20flange%20-%20fixed%20joint.png)

   <details>
   <summary>Alternative: do the same three changes with a script</summary>

   Open **Window → Script Editor**, set `ROBOT` and `FLANGE` from the table
   above, paste the script and click **Run**:

   ```python
   import omni.usd
   from pxr import UsdPhysics, Sdf

   ROBOT = "/World/ur5e"             # Franka: "/World/franka"
   FLANGE = f"{ROBOT}/wrist_3_link"  # Franka: f"{ROBOT}/panda_hand"
   GRIPPER = f"{FLANGE}/Robotiq_2F_85/Robotiq_2F_85"

   stage = omni.usd.get_context().get_stage()
   gripper = stage.GetPrimAtPath(GRIPPER)
   if not gripper:
       raise RuntimeError(f"No gripper at {GRIPPER}: check ROBOT, FLANGE and step 2")

   # One articulation for arm + gripper: remove the gripper's own root.
   gripper.RemoveAPI(UsdPhysics.ArticulationRootAPI)
   gripper.RemoveAppliedSchema("PhysxArticulationAPI")
   # Stop the gripper from being fixed to the world.
   stage.GetPrimAtPath(f"{GRIPPER}/root_joint").SetActive(False)

   # Weld the gripper base to the robot flange.
   joint = UsdPhysics.FixedJoint.Define(stage, f"{ROBOT}/RobotiqFixedJoint")
   joint.CreateBody0Rel().SetTargets([Sdf.Path(FLANGE)])
   joint.CreateBody1Rel().SetTargets([Sdf.Path(f"{GRIPPER}/base_link")])
   ```

   </details>

5. **Check the result.** Press **Play**. The arm moves straight to its home
   pose with the gripper on its flange, pointing away from the arm, and the
   **Console** shows no `PhysicsUSD` or `PhysxMimicJointAPI` errors.

   ![Robot initial position](../static/img/UR5e%20with%202F85%20initialisation.png)

> **Note:** If the gripper falls apart or spins, and the Console shows
> `failed to find internal joint object for PhysxMimicJointAPI`, the gripper
> joints are not part of the robot's articulation. Press **Stop**, check
> that the inner `Robotiq_2F_85` prim no longer has an **Articulation Root**
> section and that `root_joint` is deactivated (step 4), and press **Play**
> again.
   
## Test the gripper from the GUI

Here is a quick way to play with the robot and see the gripper work. Do
these steps **while the simulation runs**: the joint motors then move the
robot, smoothly, like a real robot.

1. Press **Play**.
2. Open **Tools → Physics → Physics Inspector** and select the robot's
   **articulation root**: `/World/ur5e/root_joint` for the UR5e (on the
   Franka Panda, the root is `/World/franka`). Sliders appear for each robot
   joint, including the gripper joint.
3. **Choose what the sliders change.** In the inspector toolbar, open the
   options menu and, under the slider options, pick
   **Joint Drives Target Position**. The sliders then set where each joint's
   motor should go. (**Joint States Position** would teleport the joints
   instead.)
4. **Move the arm.** Drag the slider of an arm joint, for example
   `wirst_3_joint`.
5. **Open and close the gripper.** Drag the `finger_joint` slider.
6. Press **Stop**. The robot goes back to its starting pose, but the targets
   you changed **stay changed**: on the next **Play**, the motors drive the
   robot to the last slider values. To start from the home pose again, set
   the targets back to the values of §1.1 (or §1.2 for the Franka) with the
   same sliders, during **Play**.

![Move the robot with Physics Inspector](../static/img/UR5e%20physics%20inspector.png)

<details>
<summary>Alternative: move the robot with the Joint Inspector</summary>

Press **Play** and open **Tools → Robotics → Joint Inspector**. It shows one
robot at a time: pick the arm (`/World/ur5e`) or the gripper
(`/World/ur5e/wrist_3_link/Robotiq_2F_85`) in the robot drop-down at the top,
then change a joint's **Target Position**. The drive moves the joint there.

The targets you change **stay changed** after **Stop**, so the robot no
longer starts in its home pose. Set them back to the values of §1.1 when you
are done.

</details>

