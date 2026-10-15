# Get started

Attach a Robotiq gripper to the robot of [Setup your scene](1-Setup_your_scene.md),
then open and close it from the Isaac Sim GUI. The steps use the 2F-85 on a
UR5e; the other grippers and the Franka Panda work the same way.

## Attach the Robotiq gripper to the robot

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
   | [UR5e](1-Setup_your_scene.md#universal-robots-ur5e) | `/World/ur5e` | `/World/ur5e/wrist_3_link` |
   | [Franka Panda](1-Setup_your_scene.md#franka-panda) | `/World/franka` | `/World/franka/panda_hand` |

   The Robotiq gripper lines up with both flange bodies as is: you do not need to
   move or rotate it.

   ![Drag and drop the gripper on the flange body](img/UR5e%20attach%20gripper.png)

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

   ![Select gripper variant](img/Ur5e%202F85%20variant.png)

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

      ![Delete gripper root articulation 1](img/UR5e%20delete%202F85%20root%20articulation.png)

      ![Delete gripper root articulation 2](img/UR5e%20delete%202F85%20root%20articulation%202.png)

   2. **Disable the gripper's world joint.** Expand the inner `Robotiq_2F_85`
      prim, right-click `root_joint` and choose **Deactivate**.
   
      ![Desactivate gripper root joint](img/UR5e%20desactivate%202F85%20root%20joint.png)

   3. **Weld the gripper to the flange.** Click the flange body (`FLANGE` in
      the table above), then **Ctrl+click** the gripper's `base_link`, inside
      the inner `Robotiq_2F_85` prim. The order matters: flange first. Then
      choose **Create → Physics → Joint → Fixed Joint**. Isaac Sim creates
      `base_link/FixedJoint` between the two bodies.

      ![Weld the gripper to the flange](img/UR5e%20attach%202F85%20to%20flange%20-%20fixed%20joint.png)

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

   ![Robot initial position](img/UR5e%20with%202F85%20initialisation.png)

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
   the targets back to the home pose of [Setup your scene](1-Setup_your_scene.md#setup-the-robot) with the
   same sliders, during **Play**.

![Move the robot with Physics Inspector](img/UR5e%20physics%20inspector.png)

<details>
<summary>Alternative: move the robot with the Joint Inspector</summary>

Press **Play** and open **Tools → Robotics → Joint Inspector**. It shows one
robot at a time: pick the arm (`/World/ur5e`) or the gripper
(`/World/ur5e/wrist_3_link/Robotiq_2F_85`) in the robot drop-down at the top,
then change a joint's **Target Position**. The drive moves the joint there.

The targets you change **stay changed** after **Stop**, so the robot no
longer starts in its home pose. Set them back to the home pose when you are done.

</details>

---

**Next:** [Do a pick and place](3-Do_a_pick_and_place.md): pick up the cube with a script.
