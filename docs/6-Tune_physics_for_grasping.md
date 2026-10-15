# Tune physics for grasping

How a gripper holds an object depends on more than the gripper asset: the
object and a few scene settings matter too. This page covers what applies to
any gripper. Each gripper's own guide covers its specific settings (see
[the gripper list](../grippers/README.md)).

If a grasp fails, look up the symptom below: first in **All engines**, then
in the section for the engine your gripper variant uses (`PhysX` or
`Newton`).

## All engines

| Symptom | Likely cause | Fix |
|---|---|---|
| The gripper doesn't move, or doesn't close | The gripper variant doesn't match the engine Isaac Sim runs. A Newton variant under PhysX loads but doesn't close | Use a PhysX variant with the regular app (`isaac-sim.bat` / `isaac-sim.sh`), and a Newton variant with the Newton app (`isaac-sim.newton.bat` / `isaac-sim.newton.sh`). |
| The fingers move oddly or fight each other | Targets are sent to joints that follow the driven one | Command only the gripper's driven joint. Its guide names it. |
| The fingers close through the object as if it weren't there | The object has no collider | Add one: **Add → Physics → Collider** on the object. |
| The fingers pass through the object, or it flies away on contact | The object has no mass: primitive shapes often come with mass 0 and density 0 | Give the object a mass, for example `physics:mass = 0.1` kg. |
| The object slides out of the closed fingers | Too little friction between pads and object | Bind a physics material with friction to the object, for example `physics:staticFriction = 1` and `physics:dynamicFriction = 0.9` (the demo's values). |

![A 2F-85 closing on a small cube](img/mimic-grasp-setup.png)

*Test setup (PhysX): a 2F-85 closing on a small cube.*

| Before | After |
|---|---|
| ![Fingers passing through the cube](img/mimic-passthrough-before.png) | ![Stable grasp after the fix](img/mimic-grasp-after.png) |
| Cube with mass 0: the fingers pass straight through it. | With mass, contact offsets and a capped grip force: the cube is held. |

## PhysX

| Symptom | Likely cause | Fix |
|---|---|---|
| The fingers press into the object | Contact distances too small for a small object | On the object, set `physxCollision:contactOffset = 0.002` and `physxCollision:restOffset = 0.0005` (metres). |
| The object vibrates in the grip | No damping on the object, or too few solver iterations | On the object, set `physxRigidBody:linearDamping = 0.1` and `physxRigidBody:solverVelocityIterationCount = 4`. On the articulation root, raise `physxArticulation:solverVelocityIterationCount` (the demo uses 8). |
| Objects pass through the fingers only during fast moves | The object moves further than its thickness in one physics step | On that object only, turn on `physxRigidBody:enableSpeculativeCCD`. |

## Newton

Newton ignores the `physx*` settings of the PhysX section.

| Symptom | Likely cause | Fix |
|---|---|---|
| The object sinks into the pads, or the grip feels spongy | The object's contact is softer than the pads' | On the object's collider, set MuJoCo's contact settings `mjc:solref` and `mjc:solimp` (explained below), close to the pads' values given in the gripper's guide. The MuJoCo defaults are `[0.02, 1]` and `[0.9, 0.95, 0.001, 0.5, 2]`. |
| The simulation runs much slower than real time | Newton's cost depends mostly on the number of physics steps | Lower `newton:timeStepsPerSecond` on `/PhysicsScene`. The demo uses 1000; 500 nearly doubled the speed in our tests. Fewer steps also limit how stiff contacts can be (see below). |

What `mjc:solref` and `mjc:solimp` mean:

- **`solref = [time constant, damping ratio]`** is the contact's
  spring-damper. A smaller time constant gives a stiffer contact (the
  stiffness grows as 1 / time constant²), but it must stay at least twice
  the physics step: 0.002 s at 1000 steps per second. A damping ratio of 1
  gives critical damping; above 1, it removes chatter but feels slower;
  below 1, the contact bounces.
- **`solimp`** sets how firmly the no-penetration force is applied as the
  contact goes deeper.

In short, `solref` sets how springy the contact is, and `solimp` how firmly
it is enforced.

## Where to find these settings in Isaac Sim

The tables use the settings' USD names. The Isaac Sim GUI shows them under
different labels. To change one, select the prim in the **Stage** panel,
then find its label in the **Property** panel. Labels marked *(Advanced)*
are in the collapsed **Advanced** part of their section.

| Setting | Prim to select | Property panel label |
|---|---|---|
| `physics:mass` | The object | **Mass** |
| `physics:staticFriction`, `physics:dynamicFriction` | The physics material bound to the object | **Static Friction**, **Dynamic Friction** |
| `physxCollision:contactOffset` | The object | **Contact Offset** *(Advanced)* |
| `physxCollision:restOffset` | The object | **Rest Offset** *(Advanced)* |
| `physxRigidBody:linearDamping` | The object | **Linear Damping** |
| `physxRigidBody:solverVelocityIterationCount` | The object | **Solver Velocity Iteration Count** *(Advanced)* |
| `physxRigidBody:enableSpeculativeCCD` | The object | **Enable Speculative CCD** *(Advanced)* |
| `physxArticulation:solverVelocityIterationCount` | The prim with the robot's articulation root (`root_joint` on the UR5e) | **Solver Velocity Iteration Count** |
| `mjc:solref` | The object's collider | **SolMix**: Isaac Sim 6.1 gives it the wrong label. Check the USD name in **Raw USD Properties** |
| `mjc:solimp` | The object's collider | **SolImp** |
| `newton:timeStepsPerSecond` | `/PhysicsScene` | No label defined: look for `newton:timeStepsPerSecond` |

Two shortcuts:

- Type the label, for example "contact offset", in the **search box** at
  the top of the Property panel.
- The **Raw USD Properties** section, at the bottom of the Property panel,
  lists every setting under its USD name, as written in the tables above.

If a setting isn't shown at all, the prim lacks the physics property that
adds it. Add it with **Add → Physics** at the top of the Property panel,
for example **Rigid Body**, **Collider** or **Mass**. A cube made with
**Create → Shape** has no mass and no rigid-body settings until you do.

Further reading: NVIDIA's
[gripper tuning example](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/gripper_tuning_example.html)
and [joint tuning](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/joint_tuning.html).
