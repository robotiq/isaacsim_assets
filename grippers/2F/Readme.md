# 2F85/140 Robotiq Adaptive Grippers Isaac Sim assets

# Variants

The 2F-85 exposes two **independent** variant sets on its root prim
(`/World/Robotiq_2F_85`), selected in the Property panel →
**Variants** section.


- **`Fingertip`** — what is mounted on the finger tip
  - `Standard`
  - `Tactile_TSF85`
- **`Physics`** — how the finger linkage is simulated
  - `None` (kinematic only)
  - `Physx_parallel_grip`: Fingers stay parallel. Physic simulation is done
    with Physx engine
  - `Physx_compliant`: The gripper can close in encompassing mode, having its
    fingers surround objects. Physic simulation is done
    with Physx engine
  - `Newton_parallel_grip`: Fingers stay parallel. Physic simulation is done
    with Newton.
  - `Newton_compliant`: The gripper can close in encompassing mode, having its
    fingers surround objects. Physic simulation is done
    with Newton.

![Variant](/static/img/Ur5e%202F85%20variant.png)

![Parallel](./static/img/Parallel.png)

![Encompassing](./static/img/encompassing.png)

## Physics engines

Gripper variant let you choose `Physx` or `Newton` to simulate physics.

| | `Physx` | `Newton` |
|---|---|---|
| Description | Isaac Sim's default engine | Isaac Sim's newer engine, built on MuJoCo-Warp |
| Recommended usage | Gripper use in parallel mode | Gripper use encompassing mode |
| Compute footprint  | Low | High |

| | `Parallel` | `Compliant` |
|---|---|---|
| Behavious | Fingers stays parallel | The gripper can encompass objects.  |
| Linkage modelled | Mimic joints and joint drives couple the finger joints to `finger_joint` | The closed finger linkage is modelled natively, as MuJoCo equality constraints |

## What a Physics variant needs besides selecting it

Selecting a `Physics` variant sets up the gripper itself. The scene and the
way you run Isaac Sim must match it:

| | `Physx_*` variants | `Newton_*` variants |
|---|---|---|
| **Isaac Sim app** | The regular app: `isaac-sim.bat` (Windows), `isaac-sim.sh` (Linux) | The Newton app: `isaac-sim.newton.bat` (Windows), `isaac-sim.newton.sh` (Linux). Under the regular app, a Newton gripper loads but doesn't close |
| **Physics scene** | The stage needs a physics scene: **Create → Physics → Physics Scene** | Same |
| **Finger drive** | Soft by default. To hold objects while the arm moves, raise its stiffness and damping: see [Tuning → PhysX variants](#physx-variants) | Already tuned in the asset |
| **After every Play** | Nothing | Run [`Robotiq_2F_85/newton/apply_gripper_tuning.py`](Robotiq_2F_85/newton/apply_gripper_tuning.py) from **Window → Script Editor**, or the fingers go floppy |
| **Robot arm** | Nothing | The arm runs on Newton too and needs its own settings: see step 5 of [Setup the robot](../../doc/1-Setup_a_scene.md#universal-robot-ure5) |
| **Objects to grasp** | Mass, collider and friction, plus PhysX contact settings: see [Tuning](#tuning) | Mass, collider and friction, plus MuJoCo contact settings: see [Tuning](#tuning) |
| **ROS 2 joint states** | The standard **Joint States** graph works | On Isaac Sim 6.1, the standard **Joint States** graph crashes Isaac Sim at Play, because it asks Newton for joint efforts. The teleop demo replaces it with [`demo/teleop/newton_runtime.py`](demo/teleop/newton_runtime.py) |
| **Isaac Sim version** | 6.x | 6.1. On 6.0.1, the bundled Newton needs a fix: see the box in [§5.2](#52-why-newton-is-better-for-the-compliant-variant) |

The 2F-140 has PhysX variants only; the Newton column applies to the 2F-85.

With every variant, command **only `finger_joint`**: 0 is open, and the
joint limit is fully closed: 47° (0.82 rad) on the 2F-85, 45° (0.79 rad) on
the 2F-140. The other finger joints follow it.

`None` has no physics: the gripper is posed, not simulated, and needs none
of the above.

## Tuning

How the gripper holds an object depends on more than the asset: the object
and a few settings in your scene matter too. If a grasp fails, look up the
symptom below: first in **All engines**, then in the section for the engine
your gripper variant uses.

### All engines

| Symptom | Likely cause | Fix |
|---|---|---|
| The gripper doesn't move, or doesn't close | The gripper variant doesn't match the engine Isaac Sim runs. A `Newton_*` variant under PhysX loads, but doesn't close | Use a `Physx_*` variant with the regular app (`isaac-sim.bat` / `isaac-sim.sh`), and a `Newton_*` variant with the Newton app (`isaac-sim.newton.bat` / `isaac-sim.newton.sh`). |
| The fingers move oddly or fight each other | Targets are sent to the passive finger joints | Command **only `finger_joint`**. The other finger joints follow it through the linkage. |
| The fingers close through the object as if it weren't there | The object has no collider | Add one: **Add → Physics → Collider** on the object. |
| The fingers pass through the object, or it flies away on contact | The object has no mass: primitive shapes often come with mass 0 and density 0 | Give the object a mass, for example `physics:mass = 0.1` kg. |
| The object slides out of the closed fingers | Too little friction between pads and object | Bind a physics material to the object, with friction, for example `physics:staticFriction = 1` and `physics:dynamicFriction = 0.9` (the demo's values). |

![The 2F-85 closing on a small cube](static/img/mimic-grasp-setup.png)

*Test setup (PhysX): the 2F-85 closing on a small cube.*

| Before | After |
|---|---|
| ![Fingers passing through the cube](static/img/mimic-passthrough-before.png) | ![Stable grasp after the fix](static/img/mimic-grasp-after.png) |
| Cube with mass 0: the fingers pass straight through it. | With mass, contact offsets and a capped grip force: the cube is held. |

### PhysX variants

For `Physx_parallel_grip` and `Physx_compliant`. The asset ships these
variants with a soft finger drive, so most of the fixes below are needed in
any scene where the robot moves while gripping.

| Symptom | Likely cause | Fix |
|---|---|---|
| The fingers press into the object | Contact distances too small for a small object | On the object, set `physxCollision:contactOffset = 0.002` and `physxCollision:restOffset = 0.0005` (metres). |
| The object slips or drops while the arm moves | The finger drive is too soft | On `finger_joint`, raise `drive:angular:physics:stiffness` to 50 and `drive:angular:physics:damping` to 3. |
| The fingers crush through the object | Grip force too high | Lower `finger_joint` `drive:angular:physics:maxForce`. The demo uses 1 N·m. |
| The object vibrates in the grip | No damping on the object, or too few solver iterations | On the object, set `physxRigidBody:linearDamping = 0.1` and `physxRigidBody:solverVelocityIterationCount = 4`. On the articulation root, raise `physxArticulation:solverVelocityIterationCount` (the demo uses 8). |
| `Physx_compliant`: the finger linkage wobbles or goes unstable | The closed linkage needs more solver iterations | On the four finger bodies (`{left,right}_{outer,inner}_finger`), set `physxRigidBody:solverPositionIterationCount = 64` and `solverVelocityIterationCount = 8`. On `/PhysicsScene`, set `physxScene:enableStabilization = True`. If it stays unstable, use `Newton_compliant`. |
| Objects pass through the fingers only during fast moves | The object moves further than its thickness in one physics step | On that object only, turn on `physxRigidBody:enableSpeculativeCCD`. |

### Newton variants

For `Newton_parallel_grip` and `Newton_compliant`. The gripper side is
already tuned in the asset: finger drive gains, joint armature and damping,
the linkage constraints, and stiffer contacts on the finger pads. Newton
ignores the `physx*` settings of the PhysX section.

| Symptom | Likely cause | Fix |
|---|---|---|
| The fingers go floppy, or the object slips although friction is set | Two gripper settings can't be saved in USD and are lost at every Play | After **every** Play, run [`Robotiq_2F_85/newton/apply_gripper_tuning.py`](Robotiq_2F_85/newton/apply_gripper_tuning.py) from **Window → Script Editor**. It sets MuJoCo's anti-slip option (`impratio = 10`) and stiffens the coupling between the two fingers. |
| The object sinks into the pads, or the grip feels spongy | The object's contact is softer than the pads' | On the object's collider, set MuJoCo's contact settings `mjc:solref` and `mjc:solimp` (explained below). The pads use `mjc:solref = [0.004, 2]` and `mjc:solimp = [0.95, 0.99, 0.001, 0.5, 2]`. The demo leaves its objects at the MuJoCo defaults (`[0.02, 1]` and `[0.9, 0.95, 0.001, 0.5, 2]`); object values haven't been tuned. |
| The simulation runs much slower than real time | Newton's cost is mostly the number of physics steps | Lower `newton:timeStepsPerSecond` on `/PhysicsScene`. The demo uses 1000; 500 nearly doubled the speed in our tests. Fewer steps also limit how stiff contacts can be (see below). |

What `mjc:solref` and `mjc:solimp` mean:

- **`solref = [time constant, damping ratio]`** is the contact's
  spring-damper. A smaller time constant gives a stiffer contact (the
  stiffness grows as 1 / time constant²), but it must stay at least twice
  the physics step: 0.002 s at 1000 steps per second. A damping ratio of 1
  is critically damped; above 1 removes chatter but feels slower; below 1
  bounces.
- **`solimp`** sets how firmly the no-penetration force is applied as the
  contact goes deeper.

In short, `solref` sets how springy the contact is, and `solimp` how firmly
it's enforced.

### Where to find these settings in Isaac Sim

The tables above use the settings' USD names. The Isaac Sim GUI shows them
under different labels. To change one, select the prim in the **Stage**
panel, then find its label in the **Property** panel. Labels marked
*(Advanced)* are in the collapsed **Advanced** part of their section.

| Setting | Prim to select | Property panel label |
|---|---|---|
| `physics:mass` | The object | **Mass** |
| `physics:staticFriction`, `physics:dynamicFriction` | The physics material bound to the object | **Static Friction**, **Dynamic Friction** |
| `physxCollision:contactOffset` | The object | **Contact Offset** *(Advanced)* |
| `physxCollision:restOffset` | The object | **Rest Offset** *(Advanced)* |
| `physxRigidBody:linearDamping` | The object | **Linear Damping** |
| `physxRigidBody:solverVelocityIterationCount` | The object, or a finger body | **Solver Velocity Iteration Count** *(Advanced)* |
| `physxRigidBody:solverPositionIterationCount` | A finger body | **Solver Position Iteration Count** *(Advanced)* |
| `physxRigidBody:enableSpeculativeCCD` | The object | **Enable Speculative CCD** *(Advanced)* |
| `drive:angular:physics:stiffness`, `damping`, `maxForce` | `finger_joint`, in the gripper's `Joints` folder | **Stiffness**, **Damping**, **Max Force**, in the angular drive section |
| `physxArticulation:solverVelocityIterationCount` | The prim with the robot's articulation root (`root_joint` on the UR5e) | **Solver Velocity Iteration Count** |
| `physxScene:enableStabilization` | `/PhysicsScene` | **Enable Stabilization** *(Advanced)* |
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

Reference reading: NVIDIA's
[rigging closed-loop structures](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/rig_closed_loop_structures.html),
[gripper tuning example](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/gripper_tuning_example.html),
and [joint tuning](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/joint_tuning.html).

---

## 4. Physics of the gripper and physical properties

### 4.1 How the 2F-85 mechanism works (and how compliance works)

The 2F-85 finger is an **underactuated five-bar linkage** — two kinematic DOF,
one driven and one compliant. Two links form a *virtual extendable link*; a
mechanical **pin** constrains the *minimal length* of that virtual link by
limiting the inner angle between the two links.

- At **minimal length**, the pin blocks the compliant DOF, so the linkage
  acts as an effective parallel **four-bar** and keeps the fingertip in a **parallel** grip.
- When the virtual link **extends**, the fingertip **rotates inward**, producing the **encompassing** grip.

![Parallel grip vs encompassing grip](static/img/2f85-grip-modes.png)

*Left: virtual link at minimal length → parallel grip. Right: virtual link
extended → fingertip rotates inward for an encompassing grip on a round object.*

An optional **second pin** can fully lock the relative angle between the two
links, forcing a *constant parallel grip* (no encompassing behavior). This configuration is rarely used.

![Optional pin locking the parallel grip](static/img/2f85-optional-pin.png)

*The optional pin (green) fully locks the angle between the two links, forcing a
constant parallel grip.*

### 4.2 The redundancy / driving spring

Because it is underactuated, one DOF is governed by a **spring** that tends to
keep the finger parallel whenever possible. This is the "compliance" of the real
gripper — and reproducing it faithfully is the whole point of the compliant variant.

<img src="images/2f85-underactuation-spring.png" alt="The underactuation spring in the linkage" width="311">

*The passive DOF is governed by a spring (magenta) that biases the finger back
toward the parallel configuration.*

On the real hardware there is a **torsional spring at the inner-finger joint**:

- **Pre-loaded by 90°**
- **Spring constant ≈ 0.0004 N·m/deg**
- In the open (parallel-finger) configuration this gives a **≈ 37 N·mm**
  restoring moment.

Modelling this spring is what keeps the simulated fingertips parallel. It is accomplished by setting a drive with the aforementioned stiffness and target position.

### 4.3 Maximum grip force

Spec: **2F-85 grip force is adjustable from 20 to 235 N.**

The transmission from the driving joint to fingertip linear position is close to
linear. Estimating the ratio as the full-span average: the driving joint moves
**49°** to drive the fingers their full **85 mm** span. Per finger:

```
r = 85e-3 / 2 / (49/180·π) ≈ 0.0498 m   (effective lever arm)
```

![Finger span vs driving-joint angle](static/img/2f85-transmission-curve.png)

*Finger span (mm) vs driving-joint position (deg): near-linear over the full
0–49° / 85 mm stroke, which justifies the single average lever-arm estimate.*

Motor torque `t = r · f`. For the spec force limits `f_max = (20, 235) N` per
finger, `t = (0.996, 11.70) N·m` per finger. The joint drives **two** fingers, so
×2:

```
driving-joint theoretical torque range ≈ (2, 23.4) N·m
```

**Tuning the drive `maxForce`:** using sensors for the contact forces at the fingertips, we can tune the drive max force to fit the desired max force for our application. This gives us an effective range of roughly **[2, 21.5] N·m**, which fits the theoretical range of (2, 23.4) N·m.

### 4.4 Motor properties

<!--

The motor's datasheet is specified at 48 V, but **the gripper drives it at
24 V**, so the figures must be de-rated. The scaling rules for a brushless DC
motor:

- **Nominal (continuous) torque** is thermally limited (set by current, not
  voltage) → **≈ voltage-independent**.
- **Stall torque** scales **linearly with voltage** (stall current ∝ V/R) →
  ×24/48.
- **Max (no-load) speed** scales **linearly with voltage** → ×24/48.
- **Rotor inertia** is a mechanical property → **voltage-independent**.

48 V datasheet values (source figures, before de-rating to 24 V):
  Nominal (max continuous) torque: 134 mN·m
  Stall torque:                    915 mN·m
  Max (no-load) speed:             10 000 rpm
  Rotor inertia:                   181 g·cm²
-->

| Spec |  |
|---|---|
| Nominal (max continuous) torque | ≈ 134 mN·m |
| Stall torque | ≈ 458 mN·m |
| Max (no-load) speed | ≈ 5 000 rpm (datasheet-scaled; ~2240 rpm is the gripper's actual operating figure) |
| Rotor inertia | 181 g·cm² (mechanical — unchanged) |

The gear ratio between the motor and the Isaac Sim driven joint varies between [26 – 32],
depending on the gripper position. This means the motor has to move by ~30 degrees for the driven joint (out_knuckle) to move by 1 degree, on average.

### 4.5 Drive max speed

Spec finger speed is **2–150 mm/s**. Converting to the driving-joint angular
rate via the same 85 mm ↔ 49° mapping:

```
[2, 150] mm/s  →  /85 · 49  →  ≈ [1.15, 86] deg/s at the driving joint
```

### 4.6 How joint-drive force is evaluated (why more stiffness ≠ more force)

PhysX uses an **implicit** spring/damper formulation for joint drives. The upside
is it prevents excessively large forces even with a poorly chosen timestep, so
the sim is far less likely to explode. The counter-intuitive downside: **beyond a
point, increasing stiffness gives *less* force, not more** (and likewise for
damping). Roughly, the implicit spring force is the explicit force `s·dx` divided
by a denominator that grows unless the timestep is small relative to the drive's
natural frequency. Practical rule: **keep the drive's natural frequency close to
the simulation frequency**, and tune force via `maxForce` + reasonable stiffness
rather than cranking stiffness arbitrarily.

![Implicit drive force formula](static/img/implicit-drive-force-formula.png)

*The implicit spring force: the explicit term `s·dx` is divided by
`dt²·nf² + 1`, so once the timestep is large relative to the drive's natural
frequency `nf`, more stiffness yields less force.*

Reference:
[gripper tuning example](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/gripper_tuning_example.html)
and Erin Catto's
[Soft Constraints (GDC 2011)](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf).

---

## 5. Using Isaac with Newton (and why it's better for the compliant variant)

### 5.1 How to use Isaac with Newton

**Newton** is Isaac Sim's newer physics backend, built on **MuJoCo-Warp**. Isaac
Sim 6 ships it bundled. You select the Newton backend on the physics scene; the
gripper asset is authored with Newton-specific (`mjc:`) attributes and MuJoCo
contact tuning rather than PhysX ones.

The Newton 2F-85 is the `Newton_compliant` variant of the unified asset
([`Robotiq_2F_85/`](Robotiq_2F_85/)) — select `Physics = Newton_compliant`, drop
it in a scene with a `PhysicsScene`, and play. It is **open by default**
(`finger_joint` drive target 0); drive `finger_joint` toward **~0.8 rad (≈45°)**
to close.

**Tuning:** the gripper side is tuned in the asset, except two settings
that must be re-applied after every Play with
[`newton/apply_gripper_tuning.py`](Robotiq_2F_85/newton/apply_gripper_tuning.py).
See [Tuning → Newton variants](#newton-variants) for that step and for the
contact settings of the objects you grasp.

**Performance note:** the UR5 + gripper Newton sim is **physics-bound**. The real
speed lever is the **timestep** (`newton:timeStepsPerSecond`), not solver
iterations. Dropping 1000 Hz → 500 Hz nearly doubled real-time factor (0.16× →
0.27×) by doing ~half the substeps, while render stayed flat (~8.5 ms). Further
levers to try are `use_mujoco_cpu` (kills GPU kernel-launch latency on this tiny
model) and `cone=pyramidal` (cheaper friction) — *not* more iterations.

### 5.2 Why Newton is better for the compliant variant

The loop (five-bar) variant is where Newton earns its place:

- **The five-bar loop closure is modelled natively** as MuJoCo equalities, which
  are far more stable at the joint limits than PhysX's maximal-coordinate loop
  constraints (which showed residual compliance and ringing at the joint limits;
  see [demo/teleop/PHYSICS_TUNING.md](demo/teleop/PHYSICS_TUNING.md)).
- **Correct joint-limit compliance.** The coupler joints' limit stiffness/damping
  (`mjc:solreflimit`) is authored per-radian and holds the fingertip parallel
  with an overdamped limit — after a subtle importer bug was fixed (see box
  below).
- **The parallel-grip spring holds** thanks to a deliberately inflated link
  inertia: `physics:diagonalInertia = (0.001, 0.001, 0.001)` is **~100× the
  physical value, on purpose** — it stabilizes the soft loop-closure equalities so
  the weak parallel-grip spring holds. With physically-small inertias the linkage
  goes floppy and the fingertips sag ~20° out of parallel. Treat it like the
  `mjc:armature`/`damping` tuning: **do not** replace it with CAD
  inertias without re-tuning the loop-closure `solref`.
- Critical joint `mjc:damping` prevents the fingers from ringing;
  `MjcEqualityJointAPI` is applied **on the joint prim**.

> **Isaac Sim 6.0.1 only: joint-limit bug.** The Newton version bundled with
> Isaac Sim 6.0.1 (1.2.1) scales angular joint-limit gains about 57× too
> high, so the finger limits overshoot by about 7° and ring. Newton PR
> **#2736** fixes it; on 6.0.1, apply that fix to the Newton install by hand
> (see [Robotiq_2F_85/newton/README.md](Robotiq_2F_85/newton/README.md)).
> Isaac Sim 6.1 bundles Newton 1.5.0, which already includes the fix.

### 5.3 Where to find the asset

- **Newton 2F-85:** the `Newton_compliant` variant of
  [`Robotiq_2F_85/`](Robotiq_2F_85/) — physics in
  `payloads/Robotiq_2F_85_newton_compliant_physics.usda` (bodies, joints, the
  five-bar loop closure, and collision cooked from the shared visual CADs), plus
  the runtime [`newton/apply_gripper_tuning.py`](Robotiq_2F_85/newton/apply_gripper_tuning.py).

**Provenance:** both the visual meshes (`Defeatured_2F_85_*`, shared with the
PhysX asset) and the Newton physics parameters (masses/inertias, joints, the
five-bar loop closure, and the MuJoCo contact/friction tuning) are Robotiq's own.

**Self-collision:** Newton/MuJoCo auto-excludes only kinematic parent-child body
pairs. The four-bar's `outer_finger`↔`inner_finger` (coupler↔follower) are joined
only by the spherical loop-closure equality — not a parent-child joint — so they
are excluded explicitly via `physics:filteredPairs` (both sides). Without it the
convex hulls overlap inside the linkage and the fingers jam on self-contact
instead of relaxing to the parallel pose.

---

## 6. Testing against the SimReady Foundation

### 6.1 What it is and where to find it

The **SimReady Foundation** is NVIDIA's open validation suite for checking that a
USD asset meets the "SimReady" specification (correct physics, articulation,
Isaac metadata, materials, etc.): <https://github.com/NVIDIA/simready-foundation>.
Use it to validate an asset you maintain before shipping it.

### 6.2 How to use it (and what we found)

**Setup gotchas:**

- The repo's default requirements target Python 3.10, but `simready-validate`
  needs **≥3.11** (we used a **Python 3.12** venv; 3.10 was rejected).
- `pip install -r nv_core/validator_sample/requirements.txt` →
  `simready-validate 2026.4.9`.
- Add **`numpy` + `Pillow`** manually — the material/physics validators import
  them but they're not in `requirements.txt`.

**Running it:** the 2F-85 asset has no `profile_id` metadata, so profile
inference fails and you must specify one. Since it's a gripper (robot body with
driven joints + articulation), run the three **Robot-Body-\*** profiles against
the root `Robotiq_2F_85.usda`.

**Result — all three profiles FAILED, but most checks pass.**
Robot-Body-Neutral is closest: everything passes *except* the driven-joints
feature.

| Feature | Neutral | Runnable | Isaac |
|---|:-:|:-:|:-:|
| Minimal (FET001) | ✅ | ✅ | ✅ |
| RBD Physics (FET003) | ✅ | ✅ | ✅ |
| Multi-body neutral (FET004) | ✅ | — | — |
| Base articulation neutral (FET024) | ✅ | ✅ | — |
| **Driven joints (FET022)** | ❌ | ❌ | ❌ |
| Multi-body PhysX (RB.011) | — | ❌ | ❌ |
| Base articulation PhysX (BA.002) | — | ❌ | ❌ |
| Robot core (RC.\*) | — | ❌ | ❌ |
| Isaac composition (ISA.001) | — | — | ❌ |

**The actual gaps to close (in priority order):**

1. **Driven joints (DJ.001–003, all profiles)** — the core blocker. The
   mimic/driven joints lack a proper `JointStateAPI`, drive configuration, and
   joint limits / state-consistency.
2. **RB.011 (PhysX profiles)** — rigid bodies lack explicit mass and have
   collision shapes with zero/undefined volume, so mass can't be auto-computed.
3. **BA.002** — collision meshes on non-adjacent links overlap/intersect.
4. **RC.\* + ISA.001 (Isaac/Runnable)** — missing Isaac robot metadata: valid
   `isaac:robotType`, root-joint pinning, physics in a dedicated physics layer,
   thumbnail placement, and the structured Isaac payload/reference composition.

---
