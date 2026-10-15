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
  - `Physx_parallel_grip`: The fingers stay parallel. Physics is simulated
    with PhysX.
  - `Physx_compliant`: The gripper can close in encompassing mode, with its
    fingers wrapping around objects. Physics is simulated
    with PhysX.
  - `Newton_parallel_grip`: The fingers stay parallel. Physics is simulated
    with Newton.
  - `Newton_compliant`: The gripper can close in encompassing mode, with its
    fingers wrapping around objects. Physics is simulated
    with Newton.

The 2F-140 (`/World/Robotiq_2F_140`) exposes the same `Physics` variants; its
`Fingertip` options are `Flat_Overmolded` (default), `Silicone`, `V_Groove` and
`Tactile_TSF140`.

Layer ownership: `payloads/<gripper>_fingertip_physics.usda` holds only the
engine-neutral fingertip bodies, mass and welds, and every Physics variant (PhysX and
Newton) sublayers it. All PhysX-only content (the articulation root and `root_joint`,
plus the `/Meshes` collision groups and every PhysX collider, including the tip colliders for each `Fingertip` variant)
lives in `payloads/<gripper>_physx_common_physics.usda`, shared by `Physx_parallel_grip`
and `Physx_compliant`. Newton variants never sublayer it and author their own colliders in
`<gripper>_newton_common_physics.usda`.

![Variant](img/Ur5e%202F85%20variant.png)

![Parallel](img/Parallel.png)

![Encompassing](img/encompassing.png)

## Physics engines

The gripper variants let you choose `Physx` or `Newton` to simulate physics.

| | `Physx` | `Newton` |
|---|---|---|
| Description | Isaac Sim's default engine | Isaac Sim's newer engine, built on MuJoCo-Warp |
| Recommended usage | Gripper use in parallel mode | Gripper use in encompassing mode |
| Compute footprint  | Low | High |

| | `Parallel` | `Compliant` |
|---|---|---|
| Behavior | The fingers stay parallel | The gripper can encompass objects.  |
| Linkage modelled | Mimic joints and joint drives couple the finger joints to `finger_joint` | The closed finger linkage is modelled natively, as MuJoCo equality constraints |

## What a Physics variant needs besides selecting it

Selecting a `Physics` variant sets up the gripper itself. The scene and the
way you run Isaac Sim must match it:

| | `Physx_*` variants | `Newton_*` variants |
|---|---|---|
| **Isaac Sim app** | The regular app: `isaac-sim.bat` (Windows), `isaac-sim.sh` (Linux) | The Newton app: `isaac-sim.newton.bat` (Windows), `isaac-sim.newton.sh` (Linux). Under the regular app, a Newton gripper loads but doesn't close |
| **Physics scene** | The stage needs a physics scene: **Create → Physics → Physics Scene** | Same |
| **Finger drive** | Soft by default. To hold objects while the arm moves, raise its stiffness and damping: see [Tuning → PhysX variants](#physx-variants) | Already tuned in the asset |
| **After every Play** | Nothing | Run [`Robotiq_2F_85/newton/apply_gripper_tuning.py`](../../Robotiq_2F_85/newton/apply_gripper_tuning.py) from **Window → Script Editor**, or the fingers go floppy |
| **Robot arm** | Nothing | The arm runs on Newton too and needs its own settings: see step 5 of [Setup the robot](../../../docs/1-Setup_your_scene.md#universal-robots-ur5e) |
| **Objects to grasp** | Mass, collider and friction, plus PhysX contact settings: see [Tune physics for grasping](../../../docs/6-Tune_physics_for_grasping.md#physx) | Mass, collider and friction, plus MuJoCo contact settings: see [Tune physics for grasping](../../../docs/6-Tune_physics_for_grasping.md#newton) |
| **ROS 2 joint states** | The standard **Joint States** graph works | On Isaac Sim 6.1, the standard **Joint States** graph crashes Isaac Sim at Play, because it asks Newton for joint efforts: see [Control from ROS](../../../docs/4-Control_from_ROS.md) |
| **Isaac Sim version** | 6.x | 6.1. On 6.0.1, the bundled Newton needs a fix: see the box in [§5.2](#52-why-newton-is-better-for-the-compliant-variant) |

The table applies to both the 2F-85 and the 2F-140.

With every variant, command **only `finger_joint`**: 0 is open, and the
joint limit is fully closed: 47° (0.82 rad) on the 2F-85, 45° (0.79 rad) on
the 2F-140. The other finger joints follow it.

`None` has no physics: the gripper is posed, not simulated, and needs none
of the above.

## Tuning

Start with the general guide,
[Tune physics for grasping](../../../docs/6-Tune_physics_for_grasping.md): the
object (mass, collider, friction) and the scene settings matter as much as
the gripper. This section covers the settings specific to the 2F-85 and
2F-140. Command **only `finger_joint`** with every variant: the other finger
joints follow it.

### PhysX variants

For `Physx_parallel_grip` and `Physx_compliant`. The asset ships these
variants with a soft finger drive, so most of the fixes below are needed in
any scene where the robot moves while gripping.

| Symptom | Likely cause | Fix |
|---|---|---|
| The object slips or drops while the arm moves | The finger drive is too soft | On `finger_joint`, raise `drive:angular:physics:stiffness` to 50 and `drive:angular:physics:damping` to 3. |
| The fingers crush through the object | Grip force too high | Lower `finger_joint` `drive:angular:physics:maxForce`. The demo uses 1 N·m. |
| `Physx_compliant`: the finger linkage wobbles or goes unstable | The closed linkage needs more solver iterations | On the four finger bodies (`{left,right}_{outer,inner}_finger`), set `physxRigidBody:solverPositionIterationCount = 64` and `solverVelocityIterationCount = 8`. On `/PhysicsScene`, set `physxScene:enableStabilization = True`. If it stays unstable, use `Newton_compliant`. |

### Newton variants

For `Newton_parallel_grip` and `Newton_compliant`. The gripper side is
already tuned in the asset: finger drive gains, joint armature and damping,
the linkage constraints, and stiffer contacts on the finger pads
(`mjc:solref = [0.004, 2]`, `mjc:solimp = [0.95, 0.99, 0.001, 0.5, 2]`; the
demo leaves its objects at the MuJoCo defaults, not tuned).

| Symptom | Likely cause | Fix |
|---|---|---|
| The fingers go floppy, or the object slips although friction is set | Two gripper settings can't be saved in USD and are lost at every Play | After **every** Play, run [`Robotiq_2F_85/newton/apply_gripper_tuning.py`](../../Robotiq_2F_85/newton/apply_gripper_tuning.py) from **Window → Script Editor**. It sets MuJoCo's anti-slip option (`impratio = 10`) and stiffens the coupling between the two fingers. |

### Where to find these settings in Isaac Sim

Select the prim in the **Stage** panel, then find its label in the
**Property** panel. Object and scene settings are in the
[general guide](../../../docs/6-Tune_physics_for_grasping.md#where-to-find-these-settings-in-isaac-sim).

| Setting | Prim to select | Property panel label |
|---|---|---|
| `drive:angular:physics:stiffness`, `damping`, `maxForce` | `finger_joint`, in the gripper's `Joints` folder | **Stiffness**, **Damping**, **Max Force**, in the angular drive section |
| `physxRigidBody:solverPositionIterationCount`, `solverVelocityIterationCount` | A finger body | **Solver Position / Velocity Iteration Count** *(Advanced)* |
| `physxScene:enableStabilization` | `/PhysicsScene` | **Enable Stabilization** *(Advanced)* |

Reference reading: NVIDIA's
[rigging closed-loop structures](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/rig_closed_loop_structures.html).

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

![Parallel grip vs encompassing grip](img/2f85-grip-modes.png)

*Left: virtual link at minimal length → parallel grip. Right: virtual link
extended → fingertip rotates inward for an encompassing grip on a round object.*

An optional **second pin** can fully lock the relative angle between the two
links, forcing a *constant parallel grip* (no encompassing behavior). This configuration is rarely used.

![Optional pin locking the parallel grip](img/2f85-optional-pin.png)

*The optional pin (green) fully locks the angle between the two links, forcing a
constant parallel grip.*

### 4.2 The redundancy / driving spring

Because it is underactuated, one DOF is governed by a **spring** that tends to
keep the finger parallel whenever possible. This is the "compliance" of the real
gripper — and reproducing it faithfully is the whole point of the compliant variant.

<img src="img/2f85-underactuation-spring.png" alt="The underactuation spring in the linkage" width="311">

*The passive DOF is governed by a spring (magenta) that biases the finger back
toward the parallel configuration.*

On the real hardware there is a **torsional spring at the inner-finger joint**:

- **Pre-loaded by 90°**
- **Spring constant ≈ 0.0004 N·m/deg** (2F-85), **≈ 0.00068 N·m/deg** (2F-140)
- In the open (parallel-finger) configuration, this gives a **≈ 37 N·mm** (2F-85), **≈ 61 N·mm** (2F-140)
  restoring moment.

Modelling this spring is what keeps the simulated fingertips parallel. This is done with a joint drive that uses this stiffness and target position.

The spring on the real hardware: its free and pre-constrained positions on
the finger, and its drawing.

![Free and pre-constrained spring positions](img/2f85-link-angles.png)

*Spring positions on the finger: free position, pre-constrained position, and
the reference used to measure the angle.*

![2F-85 torsion spring drawing](img/2f85-torsion-spring.png)

*Spring drawing (mm): 120° free angle, 6-1/6 coils, 0.81 mm wire, 7.75 mm inside
diameter.*

### 4.3 Maximum grip force

Spec: **2F-85 grip force is adjustable from 20 to 235 N.**

The transmission from the driving joint to the fingertip linear position is close to
linear. We estimate the ratio as the full-span average: the driving joint moves
**49°** to drive the fingers their full **85 mm** span. Per finger:

```
r = 85e-3 / 2 / (49/180·π) ≈ 0.0498 m   (effective lever arm)
```

![Finger span vs driving-joint angle](img/2f85-transmission-curve.png)

*Finger span (mm) vs driving-joint position (deg): near-linear over the full
0–49° / 85 mm stroke, which justifies the single average lever-arm estimate.*

Motor torque `t = r · f`. For the spec force limits `f_max = (20, 235) N` per
finger, `t = (0.996, 11.70) N·m` per finger. The joint drives **two** fingers, so
×2:

```
driving-joint theoretical torque range ≈ (2, 23.4) N·m
```

**Tuning the drive `maxForce`:** with sensors that measure the contact forces at the fingertips, we can tune the drive max force to match the grip force needed by the application. This gives us an effective range of roughly **[2, 21.5] N·m**, which fits the theoretical range of (2, 23.4) N·m.

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

The gear ratio between the motor and the Isaac Sim driven joint varies between 26 and 32,
depending on the gripper position. On average, the motor has to turn by ~30 degrees for the driven joint (out_knuckle) to move by 1 degree.

### 4.5 Drive max speed

Spec finger speed is **2–150 mm/s**. Converting to the driving-joint angular
rate via the same 85 mm ↔ 49° mapping:

```
[2, 150] mm/s  →  /85 · 49  →  ≈ [1.15, 86] deg/s at the driving joint
```

### 4.6 How joint-drive force is evaluated (why more stiffness ≠ more force)

PhysX uses an **implicit** spring/damper formulation for joint drives. The upside
is that it prevents excessively large forces even with a poorly chosen timestep, so
the sim is far less likely to explode. The counter-intuitive downside: **beyond a
point, increasing stiffness gives *less* force, not more** (and likewise for
damping). Roughly, the implicit spring force is the explicit force `s·dx` divided
by a denominator that grows unless the timestep is small relative to the drive's
natural frequency. Practical rule: **keep the drive's natural frequency close to
the simulation frequency**, and tune force via `maxForce` + reasonable stiffness
rather than cranking stiffness arbitrarily.

![Implicit drive force formula](img/implicit-drive-force-formula.png)

*The implicit spring force: the explicit term `s·dx` is divided by
`dt²·nf² + 1`, so once the timestep is large relative to the drive's natural
frequency `nf`, more stiffness yields less force.*

Reference:
[gripper tuning example](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/gripper_tuning_example.html)
and Erin Catto's
[Soft Constraints (GDC 2011)](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf).

### 4.7 How the PhysX variants model the linkage

Joint by joint, what each PhysX variant does with the finger linkage. Joint
values are read from the 2F-85 payloads; the 2F-140 payloads follow the same
structure, with their own values.

#### The finger linkage

The 2F-85 finger is a **closed five-bar linkage** with two degrees of freedom:
one driven, and one underactuated compliance DOF that a pin limits (see
[§4.1](#41-how-the-2f-85-mechanism-works-and-how-compliance-works)).
A physics articulation must be a tree, so it cannot hold the closed loop
directly. The two PhysX variants solve this differently (below).

![2F-85 finger linkage with the joints labelled](img/2f85-linkage-labeled.png)

*The 2F-85 finger linkage: the blue and red links close a kinematic loop (red
arrow). `out_knuckle` is the driven joint.*

The short names in the figure map to these joints (left side; the right side
mirrors them):

| Figure | Joint | Connects |
|---|---|---|
| out_knuckle | `finger_joint` | `base_link` → `left_outer_knuckle` |
| out_finger | `left_outer_finger_joint` | `left_outer_knuckle` → `left_outer_finger` |
| in_finger | `left_inner_finger_joint` | `left_outer_finger` → `left_inner_finger` |
| in_finger_knuckle | `left_inner_finger_knuckle_joint` | `left_inner_finger` → `left_inner_knuckle` |
| in_knuckle | `left_inner_knuckle_joint` | `base_link` → `left_inner_knuckle` |

#### `Physx_parallel_grip`: mimic joints

The loop is cut to leave a tree, and **mimic joints** make the remaining
joints follow `finger_joint`. The coupling is rigid, so only the parallel grip
is reproduced, but every joint stays in the articulation solver: cheaper and
more stable.

| Joint | Role | Gearing on `finger_joint` |
|---|---|---|
| `finger_joint` | Driven, the only joint with a drive | — |
| `left_inner_finger_joint` | Mimic | −1 |
| `left_inner_finger_knuckle_joint` | Mimic | +1 |
| `left_outer_finger_joint` | Deactivated, replaced by a fixed joint `left_outer_knuckle` → `left_outer_finger` | — |
| `left_inner_knuckle_joint` | Deactivated: this cut keeps the tree | — |

The right side mirrors this with the gearing signs flipped, and
`right_outer_knuckle_joint` mimics `finger_joint` with gearing −1, so one
command moves both fingers. Values from
[`Robotiq_2F_85_parallel_grip_physics.usda`](../../Robotiq_2F_85/payloads/Robotiq_2F_85_parallel_grip_physics.usda).

![2F-85 mimic-joint layout](img/2f85-mimic-joints.png)

*The left-side chain in the parallel-grip variant: the **driven** joint
(out_knuckle), the **fixed** weld (out_finger), and the mimic joints (magenta
arrows) that follow the driven joint.*

#### `Physx_compliant`: loop closure

Nothing is welded or cut: all five joints are kept as revolutes, so the
fingertip pad can rotate under load like the real hardware. One joint per side,
`left_inner_finger_joint` (in_finger), is excluded from the articulation
(`physics:excludeFromArticulation = 1`) and the solver enforces it as a
loop-closure constraint. That joint also carries the underactuation spring
(see [§4.2](#42-the-redundancy--driving-spring)): a drive with
stiffness 0.0004 and target 92°.

| Joint | Role |
|---|---|
| `finger_joint` | Driven |
| `left_outer_finger_joint` | Free revolute, limited to 0…45° (the pin) |
| `left_inner_finger_knuckle_joint` | Free revolute |
| `left_inner_knuckle_joint` | Free revolute |
| `left_inner_finger_joint` | **Loop closure**, outside the articulation; carries the spring drive |

`right_outer_knuckle_joint` mimics `finger_joint` with gearing −1 so both sides
close together; the compliant DOFs of each side can still move independently,
depending on contact. Command only `finger_joint`. Values from
[`Robotiq_2F_85_compliant_physics.usda`](../../Robotiq_2F_85/payloads/Robotiq_2F_85_compliant_physics.usda)
and the shared limits in
[`Robotiq_2F_85_kinematics_physics.usda`](../../Robotiq_2F_85/payloads/Robotiq_2F_85_kinematics_physics.usda).

##### The outer-finger limit reproduces the pin

The limit on `*_outer_finger_joint` stands for the mechanical pin that caps
the minimal length of the virtual link. If it is wrong, the fingertip pivots
outward instead of staying parallel. The stock NVIDIA compliant asset had this
problem; ours limits `left_outer_finger_joint` to 0…45° and
`right_outer_finger_joint` to −45…0°.

![Compliant variant with the fingertip pivoted outward](img/loop-fingertip-pivot.png)

*Compliant variant with a wrong outer-finger limit: the fingertip pivots
outward instead of holding parallel.*

**When to pick which:** mimic for a fast, robust parallel pinch; loop when the
grip's realism matters (compliance, the encompassing grip), preferably under
Newton (see [§5](#5-using-isaac-with-newton-and-why-its-better-for-the-compliant-variant)).

---

## 5. Using Isaac with Newton (and why it's better for the compliant variant)

### 5.1 How to use Isaac with Newton

**Newton** is Isaac Sim's newer physics backend, built on **MuJoCo-Warp**. Isaac
Sim 6 includes it. You select the Newton backend on the physics scene; the
gripper asset is authored with Newton-specific (`mjc:`) attributes and MuJoCo
contact tuning rather than PhysX ones.

The Newton 2F-85 comes as two variants of the unified asset
([`Robotiq_2F_85/`](../../Robotiq_2F_85/)): `Newton_compliant`, where the coupler
(`outer_finger_joint`) stays live so the fingers wrap around the object, and
`Newton_parallel_grip`, where the coupler is welded so the fingers stay parallel.
Select `Physics = Newton_compliant` or `Newton_parallel_grip`, add the gripper to a scene
with a `PhysicsScene`, and press Play. The 2F-140 has the same two Newton variants
([`Robotiq_2F_140/newton/`](../../Robotiq_2F_140/newton/)). It is **open by default**
(`finger_joint` drive target 0); drive `finger_joint` toward **~0.8 rad (≈45°)**
to close.

**Tuning:** the gripper side is tuned in the asset, except two settings
that must be re-applied after every Play with
[`newton/apply_gripper_tuning.py`](../../Robotiq_2F_85/newton/apply_gripper_tuning.py).
See [Tuning → Newton variants](#newton-variants) for that step, and
[Tune physics for grasping](../../../docs/6-Tune_physics_for_grasping.md#newton)
for the contact settings of the objects you grasp.

**Performance note:** the UR5e + gripper Newton simulation is **physics-bound**. The real
speed lever is the **timestep** (`newton:timeStepsPerSecond`), not solver
iterations. Going from 1000 Hz to 500 Hz nearly doubled the real-time factor (0.16× →
0.27×) by doing ~half the substeps, while render time stayed flat (~8.5 ms). Other
levers to try are `use_mujoco_cpu` (removes the GPU kernel-launch latency on this tiny
model) and `cone=pyramidal` (cheaper friction) — *not* more iterations.

### 5.2 Why Newton is better for the compliant variant

The loop (five-bar) variant is where Newton earns its place:

- **The five-bar loop closure is modelled natively** as MuJoCo equalities, which
  are far more stable at the joint limits than PhysX's maximal-coordinate loop
  constraints (which showed residual compliance and ringing at the joint limits;
  see [demo/teleop/PHYSICS_TUNING.md](../../demo/teleop/PHYSICS_TUNING.md)).
- **Correct joint-limit compliance.** The coupler joints' limit stiffness/damping
  (`mjc:solreflimit`) is authored per-radian and holds the fingertip parallel
  with an overdamped limit — after a subtle importer bug was fixed (see box
  below).
- **The parallel-grip spring holds** thanks to a deliberately inflated link
  inertia: `physics:diagonalInertia = (0.001, 0.001, 0.001)` is **~100× the
  physical value, on purpose** — it stabilizes the soft loop-closure equalities so
  the weak parallel-grip spring holds. With physically small inertias the linkage
  goes floppy and the fingertips sag ~20° out of parallel. Treat it like the
  `mjc:armature`/`damping` tuning: **do not** replace it with CAD
  inertias without re-tuning the loop-closure `solref`.
- Critical joint `mjc:damping` prevents the fingers from ringing;
  `MjcEqualityJointAPI` is applied **on the joint prim**.

> **Isaac Sim 6.0.1 only: joint-limit bug.** The Newton version bundled with
> Isaac Sim 6.0.1 (1.2.1) scales angular joint-limit gains about 57× too
> high, so the finger limits overshoot by about 7° and ring. Newton PR
> **#2736** fixes it; on 6.0.1, apply that fix to the Newton install by hand
> (see [Robotiq_2F_85/newton/README.md](../../Robotiq_2F_85/newton/README.md)).
> Isaac Sim 6.1 bundles Newton 1.5.0, which already includes the fix.

### 5.3 Where to find the asset

- **Newton 2F-85:** the `Newton_compliant` and `Newton_parallel_grip` variants of
  [`Robotiq_2F_85/`](../../Robotiq_2F_85/) — physics in the shared
  `payloads/Robotiq_2F_85_newton_fourbar_physics.usda` (joints, drive and the
  five-bar loop closure) and `payloads/Robotiq_2F_85_newton_common_physics.usda`
  (bodies and collision cooked from the shared visual CADs), with the thin
  `..._newton_compliant_physics.usda` / `..._newton_parallel_grip_physics.usda`
  on top, plus the runtime [`newton/apply_gripper_tuning.py`](../../Robotiq_2F_85/newton/apply_gripper_tuning.py).

**Provenance:** both the visual meshes (`Defeatured_2F_85_*`, shared with the
PhysX asset) and the Newton physics parameters (masses/inertias, joints, the
five-bar loop closure, and the MuJoCo contact/friction tuning) are Robotiq's own.
