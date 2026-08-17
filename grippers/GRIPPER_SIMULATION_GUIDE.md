# Simulating Robotiq grippers in Isaac Sim

A practical guide for anyone who wants to put a Robotiq gripper (primarily the
**2F-85**) into an NVIDIA Isaac Sim scene and have it grasp reliably. It starts
from first principles (what a variant, a mimic joint, a USD file are), walks
through a quick-start setup on a robot, then covers the problems you *will* hit
and how they were solved, the underlying physics of the gripper, and the newer
**Newton** physics backend.

The gripper assets referenced throughout live in this repo alongside this guide:

| Asset | Backend | Path |
|---|---|---|
| 2F-85 (PhysX, parallel_grip + compliant variants) | PhysX | [`Gripper_2F85/`](Gripper_2F85/) |
| 2F-85 (Newton / MuJoCo-Warp) | Newton | [`Gripper_2F85_newton/`](Gripper_2F85_newton/) |

The exact tuned USD values are given inline with each fix in §3.

---

## 1. Basics of Isaac Sim

### 1.1 What is NVIDIA Omniverse

NVIDIA Omniverse is a collection of GPU-accelerated libraries and microservices
for building physical-AI simulation applications and agentic workflows. Isaac
Sim is the robotics application built on top of Omniverse — it gives you the
physics engine (PhysX, and now Newton), the USD-based scene, and the tooling
(Stage, Property panel, Script Editor, extensions) you use to assemble and drive
a robot.

### 1.2 What is a USD file

USD (Universal Scene Description) is the file format and scene-composition system
underneath everything in Omniverse. A few things worth knowing before you edit a
gripper asset:

- **`.usda` = ASCII** (human-readable, text-diffable, used for small config and
  physics layers). **`.usd` = binary/crate** (compact, used for heavy geometry
  and top-level composition). Same data model, different encoding.
- USD scenes are **composed** from layers via *sublayers*, *references*, and
  *payloads*. A stronger layer's "opinion" overrides a weaker one **in place** —
  this is how the gripper swaps its physics behavior without touching geometry
  (see §5.3 and the layer graph below).
- A **prim** is a node in the scene tree (a body, a joint, a mesh, a scope).
  **APIs** (schemas) are tags applied to a prim that give it behavior — e.g.
  `PhysicsDriveAPI`, `PhysicsMimicJointAPI`, `PhysicsArticulationRootAPI`.

### 1.3 What are variants

A USD **variant set** lets a single prim carry several interchangeable
configurations, selected by a dropdown in the Property panel. The 2F-85 has a
`Physics` variant set on its root prim (`.../Robotiq_2F_85_edit`) with these
options:

| Variant | What it gives you |
|---|---|
| `None` | No physics at all — pure kinematic geometry |
| `Physx_parallel_grip` | One driven joint + **mimic** coupling for the passive joints |
| `Physx_compliant` | The real five-bar linkage modelled as a closed kinematic loop |

To change it: select the root prim → Property panel → **Variants** section →
`Physics` dropdown. See §2.1 for how to choose, and §5.2 for why the compliant variant
is best served by Newton.

### 1.4 What is a mimic joint

An **articulation** is a tree of rigid bodies (links) connected by joints, solved
by the physics engine as one cohesive unit. Articulations **must be trees — no
loops allowed** — yet many real mechanisms (four-bar linkages, parallel-jaw
mechanisms, geared stages) contain closed loops, or joints that must move
together.

A **mimic joint** is the lightweight way to express that coupling. A
`PhysicsMimicJointAPI` applied to a joint adds a single solver constraint tying it
to a *reference* joint:

```
q_mimic = gearing * q_reference + offset
```

Whatever the reference joint does, the mimic joint follows (velocity is
constrained the same way, `v_mimic = gearing * v_reference`). It is a **hard
constraint** enforced every solver iteration, and forces propagate back — if the
mimic joint resists, the reference joint's drive feels it. In practice you put a
drive on the reference joint and let the mimic joints follow for free.

Two common uses:

- **Gear / parallel coupling** — force two joints to move together at a fixed
  ratio (scissor mechanisms, parallel grippers, geared stages).
- **Closing a kinematic loop** — cut one joint of the loop so the remainder is a
  tree, then re-impose the cut coupling algebraically with a mimic joint.

Mimic joints can also be given **compliance** (the constraint modelled as a
spring-damper via a natural frequency + damping ratio) rather than being
perfectly rigid — useful when a hard mimic constraint would otherwise fight a
hard contact.

> **Mimic vs. gear joint:** a mimic joint is an *API applied to an existing
> joint* and works **inside** the articulation solver (cheap). A *gear joint* is a
> separate joint prim — a general constraint outside the articulation — for true
> gear trains where both joints must be independent articulation joints.

The 2F-85 uses mimic joints for its finger linkage; that gripper-specific setup
is covered in §2.1.

### 1.5 Connecting an AI agent to Isaac via MCP (and why it's so useful)

You can plug **Claude Code** (or any MCP client) into a *live* Isaac Sim session
through the **Model Context Protocol** bridge. Once connected, the agent can
build Action Graphs, inspect prims, step physics, read/write joint state, run
Python against the stage, and save — all from inside a conversation, without
pasting scripts into the Script Editor.

Why it matters for gripper work: it makes tuning much faster — sweeping
drive gains and contact offsets, diagnosing problems, poking the live stage,
measuring, and adjusting, all from the conversation.

Architecture (two processes): the **server** (`isaacsim-mcp-server`, a PyPI CLI)
runs on the client side and speaks MCP over stdio; the **extension**
(`isaac.sim.mcp_extension`) runs *inside* Isaac Sim and listens on TCP
`127.0.0.1:8766`. It needs a venv, the extension clone, an `.mcp.json`, and a
launcher script.

Two gotchas worth repeating:

- **Launch Isaac from a clean shell** — do *not* `source /opt/ros/humble` first,
  or Humble's Python 3.10 shadows Isaac's 3.11/3.12 and the bridge refuses to
  load. Source ROS only in the consumer terminals.
- **`get_isaac_logs` is the best diagnostic** — many graph/physics errors never
  appear in tool responses, only in Isaac's console.

### 1.6 Fixing CUDA errors after a crash

If Isaac (or any CUDA process) dies hard and subsequent runs throw CUDA / UVM
errors, reload the UVM kernel module:

```bash
sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm
```

This clears a wedged `nvidia_uvm` state without a full reboot.

---

## 2. Gripper quick-start guide

Goal: attach a 2F-85 to a robot (e.g. a UR5e) and have it driven as part of the
robot's articulation.

### 2.1 Choose the variant: mimic or loop

The 2F-85 finger is a **closed five-bar linkage** — two kinematic DOF, one
driven and one underactuated compliance DOF that a pin limits (see §4.1) — which
a tree-structured articulation can't hold directly (see §1.4). The two variants
resolve this differently:

![2F-85 finger linkage with the joints labelled](images/2f85-linkage-labeled.png)

*The 2F-85 finger linkage: the blue and red links close a kinematic loop (red
arrow). `out_knuckle` is the driven joint*

#### **`Physx_parallel_grip`**

Cuts the loop to leave a tree, then re-imposes the coupling
  with **mimic joints**: you drive `finger_joint` and the passive joints follow
  algebraically, imposed as a hard constraint. Because that coupling is rigid it reproduces **only the parallel
  grip**, but every joint stays inside the articulation solver — so it is
  **cheaper and more stable**. The pragmatic default under PhysX.

**Joint setup** — shown for the **left** side only; the
right side mirrors it with the gearing signs flipped to keep the fingertips
parallel. Of the five joints of the physical linkage, the parallel_grip variant drives
one, welds one, mimics two, and cuts the last to break the loop:

| Joint | Role | Gearing |
|---|---|---|
| out_knuckle | Driver — the only joint with a `JointDriveAPI` | — |
| in_finger | Mimic of `finger_joint` | +1 |
| in_finger_knuckle | Mimic of `finger_joint` | +1 |
| out_finger | **Fixed** weld `left_outer_knuckle` → `left_outer_finger` — not driven | — |
| in_knuckle | **Cut** — the `base_link`↔`left_inner_knuckle` joint is omitted so the branch stays a tree | — |

(Values read from
[`payloads/Robotiq_2F_85_physics_parallel_grip.usda`](Gripper_2F85/payloads/Robotiq_2F_85_physics_parallel_grip.usda).
The compliant variant instead keeps `left_inner_knuckle_joint` and closes it as a
loop-closure constraint.)

![2F-85 mimic-joint layout](images/2f85-mimic-joints.png)

*The same left-side chain on the mechanism: the **driven** joint (out_knuckle),
the **fixed** weld (out_finger), and the two mimic joints (magenta arrows) that
follow it.*

#### **`Physx_compliant`**

Keeps every joint and closes the five-bar as a
  maximal-coordinate **loop-closure constraint** (the extra joints are marked
  `excludeFromArticulation = True`). This preserves the finger's underactuated
  DOF, so the fingertip pad can rotate under load like the real hardware,
  modelling true **compliance** and the **encompassing grip**. The cost is a
  heavier solve, small residual compliance at the loop-closure joints, and —
  under PhysX — a tendency to instability. **Much better
  behaved under Newton** (§5).

> Under **`Physx_compliant`, command only `finger_joint`.** The remaining joints are
> solved by the loop-closure constraint.

**Joint setup** — Here nothing is
welded and nothing is cut: all five joints are kept as real revolutes, so the
fingertip pad can rotate under load like the real hardware. The loop is closed by
excluding **one** joint from the articulation and letting the solver enforce it
as a maximal-coordinate constraint:

| Joint | Role |
|---|---|
| out_knuckle | Driven — `base_link` → `left_outer_knuckle` |
| out_finger | Free revolute — `left_outer_knuckle` → `left_outer_finger` |
| in_finger | Free revolute — `left_outer_finger` → `left_inner_finger` |
| in_finger_knuckle | Free revolute — `left_inner_finger` → `left_inner_knuckle` |
| in_knuckle | **Loop closure** — `excludeFromArticulation = True`; `base_link` → `left_inner_knuckle`, solved as a maximal-coordinate constraint |

So where the parallel_grip variant *welds* `out_finger`, the
compliant variant keeps both as real revolutes and instead closes the ring at
`in_knuckle`. The two fingers' driven joints are tied together by a single mimic joint
(`right out_knuckle`, gearing −1 on `left out_knuckle`) so both sides move
symmetrically from the one `finger_joint` command. The compliant DOFs may act
asymmetrically depending on the physical interaction with the environment.

(Values read from
[`payloads/Robotiq_2F_85_physics_compliant.usda`](Gripper_2F85/payloads/Robotiq_2F_85_physics_compliant.usda).
This closed loop is what gives the more realistic contact behavior — and, under
PhysX, the residual compliance and limit instability tuned away in §3.2.)

**When to pick which.** Reach for **mimic** when you only need a fast, robust
parallel pinch; reach for **loop** when the grip's realism (compliance, the
encompassing grip of §4.1, sim-to-real fidelity) is what you're after — and run
it under Newton (§5), which tames the residual compliance and limit instability
that the compliant variant shows under PhysX (§3.2).

### 2.2 Reference the gripper onto the robot

1. **Drag the gripper into the stage**, onto the link you want to attach it to.
   The gripper prim **must be a child of the flange / `wrist_3` body**, e.g.
   `/World/ur5e/wrist_3_link/Robotiq_2F_85_edit`.
2. Reference the *config* file for your chosen variant
   (`configuration/Robotiq_2F_85_config_physics_parallel_grip.usda` or
   `…_compliant.usda`), not the `edit`/`robot` authoring files.

### 2.3 Attach it with a fixed joint

Create a fixed joint between the wrist and the gripper base, targeting these two
bodies:

```
Body 0: /World/ur5e/wrist_3_link
Body 1: /World/ur5e/wrist_3_link/Robotiq_2F_85_edit/Robotiq_2F_85/base_link
```

### 2.4 Remove the gripper's own articulation root

The standalone gripper asset ships with its **own** `PhysicsArticulationRootAPI`
so it can simulate by itself. **When you mount it on a robot, remove that
articulation root** — the gripper's joints must belong to the *robot's*
articulation (rooted at, e.g., `/World/ur5e/root_joint`), not form a second,
competing articulation. Leaving two articulation roots is a very common cause of
a gripper that won't drive or behaves erratically.

![Articulation Root API in the Physics property panel](images/articulation-root-panel.png)

*The `Articulation Root` block in the Physics panel — delete this API from the
gripper when mounting it on a robot.*

---

## 3. Troubleshooting

The failure modes below are the ones actually reproduced during bring-up. Each
fix lists the concrete USD attribute values to set, tuned for reliable grasping
in Isaac Sim 6. None of this is required for the gripper to *function* — but it is
what makes it actually hold an object through motion.

### 3.1 Mimic version

#### The gripper passes through the gripped object

![2F-85 closing on a cube](images/mimic-grasp-setup.png)

*The test setup: the 2F-85 closing on a small cube — the scenario where the
fingers tunnel through the object before the fixes below.*

Two independent root causes, fix both:

1. **The object has zero mass.** Primitive-shape assets often ship with
   `physics:mass = 0.0` *and* `physics:density = 0.0`. PhysX then treats inertia
   as undefined (`centerOfMass = (-inf,-inf,-inf)`, `diagonalInertia = (0,0,0)`)
   and *any* contact force accelerates it infinitely — it tunnels through
   everything. **Set `physics:mass = 0.1 kg`.**
2. **Contact tolerances too small and grip force too high** for a cm-scale
   object:

   | Parameter | Before | After |
   |---|---|---|
   | object `physxCollision:contactOffset` | `-inf` (scene default, too small) | **2 mm** |
   | object `physxCollision:restOffset` | `-inf` | **0.5 mm** |
   | `finger_joint` drive `maxForce` | 26 N·m | **10 N·m** |

   `restOffset = 0.5 mm` is a hard "skin" PhysX won't let objects compress
   through. `contactOffset = 2 mm` is where PhysX starts generating contacts
   (must be `> restOffset`).

> The pad meshes are inside referenced instance proxies, so USD blocks authoring
> offsets on them. Setting the offsets on the **object** side alone is enough —
> PhysX sums the pair, so object-offset + pad-scene-default still yields a visible
> skin.

| Before | After |
|---|---|
| ![Fingers passing through the cube](images/mimic-passthrough-before.png) | ![Stable grasp after the fix](images/mimic-grasp-after.png) |
| Zero-mass cube: the fingers tunnel straight through it. | With mass, contact offsets and capped `maxForce`: the cube is held. |

If snappy motions still tunnel (relative velocity × `physics_dt` > object
thickness — reachable at ~0.5 m/s with a 4 ms step and a 2 mm gap), enable
**speculative CCD** on the small fast object (see §3.3, CCD note).

#### Object drops after a few robot moves

The finger drive is too soft and picks up slack under inertial loading. Stiffen
the `finger_joint` drive:

| Attribute | From | To |
|---|---|---|
| `drive:angular:physics:stiffness` | 3 (asset default 0.17) | **50** |
| `drive:angular:physics:damping` | 0.0002 | **3.0** |

Low risk because `restOffset = 0.5 mm` still prevents penetration. (Cranking
stiffness *far* higher was tried and reverted — it transferred vibration into the
arm; K=50 / C=3.0 hits the right damping ratio for the finger inertia.)

#### Object vibrates in the gripper

Two causes:

- **Object `linearDamping = 0.0`** — every contact perturbation gives the object
  a velocity and nothing decays it, so it oscillates around the grip pose. Set
  **`linearDamping = 0.1`**.
- **Velocity iteration count = 1** on both the articulation and the object. PhysX
  does plenty of position passes but only the minimum velocity pass, so contact
  *velocity* errors aren't resolved → jitter. Raise object
  **`solverVelocityIterationCount` 1 → 4** and the articulation's
  **`solverVelocityIterationCount` 1 → 2**.

### 3.2 Loop version

#### Public asset has bad joint limits — our version fixes them

The stock NVIDIA compliant variant has wrong limits on the outer-finger joints, which
lets the **fingertip pivot outward** instead of staying parallel. This is because
the joint limit must *reproduce the mechanical pin* that constrains the linkage
(see §4.1). The fix, applied to our asset, sets the limit on
`right_outer_finger_joint`:

![Compliant variant with the fingertip pivoted outward](images/loop-fingertip-pivot.png)

*Compliant variant with the stock (bad) joint limit — the fingertip pivots outward
instead of holding parallel.*

| Joint limit | Default (public asset) | Our value |
|---|---|---|
| `lowerLimit` | 0° | **−180°** |
| `upperLimit` | 180° | **0°** |

This limit encodes the pin that caps the minimal length of the virtual link,
keeping the fingertip parallel in the open configuration.

#### Tuning the joint-limit contact (instability)

Even with correct limits, the compliant variant can go unstable at the limits under
PhysX before tuning, and settle nicely after. Tune the limit stiffness/damping and
raise the per-body solver iterations:

- The four loop-closure bodies (`{left,right}_{outer,inner}_finger`) get high
  per-body iterations (`solverPositionIterationCount = 64`,
  `solverVelocityIterationCount = 8`) — PhysX uses the max across a constraint
  island, so this tightens the loop closures.
- Enable scene stabilization: `/PhysicsScene physxScene:enableStabilization =
  True` — an extra pass that re-converges constraints between frames; cheap and
  measurably tightens the loops.

Reference reading: NVIDIA's
[rigging closed-loop structures](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/rig_closed_loop_structures.html),
[gripper tuning example](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/guides/gripper_tuning_example.html),
and [joint tuning](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup_tutorials/joint_tuning.html).

> **The compliant variant's residual compliance and limit instability under PhysX are
> exactly why we moved it to the Newton backend** (§5), which models the five-bar
> and its limits far more robustly.

### 3.3 Cross-cutting: CCD (tunneling)

Physics steps in discrete chunks (~4 ms `physics_dt` here). The default
**Discrete Collision Detection (DCD)** only tests overlap at the *end* of each
step, so a body that moves clear across a thin feature between frames is never
seen → **tunneling**. Rough threshold: DCD misses when
`relative_velocity * physics_dt > thin_dimension_thickness` (≈0.5 m/s for a 2 mm
gap at 4 ms).

**Continuous Collision Detection (CCD)** sweeps each body's motion and solves for
the time-of-impact. Two PhysX modes:

| | `enableCCD` (full) | `enableSpeculativeCCD` (speculative) |
|---|---|---|
| Method | True swept volume + TOI, linear **and** angular | Inflate broad-phase AABB by expected motion, run DCD on it |
| Cost | High (sub-steps near contacts) | Low |
| Catches | Linear **and** rotational tunneling | Linear only |
| Side effects | Can add "stickiness" | Almost none |

**Speculative CCD is the right default.** Turn it on **per-body** for the small,
fast object that's slipping (fingertips, thin pads, projectiles) — *not* on its
environment (statics get nothing from it). On a busy scene, CCD on every dynamic
body can multiply physics cost 2–5×.

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

![Parallel grip vs encompassing grip](images/2f85-grip-modes.png)

*Left: virtual link at minimal length → parallel grip. Right: virtual link
extended → fingertip rotates inward for an encompassing grip on a round object.*

An optional **second pin** can fully lock the relative angle between the two
links, forcing a *constant parallel grip* (no encompassing behavior). This configuration is rarely used.

![Optional pin locking the parallel grip](images/2f85-optional-pin.png)

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

![Finger span vs driving-joint angle](images/2f85-transmission-curve.png)

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

![Implicit drive force formula](images/implicit-drive-force-formula.png)

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

Our Newton 2F-85 asset ([`Gripper_2F85_newton/`](Gripper_2F85_newton/))
is standalone (gripper at the origin, its own articulation root) — drop it in a
scene with a `PhysicsScene` and play. It is **open by default**
(`finger_joint` drive target 0); drive `finger_joint` toward **~0.8 rad (≈45°)**
to close.

**One runtime step after every stop → play:** run
[`apply_gripper_tuning.py`](Gripper_2F85_newton/apply_gripper_tuning.py)
once (Script Editor or MCP bridge). It re-applies the only two settings that
cannot be persisted in USD:

1. `opt.impratio = 10` — the anti-slip setting (raising the friction *impedance
   ratio* is what actually stopped the grasp slipping — not the friction
   coefficient);
2. the near-rigid finger-coupling equality `solref` — the equality has no USD
   prim (it is synthesized at model build).

Everything else (drive gains, armature, `mjc:damping`, spring stiffness,
`solreflimit`, loop-closure equalities, contact tuning) is baked into the
`.usda` and rebuilds natively on load.

**Newton contact tuning knobs** (per-geom, on pads and object):

- **`geom_solref = [timeconst, dampratio]`** — the contact spring-damper.
  Smaller `timeconst` = stiffer (less sink-in); effective stiffness ∝ 1/timeconst²,
  damping ∝ 1/timeconst. MuJoCo default 0.02; pads stiffened to **0.004** to stop
  the object sinking in. **Hard floor: `timeconst ≥ 2·timestep`** (0.002 s at
  500 Hz), so 0.004 is near the stiff limit. `dampratio = 1` is critically
  damped; `>1` overdamped (kills chatter, slightly sluggish); `<1` bouncy.
- **`geom_solimp = [dmin, dmax, width, midpoint, power]`** — contact *impedance*
  (how firmly the no-penetration force is applied vs. penetration depth). Pads
  `[0.95, 0.99, 0.001, 0.5, 2.0]` bite harder than the cube
  `[0.9, 0.95, 0.001, 0.5, 2.0]`.

Rule of thumb: **`solref` sets the timing/springiness, `solimp` sets how firmly
it's applied.** Together they set penetration depth and force smoothness.

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
  constraints (which showed the residual compliance and limit ringing of §3.2).
- **Correct joint-limit compliance.** The coupler joints' limit stiffness/damping
  (`mjc:solreflimit`) is authored per-radian and holds the fingertip parallel
  with an overdamped limit — after a subtle importer bug was fixed (see box
  below).
- **The parallel-grip spring holds** thanks to a deliberately inflated link
  inertia: `physics:diagonalInertia = (0.001, 0.001, 0.001)` is **~100× the
  physical value, on purpose** — it stabilizes the soft loop-closure equalities so
  the weak parallel-grip spring holds. With physically-small inertias the linkage
  goes floppy and the fingertips sag ~20° out of parallel. Treat it like the
  `mjc:armature`/`damping` tuning: **do not** replace it with CAD/menagerie
  inertias without re-tuning the loop-closure `solref`.
- Critical joint `mjc:damping` prevents the fingers from ringing;
  `MjcEqualityJointAPI` is applied **on the joint prim**.

> **Known importer bug (angular joint limits) — fixed in our setup.** Newton's
> `parse_usd` divides angular limit gains by π/180 assuming they were authored
> per-degree, but `mjc:solreflimit` is always per-radian. Result: coupler limit
> gains mis-scaled by the ~57× deg→rad factor → the limit overshot ~7° and rang.
> This is fixed upstream (Newton PR **#2736**) but Isaac Sim 6.0.1-rc.7 bundles
> the older Newton 1.2.1 that still has it. A local backport patch cancels the
> spurious division so per-radian `solreflimit` survives; overshoot dropped from
> ~7° to <1°. **Two-part dependency:** the tuning value lives in the USD
> (round-trips across restarts), but the *fix* lives in the Newton install — if
> Isaac/Newton is reinstalled or upgraded to a version without #2736, **re-apply
> the patch** or the limits regress.

### 5.3 Where to find the asset

- **Newton 2F-85:** [`Gripper_2F85_newton/`](Gripper_2F85_newton/)
  — `Robotiq_2F85_newton.usda` (structure + tuning), `geometry/` (per-body
  meshes, git-LFS), `apply_gripper_tuning.py`, and its own
  [`README.md`](Gripper_2F85_newton/README.md).

**Provenance:** the *physics model* derives from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85)
2F-85 (`2f85.xml`) — bodies, masses/inertias, joints, five-bar closure, and
contact/friction tuning. The *visual meshes* are Robotiq's detailed CAD
(`Defeatured_2F_85_*`, shared with the PhysX asset). One import subtlety worth
knowing: the MJCF→USD import **dropped the 6 MuJoCo `<contact><exclude>`
self-collision pairs**; they were re-authored as `physics:filteredPairs`
(base↔driver, base↔spring_link, coupler↔follower, both sides). Without them the
fingers jam on self-contact instead of relaxing to the parallel pose.

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
the root `Robotiq_2F_85_edit.usd`.

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
