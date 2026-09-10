# UR5e + 2F-85 teleop on the Newton backend

Work-in-progress rig for teleoperating the UR5e with the **Newton (MuJoCo-Warp)**
2F-85, so grasping is simulated with the four-bar's real compliance instead of
PhysX's `Physx_Loop` approximation.

Measured on an RTX A4000 Laptop (8 GB). **Read "Known problems"
before demoing this** — it works, but it is not yet solid.

## Why the arm is kinematic

Newton needs a 1 ms timestep for the gripper's soft loop-closure equalities.
With the arm dynamic the whole model is far too slow:

| configuration | nv | RTF |
|---|---|---|
| arm dynamic | 32 | **0.090** (11x slower than realtime) |
| arm physics off, gripper dynamic | 26 | **0.44** (5x faster) |

The arm has **zero colliders** — only the gripper ever touches anything — so it
was paying full constraint-solve cost for nothing. It is therefore taken out of
physics (`Physics` variant -> `None`) and driven kinematically, while the
gripper stays a fully dynamic Newton articulation.

Where the time actually goes (all measured, not assumed):

* render 1280x720 -> 320x180 (16x fewer pixels) gained only **+5%** RTF, so
  rendering is *not* the bottleneck
* the model has 25 geoms and no arm colliders, so contacts are *not* the
  bottleneck
* it is the **constraint solve** at 1000 Hz

## Forward kinematics

```
T_i_world(q) = ARM_BASE . Rz(180deg) . F_i(q)
```

`F_i` is plain URDF FK from `ur_description`'s joint origins/axes (every joint
axis is local Z). The `Rz(180)` is a **left-multiplied base rotation** and is
the whole trick: the asset's link frames differ from `ur_description`'s by a
base rotation, which **cannot** be absorbed by right-multiplied per-link
offsets. Trying that gave ~1.3 m error, and ~8 cm even after searching all 4096
sign/pi-offset combinations. Verified against live physics poses: worst link
error **0.1 mm** at zero config, **0.02 mm** at the working start pose.

The arm links are **flat siblings** under `/World/ur5e`, not a nested chain, so
each link's local transform is its arm-relative pose and is written directly.

## Why not MoveIt Servo

Servo's control loop runs on the **wall** clock: measured 1726 commands per
*simulated* second against a configured 250, while the sim runs at RTF ~0.1.
That mismatch is a ~3 Hz limit cycle that no gain change removes (tested:
`publish_period`, `override_velocity_scaling_factor`, command magnitude,
`use_smoothing`). Jazzy also deleted `low_latency_mode`, which is what used to
pace Servo's output by its input.

`newton_kinematic_teleop.py` instead runs on Kit's update tick, so controller,
physics and clock share one timebase and the oscillation cannot occur. It reads
`/dev/input/js0` directly — no ROS, because Isaac's bundled Python and ROS 2's
`rclpy` are different interpreters (the same reason `launch_isaac_with_mcp.sh`
insists on a clean shell).

## Running it

```bash
# 1. author the scene (pure USD, outside Isaac)
USDLIB=$(dirname $(find ~/isaacsim/extscache -maxdepth 2 -type d -path "*omni.usd.libs*/pxr" | head -1))
PYTHONPATH="$USDLIB:$PYTHONPATH" LD_LIBRARY_PATH="$USDLIB/bin:$LD_LIBRARY_PATH" \
  ~/isaacsim/python.sh grippers/demo/newton_teleop/author_scene.py

# 2. launch Isaac with the NEWTON experience (clean shell, no /opt/ros sourced)
source grippers/demo/newton_teleop/isaac-demo-dds.sh
cd grippers/demo/mcp_bridge
ISAACSIM_LAUNCHER=isaac-sim.newton.sh MCP_EXT_ROOT=<your-mcp-ext> ./launch_isaac_with_mcp.sh

# The MCP bridge is optional -- it only lets an agent drive the stage. Without
# it, launch Isaac with the Newton experience directly and run the tuning and
# teleop scripts from the Script Editor.

# 3. in Isaac, one call: checks the Newton patch, opens the scene, plays,
#    applies the runtime gripper tuning -- and refuses if the patch is missing
python3 isaac_rpc.py script setup_demo.py

# 4. drive it
python3 isaac_rpc.py script newton_kinematic_teleop.py
```

The arm's Physics variant and the gripper's articulation root are baked into
the scene by `author_scene.py` — they are not manual steps. If you find
yourself setting them by hand, the scene is stale; re-author it (step 1).

**After every stop -> play, call `retune()`.** The two settings that cannot
live in USD are held in the built `mujoco_warp` model, which is discarded and
rebuilt on each play. `setup_demo.py` does it once; nothing does it for you
afterwards.

Controls (DualSense): right stick = linear x/y, left stick = roll/pitch,
d-pad up/down = z, L1/R1 = yaw, R2 = gripper, Triangle/Cross = speed.

## Known problems

**~~Passive joints buzz ~3-4 deg at rest~~ — SOLVED, see `apply_newton_2736_patch.py`.**
This was Newton's USD importer dividing angular limit GAINS by pi/180 while
`mjc:solreflimit` is per-radian, i.e. ~57x too stiff, so the coupler limits rang.
It is documented in `../GRIPPER_SIMULATION_GUIDE.md` §5.2 as a known importer bug
fixed upstream in Newton PR **#2736**; Isaac 6.0.0 bundles newton 1.2.0, which
predates it, and **the fix lives in the Isaac install, not in git**.

| joint at rest | unpatched | patched |
|---|---|---|
| `left_follower` | 3.24 deg | **0.001 deg** |
| `right_follower` | 3.85 deg | **0.001 deg** |
| `left_coupler` | 2.01 deg | **0.000 deg** |

Re-run the patch script after any Isaac/Newton reinstall or the buzz returns.

**The gripper base is infinitely stiff.** Its pose is *set* each tick, so
contact forces cannot push the arm back. Driving the fingers hard into the floor
therefore has nowhere to dissipate and can diverge. A `_pads_touching()`
feedback clamp blocks commanded downward motion once the pads reach the floor,
but that guard has **never actually fired in a test** — treat it as unverified.
The robust fix is a compliant base (free joint + weld to a driven target, i.e.
MuJoCo's mocap-weld pattern) so the base can yield. Note Newton rejected a
plain `FixedJoint` in this scene, so the weld needs a different expression.

**Root motion is discontinuous.** Physics runs at 1 ms; the root is updated once
per Kit tick (~16 ms), so it sits still for ~16 steps then jumps. Setting the
root velocity does not help: the gripper is a **fixed-base** articulation with no
root DOF, so `set_linear_velocity` succeeds but reads back zero. Fixing this
properly needs either per-physics-step root updates or the compliant base above.

**Finger tracking is sluggish** — commanded 0.80 rad reaches ~0.28 within a
normal settle window. Full closure to 0.787 rad does work, but only when the
target is ramped; a step input diverges, hence `GRIP_RATE`.

## Settings that must not be changed

* **`newton:timeStepsPerSecond` 500 is fine _once Newton PR #2736 is patched in_
  (`apply_newton_2736_patch.py`).** Unpatched, 500 and 250 both diverge to NaN --
  but that was a *consequence of the importer bug*, not a real constraint.
  Patched, 500 Hz is stable and faster: RTF 0.420 -> **0.486**. 250 Hz is still
  untested post-patch.
* **Do not set `mjc:option:tolerance`.** 1e-6 is Newton's own default; the
  schema advertises 1e-8, and authoring that changes nothing useful.
* **`mjc:option:iterations` is not honoured** from USD at all (authored 30, the
  model still builds with 100), so it is not a usable cost lever.
* **Command only `finger_joint`.** The coupler/follower/spring-link joints are
  passive and solved by the equalities. Sending a full `joint_positions` vector
  puts targets on them, fights the constraints and explodes the gripper on the
  first close. `gamepad_teleop.py` documents this too.
* **Never call `omni.kit.app.update()` from inside an `execute_script` RPC.** It
  re-enters Kit's main loop, corrupts its asyncio state (`Cannot enter into
  task`) and permanently degrades the session — RTF fell 0.44 -> 0.12 this way.
  Restart Isaac before demoing.

## Gotchas that cost real time

* Trigger polarity is **not** reliable. This DualSense rests at `-1.0` while
  `gamepad_teleop.py` documents `+1.0` as released; hard-coding either makes the
  gripper slam shut at startup. The driver calibrates the rest value at runtime.
* Isaac's renderer **ignores scale xformOps** on `Cube`/`Cylinder` gprims: USD
  reported 0.02 m while the viewport drew a 1 m cube that engulfed the whole
  robot and looked like "the robot is missing". Set intrinsic `size`/`radius`.
* A bare asset with `endTimeCode = 0` cannot advance: the timeline reports
  `playing=True` while `t` stays `0.000` and physics never steps.
* With Fabric active, `ComputeLocalToWorldTransform` returns **authored** values
  for physics-driven prims — identical for every joint configuration. Read live
  poses through `SingleRigidPrim` / `SingleArticulation` instead.
* GPU pipeline means torch tensors: `joint_indices` and the velocity setters
  reject numpy (`'numpy.ndarray' object has no attribute 'to'`,
  `unsqueeze(): argument 'input' must be Tensor`).
* Every stop -> play can leak a `mujoco_warp` Model; if `Models found` is > 1,
  measurements are polluted. Restart.

## Files

| file | purpose |
|---|---|
| `author_scene.py` | authors `ur5e_2F85_newton.usda` (pure USD, run outside Isaac) |
| `newton_kinematic_teleop.py` | the in-Isaac driver: FK, Jacobian jog, gripper, gamepad |
| `ur5e_2F85_newton.usda` | generated scene, checked in for convenience |
| `setup_demo.py` | one-call session setup: verifies the patch, opens the scene, plays, tunes; `retune()` after each stop -> play |
| `apply_newton_2736_patch.py` | **backports Newton PR #2736** — removes the 3-4 deg buzz and unlocks 500 Hz; re-run after any Isaac reinstall |
| `isaac-demo-dds.sh` | **DDS isolation — source in every terminal**, see below |
| `70-dualsense-hidraw.rules` | udev rule so `pydualsense` can open the DualSense for R2 haptics |

## DDS isolation is not optional

With ROS 2 defaults (domain 0 + subnet discovery) this laptop joined a shared
DDS graph containing **live robot drivers** — `/frankahardwareinterface`,
`/fr3_arm_controller`, `/dashboard_client`, GELLO — and
`/forward_position_controller/commands`, exactly what MoveIt Servo publishes to,
already had two remote subscribers. Source `isaac-demo-dds.sh` in every
terminal (Isaac's and ROS's) before running anything that publishes.
