# Plan: `SensorSim = Precise` (TSF-85 deformable-pad sensor)

Status: **plan; spikes done, implemented (variant, reader, CNN, tests, docs); density 4e4 chosen (spike 12); next: runtime test green, SimReady validation, PR, nothing implemented yet.** Branch `kepler/add-tsf-precise-variant`.

## Goal

Add a third `SensorSim` variant, `Precise`, under `Fingertip=Tactile_TSF85` of the 2F-85, next to `Raycast` and
`EffectiveResponse`. It reproduces what is in <https://github.com/Lab-CORO/TSF-85> (ETS CoRo lab): a deformable
sensing pad whose node displacements feed a CNN that predicts the tactile map. It is chosen only by selecting the
variant on our existing gripper.

## Decisions taken

| | |
|---|---|
| Engine | **PhysX only** (deformable bodies). Selecting `Precise` under Newton must fail with a clear error, not return zeros. |
| Isaac Sim | Target **6.2** (`~/isaacsim-6.2.0`). One comparison run on 5.1 (upstream's validated version) to tell port bugs from upstream bugs. CI and runtime tests are still on 6.1: bumping them is a separate decision. |
| Gripper | **No second gripper.** `Robotiq_2F_85_modified.usd` is not imported. Their mounting is adapted as overrides inside the `Precise` variant of our own asset. |
| Imported | Only the sensor (`assets/TSF_85.usd`), `tactile_cnn.onnx`, `sensor_config.json`, and the pipeline logic. |
| Licence | Ported code is **relicensed as BSD-3 (this repo's licence), option B**, under the ownership agreement with ETS CoRo. Provenance and citations are kept (see "Attribution"). |
| Output | **7 x 4 per finger**, same grid as the other readers (upstream's CNN output is 7 x 4 = 28 values). The 18 x 12 grid is the CNN *input* (216 sampled mesh nodes) and stays internal to the reader. `readers.py` / plot constants do not change. |

## Open items

- **What the 28 CNN values mean** (force, pressure, normalised response): read upstream's dataset / training
  definition before choosing which `quantity` the reader exposes. Upstream normalises by `the_max` = 0.0016163.
  Until known, do not describe `Precise` as calibrated force.
- **Mount frames:** done, see "Findings: mount comparison". Remaining: how the deformable attaches to our
  fingertip (see below).
- **onnxruntime on Isaac 6.2:** whether its bundled Python can use a compatible onnxruntime build.
- **Legal:** the upstream repo still says Apache-2.0 publicly (unfilled copyright line). Keep the ownership
  agreement where the team can find it, have legal confirm it covers every imported file (CNN model and the 26 MB
  USD included), and ask Lab-CORO to align their repo's licence info.

## Findings: mount comparison (read-only, files only)

- **Upstream's modified gripper contains no sensor.** `Robotiq_2F_85_modified.usd` only swaps the inner-finger
  pad for an open-pad finger (like our `tip_standard` deactivation). The sensors are mounted in
  `examples/scenes/scene_cylinder.usd`: `TSF_85_left` / `TSF_85_right`, each a rigid `Case` with a `FixedJoint` to
  `<side>_inner_finger`. (Their README links a `docs/attaching_sensors.md` that does not exist.)
- **The two mounts nearly coincide**, in the inner-finger frame (mm):

  | | x | y (gripper opening axis) | z (along finger) |
  |---|---|---|---|
  | ours, `tip_tsf85` (right) | -13.4 .. 13.4 | 39.5 .. 62.7 | 102.4 .. 153.9 |
  | upstream `Case` (right) | -14.2 .. 14.2 | 41.4 .. 62.7 | 110.4 .. 154.4 |
  | upstream `Deformable` (right) | -13.0 .. 13.0 | 38.4 .. 41.5 | 112.0 .. 153.1 |

  Same outer face (y = 62.7) and same far end (z about 154). Their case is about 2 mm wider and 8 mm shorter than
  ours (ours is the detailed CAD; theirs has a simplified 1.08 M-point collision mesh). The deformable pad surface
  sits about 1 mm inside our pad face. So the offset to apply is small, and no re-orientation is needed.
- **The deformable is tiny:** `/World/Deformable/simulation_mesh`, 432 nodes / 280 tets, 26 x 3 x 40 mm. Material:
  E = 1.289 MPa, nu = 0.1729, elasticity damping 0.002, dynamic friction 2.2 (E and the 3 mm thickness are where
  our `padStiffness` = 0.43 N/mm3 already comes from). Authored with the new `OmniPhysics` deformable schema
  (`OmniPhysicsVtxXformAttachment`, `OmniPhysicsElementCollisionFilter`, `omniphysics:restShapePoints`), so it
  is not 5.1-only schema; whether it loads on 6.2 is still the spike's question.
- **Node order matters:** `sensor_config.json` selects the 216 input nodes by id, so the deformable must be
  imported as authored (same 432 points and ordering).
- **Their runtime needs** `/physics/enableDeformableBodies = True` and GPU dynamics on the PhysicsScene.
- **Consequence for the variant:** import only `/World/Deformable` (plus its material and attachment), not their
  1 M-point `Case` mesh. Keep our `TSF_85_case.usd` as the visual and attach the deformable to the fingertip body.
  To check in the spike: attachment targets, and whether our `left_fingertip` / `right_fingertip` links are
  the right bodies to attach to (upstream attaches to `*_inner_finger`).

## Findings: spike 1, standalone sensor on Isaac 6.2 (PhysX, GUI)

Upstream's `TSF_85.usd` (case made kinematic) with a 0.3 kg cube resting on the pad, then 120 frames:

- **The deformable loads and simulates on 6.2** with `/physics/enableDeformableBodies = True` and GPU dynamics on the
  PhysicsScene. The 432 node positions are readable each frame from `simulation_mesh.points` through USD (no
  tensor API needed). About 19 ms per frame with the GUI at 240 Hz physics.
- **A new 6.2 stage defaults to Newton**, which has no deformables: nothing moved until the engine was switched to
  PhysX (`SimulationManager.switch_physics_engine("physx")`, as `tests/runtime/_scene.py` does). The `Precise`
  reader must check the engine and fail clearly otherwise.
- **Upstream's `Case/FixedJoint` points at a `GroundPlane` that does not exist in `TSF_85.usd`**, so the joint is
  not created and the case falls. We never use their case, so this does not matter for us.
- **The CNN runs** on the 216 configured nodes' y displacement divided by `the_max` (onnxruntime 1.31 installed in
  `~/isaacsim-6.2.0`, Python 3.12). Output minus the zero-input baseline was 20..440, with no clean spatial
  pattern under the cube. Not conclusive: the units are unknown (see open items), and my displacement-to-input
  convention (y component, relative to the first frame) is a guess until I read upstream's `dz` code.
- **Unexplained, to check before trusting the sensor:** the pad bottomed out about 3 mm under a 2.9 N load
  (cube face reached the case face), and node displacement reached 2 mm. With E = 1.289 MPa that load should
  compress the pad about 0.02 mm. Either the attachment is much softer than the material, the cube pierces the
  pad, or the contact is dynamic (landing). To be settled with a static, controlled press.
- **Upstream's `dz`** (read from `extension.py`): `(R_case @ rest + T_case - current)[:, DEFORM_AXIS]` with
  `DEFORM_AXIS = 0`, over the 216 `node_ids`, divided by `the_max`, fed to the CNN as `(1, 18, 12, 1)`; the RIGHT
  pad's grid is mirrored (`fliplr`). It depends on how their scene poses the case (their world X is the pad
  normal), so it cannot be copied as is: we need a sensor-frame version (normal = local y), with the sign and the
  right-pad mirror to be fixed against their data.
- **Spike 2, load sweep (kinematic case, cube of 0.05 / 0.3 / 1 kg on the pad, 240 frames each):**

  | load | outer-face compression mean / max | cube |
  |---|---|---|
  | 0.49 N (1.5 kPa) | 0.18 / 0.62 mm | rests 0.9 mm into the pad |
  | 2.9 N (9 kPa) | 0.73 / 3.3 mm | stops at the case face |
  | 9.8 N (30 kPa) | 0.74 / 3.1 mm | stops at the case face |

  So the pad as authored is about **50x softer than E = 1.289 MPa** (about 25 kPa effective at 1.5 kPa) and **bottoms
  out at about 3 N**, which is not a usable tactile range. Candidates: the attachment to the case is soft, the
  deformable solver settings (`resolution`, damping), or something their scene changes at runtime. To settle
  by varying E and the attachment in the spike.
- **Spikes 3-6, why the pad is soft (all 6.2 PhysX, fresh stage each, case kinematic, free cube dropped on the pad):**
  - Young's modulus **has no effect**: 1.29 vs 64.5 MPa give the same result in every configuration (also on a
    fresh stage, so it is not a stale-cook problem). Density 0 vs 1000 kg/m3: no effect either.
  - The attachment is **not** the cause: with it disabled and a static box behind the pad, same result.
  - All the API schemas are applied correctly on the body, material, meshes and attachment (read from the raw
    list-op, `GetAppliedSchemas()` hides the unregistered ones).
  - **Solver iterations matter**: `physxDeformableBody:solverPositionIterationCount` 4 / 8 / 16 crush the pad to
    the case face under 0.3 kg (cube face at y = -21.3 mm); 32 is partial (-2.2 mm); **64 holds** it (0.42 mm
    indentation at 0.3 kg).
  - But even at 64 iterations the pad **crushes again at 3 kg (29 N) and above**, to the case face, at E = 1.3 and at
    64.5 MPa alike, up to 294 N. Real grips are tens to a couple hundred newtons, so the pad as imported cannot
    carry a grip, and node displacement saturates at the 3 mm thickness.
  - The same numbers come out of a standalone script (`/tmp/tsfprecise/press.py`) and of the pytest harness, so
    it is not the harness. Isaac 5.1 on this machine segfaults at startup (`carb.tasking`), so the same test could
    not be run there to see whether this is a 6.x regression.
  - Reading: the pad behaves as if it had no elastic resistance to a rigid body in contact. The 0.4 mm
    indentation at 64 iterations does not vary with load or E, so it is a contact skin, not elasticity. The pad is
    `PhysxAutoDeformableBodyAPI` with remeshing on (cooked from the 8-point `Cube` at resolution 17) and its
    collision against the case is filtered; both are candidates (a deformable squeezed between a rigid body and a
    filtered collider), not yet tested.
- **Spike 7, the remaining candidates (3 kg at 64 iterations, and 0.3 kg at default), none helps:** disabling the pad/case
  collision filter, turning remeshing and auto-simplification off (the pad is then crushed even further, to
  about -4.3 mm), and a larger contact offset on the collision mesh all leave the cube on the case face.
  **Status: the imported deformable cannot carry a rigid load on Isaac 6.2 PhysX with any setting tried, so the
  `Precise` variant is blocked on the pad, not on the port.** Needs the authors' scene or a pad we author.
- **Node cross-section check (`/tmp/tsfprecise/pad_cross_section.png`, 6.2, 64 iterations), numbers for the 36 outer-face
  nodes under the cube:** 0.5 N: thickness 3.04 mm (no change); 2.9 N: 2.67 mm (-12%); 29 N: **0.64 mm (-79%)**, all
  36 nodes moved to about -21.8 mm. So the pad is genuinely compressed (not pierced) and the 3 mm saturation was
  real. (An earlier note here said the cube pierced the pad; that was a misreading of the plot and is retracted.)
  The response is far softer than E = 1.289 MPa (which gives about 7% at 29 N) and does not depend on E.
- **Spike 8, THE FIX CANDIDATE: pad density (mass ratio).** Thickness under a 18 x 18 mm cube (rest 3.04 mm), E = 1.289 MPa:

  | pad density (kg/m3) | 29 N | 294 N |
  |---|---|---|
  | 1 000 (authored: 0 -> default) | 0.62 mm | 0.66 mm |
  | 100 000 | 3.06 mm | 2.64 mm (-13%) |
  | 1 000 000 and up | 3.05 mm | 3.05 mm |

  With a 3 g pad under a 3 kg cube (1000:1) the position-based solver cannot converge, so the pad behaves as if it had no
  stiffness whatever E is. Raising the pad's density to about 1e5 kg/m3 makes it resist and respond: -13% at 294 N is
  about what E = 1.3 MPa predicts at 0.45 MPa. (Tried without effect: E, attachment, filter, remesh, contact offset,
  solver iterations beyond 64.) To do: pick the density and iteration count, then re-check E now matters, then the
  CNN input and a real fingertip grasp.
- **Spike 9, E at density 1e5:** pad thickness at 294 N is 2.50 mm (E = 0.1 MPa), 2.64 mm (1.29 MPa), 2.66 mm (12.9 MPa):
  E matters, saturating (thin bonded layer). 29 N: 3.05 mm, 98 N: 2.93-2.96 mm.
- **Spike 10, CNN input convention (settled by physical consistency, no authors' data):**
  - The 216 `node_ids` do **not** form a spatial 18 x 12 grid in this file's node numbering (133 outer-layer, 83
    back-layer nodes, scattered in x and z); the numbering used when the config was made differs. Ignore them.
  - The mesh is a regular lattice of 12 (x) x 18 (z) x 2 (y) nodes. Build the 18 x 12 CNN input from the **outer
    layer** (y = -24.34 mm), rows = z descending (row 0 = fingertip end, as the first ids of the config), columns
    = x ascending, value = **current y - rest y** (compression is positive), divided by `the_max` = 1.616 mm.
  - With this input the 7 x 4 output follows the contact: centred cube gives centroid (row 3.3, col 1.55) = the map
    centre; moving the cube -6 -> +6 mm in x moves the column centroid +1.55; -10 -> +10 mm in z moves the row
    centroid by 1.6 (about half the expected 3.2 rows). The opposite sign, or the back layer, does not localise.
  - The CNN output is a map of the contact; its units are still unknown (values in the hundreds for 98 N), so
    `Precise` must be documented as an uncalibrated map, not newtons, until the authors answer.
- **Spike 11, mounted on our fingertip (6.2 PhysX, `Physx_parallel_grip`, `Tactile_TSF85`, closing on the repo's fixed test box
  12 x 32.2 x 16.5 mm):** works end to end.
  - Recipe (in `tests/runtime/test_precise_mount_spike.py`, untracked): per finger, de-instance
    `<side>_fingertip/visuals`, turn the `TSF_85_case` collider off, add a rigid **backing** body (28.4 x 21.3 x
    44 mm box at finger-frame y = +-52.05, z = 132.4 mm, mass 3 g) fixed-jointed to the fingertip, reference the upstream
    `/World/Deformable` as a sibling of the articulation prim, translated to (-+14.2, +-62.71, 110.42) mm in the finger
    frame (left pad also rotated 180 deg about z), deactivate its own `attachment`, density 1e5, 64 solver iterations,
    gravity off, then `deformableUtils.create_auto_deformable_attachment(pad, backing)` with overlap offset 0.5 mm.
    Needs `enableDeformableBodies` and GPU dynamics on the PhysicsScene.
  - Open: pads sit still (0.000 mm). Closed on the box: no NaN, no instability, the compression grid shows a clean
    contact patch of the box's width (cols 3-8 of 12), up to 0.38 mm, 25.4 mm of finger travel.
  - The nodes are in world-aligned pad-prim coordinates, so the rigid motion of the backing body must be removed
    (the pad prim does not move): `compression = sign * (q - q0)_y` with `q = P_world @ inverse(backing_world)`.
    There is a uniform residual of about -30 um outside the contact: tare at the open pose.
  - The CNN map peaks where the patch maps (rows 3-5, cols 1-2). Values up to about 600 for 0.38 mm.
- **Spike 12, the built variant, and the density choice (6.2, `Physx_parallel_grip`):**
  - Authored `SensorSim=Precise` composes and cooks; the pad follows the fingers; open: 0.000 mm.
  - Without the **element collision filter** the pad's back layer collided with the backing and was pushed off it (outer-layer
    ends 0.8 mm away from the backing); collision groups cannot replace it (the pad detaches); the data PhysX cooks at the
    first play (216 attached back-layer vertices, 1332 filtered elements, like upstream) is captured by
    `tools/capture_tsf85_pad_attachment.py` and embedded in the payload.
  - Density vs signal in the gripper (box 12 x 16.5 mm): 2e4 950 um (100 % under the box columns), 3e4 310 um (99 %), 4e4 120 um
    (93-95 %), 6e4 80 um (78-86 %), 1e5 55 um (60 %). Attached pad under an 18 x 18 mm cube (rest 3.04 mm): 3e4 2.64 / 1.29 mm at 29 / 98 N;
    4e4 2.74 / 1.79 / 1.19 mm at 29 / 98 / 294 N; 6e4 2.82 / 2.15 / 0.67 mm. **Chosen: 4e4.**
  - `Physx_compliant`: the finger closes to 1.5 rad on the box, crushes the pad and does not reopen: not supported.
  - Isaac lock hygiene: a headless script that created a new stage per case hit `omni.rtx: Out of resource descriptors` and hung
    for 45 minutes holding the lock; wrap runs in `timeout` (which also counts the queue wait) and watch the log for `Fatal`.
- **Environment gotchas:** run pytest for Isaac with `env -u PYTHONPATH -u AMENT_PREFIX_PATH ...`, otherwise the
  ROS Humble pytest plugins load and fail on a missing module; the throwaway spike is
  `tests/runtime/test_precise_spike.py` (not committed).

## Steps

0. **Spike (read-only first, then live).**
   - Compare mount frames (above) on the files, no Isaac needed.
   - On 6.2 (through the isaac-lock queue, ask before the GUI): compose `TSF_85.usd` on our fingertip, check the
     deformable mesh simulates, the node arrays can be read, and the ONNX model runs at an acceptable cost per
     step. Confirm the CNN output is 28 values.
   - Go / no-go for the rest.
1. **Import.** `TSF_85.usd` into `grippers/Robotiq_2F_85/parts/` (LFS, like `TSF_85_case.usd`);
   `tactile_cnn.onnx` and `sensor_config.json` into `grippers/tactile/precise/data/`.
2. **Port the pipeline** into `tactile/precise/`, dropping the Kit UI (omni.ui panel, menu, heatmap PNGs, CSV
   logging). Keep: deformable-mesh lookup, case-pose transform, node dz computation, node-id to 18 x 12 grid
   mapping, normalisation, ONNX inference.
3. **Reader.** `PreciseTactile(TactileSensor)` in `readers.py` (`prepare`, `read`, the supported
   `quantity`), registered in `make_tactile_sensor`. Raises a clear error under Newton.
4. **Variant.** New payload `payloads/Robotiq_2F_85_tactile_precise.usda` (the deformable sensing mesh and
   `robotiq:tactile:method = precise`), generated by `grippers/tactile/generate_tactile_zones.py` like the other three.
   Register `"Precise"` in the `SensorSim` variant set of `Robotiq_2F_85.usda`. `Standard` and the other
   `SensorSim` values stay unchanged (check with the per-prim physics signature, not a flatten diff).
5. **Dependency.** onnxruntime imported lazily so the other variants work without it; clear error if missing.
6. **Tests.**
   - Unit (numpy + onnx, no Isaac): node-dz to grid mapping, CNN on a fixed input, config loading. Add to
     `.github/workflows/tactile-unit.yml`.
   - Runtime (`tests/runtime/test_tactile_sensor.py`, `PRECISE_VARIANTS` in `_tactile.py`): variant composes,
     sensor created before play, pressing on a cube gives a non-zero map. Run with `--gui` under the lock.
   - Existing `EffectiveResponse` / `Raycast` tests and the SimReady baselines in `ci/simready/expected/` still pass.
7. **Demo and docs.** `TELEOP_SENSORSIM=Precise` in `grippers/demo/teleop/run_demo.sh` and `open_scene.py`;
   root `README.md` variant table; `tactile/README.md`; `grippers/demo/TACTILE_DEMO.md`. Mark `Precise` as
   simulated, PhysX only, validated on 6.2.
8. **Attribution.**
   - Each ported file: short provenance header, "Derived from Lab-CORO/TSF-85 (ETS CoRo), used under agreement
     with Robotiq". No Apache-2.0 header or licence text.
   - `tactile/README.md` (or a short `ATTRIBUTION.md`): authors (Berith De la Cruz Sanchez, Jean-Philippe
     Roberge), repo URL, and the two citations:
     - De la Cruz Sanchez & Roberge, "A hybrid elastic-hyperelastic approach for simulating soft tactile
       sensors", Frontiers in Robotics and AI 12:1639524 (2025).
     - De la Cruz-Sanchez, Kwiatkowski & Roberge, "Tactile Contact Patterns for Robotic Grasping: A Dataset of
       Real and Simulated Data" (manuscript submitted, 2024).
   - The imported binaries (USD, ONNX, JSON) get the same attribution in the README.

## Delivery

Small commits in this order: import, port, variant, tests, docs. One PR; no merge without explicit approval.
