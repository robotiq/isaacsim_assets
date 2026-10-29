# TSF-85 tactile sensor

> Training in **Isaac Lab**? The simulated sensor for it, usable on any gripper that carries a TSF, is in
> [`grippers/tactile_sensors`](../tactile_sensors/README.md). This folder is the Isaac Sim reader (variants, contact patch, live heatmap).

The TSF-85 fingertip of the 2F-85 (`Tactile_TSF85`) and of the 2F-140 (`Tactile_TSF140`, the same sensor on the
2F-140's finger) can carry a simulated TSF-85 tactile sensor: a **7×4 grid of taxels per finger**
(row 0 at the fingertip end), filled from the physics engine's contact data. Three models, selected by
`SensorSim`. **Uncalibrated:** the output is not in the real sensor's counts and has not been compared with
recordings of it (the paper's per-taxel gain and noise are not modelled).

## Selecting a model

On the gripper's root prim (`Robotiq_2F_85` or `Robotiq_2F_140`), the same `SensorSim` variants:

| variantSet | value | |
|---|---|---|
| `Fingertip` | `Tactile_TSF85` (2F-85) / `Tactile_TSF140` (2F-140) | the TSF fingertip; `SensorSim` only exists under it |
| `SensorSim` | `None` | no sensor (default) |
| `SensorSim` | `EffectiveResponse` | **recommended.** The effective contact response of the pad's contact force and patch. Rigid pad; PhysX and Newton, any step rate. |
| `SensorSim` | `Precise` | The deformable pad and the neural network of the ETS CoRo lab's TSF-85 model: the 3 mm layer is simulated as a soft body and its compression becomes the 7×4 map through the lab's CNN. PhysX only (GPU dynamics), needs `onnxruntime`. [Below.](#precise) |
| `SensorSim` | `Raycast` | A ray per taxel measures the depth of the object past a **compliant pad**, and the pad's spring turns it into a force. PhysX only, needs a step of **at least 240 Hz** (validated at 1000 Hz): at 60 Hz the pad is not resolved and the depth no longer follows the force (a held object barely sinks, a free one sinks far too deep). |

```python
prim.GetVariantSet("Fingertip").SetVariantSelection("Tactile_TSF85")   # "Tactile_TSF140" on the 2F-140
prim.GetVariantSet("SensorSim").SetVariantSelection("EffectiveResponse")
```

## Reading it

This folder has one reader per model and `make_tactile_sensor(stage, root, engine)` returns the one
for the stage's variant. Create it **before play**, then each frame while playing:

```python
sensor = make_tactile_sensor(stage, "/World/Gripper/Robotiq_2F_85", "physx")   # or "newton"; .../Robotiq_2F_140
grids = sensor.read()        # {"left": (7, 4), "right": (7, 4)}
```

`EffectiveResponse` and `Raycast` give the same maps (`Precise` gives its own, [below](#precise)); `sensor.quantity` picks what `read()` returns:

| `quantity` | method | |
|---|---|---|
| `"force"` (default) | `read_force()` | N each taxel carries; the taxels sum to the pad force |
| `"response"` | `read_response()` | the effective contact response r (Shang et al., arXiv 2610.10510, Eq. 1) |
| `"depth"` | `read_depth()` | mm past the pad surface, `Raycast` only |
| `"cnn"` (`Precise`'s only one) | `read_cnn()` | the lab's network output, in its own units, 0 at rest |

- **`EffectiveResponse`** needs the pad's contact reports on before play (`prepare` does it, and the variant keeps
  the articulation awake: a sleeping gripper reports no contacts). On Newton the force of a contact is its
  `efc.force` normal row (summed over the facets for a pyramidal cone), so it is approximate and can split unevenly
  between the pads.
- **`Raycast`** converts the depth with the soft layer's stiffness `padStiffness` = E / thickness =
  1.289 MPa / 3 mm = 0.43 N/mm³ (ETS CoRo lab TSF-85 model, github.com/Lab-CORO/TSF-85). PhysX's compliant pad
  is a spring per contact point, so how deep a given force sinks the object follows how many contact points PhysX
  makes: the raycast force is x0.65-1.75 of the true pad force (x0.7 at light grips, x1.15-1.5 on a free box, up
  to x1.7 after a fast close).
  Prefer `EffectiveResponse` for forces.

The model parameters travel on the stage as `robotiq:tactile:*` attributes of each finger's `tactile_zone_visuals`
(`method`; `rayStandoff`, `rayMaxDistance`, `padStiffness`; `sigmaMm`, `eta`, `gamma`).

## Files

The sensor is the same on both grippers, so this folder is shared; each gripper only has its generated payloads.

| | |
|---|---|
| `<gripper>/payloads/<gripper>_tactile_zones.usda` | the marker grid, shared by both models |
| `<gripper>/payloads/<gripper>_tactile_raycast.usda` | `Raycast`: the compliant pad and the ray parameters |
| `<gripper>/payloads/<gripper>_tactile_effective_response.usda` | `EffectiveResponse`: the Eq. 1 parameters, articulation kept awake |
| `Robotiq_2F_85/payloads/Robotiq_2F_85_tactile_precise.usda` | `Precise` (2F-85): the deformable pads, their rigid backings and attachments |
| `Robotiq_2F_85/parts/TSF_85_pad.usd` | the lab's deformable pad (432 nodes), see [Precise](#precise) |
| `precise.py`, `data/` | the pad lattice, compression and network (numpy and onnxruntime, no Isaac); the lab's `tactile_cnn.onnx` |
| `Robotiq_2F_85/tools/extract_tsf85_pad.py`, `Robotiq_2F_85/tools/capture_tsf85_pad_attachment.py` | how `parts/TSF_85_pad.usd` and the pad attachment data of the `Precise` payload were made |
| `generate_tactile_zones.py` | generates those payloads for every gripper in `MOUNTS` (where the sensing area sits on each gripper's inner finger, measured from its TSF tip); `python3 grippers/tactile/generate_tactile_zones.py` |
| this folder | `effective_response.py` (Eq. 1), `contact_points.py` (PhysX / Newton contacts), `readers.py`, `plot.py` |

The markers do not collide: the TSF pad is the only collider. Their front face is on the collider's surface,
which on the 2F-140 is the TSF-140 tip body: its collider (`tip_tsf140/mesh_0`) leaves out the sensing pad, whose
visual stands 1.9 mm proud of it, so a gripped object looks 1.9 mm sunk into the 2F-140's pads.

To add a gripper: measure its sensing area in the `<side>_inner_finger` frame, add it to `MOUNTS`, run the
generator, nest the `SensorSim` variant set under its TSF fingertip (as in `Robotiq_2F_140.usda`), and add it to
`FINGERTIPS` in `tests/runtime/_tactile.py` with a box preset `tests/runtime/fixed_cubes/<gripper>_tactile.json`.

Tests: `tests/runtime/test_tactile_sensor.py` (Isaac Sim, both grippers), `tests/runtime/test_tactile_precise.py` (`Precise`, 2F-85) and
`tests/unit/test_effective_response.py`, `tests/unit/test_precise.py` (numpy / scipy / onnxruntime only). Demo: [`../demo/TACTILE_DEMO.md`](../demo/TACTILE_DEMO.md).

## Precise

`SensorSim=Precise` is the TSF-85 model of the Control and Robotics Laboratory (CoRo) of the École de technologie
supérieure, Montréal ([github.com/Lab-CORO/TSF-85](https://github.com/Lab-CORO/TSF-85)), put on our asset. Each
fingertip gets the lab's **deformable pad** (a 26 × 3 × 40 mm soft body of 432 nodes, E = 1.289 MPa, ν = 0.173)
in place of the rigid pad collider, on a rigid backing fixed to the fingertip. The lab's **CNN** turns the
compression of the pad's outer 216 nodes (an 18 × 12 image) into the 7 × 4 map.

```python
sensor = make_tactile_sensor(stage, "/World/Gripper/Robotiq_2F_85", "physx")   # before play; "newton" raises
maps = sensor.read()          # {"left": (7, 4), "right": (7, 4)}: the network's units, 0 at rest
images = sensor.compression_images()   # {"left": (18, 12), ...}: pad compression in m, the network's input
```

- **Engine and requirements:** PhysX only, because deformable bodies are; `make_tactile_sensor` raises under
  Newton. Needs the GPU pipeline (`prepare` switches `/physics/enableDeformableBodies` and the scene's GPU dynamics on, before
  play) and `onnxruntime` in Isaac's Python (`<isaac>/python.sh -m pip install onnxruntime`). Validated on **Isaac
  Sim 6.2** with `Physx_parallel_grip`; `Physx_compliant` closes much further on the test box, crushes the 3 mm pad
  against it and does not reopen, so it is not supported.
- **What the map is:** the lab's network output, **not newtons and not calibrated**: the lab has not published the
  units, so it is a map of where and how hard the pad is pressed, to be compared with itself. It is 0 at rest (the
  network's response to no compression is subtracted) and follows the contact: for a bump moved over the pad its
  centroid moves with it, but the map is smeared toward its centre (the network's own behaviour).
- **Compression:** per node of the outer layer, how far it moved toward the backing along the pad normal, in the
  frame of the backing body (so the finger's own motion does not count), positive when pressed in. The compression at the first `read` (the pad at rest) is
  the zero; `tare()` sets it again. Rows run from the fingertip end, columns along the finger's +X, for both fingers.
- **Signal size:** a grip of about 10 N on the 12 × 16.5 mm test box compresses the pad by about 0.12 mm (of the
  network's 1.6 mm full scale); the map then reads around 130, and about 0 at rest (the rest noise is a few µm).
- **Pad density:** the lab's file leaves the pad's density at the schema default, a 3 g pad. PhysX's position-based
  soft-body solver cannot hold a gripper's load on a pad that light: the pad is crushed through its 3 mm whatever its
  modulus. `parts/TSF_85_pad.usd` sets the density to 4e4 kg/m³ and 64 solver iterations. With the pad attached, an 18 × 18 mm
  cube then compresses it by 10 % at 29 N, 41 % at 98 N and 61 % at 294 N (it never bottoms out), and the response under the
  gripper is localised (93-95 % under the box). Lower densities give a bigger signal but saturate sooner (3e4: 58 % at 98 N; below
  about 2e4 it is crushed at a gripper's force); higher ones are stiffer and the signal smaller (1e5: 0.06 mm under the box, 60 % localised).
  The pad's weight is off (`disableGravity`). This is a numerical stabilisation, not a physical density.
- **Attachment:** the back layer of the pad is pinned to the backing, and the pad's elements near the backing do not
  collide with it. Both are cooked by PhysX at the first play; `tools/capture_tsf85_pad_attachment.py` records them
  for the payload, so the asset does not depend on that cook at load (re-run it, then `generate_tactile_zones.py`, if the pad or backing placement changes).

### Attribution

The deformable pad, the network (`data/tactile_cnn.onnx`) and its normalisation come from the TSF-85 Isaac Sim
extension of the Control and Robotics Laboratory (CoRo) of the École de technologie supérieure (Montréal), used
under agreement with Robotiq. If you use this model in your research, please cite:

```bibtex
@article{de2025hybrid,
  title={A hybrid elastic-hyperelastic approach for simulating soft tactile sensors},
  author={De la Cruz S{\'a}nchez, Berith Atemoztli and Roberge, Jean-Philippe},
  journal={Frontiers in Robotics and AI},
  volume={12},
  pages={1639524},
  year={2025},
  publisher={Frontiers Media SA}
}

@unpublished{BerKwaJea2024,
  title={Tactile Contact Patterns for Robotic Grasping: A Dataset of Real and Simulated Data},
  author={De la Cruz-S{\'a}nchez, Berith Atemoztli and Kwiatkowski, Jennifer and Roberge, Jean-Philippe},
  note={Manuscript submitted for publication},
  year={2024},
}
```
