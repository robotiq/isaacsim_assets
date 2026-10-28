# TSF-85 tactile sensor

The TSF-85 fingertip of the 2F-85 (`Tactile_TSF85`) and of the 2F-140 (`Tactile_TSF140`, the same sensor on the
2F-140's finger) can carry a simulated TSF-85 tactile sensor: a **7×4 grid of taxels per finger**
(row 0 at the fingertip end), filled from the physics engine's contact data. Two models, selected by
`SensorSim`. **Uncalibrated:** the output is not in the real sensor's counts and has not been compared with
recordings of it (the paper's per-taxel gain and noise are not modelled).

## Selecting a model

On the gripper's root prim (`Robotiq_2F_85` or `Robotiq_2F_140`), the same `SensorSim` variants:

| variantSet | value | |
|---|---|---|
| `Fingertip` | `Tactile_TSF85` (2F-85) / `Tactile_TSF140` (2F-140) | the TSF fingertip; `SensorSim` only exists under it |
| `SensorSim` | `None` | no sensor (default) |
| `SensorSim` | `EffectiveResponse` | **recommended.** The effective contact response of the pad's contact force and patch. Rigid pad; PhysX and Newton, any step rate. |
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

Both models give the same maps; `sensor.quantity` picks what `read()` returns:

| `quantity` | method | |
|---|---|---|
| `"force"` (default) | `read_force()` | N each taxel carries; the taxels sum to the pad force |
| `"response"` | `read_response()` | the effective contact response r (Shang et al., arXiv 2610.10510, Eq. 1) |
| `"depth"` | `read_depth()` | mm past the pad surface, `Raycast` only |

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
| `generate_tactile_zones.py` | generates those payloads for every gripper in `MOUNTS` (where the sensing area sits on each gripper's inner finger, measured from its TSF tip); `python3 grippers/tactile/generate_tactile_zones.py` |
| this folder | `effective_response.py` (Eq. 1), `contact_points.py` (PhysX / Newton contacts), `readers.py`, `plot.py` |

The markers do not collide: the TSF pad is the only collider. Their front face is on the collider's surface,
which on the 2F-140 is the TSF-140 tip body: its collider (`tip_tsf140/mesh_0`) leaves out the sensing pad, whose
visual stands 1.9 mm proud of it, so a gripped object looks 1.9 mm sunk into the 2F-140's pads.

To add a gripper: measure its sensing area in the `<side>_inner_finger` frame, add it to `MOUNTS`, run the
generator, nest the `SensorSim` variant set under its TSF fingertip (as in `Robotiq_2F_140.usda`), and add it to
`FINGERTIPS` in `tests/runtime/_tactile.py` with a box preset `tests/runtime/fixed_cubes/<gripper>_tactile.json`.

Tests: `tests/runtime/test_tactile_sensor.py` (Isaac Sim, both grippers) and `tests/unit/test_effective_response.py`
(numpy / scipy only). Demo: [`../demo/TACTILE_DEMO.md`](../demo/TACTILE_DEMO.md).
