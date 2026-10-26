# TSF-85 tactile sensor

> Training in **Isaac Lab**? The simulated sensor for it, usable on any gripper that carries a TSF, is in
> [`grippers/tactile_sensors`](../../tactile_sensors/README.md). This folder is the Isaac Sim reader (variants, contact patch, live heatmap).

The `Tactile_TSF85` fingertip can carry a simulated TSF-85 tactile sensor: a **7×4 grid of taxels per finger**
(row 0 at the fingertip end), filled from the physics engine's contact data. Two models, selected by
`SensorSim`. **Uncalibrated:** the output is not in the real sensor's counts and has not been compared with
recordings of it (the paper's per-taxel gain and noise are not modelled).

## Selecting a model

On the root `Robotiq_2F_85` prim:

| variantSet | value | |
|---|---|---|
| `Fingertip` | `Tactile_TSF85` | the TSF-85 fingertip; `SensorSim` only exists under it |
| `SensorSim` | `None` | no sensor (default) |
| `SensorSim` | `EffectiveResponse` | **recommended.** The effective contact response of the pad's contact force and patch. Rigid pad; PhysX and Newton, any step rate. |
| `SensorSim` | `Raycast` | A ray per taxel measures the depth of the object past a **compliant pad**, and the pad's spring turns it into a force. PhysX only, needs a step of **at least 240 Hz** (validated at 1000 Hz): at 60 Hz the pad is not resolved and the force reads about 0. |

```python
prim.GetVariantSet("Fingertip").SetVariantSelection("Tactile_TSF85")
prim.GetVariantSet("SensorSim").SetVariantSelection("EffectiveResponse")
```

## Reading it

This folder has one reader per model and `make_tactile_sensor(stage, root, engine)` returns the one
for the stage's variant. Create it **before play**, then each frame while playing:

```python
sensor = make_tactile_sensor(stage, "/World/Gripper/Robotiq_2F_85", "physx")   # or "newton"
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
  1.289 MPa / 3 mm = 0.43 N/mm³ (ETS CoRo lab TSF-85 model, github.com/Lab-CORO/TSF-85).

The model parameters travel on the stage as `robotiq:tactile:*` attributes of each finger's `tactile_zone_visuals`
(`method`; `rayStandoff`, `rayMaxDistance`, `padStiffness`; `sigmaMm`, `eta`, `gamma`).

## Files

| | |
|---|---|
| `payloads/Robotiq_2F_85_tactile_zones.usda` | the marker grid, shared by both models |
| `payloads/Robotiq_2F_85_tactile_raycast.usda` | `Raycast`: the compliant pad and the ray parameters |
| `payloads/Robotiq_2F_85_tactile_effective_response.usda` | `EffectiveResponse`: the Eq. 1 parameters, articulation kept awake |
| `tools/generate_tactile_zones.py` | generates the three payloads; the grid frame is measured from the TSF-85 case bounds |
| `tactile/` (this folder, with this README) | `effective_response.py` (Eq. 1), `contact_points.py` (PhysX / Newton contacts), `readers.py`, `plot.py` |

The markers do not collide: the TSF-85 pad is the only collider. Tests: `tests/runtime/test_tactile_sensor.py`
(Isaac Sim) and `tests/unit/test_effective_response.py` (numpy / scipy only). Demo: [`../../demo/TACTILE_DEMO.md`](../../demo/TACTILE_DEMO.md).
