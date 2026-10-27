# Simulated TSF tactile sensors for Isaac Lab

A TSF tactile sensor is the same part whichever gripper carries it, so its simulation lives here, separate from the
grippers. A gripper only says where the sensor sits on its fingertips; anyone training in Isaac Lab then gets the taxel
maps from one line of scene config, without writing a sensor model.

```python
from robotiq_tactile_sensors.isaaclab import (
    ROBOTIQ_2F_85, ROBOTIQ_2F_140, ROBOTIQ_2F85_TSF85_CFG, TactileSensorCfg, tactile_map,
)

@configclass
class MySceneCfg(InteractiveSceneCfg):
    robot = ...   # a robot with a 2F-85 (Tactile_TSF85) or a 2F-140 (Tactile_TSF140), spawned with activate_contact_sensors=True
    tactile = TactileSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.../Robotiq_2F_85",         # the prim that holds left_fingertip, right_fingertip
        mount=ROBOTIQ_2F_85,                                        # the default; ROBOTIQ_2F_140 for the 2F-140
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Object"],           # what the pads can feel (rigid bodies)
    )

# in the observations (or read it yourself: scene["tactile"].taxel_force, shape (num_envs, 2, 7, 4), newtons)
tactile = ObsTerm(func=tactile_map, params={"sensor_cfg": SceneEntityCfg("tactile")})   # (num_envs, 56), log(1 + N)
```

| | |
|---|---|
| `sensor.taxel_force` | `(num_envs, 2, rows, cols)` N per taxel. Pad 0 = left, pad 1 = right, row 0 at the fingertip end. |
| `sensor.pad_force` | `(num_envs, 2)` N each pad carries (the taxels sum to it). |
| `sensor.object_force` | `(num_envs, 2, num_objects)` N each pad carries from each filtered object, in the order of `filter_prim_paths_expr`. |
| `sensor.taxel_positions_w` | the taxel centres in the world frame, `(num_envs, 2, rows * cols, 3)`. |
| `sensor.data` | the raw `ContactSensor` data. |
| `ROBOTIQ_2F85_TSF85_CFG`, `ROBOTIQ_2F140_TSF140_CFG` | the bare gripper on TSF fingertips as an `ArticulationCfg` (variants `Fingertip=Tactile_TSF85` / `Tactile_TSF140`, `SensorSim=EffectiveResponse`). To mount it on an arm, weld its `base_link` to the flange: see the weld in `demo/teleop/ur5robot_with_2F-85.usda`. |

Setup, once: `pip install -e grippers/tactile_sensors` into the environment that runs Isaac Lab. Isaac Lab 3.0 with the PhysX
backend (Isaac Sim 6.2).

The gripper USD assets are not part of the package: they are the `Robotiq_2F_*` folders of this repo's `grippers/`. The robot configs
find them next to the package from an editable install; with another install, pass `robotiq_2f85_tsf85_cfg(grippers_dir=...)` or set
`ROBOTIQ_GRIPPERS_DIR` to the `grippers/` folder. A missing asset is an error when the config is built, not at spawn.

## How it is simulated

1. **Contact.** A PhysX `ContactSensor` on the two pad bodies, filtered on the objects to feel, with contact points on. For
   each pad and object it gives the normal force and the average contact point. Forces under 0.05 N are dropped.
2. **Taxels.** The 7 x 4 grid of the TSF (`tsf.py`: 5.486 mm between the 7 rows, 5.975 mm between the 4 columns) is placed on each pad by the gripper's *mount* (`mounts.py`)
   and follows the pad body's pose.
3. **Spreading** (`spread.py`). Each object's force is spread over the taxels around its contact point with a Gaussian
   (sigma 6 mm) and normalised, so the taxels of a pad sum to the force the pad carries. Objects add up. A contact farther than
   8 mm (`max_contact_distance`) from every taxel is not on the sensing area (the back or the side of the finger, the part of the
   pad outside the array) and reads nothing, instead of showing up at full force on the nearest taxels.

The logic of steps 1-3 is in pure torch functions (`sensing.py`), which `tests/unit` runs without Isaac Lab; `isaaclab/sensor.py` only reads the
contact sensor's buffers and calls them.

It is a model, not a calibration. The pad is rigid (no compliance, no shear), one object gives one averaged contact point per
pad (an edge contact or two touching points of one object are blurred into one blob), sigma is a free parameter, and the
output is in newtons of normal force, not the real sensor's counts. It is cheap enough for thousands of environments, which
the Isaac Sim reader in `grippers/tactile/` (the contact-patch model of Shang et al.) is not.

## Another gripper with a TSF

Write its mount: the centre of taxel (row 0, col 0) and the directions in which the rows and the columns grow, in the frame
of each pad's rigid body (the taxel markers of the gripper's USD give them), and its pad body names.

```python
from robotiq_tactile_sensors.tsf import TSF85, FingertipMounts, PadMount

MY_GRIPPER = FingertipMounts(
    left=PadMount("left_pad", origin=(...), row_axis=(...), col_axis=(...)),
    right=PadMount("right_pad", origin=(...), row_axis=(...), col_axis=(...)),
    spec=TSF85,                                   # a TSF with another grid: TsfSpec(rows, cols, height, width, gap)
)
TactileSensorCfg(prim_path="...", mount=MY_GRIPPER, filter_prim_paths_expr=[...])
```

The pad bodies need collision shapes, and the robot must be spawned with `activate_contact_sensors=True`. The mounts of the
2F-85 (`ROBOTIQ_2F_85`) and of the 2F-140 (`ROBOTIQ_2F_140`) are in `mounts.py`: the sensing-area numbers of
`grippers/tactile/generate_tactile_zones.py`, the script that places the taxel markers in each gripper's USD. A unit test reads the
markers of each gripper's `*_tactile_zones.usda` and checks that the taxels here are those markers, so a mount that drifts from the
USD, or a USD that was not regenerated, fails. The Hand-E has no TSF fingertip.

## Checks

- `tests/unit/test_tactile_taxels.py` (CPU, in CI): the taxels against the markers in each gripper's USD; the spreading (the taxels sum to the
  force, the peak is under the contact, objects add up, batches with different contacts per env and pad give each slice's own result, the
  contact-distance gate); and the sensor logic of `sensing.py`: quaternion rotation, the left/right pad order whatever order the
  contact sensor lists its bodies in, the force floor, NaN contact points, taxels following the pad pose, a contact on a taxel lighting that taxel.
- The blind sphere-picking task (`demo/rl`) is built on it: the sensor reproduced that task's earlier private implementation exactly
  (0.0 N difference over a scripted grasp), and the trained policy still empties the box.
- `examples/check_tactile_sensor.py` (Isaac Lab, GPU): the bare gripper closes on a zero-gravity sphere and opens again;
  asserts no force when open, equal force on both pads (2F-85: 10.7 N, 2F-140: 3.2 N at its stock drive), and the force centroid
  within 6 mm of the sphere along the pad (measured: 0.05 mm on the 2F-85, 0.02 mm on the 2F-140), and no force after opening.
  `--gripper 2f85|2f140` picks the gripper. The sphere is put at the middle of the taxel arrays once the pads have almost closed
  on it (the pads slide along the fingers as the gripper closes), so it tests the mount geometry end to end.

## Limits

- PhysX only. Isaac Lab's contact sensor also exists on Newton; this has not been tried there.
- The contact-distance gate looks at the averaged contact point of each object: an object touching the pad within 8 mm of the array (the rim
  of the case just outside the taxels) still reads on the edge taxels, and a contact patch that spans the array and the case is averaged
  into one point, which can fall off the array. A push with the back of the finger, 2 cm behind the taxel plane, reads nothing. Set
  `max_contact_distance=None` to report every contact on the nearest taxels.
- The filtered objects must be rigid bodies: a static collider cannot be a contact-filter target (its filtered force reads 0
  although the unfiltered one does not). Make walls and tables kinematic rigid bodies.
