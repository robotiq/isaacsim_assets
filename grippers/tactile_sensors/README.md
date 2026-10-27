# Simulated TSF tactile sensors for Isaac Lab

A TSF tactile sensor is the same part whichever gripper carries it, so its simulation lives here, separate from the
grippers. A gripper only says where the sensor sits on its fingertips; anyone training in Isaac Lab then gets the taxel
maps from one line of scene config, without writing a sensor model.

```python
from robotiq_tactile_sensors.isaaclab import TactileSensorCfg, tactile_map, ROBOTIQ_2F85_TSF85_CFG

@configclass
class MySceneCfg(InteractiveSceneCfg):
    robot = ...   # a robot with a 2F-85 on Tactile_TSF85 fingertips, spawned with activate_contact_sensors=True
    tactile = TactileSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.../Robotiq_2F_85",         # the prim that holds left_fingertip, right_fingertip
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
| `ROBOTIQ_2F85_TSF85_CFG` | the bare 2F-85 on TSF-85 fingertips as an `ArticulationCfg` (variants `Fingertip=Tactile_TSF85`, `SensorSim=EffectiveResponse`). To mount it on an arm, weld its `base_link` to the flange: see the weld in `demo/teleop/ur5robot_with_2F-85.usda`. |

Setup, once: `pip install -e grippers/tactile_sensors` into the environment that runs Isaac Lab. Isaac Lab 3.0 with the PhysX
backend (Isaac Sim 6.2).

## How it is simulated

1. **Contact.** A PhysX `ContactSensor` on the two pad bodies, filtered on the objects to feel, with contact points on. For
   each pad and object it gives the normal force and the average contact point. Forces under 0.05 N are dropped.
2. **Taxels.** The 7 x 4 grid of the TSF (`tsf.py`, 6 mm x 5.5 mm pitch) is placed on each pad by the gripper's *mount* (`mounts.py`)
   and follows the pad body's pose.
3. **Spreading** (`spread.py`). Each object's force is spread over the taxels around its contact point with a Gaussian
   (sigma 6 mm) and normalised, so the taxels of a pad sum to the force the pad carries. Objects add up.

It is a model, not a calibration. The pad is rigid (no compliance, no shear), one object gives one averaged contact point per
pad (an edge contact or two touching points of one object are blurred into one blob), sigma is a free parameter, and the
output is in newtons of normal force, not the real sensor's counts. It is cheap enough for thousands of environments, which
the Isaac Sim reader in `Robotiq_2F_85/tactile` (the contact-patch model of Shang et al.) is not.

## Another gripper with a TSF

Write its mount: the centre of taxel (row 0, col 0) and the directions in which the rows and the columns grow, in the frame
of each pad's rigid body (the taxel markers of the gripper's USD give them), and its pad body names.

```python
from robotiq_tactile_sensors.tsf import TSF85, FingertipMounts, PadMount

MY_GRIPPER = FingertipMounts(
    left=PadMount("left_pad", origin=(...), row_axis=(...), col_axis=(...)),
    right=PadMount("right_pad", origin=(...), row_axis=(...), col_axis=(...)),
    spec=TSF85,                                   # a TSF with another grid: TsfSpec(rows, cols, pitch_row, pitch_col)
)
TactileSensorCfg(prim_path="...", mount=MY_GRIPPER, filter_prim_paths_expr=[...])
```

The pad bodies need collision shapes, and the robot must be spawned with `activate_contact_sensors=True`. Only the 2F-85
has a taxel layout today (`mounts.ROBOTIQ_2F_85`); the 2F-140's TSF fingertip is geometry only.

## Checks

- `tests/unit/test_tactile_taxels.py` (CPU, in CI): the layout against the marker grid, the spreading (the taxels sum to the
  force, the peak is under the contact, objects add up, batches are independent).
- `examples/check_tactile_sensor.py` (Isaac Lab, GPU): the bare gripper closes on a zero-gravity sphere and opens again;
  asserts no force when open, equal force on both pads (10.7 N), and the force centroid within 6 mm of the sphere along the pad
  (measured: 1 mm), and no force after opening.

## Limits

- PhysX only. Isaac Lab's contact sensor also exists on Newton; this has not been tried there.
- The filtered objects must be rigid bodies: a static collider cannot be a contact-filter target (its filtered force reads 0
  although the unfiltered one does not). Make walls and tables kinematic rigid bodies.
