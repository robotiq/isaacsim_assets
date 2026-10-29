"""``SensorSim=Precise`` of the 2F-85: the deformable TSF-85 pad of the ETS CoRo lab and its CNN (PhysX only).

The gripper closes on the fixed box of the other tactile tests (fixed_cubes/Robotiq_2F_85_tactile.json, 12 mm wide)
and opens again. The map is in the network's own units, not newtons, so what is checked is that it is zero at rest,
appears under the box on both pads, and goes back when the gripper opens. What they check is in tests/runtime/README.md.
"""

from __future__ import annotations

import numpy as np
import pytest

from _cube import add_fixed_cube
from _scene import load_gripper, move_to, step
from tactile import PreciseTactile, make_tactile_sensor
from _tactile import PRECISE_VARIANTS

GRIPPER = "Robotiq_2F_85"
PRESET = f"{GRIPPER}_tactile"
# Physx_compliant is not covered: that variant closes much further on the box (finger_joint 1.5 rad), crushes the 3 mm pad
# against it and does not reopen (see tactile/PRECISE_PLAN.md).
PHYSICS = ["Physx_parallel_grip"]
BOX_COLUMNS = range(2, 10)  # of the 12 image columns: the 12 mm box is on columns 3 to 8 of the 2.37 mm lattice
MIN_COMPRESSION_MM = 0.05  # the 3 mm pad compresses about 0.12 mm under the 12 x 16.5 mm box
MIN_MAP = 40.0  # network units: about 130 at that compression, where the rest reads under 10
RELEASED = 0.15  # the map is back under this share of its peak once the gripper is open again


def _load(simulation_app, physics):
    built = {}

    def setup(stage):
        add_fixed_cube(stage, PRESET)
        built["stage"] = stage
        built["sensor"] = make_tactile_sensor(stage, f"/World/Gripper/{GRIPPER}", "physx")

    scene = load_gripper(simulation_app, GRIPPER, physics, None, setup=setup, variants=PRECISE_VARIANTS)
    return scene, built["sensor"], built["stage"]


@pytest.mark.parametrize("physics", PHYSICS)
def test_precise_sensor(simulation_app, physics):
    scene, sensor, stage = _load(simulation_app, physics)
    assert isinstance(sensor, PreciseTactile), type(sensor)
    assert sensor.quantity == "cnn"
    step(simulation_app, 60)
    sensor.read()  # the first read takes the rest shape
    step(simulation_app, 30)
    at_rest = {s: np.abs(m).max() for s, m in sensor.read().items()}
    assert all(v < MIN_MAP / 4 for v in at_rest.values()), f"not zero at rest: {at_rest}"

    dof = scene.dof("finger_joint")
    move_to(simulation_app, scene, dof, 0.8, settle_frames=120)
    images, maps = sensor.compression_images(), sensor.read()
    peak = {}
    for side in ("left", "right"):
        img, m = images[side], maps[side]
        assert m.shape == (7, 4) and np.isfinite(m).all(), side
        assert img.max() * 1000 >= MIN_COMPRESSION_MM, f"{side}: compression {img.max() * 1000:.4f} mm"
        assert m.max() >= MIN_MAP, f"{side}: map {m.max():.1f}"
        pressed = img > 0.5 * img.max()  # the half-maximum patch: nodes beside the box are pulled along a little
        inside = pressed[:, list(BOX_COLUMNS)].sum() / pressed.sum()
        assert inside >= 0.9, f"{side}: only {inside:.0%} of the pressed nodes are under the box columns"
        peak[side] = m.max()

    move_to(simulation_app, scene, dof, 0.0, settle_frames=180)
    for side, m in sensor.read().items():
        assert np.abs(m).max() <= RELEASED * peak[side], f"{side}: {np.abs(m).max():.1f} after release (peak {peak[side]:.1f})"


def test_precise_refuses_newton(simulation_app):
    _scene, _sensor, stage = _load(simulation_app, "Physx_parallel_grip")
    with pytest.raises(ValueError, match="PhysX"):
        make_tactile_sensor(stage, f"/World/Gripper/{GRIPPER}", "newton")
