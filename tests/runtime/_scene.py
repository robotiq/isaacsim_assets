"""Shared stage setup for the runtime tests: one gripper, one Physics variant,
the matching engine, playing and ready to be driven."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

REPO = Path(__file__).resolve().parents[2]
SCENE_LAYER = REPO / "grippers" / "scene" / "Robotiq_Newton_scene_physics.usda"

def engine_for(physics: str) -> str:
    """A Physics variant starting with "Newton" runs on Newton, anything else on PhysX."""
    return "newton" if physics.startswith("Newton") else "physx"


@dataclass
class GripperScene:
    gripper: str
    physics: str
    robot: Articulation
    root: str  # prim path of the gripper's articulation root

    def dof(self, name: str) -> int:
        return self.robot.dof_names.index(name)


def load_gripper(simulation_app, gripper: str, physics: str, view=None, setup=None,
                 variants: list[tuple[str, str]] | None = None) -> GripperScene:
    """Fresh stage with the gripper at /World/Gripper on the engine matching
    ``physics``, timeline playing. Call ``view``
    (the conftest fixture) once the stage is populated, if given. ``setup(stage)``
    runs after the engine switch and before play, to add scene content (e.g. an
    object to grasp) that the engine must see when it builds its model.
    ``variants`` selects more variant sets after Physics, in order, one at a time,
    so a nested set (e.g. SensorSim, which only exists under
    Fingertip=Tactile_TSF85) can follow the set that introduces it."""
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    stage = stage_utils.create_new_stage()
    # The scene-level Newton settings the grippers need (the elliptic cone and impratio,
    # see grippers/scene).
    assert SCENE_LAYER.is_file(), SCENE_LAYER
    stage.GetRootLayer().subLayerPaths.append(str(SCENE_LAYER))
    assert stage.GetPrimAtPath("/PhysicsScene"), f"{SCENE_LAYER} did not compose a /PhysicsScene"
    stage_utils.add_reference_to_stage(
        usd_path=str(REPO / "grippers" / gripper / f"{gripper}.usda"),
        path="/World/Gripper",
        variants=[("Physics", physics)],
    )
    prim = stage.GetPrimAtPath("/World/Gripper")
    for variant_set, selection in variants or []:
        names = prim.GetVariantSets().GetNames()
        assert variant_set in names, f"no variant set {variant_set!r} on {gripper} (has {names})"
        options = prim.GetVariantSet(variant_set).GetVariantNames()
        assert selection in options, f"no {variant_set}={selection!r} on {gripper} (has {options})"
        prim.GetVariantSet(variant_set).SetVariantSelection(selection)
    if view:
        view()
    simulation_app.update()

    # Select the engine once the stage is populated (the switch resets the
    # physics scene), as Isaac's own Newton tests do.
    engine = engine_for(physics)
    assert SimulationManager.switch_physics_engine(engine), f"could not switch to {engine}"
    simulation_app.update()
    if setup:
        setup(stage_utils.get_current_stage())
        simulation_app.update()

    # The articulation root sits on the inner <gripper> prim, not the asset root.
    root = f"/World/Gripper/{gripper}"
    robot = Articulation(root)
    timeline.play()
    simulation_app.update()
    return GripperScene(gripper, physics, robot, root)


def step(simulation_app, frames: int, every: int = 0, on_sample=None) -> None:
    """Advance ``frames`` frames; call ``on_sample()`` every ``every`` frames and at the end."""
    for i in range(1, frames + 1):
        simulation_app.update()
        if on_sample and every and (i % every == 0 or i == frames):
            on_sample()


FRAME_DT = 1.0 / 60.0  # simulation_app.update() advances one 60 Hz frame

# Datasheet maximum finger speed, 150 mm/s, as a rate on the driven DOF:
# 2F: 85 mm <-> 49 deg at finger_joint (guide 4.5) -> 86 deg/s; Hand-E: the
# prismatic finger joint moves at the finger speed itself.
FINGER_RATE = {
    "Robotiq_2F_85": math.radians(86.0),
    "Robotiq_2F_140": math.radians(86.0),
    "Robotiq_Hand_E": 0.150,
}


def move_to(simulation_app, scene: GripperScene, dof: int, target: float, *, rate: float | None = None,
            settle_frames: int = 60, every: int = 0, on_sample=None) -> int:
    """Ramp the position target of ``dof`` linearly from where it is to
    ``target`` at ``rate`` (DOF units per second, default: the gripper's
    datasheet maximum finger speed), then hold it for ``settle_frames``.
    ``on_sample()`` is called every ``every`` frames throughout. Returns the
    number of ramp frames."""
    rate = FINGER_RATE[scene.gripper] if rate is None else rate
    start = float(scene.robot.get_dof_positions().numpy()[0][dof])
    frames = max(1, math.ceil(abs(target - start) / (rate * FRAME_DT)))
    for i in range(1, frames + 1):
        scene.robot.set_dof_position_targets([start + (target - start) * i / frames], dof_indices=[dof])
        simulation_app.update()
        if on_sample and every and i % every == 0:
            on_sample()
    step(simulation_app, settle_frames, every, on_sample)
    return frames
