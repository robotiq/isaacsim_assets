"""Test-side helpers for the TSF-85 tactile sensor: the grippers that carry it, their SensorSim variant
selections and the live heatmap of --gui runs. The readers themselves are grippers/tactile/."""

from __future__ import annotations

from tactile import COLS, ROWS, SIDES
from tactile.plot import TactilePlot

# Each 2F gripper's TSF fingertip variant and its pad collider under <side>_fingertip/visuals/.
FINGERTIPS = {
    "Robotiq_2F_85": ("Tactile_TSF85", "tip_tsf85/TSF_85_case"),
    "Robotiq_2F_140": ("Tactile_TSF140", "tip_tsf140/mesh_0"),
}
GRIPPERS = list(FINGERTIPS)


def sensor_variants(gripper: str, sensor_sim: str) -> list[tuple[str, str]]:
    """The Fingertip / SensorSim selections for ``gripper``'s sensor (``sensor_sim``: Raycast | EffectiveResponse)."""
    return [("Fingertip", FINGERTIPS[gripper][0]), ("SensorSim", sensor_sim)]


__all__ = ["COLS", "ROWS", "SIDES", "TactilePlot", "FINGERTIPS", "GRIPPERS", "sensor_variants"]
