"""Test-side helpers for the TSF-85 tactile sensor: the SensorSim variant selections and the
live heatmap of --gui runs. The readers themselves are grippers/tactile/."""

from __future__ import annotations

from tactile import COLS, ROWS, SIDES
from tactile.plot import TactilePlot

RAYCAST_VARIANTS = [("Fingertip", "Tactile_TSF85"), ("SensorSim", "Raycast")]
EFFECTIVE_VARIANTS = [("Fingertip", "Tactile_TSF85"), ("SensorSim", "EffectiveResponse")]

__all__ = ["COLS", "ROWS", "SIDES", "TactilePlot", "RAYCAST_VARIANTS", "EFFECTIVE_VARIANTS"]
