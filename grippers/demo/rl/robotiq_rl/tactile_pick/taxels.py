"""Taxel layout of the TSF-85 pads, in the finger-body frames of the 2F-85.

Measured from the marker grid of Robotiq_2F_85/payloads/Robotiq_2F_85_tactile_zones.usda (Fingertip=Tactile_TSF85):
7 rows along z (row 0 at the fingertip end) x 4 columns along x, on the pad surface at y = -/+ 41.6 mm. The finger
bodies sit at the gripper base frame when the gripper is open and carry the grid when it closes.
"""

import numpy as np

ROWS, COLS = 7, 4
X_COLS = np.array([-0.009, -0.003, 0.003, 0.009])  # m, 6 mm pitch
Z_ROWS = 0.1489 - 0.0055 * np.arange(ROWS)  # m, 5.5 mm pitch, row 0 at the tip
PAD_Y = {"left": -0.0416, "right": 0.0416}  # m; the pad faces the other finger


def layout(side: str) -> np.ndarray:
    """Taxel centres in the finger-body frame: ``(ROWS * COLS, 3)``, row-major (row, col)."""
    xs, zs = np.meshgrid(X_COLS, Z_ROWS)  # (ROWS, COLS)
    return np.stack([xs.ravel(), np.full(xs.size, PAD_Y[side]), zs.ravel()], axis=-1)
