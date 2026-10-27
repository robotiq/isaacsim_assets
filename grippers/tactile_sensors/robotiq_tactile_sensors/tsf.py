"""The taxel array of a TSF tactile sensor and where it sits on a gripper's fingertips. numpy only.

A TSF is the same part whichever gripper carries it, so the two are described separately:

- ``TsfSpec``: the sensor itself, a ``rows x cols`` grid of taxels with a pitch (row 0 at the fingertip end).
- ``PadMount``: where that grid sits on ONE pad body of ONE gripper, in the frame of that body: the centre of
  taxel (row 0, col 0) and the directions in which the rows and the columns grow. A gripper has two (left, right),
  grouped in ``FingertipMounts``.

To use the sensor on another gripper, write its ``FingertipMounts``: measure the taxel (0, 0) centre and the two axes in
the frame of the pad's rigid body (the taxel markers of the gripper's USD give them), see ``mounts.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class TsfSpec:
    rows: int
    cols: int
    pitch_row: float  # m, between taxel rows
    pitch_col: float  # m, between taxel columns

    @property
    def taxels(self) -> int:
        return self.rows * self.cols


TSF85 = TsfSpec(rows=7, cols=4, pitch_row=0.0055, pitch_col=0.006)
"""TSF-85: 7 x 4 taxels per finger. Pitches measured from the taxel markers of Robotiq_2F_85's USD."""


@dataclass
class PadMount:
    body: str  # name of the pad's rigid body, e.g. "left_fingertip"
    origin: tuple[float, float, float]  # m, centre of taxel (row 0, col 0), in the body frame
    row_axis: tuple[float, float, float]  # unit vector along which the rows grow (away from the fingertip end)
    col_axis: tuple[float, float, float]  # unit vector along which the columns grow


@dataclass
class FingertipMounts:
    left: PadMount
    right: PadMount
    spec: TsfSpec = field(default_factory=lambda: TSF85)

    @property
    def pads(self) -> tuple[PadMount, PadMount]:
        return (self.left, self.right)


def taxel_positions(pad: PadMount, spec: TsfSpec) -> np.ndarray:
    """Taxel centres in the pad body frame, ``(rows * cols, 3)``, row-major: taxel (r, c) is index ``r * cols + c``."""
    r, c = np.meshgrid(np.arange(spec.rows), np.arange(spec.cols), indexing="ij")
    origin, row_axis, col_axis = (np.asarray(v, dtype=np.float64) for v in (pad.origin, pad.row_axis, pad.col_axis))
    pos = origin + r[..., None] * spec.pitch_row * row_axis + c[..., None] * spec.pitch_col * col_axis
    return pos.reshape(-1, 3)
