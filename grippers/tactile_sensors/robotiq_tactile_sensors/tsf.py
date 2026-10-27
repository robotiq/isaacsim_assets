"""The taxel array of a TSF tactile sensor and where it sits on a gripper's fingertips. numpy only.

A TSF is the same part whichever gripper carries it, so the two are described separately:

- ``TsfSpec``: the sensor itself, a ``rows x cols`` grid of taxels filling a sensing area (row 0 at the fingertip end).
- ``PadMount``: where that grid sits on ONE pad body of ONE gripper, in the frame of that body: the centre of
  taxel (row 0, col 0) and the directions in which the rows and the columns grow. A gripper has two (left, right),
  grouped in ``FingertipMounts``.

The Robotiq grippers place the sensing area the same way (rows along -z from the fingertip end, columns along +x, the pad
surface at y = -/+ ``pad_y``), so a mount is three numbers: ``FingertipMounts.from_sensing_area``. They are the ``MOUNTS``
of grippers/tactile/generate_tactile_zones.py, which places the taxel markers in the USD: a unit test checks the two agree.
Another gripper can give the taxel (0, 0) centre and the two axes directly (``PadMount``), see ``mounts.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class TsfSpec:
    """The sensing area of a TSF and its taxel grid. Taxel cells are ``gap`` apart."""

    rows: int
    cols: int
    height: float  # m, span of the rows
    width: float  # m, span of the columns
    gap: float  # m, between taxel cells

    @property
    def taxels(self) -> int:
        return self.rows * self.cols

    @property
    def cell_row(self) -> float:
        """Size of one taxel cell along the rows [m]."""
        return (self.height - self.gap * (self.rows - 1)) / self.rows

    @property
    def cell_col(self) -> float:
        return (self.width - self.gap * (self.cols - 1)) / self.cols

    @property
    def pitch_row(self) -> float:
        """Distance between the centres of two taxels along the rows [m]."""
        return self.cell_row + self.gap

    @property
    def pitch_col(self) -> float:
        return self.cell_col + self.gap


TSF85 = TsfSpec(rows=7, cols=4, height=0.038, width=0.0235, gap=0.0004)
"""The TSF sensor: 7 x 4 taxels over a 38.0 x 23.5 mm sensing area (TSF-85; the 2F-140's TSF-140 tip carries the same sensor)."""


@dataclass
class PadMount:
    body: str  # name of the pad's rigid body, e.g. "left_fingertip"
    origin: tuple[float, float, float]  # m, centre of taxel (row 0, col 0) (of its marker), in the body frame
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

    @classmethod
    def from_sensing_area(
        cls,
        center_x: float,
        center_z: float,
        pad_y: float,
        spec: TsfSpec | None = None,
        left_body: str = "left_fingertip",
        right_body: str = "right_fingertip",
    ) -> "FingertipMounts":
        """The mount of the Robotiq grippers, from where the sensing area sits in the pad body frame.

        ``center_x``, ``center_z``: the centre of the sensing area; ``pad_y``: ``|y|`` of the taxel centres (the left
        pad at -y, the right one mirrored). The 7 rows grow along -z from the fingertip end, the 4 columns along +x."""
        spec = spec or TSF85
        x0 = center_x - spec.width / 2 + spec.cell_col / 2
        z0 = center_z + spec.height / 2 - spec.cell_row / 2
        down, along = (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)
        return cls(
            left=PadMount(left_body, (x0, -pad_y, z0), down, along),
            right=PadMount(right_body, (x0, pad_y, z0), down, along),
            spec=spec,
        )


def taxel_positions(pad: PadMount, spec: TsfSpec) -> np.ndarray:
    """Taxel centres in the pad body frame, ``(rows * cols, 3)``, row-major: taxel (r, c) is index ``r * cols + c``."""
    r, c = np.meshgrid(np.arange(spec.rows), np.arange(spec.cols), indexing="ij")
    origin, row_axis, col_axis = (np.asarray(v, dtype=np.float64) for v in (pad.origin, pad.row_axis, pad.col_axis))
    pos = origin + r[..., None] * spec.pitch_row * row_axis + c[..., None] * spec.pitch_col * col_axis
    return pos.reshape(-1, 3)
