"""Mounts of the TSF on the Robotiq grippers that carry one.

``ROBOTIQ_2F_85``: ``Fingertip=Tactile_TSF85`` of grippers/Robotiq_2F_85. The pad bodies are ``left_fingertip`` /
``right_fingertip``. Their frames sit at the gripper base frame when the gripper is open (z along the fingers, the jaws
close along y), and the taxel grid is on the pad surface at y = -/+ 41.6 mm: columns along x (-9 .. +9 mm), rows along -z
from the fingertip end (z = 148.9 mm). Measured from the taxel markers of ``Robotiq_2F_85_tactile_zones.usda``.
"""

from .tsf import TSF85, FingertipMounts, PadMount

ROBOTIQ_2F_85 = FingertipMounts(
    left=PadMount("left_fingertip", origin=(-0.009, -0.0416, 0.1489), row_axis=(0.0, 0.0, -1.0), col_axis=(1.0, 0.0, 0.0)),
    right=PadMount("right_fingertip", origin=(-0.009, 0.0416, 0.1489), row_axis=(0.0, 0.0, -1.0), col_axis=(1.0, 0.0, 0.0)),
    spec=TSF85,
)
