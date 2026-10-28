"""Mounts of the TSF on the Robotiq grippers that carry one: where the sensing area sits in the pad body frame.

The pad bodies are ``left_fingertip`` / ``right_fingertip``; their frames are the ones of the ``<side>_inner_finger`` bodies they are
fixed to (z along the finger, the jaws close along y). The numbers are the ``MOUNTS`` of grippers/tactile/generate_tactile_zones.py,
which places the taxel markers in each gripper's USD (tests/unit/test_tactile_taxels.py checks they agree):

- ``ROBOTIQ_2F_85``: ``Fingertip=Tactile_TSF85``. Sensing area centred at x = 0, z = 132.4 mm; the marker centres at |y| = 41.6 mm.
- ``ROBOTIQ_2F_140``: ``Fingertip=Tactile_TSF140``, the same sensor further out on the 2F-140's finger: centred at x = -0.17 mm,
  z = 187.5 mm, marker centres at |y| = 70.1 mm (the collider face is at 68.1 mm, the markers 2 mm behind it).
"""

from .tsf import FingertipMounts

ROBOTIQ_2F_85 = FingertipMounts.from_sensing_area(center_x=0.0, center_z=0.1324, pad_y=0.0416)
ROBOTIQ_2F_140 = FingertipMounts.from_sensing_area(center_x=-0.00017, center_z=0.18751, pad_y=0.06807 + 0.002)
