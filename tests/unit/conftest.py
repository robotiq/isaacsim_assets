import sys
from pathlib import Path

# the TSF-85 tactile package (grippers/tactile/), imported as ``tactile`` (numpy / scipy parts only: no Isaac needed)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "grippers"))

# the simulated TSF sensors for Isaac Lab (taxel layout, mounts, force spreading), imported as ``robotiq_tactile_sensors``
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "grippers" / "tactile_sensors"))
