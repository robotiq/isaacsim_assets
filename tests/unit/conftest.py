import sys
from pathlib import Path

# the tactile package of the 2F-85, imported as ``tactile`` (numpy / scipy parts only: no Isaac needed)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "grippers" / "Robotiq_2F_85"))
