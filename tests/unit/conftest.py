import sys
from pathlib import Path

# the TSF-85 tactile package (grippers/tactile/), imported as ``tactile`` (numpy / scipy parts only: no Isaac needed)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "grippers"))
