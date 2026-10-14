"""Locate gripper asset folders under grippers/.

A gripper folder holds a root USD named after it (``Robotiq_2F_85/Robotiq_2F_85.usda``).
It sits directly under grippers/, or one level down in a family folder that has no
root USD of its own (``Robotiq_2F/`` groups the 2F-85 and 2F-140).
"""

from __future__ import annotations

from pathlib import Path

GRIPPERS_DIR = Path(__file__).resolve().parents[2] / "grippers"


def _is_gripper(d: Path) -> bool:
    return d.is_dir() and (d / f"{d.name}.usda").is_file()


def gripper_dirs() -> list[Path]:
    """Every gripper folder, including those inside a family folder."""
    found = []
    for d in sorted(GRIPPERS_DIR.iterdir()):
        if _is_gripper(d):
            found.append(d)
        elif d.is_dir():
            found.extend(c for c in sorted(d.iterdir()) if _is_gripper(c))
    return found


def gripper_dir(gripper: str) -> Path:
    """The folder of the gripper named ``gripper`` (e.g. ``Robotiq_2F_85``)."""
    for d in gripper_dirs():
        if d.name == gripper:
            return d
    raise FileNotFoundError(f"no gripper folder named {gripper!r} under {GRIPPERS_DIR}")
