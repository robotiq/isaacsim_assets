#!/usr/bin/env python3
"""Turn the frames of a ``--record DIR`` run into videos.

    ~/isaacsim/python.sh -m pytest tests/runtime --gui --record /tmp/frames
    ~/isaacsim/python.sh tests/runtime/make_videos.py /tmp/frames /tmp/videos

Writes, as VP9 .webm (30 fps):
- ``cases/<test id>.webm``: one video per test case;
- ``grids/<test>-<gripper>.webm``: the Physics variants of one test and gripper side by
  side (2x2 for the 2F: PhysX / Newton x parallel grip / compliant; 1x2 for the Hand-E),
  shorter clips holding their last frame;
- ``all_tests.webm``: every grid one after the other, each with a title card.

Needs OpenCV (bundled with Isaac Sim's Python).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

FPS = 30
TILE = (640, 360)  # w, h
TITLE_SECONDS = 1.5
VARIANT_ORDER = ["Physx_parallel_grip", "Newton_parallel_grip", "Physx_compliant", "Newton_compliant", "PhysX", "Newton"]


def _writer(path: Path, size: tuple[int, int]) -> cv2.VideoWriter:
    path.parent.mkdir(parents=True, exist_ok=True)
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"VP90"), FPS, size)
    if not w.isOpened():
        sys.exit(f"cannot write {path}")
    return w


def _frames(folder: Path) -> list[np.ndarray]:
    out = []
    for f in sorted(folder.glob("*.png")):
        img = cv2.imread(str(f))
        if img is not None:
            out.append(cv2.resize(img, TILE, interpolation=cv2.INTER_AREA))
    return out


def _label(img: np.ndarray, text: str, scale: float = 0.7) -> np.ndarray:
    img = img.copy()
    cv2.rectangle(img, (0, 0), (img.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def _split(name: str) -> tuple[str, str, str]:
    """'test_gripper_closes-Robotiq_2F_85-Physx_parallel_grip' -> (test, gripper, variant)."""
    test, rest = name.split("-", 1)
    gripper, variant = rest.rsplit("-", 1)
    return test, gripper, variant


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", type=Path, help="the --record DIR")
    ap.add_argument("out", type=Path, help="output folder")
    args = ap.parse_args()

    groups: dict[tuple[str, str], dict[str, list[np.ndarray]]] = {}
    for folder in sorted(p for p in args.frames.iterdir() if p.is_dir()):
        frames = _frames(folder)
        if not frames:
            continue
        test, gripper, variant = _split(folder.name)
        w = _writer(args.out / "cases" / f"{folder.name}.webm", TILE)
        for f in frames:
            w.write(_label(f, f"{test}  {gripper}  {variant}", 0.5))
        w.release()
        # The fixed-cube test's compliant cases are a separate function: grid them with the others.
        groups.setdefault((test.replace("test_compliant_gripper", "test_gripper"), gripper), {})[variant] = frames
        print(f"case {folder.name}: {len(frames)} frames")

    everything = _writer(args.out / "all_tests.webm", (2 * TILE[0], 2 * TILE[1]))
    for (test, gripper), variants in groups.items():
        names = sorted(variants, key=lambda v: VARIANT_ORDER.index(v) if v in VARIANT_ORDER else 99)
        cols, rows = 2, (2 if len(names) > 2 else 1)
        n = max(len(v) for v in variants.values())
        blank = np.zeros((TILE[1], TILE[0], 3), np.uint8)
        size = (cols * TILE[0], rows * TILE[1])
        grid = _writer(args.out / "grids" / f"{test}-{gripper}.webm", size)
        title = np.zeros((2 * TILE[1], 2 * TILE[0], 3), np.uint8)
        for line, text in enumerate((test, gripper)):
            cv2.putText(title, text, (60, 300 + 80 * line), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 3, cv2.LINE_AA)
        for _ in range(int(TITLE_SECONDS * FPS)):
            everything.write(title)
        for i in range(n):
            tiles = [_label(variants[v][min(i, len(variants[v]) - 1)], v) for v in names]
            tiles += [blank] * (cols * rows - len(tiles))
            frame = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])
            grid.write(frame)
            if rows == 2:
                everything.write(frame)
            else:  # 1x2: centre it in the 2x2 canvas
                canvas = np.zeros((2 * TILE[1], 2 * TILE[0], 3), np.uint8)
                canvas[TILE[1] // 2:TILE[1] // 2 + TILE[1]] = frame
                everything.write(canvas)
        grid.release()
        print(f"grid {test}-{gripper}: {names}, {n} frames")
    everything.release()
    print(f"videos in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
