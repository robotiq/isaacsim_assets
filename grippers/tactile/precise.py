# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Robotiq.
"""The maths of ``SensorSim=Precise``, numpy (and onnxruntime for the network) only: no Isaac needed.

The variant puts the deformable pad of the TSF-85 of the Control and Robotics Laboratory (CoRo) of the Ecole de
technologie superieure (github.com/Lab-CORO/TSF-85, used under agreement with Robotiq) on each fingertip. The
pad is a 12 (x) x 18 (z) x 2 (y) lattice of 432 nodes, 3 mm thick; its outer layer (y = -24.34 mm in the pad's file
frame) touches the object. The network of that lab (``data/tactile_cnn.onnx``) turns the compression of the 216
outer-layer nodes, laid out as an 18 x 12 image, into a 7 x 4 tactile map:

    compression = how far each node of the outer layer moved toward the backing, along the pad normal,
                  in the backing's frame (m), minus the same at rest
    image       = compression / the_max, rows from the fingertip end, columns along the finger's +X
    map         = network(image) - network(0)

``the_max`` (1.616 mm) is the lab's normalisation. The units of the map are the network's own: it is not
calibrated and is not in newtons (see README.md). De la Cruz Sanchez & Roberge, "A hybrid elastic-hyperelastic
approach for simulating soft tactile sensors", Front. Robot. AI 12:1639524 (2025), doi:10.3389/frobt.2025.1639524.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parent / "data"
IMAGE_ROWS, IMAGE_COLS = 18, 12  # network input: the outer layer's nodes, z by x
MAP_ROWS, MAP_COLS = 7, 4  # network output


def outer_layer_grid(rest_points: np.ndarray, columns_along_pad_x: bool = True) -> np.ndarray:
    """Node index of each cell of the 18 x 12 image: ``(IMAGE_ROWS, IMAGE_COLS)``.

    ``rest_points``: ``(N, 3)`` nodes at rest in the pad's file frame (x across, y through the 3 mm layer, z along).
    Rows run from the highest z (the fingertip end) down; columns along +x, or along -x when
    ``columns_along_pad_x`` is False (the pad turned 180 deg about z, as the left one, so that the columns still
    run along the finger's +X).
    """
    pts = np.asarray(rest_points, dtype=np.float64)
    # the outer layer is the lowest-y plane of the lattice
    outer = np.where(np.isclose(pts[:, 1], pts[:, 1].min(), atol=1e-5))[0]
    xs = np.unique(np.round(pts[outer, 0], 5))
    zs = np.unique(np.round(pts[outer, 2], 5))[::-1]
    if len(xs) != IMAGE_COLS or len(zs) != IMAGE_ROWS or len(outer) != IMAGE_ROWS * IMAGE_COLS:
        raise ValueError(f"the pad's outer layer is {len(xs)} x {len(zs)} nodes ({len(outer)}), "
                         f"expected {IMAGE_COLS} x {IMAGE_ROWS}")
    if not columns_along_pad_x:
        xs = xs[::-1]
    grid = np.zeros((IMAGE_ROWS, IMAGE_COLS), dtype=int)
    for i in outer:
        grid[np.searchsorted(-zs, -round(pts[i, 2], 5)), np.where(xs == round(pts[i, 0], 5))[0][0]] = i
    return grid


def quat_to_matrix(q) -> np.ndarray:
    """3 x 3 rotation matrix (column convention: ``p_world = R @ p_body + t``) of a ``(w, x, y, z)`` quaternion."""
    w, x, y, z = (float(v) for v in q)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def body_frame(points_world: np.ndarray, rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    """``(N, 3)`` world points in the frame of a rigid body (``rotation``: its 3 x 3 matrix, column convention)."""
    return (np.asarray(points_world) - np.asarray(position)) @ np.asarray(rotation)


def compression(points_body: np.ndarray, rest_body: np.ndarray, sign: float = 1.0) -> np.ndarray:
    """Per node, the move along the pad normal (the body's y) since rest, positive when the pad is compressed.

    ``sign``: +1 for the pad whose outer face is on the low-y side of its backing (the right pad), -1 for the
    mirror one.
    """
    return sign * (np.asarray(points_body)[:, 1] - np.asarray(rest_body)[:, 1])


class TactileCnn:
    """The lab's network: ``read(compression)`` gives the 7 x 4 map. Needs ``onnxruntime``."""

    def __init__(self, model: Path | str = DATA / "tactile_cnn.onnx", config: Path | str = DATA / "tactile_cnn.json"):
        try:
            import onnxruntime as ort
        except ImportError as e:  # not part of Isaac Sim's Python
            raise ImportError("SensorSim=Precise needs onnxruntime: `<isaac>/python.sh -m pip install onnxruntime`") from e
        self.the_max = float(json.loads(Path(config).read_text())["the_max"])
        self.session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.baseline = self._run(np.zeros((IMAGE_ROWS, IMAGE_COLS)))

    def _run(self, image: np.ndarray) -> np.ndarray:
        x = np.asarray(image, dtype=np.float32).reshape(1, IMAGE_ROWS, IMAGE_COLS, 1)
        return self.session.run(None, {self.input_name: x})[0].reshape(MAP_ROWS, MAP_COLS)

    def read(self, compression_image: np.ndarray) -> np.ndarray:
        """``compression_image``: ``(IMAGE_ROWS, IMAGE_COLS)`` compression in m. The map is 0 at rest."""
        return self._run(np.asarray(compression_image) / self.the_max) - self.baseline
