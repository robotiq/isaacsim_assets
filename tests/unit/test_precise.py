"""SensorSim=Precise: the pad lattice, the rigid-motion-free compression and the lab's CNN. numpy and onnxruntime, no Isaac."""

import numpy as np
import pytest

from tactile import precise

PITCH_X, PITCH_Z = 0.0023682, 0.0023682  # m, the lattice of the pad
THICKNESS = 0.00308


def pad_nodes(seed=0):
    """The 12 (x) x 18 (z) x 2 (y) lattice of the pad in its file frame, in a shuffled order (the file's order is
    not spatial)."""
    x0, z0, y0 = 0.0012, 0.0021, -0.02434
    pts = np.array([(x0 + i * PITCH_X, y0 + j * THICKNESS, z0 + k * PITCH_Z)
                    for i in range(12) for j in range(2) for k in range(18)])
    return pts[np.random.default_rng(seed).permutation(len(pts))]


def test_grid_covers_the_outer_layer_once():
    pts = pad_nodes()
    grid = precise.outer_layer_grid(pts)
    assert grid.shape == (18, 12) and len(np.unique(grid)) == 216
    assert np.allclose(pts[grid][..., 1], pts[:, 1].min())


@pytest.mark.parametrize("along_x", [True, False])
def test_grid_rows_run_from_the_fingertip_end_and_columns_along_the_finger_x(along_x):
    pts = pad_nodes()
    grid = precise.outer_layer_grid(pts, columns_along_pad_x=along_x)
    z, x = pts[grid][..., 2], pts[grid][..., 0]
    assert (np.diff(z, axis=0) < 0).all() and np.allclose(z, z[:, :1])  # row 0 has the highest z
    assert ((np.diff(x, axis=1) > 0) if along_x else (np.diff(x, axis=1) < 0)).all()


def test_grid_rejects_another_lattice():
    with pytest.raises(ValueError):
        precise.outer_layer_grid(pad_nodes()[:100])


def test_quat_to_matrix():
    assert np.allclose(precise.quat_to_matrix((1, 0, 0, 0)), np.eye(3))
    r = precise.quat_to_matrix((np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)))  # 90 deg about z
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0])


def rigid(points, quat, t):
    return points @ precise.quat_to_matrix(quat).T + t  # p_world = R p + t


def test_compression_ignores_the_motion_of_the_backing():
    rest = pad_nodes()
    quat, t = (0.9, 0.1, -0.3, 0.2), np.array([0.01, -0.02, 0.03])
    r = precise.quat_to_matrix(quat)
    moved = precise.body_frame(rigid(rest, quat, t), r, t)
    assert np.allclose(precise.compression(moved, rest), 0.0, atol=1e-12)


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_compression_is_positive_when_pressed_in(sign):
    rest = pad_nodes()
    pressed = rest.copy()
    pressed[:, 1] += sign * 0.0004  # the outer layer moves toward the backing, which is on the sign side
    assert np.allclose(precise.compression(pressed, rest, sign), 0.0004)
    assert np.allclose(precise.compression(rest, pressed, sign), -0.0004)


@pytest.fixture(scope="module")
def cnn():
    pytest.importorskip("onnxruntime")
    return precise.TactileCnn()


def bump(r0, c0, amp):
    r, c = np.meshgrid(np.arange(18), np.arange(12), indexing="ij")
    return amp * np.exp(-(((r - r0) / 2.5) ** 2 + ((c - c0) / 2.0) ** 2))


def test_cnn_map_is_zero_at_rest(cnn):
    assert cnn.read(np.zeros((18, 12))).shape == (7, 4)
    assert np.abs(cnn.read(np.zeros((18, 12)))).max() == 0.0


def centroid(m):
    p = np.clip(m, 0, None)
    return (p * np.arange(7)[:, None]).sum() / p.sum(), (p * np.arange(4)[None, :]).sum() / p.sum()


def test_cnn_map_follows_the_contact(cnn):
    """The centroid of the map moves with a compression bump over the image, down the rows and across the columns.
    (The map is smeared toward its centre: its centroid spans rows 2.4 -> 3.5 and columns 0.7 -> 2.4 for a bump
    moved over the whole pad, and its argmax is not stable, so the centroid is what is checked.)"""
    cents = [centroid(cnn.read(bump(r0, c0, 0.3 * cnn.the_max))) for r0, c0 in ((3, 2), (9, 5), (14, 9))]
    rows, cols = zip(*cents)
    assert rows[0] < rows[1] < rows[2] and cols[0] < cols[1] < cols[2], cents
    assert cols[2] - cols[0] >= 1.0 and rows[2] - rows[0] >= 0.8, cents


def test_cnn_map_grows_with_the_compression(cnn):
    small, large = (cnn.read(bump(9, 5, a * cnn.the_max)).max() for a in (0.1, 0.5))
    assert 0 < small < large
