"""Unit tests of the TSF taxel layout and the force spreading (grippers/tactile_sensors): numpy / torch, no Isaac."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from robotiq_tactile_sensors.mounts import ROBOTIQ_2F_85  # noqa: E402
from robotiq_tactile_sensors.spread import spread_forces  # noqa: E402
from robotiq_tactile_sensors.tsf import TSF85, FingertipMounts, PadMount, taxel_positions  # noqa: E402


def test_2f85_layout_matches_the_marker_grid():
    """The corners of the grid measured from Robotiq_2F_85_tactile_zones.usda, in the pad body frame."""
    left = taxel_positions(ROBOTIQ_2F_85.left, TSF85)
    assert left.shape == (28, 3)
    np.testing.assert_allclose(left[0], [-0.009, -0.0416, 0.1489], atol=1e-9)  # row 0, col 0: at the fingertip end
    np.testing.assert_allclose(left[3], [0.009, -0.0416, 0.1489], atol=1e-9)  # row 0, col 3
    np.testing.assert_allclose(left[-1], [0.009, -0.0416, 0.1159], atol=1e-9)  # row 6, col 3
    right = taxel_positions(ROBOTIQ_2F_85.right, TSF85)
    np.testing.assert_allclose(right[:, [0, 2]], left[:, [0, 2]])  # the pads face each other: only y differs
    np.testing.assert_allclose(right[:, 1], -left[:, 1])


def test_layout_is_row_major_with_the_spec_pitches():
    pos = taxel_positions(ROBOTIQ_2F_85.left, TSF85).reshape(TSF85.rows, TSF85.cols, 3)
    assert np.isclose(np.linalg.norm(pos[1, 0] - pos[0, 0]), TSF85.pitch_row)
    assert np.isclose(np.linalg.norm(pos[0, 1] - pos[0, 0]), TSF85.pitch_col)


def test_a_mount_on_another_gripper_is_just_its_own_numbers():
    mounts = FingertipMounts(
        left=PadMount("tip_l", origin=(0.0, 0.0, 0.0), row_axis=(0, 1, 0), col_axis=(1, 0, 0)),
        right=PadMount("tip_r", origin=(0.0, 0.1, 0.0), row_axis=(0, 1, 0), col_axis=(1, 0, 0)),
    )
    pos = taxel_positions(mounts.right, mounts.spec)
    np.testing.assert_allclose(pos[0], [0.0, 0.1, 0.0])
    np.testing.assert_allclose(pos[mounts.spec.cols], [0.0, 0.1 + mounts.spec.pitch_row, 0.0])  # next row


def _grid(rows=7, cols=4):
    return torch.tensor(taxel_positions(ROBOTIQ_2F_85.left, TSF85), dtype=torch.float32).reshape(1, 1, rows * cols, 3)


def test_the_taxels_sum_to_the_force():
    taxel = _grid()
    contact = taxel[:, :, 10:11] + torch.tensor([0.0, 0.003, 0.001])  # near taxel 10
    out = spread_forces(taxel, contact, torch.tensor([[[7.5]]]))
    assert out.shape == (1, 1, 28)
    assert torch.isclose(out.sum(), torch.tensor(7.5), atol=1e-4)


def test_the_force_peaks_at_the_taxel_under_the_contact():
    taxel = _grid()
    for target in (0, 10, 27):
        out = spread_forces(taxel, taxel[:, :, target : target + 1], torch.tensor([[[3.0]]]))
        assert int(out.argmax()) == target


def test_no_force_no_signal_and_objects_add_up():
    taxel = _grid()
    far = taxel[:, :, :1].expand(1, 1, 2, 3).contiguous()
    assert spread_forces(taxel, far, torch.zeros(1, 1, 2)).abs().max() == 0
    a = spread_forces(taxel, taxel[:, :, 3:4], torch.tensor([[[2.0]]]))
    b = spread_forces(taxel, taxel[:, :, 20:21], torch.tensor([[[5.0]]]))
    both = spread_forces(taxel, torch.cat([taxel[:, :, 3:4], taxel[:, :, 20:21]], dim=2), torch.tensor([[[2.0, 5.0]]]))
    torch.testing.assert_close(both, a + b)


def test_batches_are_independent():
    taxel = _grid().expand(3, 2, 28, 3)
    contact = taxel[:, :, :1] * 1.0
    force = torch.tensor([1.0, 2.0, 3.0]).view(3, 1, 1).expand(3, 2, 1)
    out = spread_forces(taxel, contact, force)
    torch.testing.assert_close(out.sum(-1), force[..., 0])
