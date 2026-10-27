"""Unit tests of the TSF taxel layout and the force spreading (grippers/tactile_sensors): numpy / torch, no Isaac."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from robotiq_tactile_sensors.mounts import ROBOTIQ_2F_85, ROBOTIQ_2F_140  # noqa: E402
from robotiq_tactile_sensors.spread import spread_forces  # noqa: E402
from robotiq_tactile_sensors.tsf import TSF85, FingertipMounts, PadMount, taxel_positions  # noqa: E402
from tactile.generate_tactile_zones import MOUNTS, zone_offsets  # noqa: E402


@pytest.mark.parametrize("gripper, mounts", [("Robotiq_2F_85", ROBOTIQ_2F_85), ("Robotiq_2F_140", ROBOTIQ_2F_140)])
def test_taxels_match_the_usd_markers(gripper, mounts):
    """The taxel centres are the markers the USD generator (grippers/tactile/generate_tactile_zones.py) places in each
    ``<side>_inner_finger`` frame, for both grippers: row-major, row 0 at the fingertip end."""
    mount = MOUNTS[gripper]
    for pad, sign in ((mounts.left, -1), (mounts.right, +1)):
        expected, _, _ = zone_offsets(mount, sign)
        np.testing.assert_allclose(taxel_positions(pad, mounts.spec), np.asarray(expected), atol=1e-8)


def test_the_pads_face_each_other():
    for mounts in (ROBOTIQ_2F_85, ROBOTIQ_2F_140):
        left, right = (taxel_positions(p, mounts.spec) for p in mounts.pads)
        np.testing.assert_allclose(right[:, [0, 2]], left[:, [0, 2]])  # only y differs
        np.testing.assert_allclose(right[:, 1], -left[:, 1])
        assert (left[:, 1] < 0).all()  # the left pad is at -y


def test_row_0_is_the_fingertip_end():
    for mounts in (ROBOTIQ_2F_85, ROBOTIQ_2F_140):
        pos = taxel_positions(mounts.left, mounts.spec).reshape(mounts.spec.rows, mounts.spec.cols, 3)
        assert (np.diff(pos[:, 0, 2]) < 0).all()  # rows go down the finger, away from the tip
        assert (np.diff(pos[0, :, 0]) > 0).all()  # columns go along +x


def test_the_spec_pitches_follow_from_the_sensing_area():
    assert TSF85.taxels == 28
    assert np.isclose(TSF85.pitch_row, (0.038 - 6 * 0.0004) / 7 + 0.0004)
    assert np.isclose(TSF85.pitch_col, (0.0235 - 3 * 0.0004) / 4 + 0.0004)
    assert np.isclose(TSF85.rows * TSF85.pitch_row - TSF85.gap, TSF85.height)  # the cells fill the sensing area
    assert np.isclose(TSF85.cols * TSF85.pitch_col - TSF85.gap, TSF85.width)


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
