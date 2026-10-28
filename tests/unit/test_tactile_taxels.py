"""Unit tests of the TSF taxel layout and of the tactile sensor logic (grippers/tactile_sensors): numpy / torch, no Isaac."""

import math
import re
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from robotiq_tactile_sensors.mounts import ROBOTIQ_2F_85, ROBOTIQ_2F_140  # noqa: E402
from robotiq_tactile_sensors.spread import spread_forces  # noqa: E402
from robotiq_tactile_sensors.tsf import TSF85, FingertipMounts, PadMount, taxel_positions  # noqa: E402
from robotiq_tactile_sensors.sensing import (  # noqa: E402
    pad_object_forces,
    quat_apply,
    taxel_force_map,
    taxel_positions_world,
)


GRIPPERS = Path(__file__).resolve().parents[2] / "grippers"


def usd_markers(gripper: str, side: str) -> np.ndarray:
    """The taxel marker positions ``(28, 3)`` of a gripper's ``<gripper>_tactile_zones.usda``, in the ``<side>_inner_finger`` frame."""
    text = (GRIPPERS / gripper / "payloads" / f"{gripper}_tactile_zones.usda").read_text()
    start = text.index(f'over "{side}_inner_finger"')
    end = text.find('over "', start + 10)
    block = text[start : end if end > 0 else None]
    cubes = re.findall(r'def Cube "zone_r(\d+)_c(\d+)".*?xformOp:translate = \(([^)]*)\)', block, flags=re.S)
    out = np.full((28, 3), np.nan)
    for r, c, xyz in cubes:
        out[int(r) * 4 + int(c)] = [float(v) for v in xyz.split(",")]
    return out


@pytest.mark.parametrize("gripper, mounts", [("Robotiq_2F_85", ROBOTIQ_2F_85), ("Robotiq_2F_140", ROBOTIQ_2F_140)])
def test_taxels_are_the_markers_of_the_gripper_usd(gripper, mounts):
    """The taxel centres are the markers in the gripper's own ``*_tactile_zones.usda`` (what the readers use), so a USD that was
    not regenerated after a change of the mount, or a mount that drifted from the USD, fails here. Row-major, row 0 at the tip."""
    for pad, side in ((mounts.left, "left"), (mounts.right, "right")):
        markers = usd_markers(gripper, side)
        assert not np.isnan(markers).any(), f"{gripper} {side}: markers missing in the USD"
        np.testing.assert_allclose(taxel_positions(pad, mounts.spec), markers, atol=1e-8)


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
    assert np.isclose(TSF85.pitch_row, (0.038 - 6 * 0.0004) / 7 + 0.0004)  # 5.486 mm between rows
    assert np.isclose(TSF85.pitch_col, (0.0235 - 3 * 0.0004) / 4 + 0.0004)  # 5.975 mm between columns
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


# --- the force spreading ---------------------------------------------------------------------------------------------


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
    """A different grid, contact point and force in every env and pad: the batched result is the one of each slice alone,
    so a mix-up of the env / pad / object axes cannot pass (the maps are not linear in the contact position)."""
    g = torch.Generator().manual_seed(0)
    taxel = torch.rand(3, 2, 28, 3, generator=g) * 0.04
    contact = torch.rand(3, 2, 4, 3, generator=g) * 0.04
    force = torch.rand(3, 2, 4, generator=g) * 10
    batched = spread_forces(taxel, contact, force)
    for e in range(3):
        for p in range(2):
            alone = spread_forces(taxel[e : e + 1, p : p + 1], contact[e : e + 1, p : p + 1], force[e : e + 1, p : p + 1])
            torch.testing.assert_close(batched[e, p], alone[0, 0])
    assert (batched[0, 0] - batched[1, 0]).abs().max() > 1e-3  # and the slices do differ


def test_a_contact_off_the_array_is_ignored_with_a_max_distance():
    """The back of the finger, behind the sensing plane, is not on the array: with a max distance it reads nothing."""
    taxel = _grid()
    on = taxel[:, :, 10:11].clone()
    behind = on + torch.tensor([0.0, -0.021, 0.0])  # 21 mm behind the taxel plane (the case is that deep)
    beside = on + torch.tensor([0.0, 0.0, -0.05])  # 5 cm along the finger, off the array
    f = torch.tensor([[[5.0]]])
    assert spread_forces(taxel, behind, f).sum() > 4.99  # the old behaviour: reported at full force
    assert spread_forces(taxel, behind, f, max_distance=0.008).abs().max() == 0
    assert spread_forces(taxel, beside, f, max_distance=0.008).abs().max() == 0
    assert torch.isclose(spread_forces(taxel, on, f, max_distance=0.008).sum(), torch.tensor(5.0), atol=1e-4)


def test_a_contact_at_the_edge_of_the_array_still_reads():
    taxel = _grid()
    edge = taxel[:, :, 0:1] + torch.tensor([0.0, 0.002, 0.003])  # 3 mm past the first row, 2 mm in front of the markers
    out = spread_forces(taxel, edge, torch.tensor([[[5.0]]]), max_distance=0.008)
    assert out.sum() > 4.99


# --- the sensor logic (what the Isaac Lab sensor computes), without Isaac ---------------------------------------------


def _identity_quat(*shape):
    q = torch.zeros(*shape, 4)
    q[..., 3] = 1.0
    return q


def test_quat_apply_rotates_like_isaac_lab_quaternions_xyzw():
    z90 = torch.tensor([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])  # 90 degrees about z, (x, y, z, w)
    torch.testing.assert_close(quat_apply(z90, torch.tensor([1.0, 0.0, 0.0])), torch.tensor([0.0, 1.0, 0.0]), atol=1e-6, rtol=0)
    torch.testing.assert_close(quat_apply(z90, torch.tensor([0.0, 0.0, 2.0])), torch.tensor([0.0, 0.0, 2.0]), atol=1e-6, rtol=0)
    x180 = torch.tensor([1.0, 0.0, 0.0, 0.0])
    torch.testing.assert_close(quat_apply(x180, torch.tensor([0.0, 1.0, 3.0])), torch.tensor([0.0, -1.0, -3.0]), atol=1e-6, rtol=0)
    v = torch.randn(5, 3)
    torch.testing.assert_close(quat_apply(_identity_quat(5), v), v)


def test_the_pads_are_put_in_left_right_order():
    """The contact sensor lists its bodies in its own order: the pad order picks the left and the right one."""
    f = torch.zeros(1, 2, 3, 3)
    f[0, 0, :, 0] = torch.tensor([1.0, 2.0, 3.0])  # body 0 of the sensor
    f[0, 1, :, 0] = torch.tensor([10.0, 20.0, 30.0])  # body 1
    as_listed = pad_object_forces(f, torch.tensor([0, 1]), 0.0)
    swapped = pad_object_forces(f, torch.tensor([1, 0]), 0.0)  # the sensor lists (right, left)
    torch.testing.assert_close(as_listed[0, 0], torch.tensor([1.0, 2.0, 3.0]))
    torch.testing.assert_close(swapped[0, 0], torch.tensor([10.0, 20.0, 30.0]))
    torch.testing.assert_close(swapped[0, 1], torch.tensor([1.0, 2.0, 3.0]))


def test_forces_at_or_below_the_floor_read_zero():
    f = torch.zeros(1, 2, 3, 3)
    f[0, 0, :, 2] = torch.tensor([0.04, 0.05, 0.06])  # N, along z: the magnitude is what counts
    out = pad_object_forces(f, torch.tensor([0, 1]), 0.05)
    torch.testing.assert_close(out[0, 0], torch.tensor([0.0, 0.0, 0.06]))


def test_taxels_follow_the_pad_pose():
    local = torch.tensor(np.stack([taxel_positions(p, TSF85) for p in ROBOTIQ_2F_85.pads]), dtype=torch.float32)  # (2, 28, 3)
    pos = torch.tensor([[[1.0, 2.0, 3.0], [-1.0, 0.0, 0.5]]])  # (1, 2, 3): body 0 = right, body 1 = left
    quat = _identity_quat(1, 2)
    out = taxel_positions_world(pos, quat, local, torch.tensor([1, 0]))  # pads (left, right) = bodies (1, 0)
    torch.testing.assert_close(out[0, 0], local[0] + pos[0, 1])
    torch.testing.assert_close(out[0, 1], local[1] + pos[0, 0])
    z90 = torch.tensor([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    out = taxel_positions_world(pos, z90.expand(1, 2, 4), local, torch.tensor([0, 1]))
    torch.testing.assert_close(out[0, 0, 0], quat_apply(z90, local[0, 0]) + pos[0, 0], atol=1e-6, rtol=0)


def _scene():
    local = torch.tensor(np.stack([taxel_positions(p, TSF85) for p in ROBOTIQ_2F_85.pads]), dtype=torch.float32)
    pos = torch.zeros(1, 2, 3)
    quat = _identity_quat(1, 2)
    return local, pos, quat


def test_a_contact_on_a_taxel_lights_that_taxel_of_that_pad_and_the_sensor_order_does_not_matter():
    local, pos, quat = _scene()
    left_contact = local[0, 10]  # taxel 10 of the left pad
    for order in ([0, 1], [1, 0]):  # the sensor lists (left, right) or (right, left)
        f = torch.zeros(1, 2, 1, 3)  # (E, sensor body, object, xyz)
        c = torch.full((1, 2, 1, 3), float("nan"))  # NaN where there is no contact
        body = order.index(0)  # the sensor body that is the left pad
        f[0, body, 0] = torch.tensor([0.0, 4.0, 0.0])
        c[0, body, 0] = left_contact
        out = taxel_force_map(pos, quat, f, c, local, torch.tensor(order))
        assert torch.isfinite(out).all()  # the NaN contact point of the other pad did not poison anything
        assert int(out[0, 0].argmax()) == 10 and abs(float(out[0, 0].sum()) - 4.0) < 1e-3
        assert out[0, 1].abs().max() == 0  # the right pad feels nothing


def test_a_force_on_the_back_of_the_finger_reads_nothing_by_default():
    local, pos, quat = _scene()
    f = torch.zeros(1, 2, 1, 3)
    f[0, 0, 0, 1] = 6.0
    c = torch.zeros(1, 2, 1, 3)
    c[0, 0, 0] = local[0, 10] + torch.tensor([0.0, -0.021, 0.0])  # 21 mm behind the taxel plane
    ids = torch.tensor([0, 1])
    assert taxel_force_map(pos, quat, f, c, local, ids).abs().max() == 0
    assert taxel_force_map(pos, quat, f, c, local, ids, max_distance=None).sum() > 5.99  # the gate can be turned off
