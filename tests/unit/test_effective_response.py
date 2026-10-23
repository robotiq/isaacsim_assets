"""Eq. 1 of Shang et al. (arXiv 2610.10510) and its force / pressure variants: pure numpy, no Isaac."""

import numpy as np
import pytest

from tactile.contact_points import ContactPoint, link_contacts
from tactile.effective_response import (ETA, GAMMA, SIGMA_MM, convex_hull, effective_response, pad_maps, pad_plane,
                                        patch_force, response_from_taxel_pressure)

ROWS, COLS = 7, 4
PITCH = (5.875, 5.43)  # mm across the columns, along the rows
UV = np.stack(np.meshgrid((np.arange(COLS) - 1.5) * PITCH[0], (np.arange(ROWS) - 3) * PITCH[1], indexing="xy"), -1)


def rectangle(u0, u1, v0, v1):
    return np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]], dtype=float)


BOX = rectangle(-6, 6, -8.25, 8.25)  # a 12 x 16.5 mm box face on the grid


def test_paper_constants_are_the_defaults():
    assert SIGMA_MM == (1.5, 1.25) and ETA == 0.7 and GAMMA == 0.5


def test_convex_hull_of_a_square_with_an_interior_point():
    pts = np.vstack([BOX, [[0.0, 0.0]]])
    hull = convex_hull(pts)
    assert len(hull) == 4 and {tuple(p) for p in hull} == {tuple(p) for p in BOX}
    x, y = hull[:, 0], hull[:, 1]
    assert 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0, "counter-clockwise"


@pytest.mark.parametrize("pts", [[[1.0, 2.0]], [[1.0, 2.0], [3.0, 4.0]]])
def test_convex_hull_of_fewer_than_three_points_is_the_points(pts):
    assert np.array_equal(convex_hull(np.array(pts)), np.array(pts))


def test_no_contact_or_no_force_gives_zeros():
    assert not effective_response(np.zeros((0, 2)), 10.0, UV, PITCH).any()
    assert not effective_response(BOX, 0.0, UV, PITCH).any()
    assert not patch_force(BOX, 0.0, UV, PITCH).any()


def test_patch_force_sums_to_the_force_on_the_grid_and_less_off_it():
    assert patch_force(BOX, 30.0, UV, PITCH).sum() == pytest.approx(30.0, rel=1e-6)
    off = rectangle(5, 40, -8, 8)  # hangs over the right edge of the grid
    assert patch_force(off, 30.0, UV, PITCH).sum() < 29.0


def test_a_patch_over_the_whole_grid_gives_r_equal_to_pressure_to_the_gamma():
    big = rectangle(-60, 60, -60, 60)
    force = 40.0
    pressure = force / (120 * 120)  # N / mm^2
    r = effective_response(big, force, UV, PITCH)
    assert r == pytest.approx(np.full((ROWS, COLS), pressure ** GAMMA), rel=5e-3)  # the raster quantises the patch edge


def test_r_scales_as_force_to_the_gamma():
    r1 = effective_response(BOX, 10.0, UV, PITCH)
    r4 = effective_response(BOX, 40.0, UV, PITCH)
    assert r4 == pytest.approx(r1 * 4 ** GAMMA, rel=1e-9)


def test_pad_maps_agrees_with_the_single_purpose_functions():
    force, r = pad_maps(BOX, 25.0, UV, PITCH)
    assert np.array_equal(force, patch_force(BOX, 25.0, UV, PITCH))
    assert np.array_equal(r, effective_response(BOX, 25.0, UV, PITCH))


def test_taxel_pressure_of_a_whole_taxel_patch_matches_the_patch():
    """The raycast path (a pressure per taxel) and the contact path (a force and a patch) agree on a patch
    that covers whole taxels, up to the raster."""
    u0, u1 = UV[0, 1, 0] - PITCH[0] / 2, UV[0, 2, 0] + PITCH[0] / 2
    v0, v1 = UV[2, 0, 1] - PITCH[1] / 2, UV[4, 0, 1] + PITCH[1] / 2
    patch = rectangle(u0, u1, v0, v1)
    force = 30.0
    pressure = np.zeros((ROWS, COLS))
    pressure[2:5, 1:3] = force / ((u1 - u0) * (v1 - v0))
    from_patch = effective_response(patch, force, UV, PITCH)
    from_pressure = response_from_taxel_pressure(pressure, UV, PITCH)
    assert np.abs(from_patch - from_pressure).max() <= 0.03 * from_patch.max()


def test_a_centred_box_reads_mirror_symmetric():
    r = effective_response(BOX, 30.0, UV, PITCH)
    assert np.abs(r - r[:, ::-1]).max() <= 0.05 * r.max()
    assert np.abs(r - r[::-1]).max() <= 0.05 * r.max()


def test_pad_plane_recovers_the_grid_it_was_built_from():
    rot = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)  # an arbitrary rotation
    local = np.concatenate([UV / 1000.0, np.zeros((ROWS, COLS, 1))], axis=-1)  # m, in the pad plane
    pts = local @ rot.T + np.array([0.1, -0.2, 0.3])
    o, u, v, uv, pitch = pad_plane(pts)
    assert pitch == pytest.approx(PITCH, rel=1e-6)
    assert uv == pytest.approx(UV - UV.reshape(-1, 2).mean(axis=0), abs=1e-6)
    assert u == pytest.approx(rot[:, 0]) and v == pytest.approx(rot[:, 1])


def test_link_contacts_sums_forces_per_link():
    n = np.array([0.0, 0.0, 1.0])
    physx = [ContactPoint("pad", np.zeros(3), n, 3.0, -1e-4, np.array([0.0, 0.0, 3.0])),
             ContactPoint("pad", np.zeros(3), n, 4.0, -2e-4, np.array([0.0, 4.0, 0.0]))]
    c = link_contacts(physx)["pad"]
    assert c.force == pytest.approx(5.0) and c.min_separation == pytest.approx(-2e-4) and c.points == 2
    newton = [ContactPoint("pad", np.zeros(3), n, 3.0, 0.0), ContactPoint("pad", np.zeros(3), n, 4.0, 0.0)]
    assert link_contacts(newton)["pad"].force == pytest.approx(7.0)
