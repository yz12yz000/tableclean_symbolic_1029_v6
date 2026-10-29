"""Measured occupancy fits must not alter the original free-radius grasp fit."""
import numpy as np
import pytest

from rim_locator.circle_fit import fit_measured_rim_circle


def arc(radius=.065, degrees=100, noise=.0007):
    rng = np.random.default_rng(31)
    theta = np.linspace(.2, .2 + np.deg2rad(degrees), 600)
    radial = radius + rng.normal(0, noise, len(theta))
    return np.c_[.65 + radial * np.cos(theta), .08 + radial * np.sin(theta),
                 np.full(len(theta), -.02)]


@pytest.mark.parametrize('radius', [.065, .0925])
def test_measured_partial_rim_size_center_and_order_invariance(radius):
    points = arc(radius)
    fit = fit_measured_rim_circle(points, [.065, .0925])
    assert fit['measured_radius'] == radius
    assert np.linalg.norm(np.array(fit['center_xy']) - [.65, .08]) < .002
    assert fit_measured_rim_circle(points[::-1], [.0925, .065]) == fit


@pytest.mark.parametrize('points,radii', [
    (arc(degrees=35), [.065, .0925]),
    (arc()[::50], [.065]),
    (arc(degrees=90), [.0925]),
    (np.c_[np.linspace(0, .15, 200), np.zeros((200, 2))], [.065, .0925]),
    (np.full((100, 3), np.nan), [.065]),
])
def test_unresolved_geometry_is_rejected(points, radii):
    with pytest.raises(ValueError, match='measured_rim_'):
        fit_measured_rim_circle(points, radii)


def test_similar_sizes_with_ambiguous_support_are_rejected():
    with pytest.raises(ValueError, match='ambiguous_size'):
        fit_measured_rim_circle(arc(degrees=100), [.065, .066])
