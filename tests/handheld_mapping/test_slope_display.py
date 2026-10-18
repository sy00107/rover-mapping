"""Short colour gaps may be repaired; broad flats, steps and unknowns must survive."""
import copy

import numpy as np
import pytest

from experiments.handheld_mapping.slope_display import continuous_slope_mask
from experiments.handheld_mapping.surface_colors import (
    GROUND, OBSTACLE, UNKNOWN, PALETTE, SLOPE_COLOR, surface_point_colors,
)


def ramp(res=.05):
    size = 25
    y, x = np.mgrid[:size, :size]
    gradient = np.tan(np.deg2rad(10.))
    layers = dict(
        ground=np.ones((size, size), bool), surface_observed=np.ones((size, size), bool),
        reference_ground_height_m=(x+.5)*res*gradient,
        reference_ground_gradient_x=np.full((size, size), gradient),
        reference_ground_gradient_y=np.zeros((size, size)),
    )
    return dict(layers=layers, resolution=res, bounds=np.array([0., size*res, 0., size*res]),
                grid_from_geometry=np.eye(4))


@pytest.mark.parametrize('width', [1, 2, 3])
def test_short_flat_colour_island_in_height_consistent_ramp_is_repaired(width):
    terrain = ramp()
    terrain['layers']['reference_ground_gradient_x'][11:11+width, 11:11+width] = 0.
    original = copy.deepcopy(terrain['layers'])
    pink, stats = continuous_slope_mask(terrain)
    assert pink[11:11+width, 11:11+width].all()
    assert stats['corrected_cells'] == width**2
    for key in original:
        np.testing.assert_array_equal(terrain['layers'][key], original[key])


def test_broad_real_flat_landing_and_slope_endpoint_remain_green():
    terrain = ramp()
    layers = terrain['layers']
    layers['reference_ground_gradient_x'][:, 9:17] = 0.
    # A genuine 40 cm landing: height is constant here and slope continues afterward.
    h = layers['reference_ground_height_m']
    rise = h[0, 16]-h[0, 9]
    h[:, 9:17] = h[0, 9]
    h[:, 17:] -= rise
    layers['reference_ground_gradient_x'][:, :3] = 0.
    pink, _ = continuous_slope_mask(terrain)
    assert not pink[:, 9:17].any()
    assert not pink[:, :3].any()


def test_small_height_discontinuity_and_unobserved_hole_are_not_recoloured():
    terrain = ramp()
    layers = terrain['layers']
    layers['reference_ground_gradient_x'][12, 12] = 0.
    layers['reference_ground_height_m'][12, 12] += .08
    pink, stats = continuous_slope_mask(terrain)
    assert not pink[12, 12]
    assert stats['corrected_cells'] == 0
    layers['reference_ground_height_m'][12, 12] -= .08
    layers['surface_observed'][11:14, 11:14] = False
    pink, _ = continuous_slope_mask(terrain)
    assert not pink[12, 12]


def test_obstacle_unknown_and_disable_switch_survive_slope_colour_repair():
    terrain = ramp()
    terrain['layers']['reference_ground_gradient_x'][12, 12] = 0.
    points = np.tile([.625, .625, .11], (3, 1))
    labels = np.array([GROUND, OBSTACLE, UNKNOWN])
    corrected = surface_point_colors(points, labels, terrain)
    np.testing.assert_allclose(corrected, [SLOPE_COLOR, PALETTE[OBSTACLE], PALETTE[UNKNOWN]])
    raw = surface_point_colors(points, labels, terrain, smooth_slope_colors=False)
    np.testing.assert_allclose(raw, [PALETTE[GROUND], PALETTE[OBSTACLE], PALETTE[UNKNOWN]])


def test_coarse_cell_wider_than_gap_limit_is_not_bridged():
    terrain = ramp(.25)
    terrain['layers']['reference_ground_gradient_x'][12, 12] = 0.
    pink, stats = continuous_slope_mask(terrain)
    assert not pink[12, 12]
    assert stats['corrected_cells'] == 0
