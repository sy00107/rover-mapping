"""Geometry checks for the user-confirmed fixed ramp junction, no sensors."""
import numpy as np
import pytest

from experiments.handheld_mapping.slope_step import step_geometry


def ramp(y0, y1, slope, offset=0):
    x, y = np.meshgrid(np.linspace(0, 1, 12), np.linspace(y0, y1, 6))
    return np.c_[x.ravel(), y.ravel(), (slope*x+offset).ravel()]


def test_step_has_vertical_face_and_preserves_both_ramp_edge_heights():
    first = ramp(-.5, 0, .17)
    second = ramp(.1, .5, .58, .08)
    before = first.copy(), second.copy()
    xyz, faces, info = step_geometry(first, second)
    panels = xyz.reshape(-1, 8, 3)
    # The middle face shares exact XY, with independent ramp heights.
    assert np.allclose(panels[:, 1, :2], panels[:, 2, :2])
    assert np.allclose(panels[:, 5, :2], panels[:, 6, :2])
    assert np.all(panels[:, 2, 2] > panels[:, 1, 2])
    assert np.allclose(panels[:, 0, 2], .17*panels[:, 0, 0])
    assert np.allclose(panels[:, 3, 2], .58*panels[:, 3, 0]+.08)
    assert xyz[:, 0].min() >= 0 and xyz[:, 0].max() <= 1
    assert np.isfinite(xyz).all() and faces.max() < len(xyz)
    assert info['semantic'] == 'ramp_step' and info['display_only'] and info['inferred']
    assert np.array_equal(first, before[0]) and np.array_equal(second, before[1])


def test_step_follows_rotated_fixed_map_and_rejects_large_or_missing_gap():
    first, second = ramp(-.5, 0, .17), ramp(.1, .5, .58, .08)
    theta = .7
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    rotated = []
    for p in (first, second):
        q = p.copy(); q[:, :2] = p[:, :2]@rotation.T+[4, -2]; q[:, 2] += .4
        rotated.append(q)
    xyz, faces, info = step_geometry(*rotated)
    assert len(faces) > 0 and info['step_height_range_m'][1] > .4
    with pytest.raises(ValueError, match='bounded'):
        step_geometry(first, ramp(.6, 1, .58))
    q = second.copy(); q[:, 0] += 3
    with pytest.raises(ValueError, match='span'):
        step_geometry(first, q)


def test_unstable_plane_and_invalid_inputs_are_rejected():
    first, second = ramp(-.5, 0, .17), ramp(.1, .5, .58)
    first[0, 2] += 2
    with pytest.raises(ValueError, match='RMS'):
        step_geometry(first, second)
    second[0, 0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        step_geometry(ramp(-.5, 0, .17), second)
