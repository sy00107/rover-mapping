"""거리 손실·경계 둥글어짐·근거 없는 연결 없이 원거리 표면을 보완한다."""
import numpy as np
import pytest

from experiments.handheld_mapping.depth_filter import DepthFilter, multiview_filter, spatial_filter
from experiments.handheld_mapping.surface_cleanup import smooth_far_depth, clean_surface_mesh


def test_smoothing_reduces_far_noise_and_preserves_near_pixels_holes_and_steps():
    rng = np.random.default_rng(17)
    depth = np.rint(1800+rng.normal(0, 9, (70, 100))).astype(np.uint16)
    depth[:, :20] = rng.integers(850, 870, (70, 20), dtype=np.uint16)
    depth[:, 70:] += 200
    depth[35, 45] = 0
    original = depth.copy()
    result = smooth_far_depth(depth, .001, .25, 2.1)
    np.testing.assert_array_equal(depth, original)
    np.testing.assert_array_equal(result[:, :20], depth[:, :20])
    assert result[35, 45] == 0
    assert result[5:25, 30:60].std() < .65*depth[5:25, 30:60].std()
    assert np.median(result[:, 69]) < 1820 and np.median(result[:, 70]) > 1980
    assert abs(float(np.median(result[5:25, 30:60]))-1800) < 3


def test_enclosed_single_hole_needs_two_measured_references_and_large_gaps_stay_empty():
    info = {'k': [40., 0., 19.5, 0., 40., 14.5, 0., 0., 1.], 'depth_scale_m': .001}
    depth = np.full((30, 40), 1800, np.uint16)
    depth[10, 10] = 0
    depth[20:23, 20:23] = 0
    depth[:, :2] = 0
    reference = np.full_like(depth, 1800)
    config = DepthFilter(max_depth_m=2., surface_cleanup=True)
    result, stats = multiview_filter(depth, info, np.eye(4),
                                    [(reference, np.eye(4)), (reference, np.eye(4))], config)
    assert result[10, 10] == 1800 and stats['supported_holes_filled'] == 1
    assert not result[20:23, 20:23].any() and not result[:, :2].any()
    reference[10, 10] = 0
    result, stats = multiview_filter(depth, info, np.eye(4),
                                    [(reference, np.eye(4)), (reference, np.eye(4))], config)
    assert result[10, 10] == 0 and stats['supported_holes_filled'] == 0


def test_far_points_require_two_matches_and_45mm_limit_but_near_keeps_existing_policy():
    info = {'k': [40., 0., 19.5, 0., 40., 14.5, 0., 0., 1.], 'depth_scale_m': .001}
    depth = np.full((30, 40), 1900, np.uint16)
    depth[:, :10] = 800
    reference = depth.copy()
    reference[:, 25:] += 60  # Accepted by the old 2m tolerance, rejected by cleanup.
    config = DepthFilter(max_depth_m=2., surface_cleanup=True)
    result, _ = multiview_filter(depth, info, np.eye(4), [(reference, np.eye(4))], config)
    np.testing.assert_array_equal(result[:, :10], depth[:, :10])
    assert not result[:, 10:].any()
    result, _ = multiview_filter(depth, info, np.eye(4), [(reference, np.eye(4))]*2, config)
    assert (result[:, 10:25] == 1900).all() and not result[:, 25:].any()
    fallback, _ = multiview_filter(depth, info, np.eye(4), [], config)
    np.testing.assert_array_equal(fallback[:, :10], depth[:, :10])
    assert not fallback[:, 10:].any()


def test_hole_next_to_another_depth_surface_is_not_completed():
    from experiments.handheld_mapping.surface_cleanup import supported_hole_candidates
    depth = np.full((15, 15), 1800, np.uint16)
    depth[7, 7] = 0
    depth[7, 6] = 900
    result, candidates = supported_hole_candidates(depth, .001, 2.)
    assert not candidates[7, 7] and result[7, 7] == 0


def test_far_smoothing_preserves_sloped_surface():
    y, x = np.mgrid[:60, :80]
    # An affine inverse depth is the projection of a plane.
    truth = 1./(.56+.0003*x-.0002*y)
    depth = np.rint(truth*1000).astype(np.uint16)
    filtered, _ = spatial_filter(depth, .001, DepthFilter(max_depth_m=2., surface_cleanup=True))
    valid = filtered[4:-4, 4:-4] > 0
    assert np.max(np.abs(filtered[4:-4, 4:-4][valid]*.001-truth[4:-4, 4:-4][valid])) < .002


def test_mesh_cleanup_keeps_near_small_objects_and_far_large_objects_without_bridges():
    import open3d as o3d
    near = o3d.geometry.TriangleMesh.create_box(.02, .02, .02).translate([.5, 0., 0.])
    speck = o3d.geometry.TriangleMesh.create_box(.01, .01, .01).translate([2., 0., 0.])
    obstacle = o3d.geometry.TriangleMesh.create_box(.2, .2, .2).translate([2., 1., 0.])
    mesh = near+speck+obstacle
    near_original = np.asarray(mesh.vertices)[:8].copy()
    cleaned, stats = clean_surface_mesh(mesh, [[0., 0., 0.]])
    assert stats['removed_components'] == 1 and stats['removed_triangles'] == 12
    assert len(cleaned.triangles) == 24
    np.testing.assert_array_equal(np.asarray(cleaned.vertices)[:8], near_original)
    assert stats['max_vertex_shift_m'] <= .00500001
    assert len(cleaned.cluster_connected_triangles()[1]) == 2
    assert not stats['new_faces_or_bridges']


def test_mesh_with_only_far_specks_returns_empty_without_native_smoothing_error():
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh.create_box(.01, .01, .01).translate([2., 0., 0.])
    cleaned, stats = clean_surface_mesh(mesh, [[0., 0., 0.]])
    assert not len(cleaned.triangles) and not len(cleaned.vertices)
    assert stats['max_vertex_shift_m'] == 0.


@pytest.mark.parametrize('invalid', [None, 1, 'yes'])
def test_cleanup_setting_must_be_boolean(invalid):
    with pytest.raises(ValueError):
        DepthFilter(surface_cleanup=invalid)
