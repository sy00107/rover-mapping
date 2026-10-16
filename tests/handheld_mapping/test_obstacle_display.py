"""표시 평활화가 주황 표면 잡음을 줄이고 지면·경계·분류·저장값을 보존한다."""
import numpy as np
import pytest

from experiments.handheld_mapping.obstacle_display import smooth_obstacle_display
from experiments.handheld_mapping.surface_colors import GROUND, OBSTACLE, UNKNOWN, PALETTE


def noisy_surface(size=45):
    import open3d as o3d
    y, x = np.mgrid[:size, :size]*.01
    z = .3*x+np.random.default_rng(9).normal(0., .0015, x.shape)
    vertices = np.c_[x.ravel(), y.ravel(), z.ravel()]
    a = (np.arange(size-1)[:, None]*size+np.arange(size-1)[None, :]).ravel()
    faces = np.r_[np.c_[a, a+1, a+size], np.c_[a+1, a+size+1, a+size]]
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    labels = np.where(x.ravel() < .1, GROUND, OBSTACLE).astype(np.uint8)
    labels[x.ravel() > .42] = UNKNOWN
    mesh.vertex_colors = o3d.utility.Vector3dVector(PALETTE[labels])
    return mesh, labels


def test_obstacle_noise_reduced_while_ground_unknown_boundary_and_topology_stay_fixed():
    mesh, labels = noisy_surface()
    original = np.asarray(mesh.vertices).copy()
    faces = np.asarray(mesh.triangles).copy()
    colors = np.asarray(mesh.vertex_colors).copy()
    stats = smooth_obstacle_display(mesh, labels)
    result = np.asarray(mesh.vertices)
    interior = ((original[:, 0] > .16) & (original[:, 0] < .38)
                & (original[:, 1] > .06) & (original[:, 1] < .38))
    residual_before = original[interior, 2]-.3*original[interior, 0]
    residual_after = result[interior, 2]-.3*result[interior, 0]
    assert residual_after.std() < .75*residual_before.std()
    assert abs(residual_after.mean()) < .0005
    np.testing.assert_array_equal(result[labels != OBSTACLE], original[labels != OBSTACLE])
    boundary = (original[:, 1] == 0.) | np.isclose(original[:, 1], .44)
    np.testing.assert_array_equal(result[boundary], original[boundary])
    np.testing.assert_array_equal(mesh.triangles, faces)
    np.testing.assert_array_equal(mesh.vertex_colors, colors)
    assert stats['smoothed_vertices'] > 100 and stats['max_vertex_shift_m'] <= .01000001
    assert np.isfinite(mesh.vertex_normals).all()


def test_sharp_box_corners_and_separate_components_are_preserved():
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh.create_box()+o3d.geometry.TriangleMesh.create_box().translate([1.01, 0., 0.])
    original = np.asarray(mesh.vertices).copy()
    stats = smooth_obstacle_display(mesh, np.full(len(original), OBSTACLE))
    np.testing.assert_array_equal(mesh.vertices, original)
    assert stats['smoothed_vertices'] == 0
    assert len(mesh.cluster_connected_triangles()[1]) == 2


def test_no_obstacles_does_not_change_mesh():
    mesh, labels = noisy_surface()
    original = np.asarray(mesh.vertices).copy()
    stats = smooth_obstacle_display(mesh, np.full(len(labels), GROUND))
    np.testing.assert_array_equal(mesh.vertices, original)
    assert stats['obstacle_vertices'] == stats['smoothed_vertices'] == 0


def test_label_count_mismatch_is_rejected():
    mesh, _ = noisy_surface()
    with pytest.raises(ValueError, match='labels'):
        smooth_obstacle_display(mesh, [])
