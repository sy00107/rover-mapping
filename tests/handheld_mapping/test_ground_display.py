"""작은 초록 내부 구멍만 채우고 외곽·큰 구멍·장애물·미분류는 보존한다."""
import numpy as np
import pytest

from experiments.handheld_mapping.ground_display import fill_small_ground_gaps, _triangulate_polygon
from experiments.handheld_mapping.surface_colors import GROUND, OBSTACLE, UNKNOWN, PALETTE


def ground_with_hole(width=3, slope=.3):
    import open3d as o3d
    size = 21
    y, x = np.mgrid[:size, :size]*.01
    vertices = np.c_[x.ravel(), y.ravel(), (slope*x).ravel()]
    a = (np.arange(size-1)[:, None]*size+np.arange(size-1)[None, :]).ravel()
    cells_x, cells_y = a % size, a//size
    keep = ~((cells_x >= 8) & (cells_x < 8+width) & (cells_y >= 8) & (cells_y < 8+width))
    a = a[keep]
    faces = np.r_[np.c_[a, a+1, a+size], np.c_[a+1, a+size+1, a+size]]
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    labels = np.full(len(vertices), GROUND)
    mesh.vertex_colors = o3d.utility.Vector3dVector(PALETTE[labels])
    return mesh, labels


def test_small_ground_hole_is_capped_on_its_slope_without_moving_vertices_or_colours():
    mesh, labels = ground_with_hole()
    vertices, colors, faces = (np.asarray(value).copy() for value in
                               (mesh.vertices, mesh.vertex_colors, mesh.triangles))
    stats = fill_small_ground_gaps(mesh, labels)
    assert stats['filled_holes'] == 1 and stats['added_triangles'] == 10
    np.testing.assert_array_equal(mesh.vertices, vertices)
    np.testing.assert_array_equal(mesh.vertex_colors, colors)
    np.testing.assert_array_equal(np.asarray(mesh.triangles)[:len(faces)], faces)
    cap = vertices[np.asarray(mesh.triangles)[len(faces):]]
    assert np.all((cap[:, :, 0] >= .08) & (cap[:, :, 0] <= .11))
    assert np.allclose(cap[:, :, 2], .3*cap[:, :, 0])
    assert np.all(np.linalg.norm(np.cross(cap[:, 1]-cap[:, 0], cap[:, 2]-cap[:, 0]), axis=1) > 1e-10)


@pytest.mark.parametrize('label', [OBSTACLE, UNKNOWN])
def test_non_green_boundary_never_becomes_a_green_patch(label):
    mesh, labels = ground_with_hole()
    labels[8*21+8] = label
    before = np.asarray(mesh.triangles).copy()
    stats = fill_small_ground_gaps(mesh, labels)
    assert stats['filled_holes'] == 0
    np.testing.assert_array_equal(mesh.triangles, before)


def test_large_hole_and_small_outer_island_are_not_filled():
    import open3d as o3d
    mesh, labels = ground_with_hole(width=8)
    assert fill_small_ground_gaps(mesh, labels)['filled_holes'] == 0
    island = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[0, 0, 0], [.02, 0, 0], [0, .02, 0]]),
                                     o3d.utility.Vector3iVector([[0, 1, 2]]))
    assert fill_small_ground_gaps(island, np.full(3, GROUND))['added_triangles'] == 0


def test_concave_hole_triangulation_keeps_area_and_rejects_crossed_boundary():
    points = np.array([[0., 0.], [3., 0.], [3., 1.], [1., 1.], [1., 3.], [0., 3.]])
    faces = _triangulate_polygon(points)
    p = points[np.asarray(faces)]
    area = np.abs(np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0])).sum()/2
    assert len(faces) == 4 and area == pytest.approx(5.)
    assert _triangulate_polygon(np.array([[0., 0.], [1., 1.], [0., 1.], [1., 0.]])) == []
