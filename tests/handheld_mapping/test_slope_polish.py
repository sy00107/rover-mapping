"""The fixed ramp polish fills holes without changing source or orange objects."""
import numpy as np
import pytest

from experiments.handheld_mapping.slope_polish import polish_ramps
from experiments.handheld_mapping.surface_colors import PALETTE, OBSTACLE, SLOPE_COLOR


def test_planar_replacement_covers_hole_preserves_obstacle_and_source():
    import open3d as o3d
    from tests.handheld_mapping.test_ground_display import ground_with_hole

    mesh, _ = ground_with_hole(width=8)
    obstacle = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector([[.08, .08, .03], [.10, .08, .03], [.08, .10, .03]]),
        o3d.utility.Vector3iVector([[0, 1, 2]]))
    obstacle.paint_uniform_color(PALETTE[OBSTACLE])
    mesh += obstacle
    before = [np.asarray(a).copy() for a in (mesh.vertices, mesh.triangles, mesh.vertex_colors)]
    patch = np.array([[0, 0, 0], [.2, 0, .06], [.2, .2, .06], [0, .2, 0]])
    result, stats = polish_ramps(mesh, [patch])
    for original, expected in zip((mesh.vertices, mesh.triangles, mesh.vertex_colors), before):
        np.testing.assert_array_equal(original, expected)
    xyz, faces = np.asarray(result.vertices), np.asarray(result.triangles)
    colors = np.asarray(result.vertex_colors)
    pink = np.all(np.isclose(colors[faces], SLOPE_COLOR), axis=(1, 2))
    triangles = xyz[faces[pink]]
    assert np.allclose(triangles[:, :, 2], .3*triangles[:, :, 0])
    # Exact whole footprint area means the missing central 8 cm square is filled.
    edges = triangles[:, 1:, :2]-triangles[:, :1, :2]
    area = np.abs(edges[:, 0, 0]*edges[:, 1, 1]-edges[:, 0, 1]*edges[:, 1, 0]).sum()/2
    assert area == pytest.approx(.04)
    orange = np.all(np.isclose(colors[faces], PALETTE[OBSTACLE]), axis=(1, 2))
    np.testing.assert_allclose(xyz[faces[orange]], np.asarray(obstacle.vertices)[None])
    assert len(faces) == 3 and stats['display_only'] and stats['inferred']


def test_unclassified_geometry_is_rejected():
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh()
    with pytest.raises(ValueError, match='colours'):
        # A geometry without classifications must not silently erase structures.
        mesh.vertices = o3d.utility.Vector3dVector([[0, 0, 0]])
        polish_ramps(mesh, [])
