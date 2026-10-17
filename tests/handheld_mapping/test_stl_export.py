import hashlib
import json

import numpy as np
import pytest

from experiments.handheld_mapping.stl_export import (
    export_stl, prevent_face_damage, smooth_export_surface, supported_ground_patch)
from experiments.handheld_mapping.surface_colors import GROUND, OBSTACLE, UNKNOWN
from test_ground_display import ground_with_hole
from test_obstacle_display import noisy_surface


def terrain_for_plane(output, slope=.3):
    y, x = np.mgrid[:50, :50]
    yes = np.ones((50, 50), bool)
    return {'layers': {'ground': yes.copy(), 'surface_observed': yes.copy(),
                      'obstacle_observed': yes.copy(), 'reference_ground_height_m': slope*(x+.5)*.01,
                      'reference_ground_gradient_x': np.full(x.shape, slope),
                      'reference_ground_gradient_y': np.zeros(x.shape)},
            'bounds': np.array([0., .5, 0., .5]), 'resolution': .01,
            'grid_from_geometry': np.eye(4), 'output_dir': str(output),
            'ground_tolerance_m': .02, 'obstacle_threshold_m': .06}


@pytest.mark.parametrize('label,cap', [(GROUND, .008), (OBSTACLE, .015)])
def test_noise_reduced_without_flattening_ramp_or_moving_boundary(label, cap):
    mesh, _ = noisy_surface()
    original = np.asarray(mesh.vertices).copy()
    faces = np.asarray(mesh.triangles).copy()
    smooth_export_surface(mesh, np.full(len(original), label))
    points = np.asarray(mesh.vertices)
    interior = ((original[:, 0] > .06) & (original[:, 0] < .38)
                & (original[:, 1] > .06) & (original[:, 1] < .38))
    before = original[interior, 2]-.3*original[interior, 0]
    after = points[interior, 2]-.3*points[interior, 0]
    assert after.std() < .75*before.std()
    assert abs(after.mean()) < .0005
    assert np.linalg.norm(points-original, axis=1).max() <= cap+1e-10
    boundary = (original[:, 0] == 0.) | np.isclose(original[:, 0], .44)
    np.testing.assert_array_equal(points[boundary], original[boundary])
    np.testing.assert_array_equal(mesh.triangles, faces)


def test_unknowns_and_sharp_box_edges_are_preserved():
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh.create_box()+o3d.geometry.TriangleMesh.create_box().translate([1.01, 0, 0])
    original = np.asarray(mesh.vertices).copy()
    smooth_export_surface(mesh, np.full(len(original), OBSTACLE))
    np.testing.assert_array_equal(mesh.vertices, original)
    mesh, _ = noisy_surface()
    original = np.asarray(mesh.vertices).copy()
    smooth_export_surface(mesh, np.full(len(original), UNKNOWN))
    np.testing.assert_array_equal(mesh.vertices, original)


def test_guard_reverts_category_change_and_face_collapse(tmp_path):
    import open3d as o3d
    mesh, labels = ground_with_hole()
    original = np.asarray(mesh.vertices).copy()
    damaged = original.copy()
    damaged[100, 2] += .1
    damaged[101] = damaged[102]
    mesh.vertices = o3d.utility.Vector3dVector(damaged)
    report = prevent_face_damage(mesh, original, labels, terrain_for_plane(tmp_path))
    assert report['reverted_vertices'] >= 2
    assert report['category_changes_after_guard'] == 0
    points = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    before = np.cross(original[faces[:, 1]]-original[faces[:, 0]], original[faces[:, 2]]-original[faces[:, 0]])
    after = np.cross(points[faces[:, 1]]-points[faces[:, 0]], points[faces[:, 2]]-points[faces[:, 0]])
    assert np.all(np.einsum('ij,ij->i', before, after) > 0)


@pytest.mark.parametrize('supported', [True, False])
def test_real_export_larger_supported_hole_source_immutable_and_binary_valid(tmp_path, monkeypatch, supported):
    import open3d as o3d
    from experiments.handheld_mapping import stl_export
    from experiments.handheld_mapping.ground_display import fill_small_ground_gaps
    mesh, labels = ground_with_hole(width=5)
    assert fill_small_ground_gaps(mesh, labels)['filled_holes'] == 0
    source = tmp_path/'source.ply'
    assert o3d.io.write_triangle_mesh(str(source), mesh)
    fingerprint = hashlib.sha256(source.read_bytes()).hexdigest()
    terrain = terrain_for_plane(tmp_path)
    if not supported:
        terrain['layers']['surface_observed'][9:12, 9:12] = False
    (tmp_path/'summary.json').write_text('{}')
    np.savez(tmp_path/'terrain.npz', **terrain['layers'])
    monkeypatch.setattr(stl_export, 'load_surface_layers', lambda _: terrain)
    output = tmp_path/'export'
    report = export_stl(source, output)
    assert report['ground_holes']['filled_holes'] == int(supported)
    assert report['source_unchanged']
    assert hashlib.sha256(source.read_bytes()).hexdigest() == fingerprint
    assert report['refined']['coordinates_verified']
    assert report['remaining_nonmanifold_edges'] == 0
    mask = np.load(output/'inferred_faces.npy', allow_pickle=False)
    assert mask.sum() == report['ground_holes']['added_triangles']
    reloaded = o3d.io.read_triangle_mesh(str(output/'map_refined.stl'))
    assert len(reloaded.triangles) == report['refined']['triangles']
    assert json.loads((output/'quality.json').read_text())['nav2_ready'] is False
    with pytest.raises(FileExistsError):
        export_stl(source, output)


def test_patch_with_unknown_interior_is_rejected(tmp_path):
    terrain = terrain_for_plane(tmp_path, slope=0.)
    triangles = np.array([[[.08, .08, 0.], [.13, .08, 0.], [.08, .13, 0.]]])
    assert supported_ground_patch(triangles, terrain)
    terrain['layers']['surface_observed'][9:11, 9:11] = False
    assert not supported_ground_patch(triangles, terrain)


def test_patch_cannot_reuse_an_existing_interior_edge():
    import copy
    import open3d as o3d
    from experiments.handheld_mapping.ground_display import fill_small_ground_gaps
    from experiments.handheld_mapping.stl_export import mesh_edges
    mesh, labels = ground_with_hole(width=5)
    trial = copy.deepcopy(mesh)
    n = len(mesh.triangles)
    fill_small_ground_gaps(trial, labels, maximum_diameter_m=.08, maximum_edges=40)
    edges, _, counts = mesh_edges(np.asarray(trial.triangles)[n:])
    a, b = edges[counts == 2][0]
    vertices = np.asarray(mesh.vertices).copy()
    middle = (vertices[a]+vertices[b])*.5
    c, d = len(vertices), len(vertices)+1
    mesh.vertices = o3d.utility.Vector3dVector(np.r_[vertices,
        [middle+[0., 0., .03], middle+[.01, -.01, .02]]])
    mesh.triangles = o3d.utility.Vector3iVector(np.r_[np.asarray(mesh.triangles),
        [[a, b, c], [b, a, d], [a, c, d], [b, d, c]]])
    labels = np.r_[labels, UNKNOWN, UNKNOWN]
    before = np.asarray(mesh.triangles).copy()
    stats = fill_small_ground_gaps(mesh, labels, maximum_diameter_m=.08, maximum_edges=40,
                                  prevent_nonmanifold=True)
    assert stats['topology_rejected_holes'] == 1
    assert stats['added_triangles'] == 0
    np.testing.assert_array_equal(mesh.triangles, before)
