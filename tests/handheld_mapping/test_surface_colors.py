"""3D 표면의 실제 높이·기울기·좌표와 지형 출처를 대조하는 표시 검증."""
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_viewer import maps, keyframes
from experiments.handheld_mapping.surface_colors import (GROUND, OBSTACLE, UNKNOWN, PALETTE,
    apply_surface_colors, classify_surface_points, load_surface_layers)


@pytest.fixture
def terrain_run(tmp_path):
    run = maps(tmp_path/'run')
    keyframes(run)
    (run/'capture_calibration.json').write_text('{}')
    dense_path = run/'dense_offline/summary.json'
    dense = json.loads(dense_path.read_text())
    dense.update(voxel_size_m=.01, depth_filter={'method': 'fixture'})
    dense['maps'][0].update(coordinate_system='gravity_z_up', world_from_slam=np.eye(4).tolist())
    dense_path.write_text(json.dumps(dense))
    folder = run/'terrain/orb_offline/map_0'
    folder.mkdir(parents=True)
    gx = np.full((4, 4), .3, np.float32)
    gy = np.zeros((4, 4), np.float32)
    h = np.tile(.3*(np.arange(4)+.5)*.25, (4, 1)).astype(np.float32)
    ground = np.ones((4, 4), bool)
    ground[1, 2] = False  # Ground near a measured obstacle but not directly seen here.
    ground[3, 3] = False
    obstacle = np.ones((4, 4), bool)
    obstacle[3, 3] = False
    h[3, 3] = np.nan
    np.savez(folder/'terrain.npz', ground=ground, surface_observed=np.ones((4, 4), bool),
             obstacle_observed=obstacle, reference_ground_height_m=h,
             reference_ground_gradient_x=gx, reference_ground_gradient_y=gy)
    names = ('capture_calibration.json', 'orb_offline/map_0_keyframes.csv')
    summary = {'source': str(run), 'map_id': 0, 'backend': 'orb_offline',
        'source_sha256': {p: hashlib.sha256((run/p).read_bytes()).hexdigest() for p in names},
        'grid': {'bounds': [0., 1., 0., 1.], 'resolution': .25},
        'grid_from_world': np.eye(4).tolist(), 'depth_filter': dense['depth_filter'],
        'ground_separation': {'surface_band_m': .04}, 'rover_limits': {'max_step_m': .06}}
    (folder/'summary.json').write_text(json.dumps(summary))
    return run


def test_same_xy_floor_and_object_get_different_colours_and_slope_is_preserved(terrain_run):
    terrain = load_surface_layers(terrain_run/'dense_offline/map_0_mesh.ply')
    points = [[.24, .125, .3*.24], [.24, .125, .3*.24+.2], [.24, .125, -.2],
              [.625, .375, .3*.625+.3], [.625, .375, .3*.625],
              [.9, .9, .27], [-.01, .1, 0.], [np.nan, 0., 0.]]
    assert classify_surface_points(points, terrain).tolist() == [GROUND, OBSTACLE, UNKNOWN,
                                                               OBSTACLE, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]


def test_geometry_and_terrain_use_their_own_saved_frames(terrain_run):
    path = terrain_run/'dense_offline/summary.json'
    dense = json.loads(path.read_text())
    world = np.eye(4)
    world[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    world[:3, 3] = [10., 20., 30.]
    dense['maps'][0]['world_from_slam'] = world.tolist()
    path.write_text(json.dumps(dense))
    point = np.array([[.125, .125, .0375]])@world[:3, :3].T+world[:3, 3]
    terrain = load_surface_layers(terrain_run/'dense_offline/map_0_mesh.ply')
    assert classify_surface_points(point, terrain).tolist() == [GROUND]


@pytest.mark.parametrize('name', ['capture_calibration.json', 'orb_offline/map_0_keyframes.csv'])
def test_changed_source_cannot_colour_using_stale_terrain(terrain_run, name):
    with (terrain_run/name).open('a') as stream:
        stream.write('\n')
    with pytest.raises(ValueError, match='입력'):
        load_surface_layers(terrain_run/'dense_offline/map_0_mesh.ply')


@pytest.mark.parametrize('key,value', [('backend', 'orbline'), ('map_id', 421), ('source', '/another/run')])
def test_mismatched_identity_rejected(terrain_run, key, value):
    path = terrain_run/'terrain/orb_offline/map_0/summary.json'
    data = json.loads(path.read_text()); data[key] = value
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='다릅니다'):
        load_surface_layers(terrain_run/'dense_offline/map_0_cloud.ply')


def test_revision_order_is_numeric_and_invalid_latest_never_falls_back(terrain_run):
    import shutil
    root = terrain_run/'terrain/orb_offline'
    for revision in (2, 10):
        shutil.copytree(root/'map_0', root/f'map_0_v{revision}')
    path = terrain_run/'dense_offline/map_0_mesh.ply'
    assert load_surface_layers(path)['output_dir'].endswith('map_0_v10')
    (root/'map_0_v10/summary.json').write_text('{}')
    with pytest.raises(ValueError):
        load_surface_layers(path)
    assert load_surface_layers(terrain_run/'dense_offline/map_421_cloud.ply') is None


@pytest.mark.parametrize('cloud', [True, False])
def test_colours_change_only_display_geometry_not_saved_file(terrain_run, cloud):
    o3d = pytest.importorskip('open3d')
    path = terrain_run/f'dense_offline/map_0_{"cloud" if cloud else "mesh"}.ply'
    if cloud:
        points = np.array([[.125, .125, .0375], [.125, .125, .3375], [.9, .9, .27]])
        geometry = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        o3d.io.write_point_cloud(str(path), geometry)
    else:
        geometry = o3d.geometry.TriangleMesh.create_box(.5, .5, .3).translate([.1, .1, .03])
        o3d.io.write_triangle_mesh(str(path), geometry)
        points = np.asarray(geometry.vertices).copy()
    saved = path.read_bytes()
    stats = apply_surface_colors(geometry, path)
    colours = np.asarray(geometry.colors if cloud else geometry.vertex_colors)
    assert stats['ground'] > 0 and stats['obstacle'] > 0
    assert len(np.unique(colours, axis=0)) >= 2
    np.testing.assert_array_equal(np.asarray(geometry.points if cloud else geometry.vertices), points)
    assert path.read_bytes() == saved
    if cloud:
        np.testing.assert_allclose(colours, PALETTE[[GROUND, OBSTACLE, UNKNOWN]])


def test_show_maps_applies_colours_by_default_and_can_disable_them(terrain_run, monkeypatch):
    o3d = pytest.importorskip('open3d')
    from experiments.handheld_mapping.viewer import show_maps
    path = terrain_run/'dense_offline/map_0_mesh.ply'
    geometry = o3d.geometry.TriangleMesh.create_box(.5, .5, .3).translate([.1, .1, .03])
    o3d.io.write_triangle_mesh(str(path), geometry)
    added, titles = [], []
    class Viewer:
        def create_window(self, **kwargs): titles.append(kwargs['window_name']); return True
        def add_geometry(self, geometry): added.append(geometry); return True
        def get_render_option(self): return SimpleNamespace()
        def get_view_control(self):
            return SimpleNamespace(**{name: lambda value: None for name in ('set_lookat', 'set_up', 'set_front', 'set_zoom')})
        def poll_events(self): return False
        def update_renderer(self): pass
        def destroy_window(self): pass
    monkeypatch.setattr(o3d.visualization, 'Visualizer', Viewer)
    show_maps([('Map 0', path)], trajectory=False)
    assert len(np.unique(np.asarray(added[-1].vertex_colors), axis=0)) >= 2
    assert '초록: 지면' in titles[-1]
    show_maps([('Map 0', path)], trajectory=False, terrain_colors=False)
    assert len(np.unique(np.asarray(added[-1].vertex_colors), axis=0)) == 1


def test_obstacle_display_keeps_source_geometry_terrain_and_classification(terrain_run):
    import open3d as o3d
    from test_obstacle_display import noisy_surface
    path = terrain_run/'dense_offline/map_0_mesh.ply'
    geometry, _ = noisy_surface()
    points = np.asarray(geometry.vertices).copy()
    points[:, :2] += .05
    points[:, 2] += .2
    geometry.vertices = o3d.utility.Vector3dVector(points)
    o3d.io.write_triangle_mesh(str(path), geometry)
    terrain_path = terrain_run/'terrain/orb_offline/map_0/terrain.npz'
    before = {p: p.read_bytes() for p in (path, terrain_path)}
    terrain = load_surface_layers(path)
    labels = classify_surface_points(points, terrain)
    stats = apply_surface_colors(geometry, path, smooth_obstacles=True)
    assert stats['obstacle_smoothing']['smoothed_vertices'] > 0
    np.testing.assert_array_equal(np.asarray(geometry.vertex_colors), PALETTE[labels])
    np.testing.assert_array_equal(np.asarray(geometry.vertices)[labels != OBSTACLE], points[labels != OBSTACLE])
    assert before == {p: p.read_bytes() for p in before}


def test_green_gap_fill_is_display_only_and_can_be_disabled(terrain_run):
    import copy
    import open3d as o3d
    from test_ground_display import ground_with_hole
    geometry, _ = ground_with_hole()
    path = terrain_run/'dense_offline/map_0_mesh.ply'
    o3d.io.write_triangle_mesh(str(path), geometry)
    terrain_path = terrain_run/'terrain/orb_offline/map_0/terrain.npz'
    before = {p: p.read_bytes() for p in (path, terrain_path)}
    original_faces = np.asarray(geometry.triangles).copy()
    disabled = copy.deepcopy(geometry)
    off = apply_surface_colors(disabled, path, fill_ground_gaps=False)
    assert off['ground_gap_fill'] is None
    np.testing.assert_array_equal(disabled.triangles, original_faces)
    on = apply_surface_colors(geometry, path, smooth_obstacles=True, fill_ground_gaps=True)
    assert on['ground_gap_fill']['filled_holes'] == 1
    assert len(geometry.triangles) == len(original_faces)+10
    assert np.all(np.asarray(geometry.vertex_colors) == PALETTE[GROUND])
    assert before == {p: p.read_bytes() for p in before}


@pytest.mark.parametrize('angle', [10., 15., 25., 35.])
def test_flat_ramp_upper_flat_stays_ground_while_object_on_ramp_is_obstacle(angle):
    from experiments.handheld_mapping.ground import GroundGrid
    from experiments.handheld_mapping.terrain import GridSpec
    from experiments.handheld_mapping.costmap import RoverLimits, build_costmap
    slope = np.tan(np.deg2rad(angle))
    x, y = np.meshgrid(np.arange(.005, 3.2, .01), np.arange(.005, 1.6, .01))
    z = slope*np.clip(x-.8, 0., 1.2)
    floor = np.c_[x.ravel(), y.ravel(), z.ravel()]
    obj = floor[(floor[:, 0] > 1.3) & (floor[:, 0] < 1.5)
                & (floor[:, 1] > .65) & (floor[:, 1] < .95)].copy()
    obj[:, 2] += .2
    grid = GroundGrid(GridSpec((0., 3.2, 0., 1.6), .05))
    for stamp, cx in enumerate((.2, 1.2, 2.5), 1):
        grid.update(np.r_[floor, obj], stamp, [cx, .8, .4+slope*np.clip(cx-.8, 0., 1.2)])
    layers = grid.layers(2)
    terrain = {'layers': layers, 'bounds': np.array(grid.spec.bounds), 'resolution': .05,
               'grid_from_geometry': np.eye(4), 'ground_tolerance_m': .02, 'obstacle_threshold_m': .06}
    floor_labels = classify_surface_points(floor, terrain)
    # Include the transition edges and elevated plateau; absolute height is not an obstacle.
    assert not np.any(floor_labels == OBSTACLE)
    assert np.mean(floor_labels == GROUND) > .95
    assert np.all(classify_surface_points(obj, terrain) == OBSTACLE)
    cost = build_costmap(layers, .05, RoverLimits(safety_radius_m=0.))
    # The same steep ground can be green in 3D and blocked for traversal in the cost layer.
    if angle > 20.:
        assert cost[8, 22] == 254
        assert floor_labels[(floor[:, 0] > 1.05) & (floor[:, 0] < 1.15)].tolist().count(OBSTACLE) == 0
    else:
        assert cost[8, 22] < 254
