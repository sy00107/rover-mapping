"""거리별 실제 TSDF/지형·표시의 일관성과 원본·기존 결과 보존을 검증한다."""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from test_core import make_run
from experiments.handheld_mapping.depth_variants import ensure_depth_variant
from experiments.handheld_mapping.gravity import fingerprint
from experiments.handheld_mapping.viewer import select_maps, load_trajectory


@pytest.fixture
def mixed_run(tmp_path):
    run = make_run(tmp_path/'run')
    depth = np.tile(np.repeat(np.array([700, 1700, 2700], np.uint16), [26, 27, 27]), (60, 1))
    cv2.imwrite(str(run/'raw/depth/1000000000.png'), depth)
    original = run/'dense'
    original.mkdir()
    (original/'summary.json').write_text('{"untouched_original":true}')
    return run


@pytest.mark.parametrize('limit,maximum', [(1., .7), (2., 1.7), (3., 2.7)])
def test_actual_tsdf_uses_requested_range_and_preserves_original_outputs(mixed_run, limit, maximum):
    import open3d as o3d
    before = {p: fingerprint(mixed_run/p) for p in ('capture_calibration.json', 'frames.jsonl',
              'orb/map_0_keyframes.csv', 'raw/depth/1000000000.png', 'dense/summary.json')}
    output = ensure_depth_variant(mixed_run, limit)
    assert output.name == f'depth_{int(limit)}m'
    path = select_maps(output, cloud=True)[0][1]
    points = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    assert len(points) > 100
    assert points[:, 2].max() == pytest.approx(maximum, abs=.025)
    assert np.all(points[:, 2] <= limit+.025)
    dense = json.loads((output/'dense/summary.json').read_text())
    assert dense['depth_max_m'] == dense['depth_filter']['max_depth_m'] == limit
    assert load_trajectory(path) == [(1., 0., 0.)]
    assert before == {p: fingerprint(mixed_run/p) for p in before}
    summary_before = fingerprint(output/'dense/summary.json')
    assert ensure_depth_variant(mixed_run, limit) == output
    assert fingerprint(output/'dense/summary.json') == summary_before


def test_changed_inputs_create_new_revision_and_keep_old_trajectory_snapshot(mixed_run):
    first = ensure_depth_variant(mixed_run, 1.)
    first_path = select_maps(first)[0][1]
    geometry_hash = fingerprint(first_path)
    source = mixed_run/'orb/map_0_keyframes.csv'
    source.write_text(source.read_text().replace('1.0,1,0,0', '1.0,2,0,0'))
    second = ensure_depth_variant(mixed_run, 1.)
    assert second.name == 'depth_1m_v2'
    assert load_trajectory(first_path) == [(1., 0., 0.)]
    assert load_trajectory(select_maps(second)[0][1]) == [(2., 0., 0.)]
    assert fingerprint(first_path) == geometry_hash


def test_terrain_projection_and_filter_obey_same_limit(tmp_path):
    from experiments.handheld_mapping.terrain_replay import replay, project_depth
    from experiments.handheld_mapping.terrain import GridSpec
    run = make_run(tmp_path/'run')
    info = json.loads((run/'capture_calibration.json').read_text())['depth']
    depth = np.tile(np.repeat(np.array([700, 1700, 2700], np.uint16), [26, 27, 27]), (60, 1))
    cv2.imwrite(str(run/'raw/depth/1000000000.png'), depth)
    for limit, expected in ((1., 1560), (2., 3180), (3., 4800)):
        points = project_depth(depth, info, np.eye(4), stride=1, depth_max=limit)
        assert len(points) == expected
        assert points[:, 2].max() <= limit
        output = tmp_path/f'terrain_{int(limit)}'
        summary = replay(run, output, 0, GridSpec((-5., 5., -5., 5.), .1), np.eye(4),
                         depth_stride=1, depth_max=limit)
        assert summary['depth_max_m'] == summary['depth_filter']['max_depth_m'] == limit
        with np.load(output/'terrain.npz', allow_pickle=False) as layers:
            reference = layers['reference_ground_height_m']
            assert np.isnan(reference).all() or np.nanmax(reference) <= limit


@pytest.mark.parametrize('limit', [float('nan'), float('inf'), -.5, .25, .3, 11.])
def test_invalid_range_fails_before_creating_output(mixed_run, limit):
    with pytest.raises(ValueError, match='depth-max'):
        ensure_depth_variant(mixed_run, limit)
    assert not (mixed_run/'depth_variants').exists()


def test_failure_preserves_original_and_is_not_cached(mixed_run):
    depth = np.full((60, 80), 2000, np.uint16)
    cv2.imwrite(str(mixed_run/'raw/depth/1000000000.png'), depth)
    with pytest.raises(ValueError, match='matching|일치'):
        ensure_depth_variant(mixed_run, 1.)
    failed = mixed_run/'depth_variants/orb/depth_1m/depth_variant.json'
    assert json.loads(failed.read_text())['status'] == 'failed'
    assert json.loads((mixed_run/'dense/summary.json').read_text())['untouched_original']


def test_cli_build_only_never_opens_a_window(mixed_run, monkeypatch):
    from experiments.handheld_mapping import viewer
    monkeypatch.setattr('sys.argv', ['viewer', str(mixed_run), '--depth-max', '1', '--build-only'])
    def unexpected(*args, **kwargs):
        raise AssertionError('GUI opened during build-only')
    monkeypatch.setattr(viewer, 'show_maps', unexpected)
    viewer.main()
    assert (mixed_run/'depth_variants/orb/depth_1m/depth_variant.json').is_file()


def test_cleanup_is_separate_cached_geometry_and_terrain_uses_the_same_filter(mixed_run):
    from experiments.handheld_mapping.depth_filter import DepthFilter
    from experiments.handheld_mapping.terrain_replay import replay
    from experiments.handheld_mapping.terrain import GridSpec
    # Three measured frames let far surfaces obtain two reference matches.
    source = json.loads((mixed_run/'frames.jsonl').read_text().splitlines()[0])
    records = []
    depth = cv2.imread(str(mixed_run/source['path']), cv2.IMREAD_UNCHANGED)
    for i in range(3):
        row = {**source, 'stamp_ns': 1_000_000_000+i*100_000_000,
               'path': f'raw/depth/{1_000_000_000+i*100_000_000}.png'}
        cv2.imwrite(str(mixed_run/row['path']), depth)
        records.append(json.dumps(row))
    (mixed_run/'frames.jsonl').write_text('\n'.join(records)+'\n')
    (mixed_run/'orb/map_0_keyframes.csv').write_text(
        'map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n'+
        ''.join(f'0,{i},{1+i*.1:.1f},1,0,0,0,0,0,1\n' for i in range(3)))
    before = {str(p): fingerprint(p) for p in (mixed_run/'raw/depth').glob('*.png')}
    baseline = ensure_depth_variant(mixed_run, 2.)
    baseline_hash = fingerprint(select_maps(baseline)[0][1])
    clean = ensure_depth_variant(mixed_run, 2., surface_cleanup=True)
    assert clean.name == 'depth_2m_clean' and clean != baseline
    assert ensure_depth_variant(mixed_run, 2., surface_cleanup=True) == clean
    assert fingerprint(select_maps(baseline)[0][1]) == baseline_hash
    assert before == {name: fingerprint(Path(name)) for name in before}
    dense = json.loads((clean/'dense/summary.json').read_text())
    assert dense['maps'][0]['surface_cleanup']['new_faces_or_bridges'] is False
    terrain = replay(mixed_run, mixed_run/'clean_terrain', 0, GridSpec((-5., 5., -5., 5.), .1),
                     np.eye(4), depth_max=2., depth_filter=DepthFilter(max_depth_m=2., surface_cleanup=True))
    assert terrain['depth_filter'] == dense['depth_filter']
    assert terrain['depth_filter_stats'] == dense['maps'][0]['depth_filter_stats']


def test_variant_can_load_its_own_terrain_and_snapshot_sources(terrain_run):
    # Reuse the known classified ramp fixture as a small independent variant layout.
    import shutil
    from experiments.handheld_mapping.surface_colors import load_surface_layers
    variant = terrain_run/'depth_variants/orb_offline/depth_1m'
    variant.mkdir(parents=True)
    for folder in ('dense_offline', 'orb_offline', 'terrain'):
        shutil.copytree(terrain_run/folder, variant/folder)
    shutil.copy2(terrain_run/'capture_calibration.json', variant/'capture_calibration.json')
    (variant/'depth_variant.json').write_text(json.dumps({'schema': 1, 'status': 'complete',
        'backend': 'orb_offline', 'source_run': str(terrain_run)}))
    layers = load_surface_layers(variant/'dense_offline/map_0_mesh.ply')
    assert Path(layers['output_dir']).is_relative_to(variant)
    assert len(load_trajectory(variant/'dense_offline/map_0_mesh.ply')) == 2


from test_surface_colors import terrain_run
