# ================================================================
# test_viewer.py
# 설명: 실제 지도 선택의 backend·map 분리·최신 미완료·경로 검증을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_viewer.py -q
# ================================================================
import json
from pathlib import Path
import pytest


def maps(run: Path,folder='dense_offline'):
    output=run/folder; output.mkdir(parents=True)
    rows=[]
    for map_id in (0,421):
        mesh=f'{folder}/map_{map_id}_mesh.ply'
        cloud=f'{folder}/map_{map_id}_cloud.ply'
        (run/mesh).touch(); (run/cloud).touch()
        rows.append({'map_id':map_id,'mesh':mesh,'cloud':cloud})
    (output/'summary.json').write_text(json.dumps({'maps':rows,'status':'partial'}))
    return run


def test_viewer_defaults_to_latest_session_and_preserves_multiple_maps(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    maps(tmp_path/'001'); latest=maps(tmp_path/'002')
    selected=select_maps(None,sessions=tmp_path)
    assert len(selected)==2
    assert selected[0]==('Map 0',(latest/'dense_offline/map_0_mesh.ply').resolve())
    assert selected[1][0]=='Map 421'
    (tmp_path/'003').mkdir()
    with pytest.raises(FileNotFoundError,match='아직|없'):
        select_maps(None,sessions=tmp_path)


def test_viewer_explicit_backend_map_cloud_and_direct_ply(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    run=maps(tmp_path/'run'); maps(run,'dense_orbline')
    selected=select_maps(run,backend='orbline',map_id=421,cloud=True)
    assert selected==[('Map 421',(run/'dense_orbline/map_421_cloud.ply').resolve())]
    direct=run/'dense_offline/map_0_mesh.ply'
    assert select_maps(direct)==[(direct.name,direct.resolve())]
    with pytest.raises(ValueError):select_maps(direct,map_id=0)
    with pytest.raises(ValueError):select_maps(run,map_id=99)
    with pytest.raises(ValueError):select_maps(run,backend='unknown')


def test_auto_prefers_camera_maps_over_partial_imu_and_allows_explicit_imu(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    run = maps(tmp_path/'run', 'dense_inertial')
    maps(run, 'dense')
    assert all(p.parent.name == 'dense' for _, p in select_maps(run))
    maps(run, 'dense_offline')
    assert all(p.parent.name == 'dense_offline' for _, p in select_maps(run))
    assert all(p.parent.name == 'dense_inertial' for _, p in select_maps(run, backend='orb_inertial'))


def test_imu_only_recording_is_still_viewable(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    run = maps(tmp_path/'run', 'dense_inertial')
    assert all(p.parent.name == 'dense_inertial' for _, p in select_maps(run))


def test_viewer_rejects_summary_path_escape_and_missing_geometry(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    run=maps(tmp_path/'run')
    summary=run/'dense_offline/summary.json'
    value=json.loads(summary.read_text())
    value['maps'][0]['mesh']='../../escape.ply'
    summary.write_text(json.dumps(value))
    with pytest.raises(ValueError):select_maps(run)
    run=maps(tmp_path/'other')
    (run/'dense_offline/map_421_mesh.ply').unlink()
    with pytest.raises(FileNotFoundError):select_maps(run)


@pytest.mark.parametrize('summary',[[],{'maps':None},{'maps':[{}]},
                                     {'maps':[{'map_id':0}]},{'maps':[{'map_id':0,'mesh':None}]}])
def test_malformed_saved_summary_is_a_user_facing_value_error(tmp_path,summary):
    from experiments.handheld_mapping.viewer import select_maps
    run=maps(tmp_path/'run')
    (run/'dense_offline/summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError):select_maps(run)


def keyframes(run,backend='orb_offline',map_id=0,rows=None):
    folder=run/backend; folder.mkdir(exist_ok=True)
    source=folder/f'map_{map_id}_keyframes.csv'
    source.write_text('map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n'+
                      (rows or f'{map_id},2,2,1,2,3,0,0,0,1\n{map_id},0,1,0,0,0,0,0,0,1\n'))
    return source


def test_trajectory_uses_matching_final_backend_map_and_timestamp_order(tmp_path):
    from experiments.handheld_mapping.viewer import load_trajectory
    run=maps(tmp_path/'run'); maps(run,'dense_orbline')
    keyframes(run)
    keyframes(run,map_id=421,rows='421,0,1,9,8,7,0,0,0,1\n')
    keyframes(run,backend='orbline',rows='0,0,1,6,5,4,0,0,0,1\n')
    assert load_trajectory(run/'dense_offline/map_0_mesh.ply')==[(0.,0.,0.),(1.,2.,3.)]
    assert load_trajectory(run/'dense_offline/map_421_cloud.ply')==[(9.,8.,7.)]
    assert load_trajectory(run/'dense_orbline/map_0_mesh.ply')==[(6.,5.,4.)]


@pytest.mark.parametrize('row',[
    '421,0,1,0,0,0,0,0,0,1\n',
    '0,0,1,nan,0,0,0,0,0,1\n',
    '0,0,inf,0,0,0,0,0,0,1\n',
    '0,0,1,broken,0,0,0,0,0,1\n',
])
def test_trajectory_rejects_mixed_map_and_invalid_poses(tmp_path,row):
    from experiments.handheld_mapping.viewer import load_trajectory
    run=maps(tmp_path/'run'); keyframes(run,rows=row)
    with pytest.raises(ValueError):load_trajectory(run/'dense_offline/map_0_mesh.ply')


def test_missing_final_trajectory_never_falls_back_to_live_or_other_backend(tmp_path):
    from experiments.handheld_mapping.viewer import load_trajectory
    run=maps(tmp_path/'run')
    keyframes(run,backend='orb_live')
    keyframes(run,backend='orbline')
    assert load_trajectory(run/'dense_offline/map_0_mesh.ply')==[]
    assert load_trajectory(tmp_path/'arbitrary.ply')==[]
    source=keyframes(run); source.write_text('wrong,columns\n')
    with pytest.raises(ValueError):load_trajectory(run/'dense_offline/map_0_mesh.ply')


def test_trajectory_geometry_preserves_3d_positions_and_endpoint_colors():
    o3d=pytest.importorskip('open3d')
    import numpy as np
    from experiments.handheld_mapping.viewer import trajectory_geometries
    points=[(0.,1.,2.),(3.,4.,5.),(6.,7.,8.)]
    line,start,end=trajectory_geometries(points)
    assert isinstance(line,o3d.geometry.LineSet)
    np.testing.assert_allclose(np.asarray(line.points),points)
    np.testing.assert_array_equal(np.asarray(line.lines),[[0,1],[1,2]])
    np.testing.assert_allclose(start.get_center(),points[0],atol=1e-12)
    np.testing.assert_allclose(end.get_center(),points[-1],atol=1e-12)
    assert np.asarray(start.vertex_colors)[0,1] > np.asarray(start.vertex_colors)[0,0]
    assert np.asarray(end.vertex_colors)[0,0] > np.asarray(end.vertex_colors)[0,1]
    assert trajectory_geometries([])==[]
    assert len(trajectory_geometries(points[:1]))==1


@pytest.mark.parametrize('trajectory,expected_count',[(True,4),(False,1)])
def test_viewer_adds_trajectory_to_same_window_and_can_hide_it(tmp_path,monkeypatch,trajectory,expected_count):
    o3d=pytest.importorskip('open3d')
    from types import SimpleNamespace
    from experiments.handheld_mapping.viewer import show_maps
    run=maps(tmp_path/'run'); keyframes(run)
    path=run/'dense_offline/map_0_mesh.ply'
    o3d.io.write_triangle_mesh(str(path),o3d.geometry.TriangleMesh.create_box())
    added=[]
    class Viewer:
        def create_window(self,**kwargs):return True
        def add_geometry(self,geometry):added.append(geometry); return True
        def get_render_option(self):return SimpleNamespace()
        def get_view_control(self):
            return SimpleNamespace(**{name:lambda value:None for name in ('set_lookat','set_up','set_front','set_zoom')})
        def poll_events(self):return False
        def update_renderer(self):pass
        def destroy_window(self):pass
    monkeypatch.setattr(o3d.visualization,'Visualizer',Viewer)
    show_maps([('Map 0',path)],trajectory=trajectory,path_measurements=False)
    assert len(added)==expected_count
    if trajectory:assert isinstance(added[1],o3d.geometry.LineSet)
