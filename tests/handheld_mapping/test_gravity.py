"""정지 중력 기준·map 분리·실제 TSDF/경로 좌표의 일치를 검사한다."""
import json
from pathlib import Path

import numpy as np
import pytest

from experiments.handheld_mapping.gravity import (alignment_matrix,derive_alignment,fingerprint,
                                                 load_alignment,mounting,stationary_reference,
                                                 stationary_anchor_reference)
from test_core import make_run


def samples(center=3_000_000_000,up=(0.,0.,1.)):
    return [{'pi_ns':center+offset,'segment':0,'acc_g':list(up),'gyro_dps':[0.,0.,0.]}
            for offset in range(-1_000_000_000,1_000_000_001,20_000_000)]


@pytest.fixture
def gravity_run(tmp_path):
    run=make_run(tmp_path/'run')
    root=Path(__file__).resolve().parents[2]
    (run/'robot.urdf').write_bytes((root/'ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf').read_bytes())
    axes=[[0,-1],[1,-1],[2,1]]
    (run/'imu_calibration.json').write_text(json.dumps({'chip_to_body':axes}))
    (run/'imu_manifest.json').write_text(json.dumps({
        'run_id':run.name,'frame':'rover_body','chip_to_body_actual':axes,
        'calibration_sha256':fingerprint(run/'imu_calibration.json'),
        'urdf_sha256':fingerprint(run/'robot.urdf')}))
    calibration=json.loads((run/'capture_calibration.json').read_text())
    calibration['stereo']={'source_camera_info':{'left':{'r':np.eye(3).ravel().tolist()}}}
    (run/'capture_calibration.json').write_text(json.dumps(calibration))
    with (run/'frames.jsonl').open('a') as output:
        output.write(json.dumps({'stream':'left','stamp_ns':1_000_000_000,
                                 'pi_receipt_monotonic_ns':3_000_000_000})+'\n')
    with (run/'imu.jsonl').open('w') as output:
        for row in samples():
            output.write(json.dumps({'acceptance':'accepted',
                         'clock':{'pi_receipt_monotonic_ns':row['pi_ns'],'segment':row['segment']},
                         'calibrated':{'acc_g':row['acc_g'],'gyro_dps':row['gyro_dps']}})+'\n')
    return run


def test_stationary_body_axes_are_not_flipped_again():
    up=np.array([.2,-.1,1.]); up/=np.linalg.norm(up)
    result=stationary_reference(samples(up=up),3_000_000_000)
    np.testing.assert_allclose(result['body_up'],up)
    assert result['samples']==101


@pytest.mark.parametrize('problem',['rotation','acceleration','gap','reset','nonfinite'])
def test_motion_gaps_and_reset_cannot_define_gravity(problem):
    rows=samples()
    if problem=='rotation':rows[50]['gyro_dps']=[0.,0.,10.]
    if problem=='acceleration':rows[50]['acc_g']=[.2,0.,1.]
    if problem=='gap':del rows[20:40]
    if problem=='reset':rows[-1]['segment']=1
    if problem=='nonfinite':rows[50]['acc_g']=[0.,0.,float('nan')]
    with pytest.raises(ValueError):stationary_reference(rows,3_000_000_000)


@pytest.mark.parametrize('placement,direction', [('forward', 1), ('backward', -1)])
def test_boundary_reference_uses_full_stationary_window_touching_anchor(placement, direction):
    anchor = 3_000_000_000
    rows = samples(center=anchor-direction*1_000_000_000)
    rows += samples(center=anchor+direction*1_000_000_000)
    rows = sorted({row['pi_ns']:row for row in rows}.values(),key=lambda row:row['pi_ns'])
    for row in rows:
        if (row['pi_ns']-anchor)*direction < 0:
            row['gyro_dps'] = [0.,0.,10.]
    with pytest.raises(ValueError):stationary_reference(rows,anchor)
    reference = stationary_anchor_reference(rows,anchor)
    assert reference['window_placement'] == placement
    assert reference['samples'] == 101
    assert reference['anchor_pi_ns'] == anchor
    assert reference['max_gyro_dps'] == 0.
    assert reference['end_pi_ns']-reference['start_pi_ns'] == 2_000_000_000


@pytest.mark.parametrize('problem',['rotation','acceleration','gap','reset','too_short','detached'])
def test_boundary_reference_does_not_bypass_stationarity_or_coverage(problem):
    anchor = 3_000_000_000
    rows = samples(center=anchor+1_000_000_000)
    if problem=='rotation':rows[0]['gyro_dps']=[0.,0.,10.]
    elif problem=='acceleration':rows[0]['acc_g']=[.2,0.,1.]
    elif problem=='gap':del rows[20:40]
    elif problem=='reset':rows[-1]['segment']=1
    elif problem=='too_short':rows=rows[:60]
    elif problem=='detached':
        for row in rows:row['pi_ns']+=3_000_000_000
    with pytest.raises(ValueError):stationary_anchor_reference(rows,anchor)


def test_offline_gravity_recovers_capture_start_without_changing_pose_or_imu(gravity_run):
    path=gravity_run/'imu.jsonl'
    data=[]
    for row in samples(center=3_000_000_000)+samples(center=4_000_000_000):
        if row['pi_ns']<3_000_000_000:row['gyro_dps']=[0.,0.,10.]
        data.append({'acceptance':'accepted','clock':{'pi_receipt_monotonic_ns':row['pi_ns'],'segment':0},
                     'calibrated':{'acc_g':row['acc_g'],'gyro_dps':row['gyro_dps']}})
    data=sorted({row['clock']['pi_receipt_monotonic_ns']:row for row in data}.values(),
                key=lambda row:row['clock']['pi_receipt_monotonic_ns'])
    path.write_text(''.join(json.dumps(row)+'\n' for row in data))
    before={p:p.read_bytes() for p in [path,gravity_run/'orb/map_0_keyframes.csv']}
    result=derive_alignment(gravity_run,'orb',0)
    assert result['stationary_reference']['window_placement']=='forward'
    assert result['anchor_keyframe_id']==0
    assert load_alignment(gravity_run,'orb',0)==result
    assert all(p.read_bytes()==content for p,content in before.items())


def test_gravity_alignment_uses_tilted_rover_and_camera_mount(gravity_run):
    from scipy.spatial.transform import Rotation
    left_body=mounting(gravity_run)
    earth_body=np.eye(4)
    earth_body[:3,:3]=Rotation.from_euler('xyz',[8.,-15.,35.],degrees=True).as_matrix()
    earth_left=earth_body@np.linalg.inv(left_body)
    body_up=earth_body[:3,:3].T@np.array([0.,0.,1.])
    matrix=alignment_matrix(earth_left,left_body,body_up)
    np.testing.assert_allclose(matrix[:3,:3]@np.array([0.,0.,1.]),[0.,0.,1.],atol=1e-12)
    np.testing.assert_allclose((matrix@earth_left)[:3,3],0.,atol=1e-12)
    assert np.linalg.det(matrix[:3,:3])==pytest.approx(1.)


def test_final_gravity_reference_is_bound_to_run_map_and_source(gravity_run):
    data=derive_alignment(gravity_run,'orb',0)
    assert data['coordinate_system']=='gravity_z_up'
    assert data['orb_fused'] is False and data['imu_clock_verified'] is False
    assert data['extrinsic_verified'] is False
    assert load_alignment(gravity_run,'orb',0)==data
    assert load_alignment(gravity_run,'orb',7) is None
    assert load_alignment(gravity_run,'orb_offline',0) is None
    source=gravity_run/'orb/map_0_keyframes.csv'
    source.write_text(source.read_text().replace('1,0,0,0,0,0,1','2,0,0,0,0,0,1'))
    with pytest.raises(ValueError,match='변경'):load_alignment(gravity_run,'orb',0)


def test_gravity_refuses_wrong_axes_or_missing_receipts(gravity_run):
    manifest=gravity_run/'imu_manifest.json'
    data=json.loads(manifest.read_text()); data['chip_to_body_actual']=[[0,1],[1,1],[2,1]]
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='축'):derive_alignment(gravity_run,'orb',0)
    data['chip_to_body_actual']=[[0,-1],[1,-1],[2,1]]; manifest.write_text(json.dumps(data))
    path=gravity_run/'frames.jsonl'
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    for row in rows:row.pop('pi_receipt_monotonic_ns',None)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError):derive_alignment(gravity_run,'orb',0)


def test_gravity_tsdf_and_camera_path_use_same_fixed_transform(gravity_run):
    import open3d as o3d
    from experiments.handheld_mapping.reconstruct import reconstruct
    from experiments.handheld_mapping.viewer import load_trajectory,map_frame
    data=derive_alignment(gravity_run,'orb',0)
    matrix=np.array(data['world_from_slam'])
    result=reconstruct(gravity_run,voxel_size=.025)
    row=result['maps'][0]
    assert row['coordinate_system']=='gravity_z_up'
    path=gravity_run/row['cloud']
    cloud=o3d.io.read_point_cloud(str(path))
    original=np.asarray(cloud.points)@matrix[:3,:3]-matrix[:3,3]@matrix[:3,:3]
    assert np.median(original[:,2])==pytest.approx(2.,abs=.03)
    assert np.median(original[:,0])==pytest.approx(1.2,abs=.06)
    np.testing.assert_allclose(load_trajectory(path),[[0.,0.,0.]],atol=1e-12)
    frame,aligned=map_frame(path)
    assert aligned
    np.testing.assert_allclose(frame,matrix)


def test_live_reference_stays_fixed_when_rover_climbs_and_isolated_per_map(gravity_run):
    from experiments.handheld_mapping.coverage import CoverageGrid
    from experiments.handheld_mapping.live import LiveMapper
    from scipy.spatial.transform import Rotation
    mapper=LiveMapper.__new__(LiveMapper)
    mapper.gravity_provider=lambda:samples()
    mapper.gravity_frames={}; mapper.gravity_error=None
    mapper.left_body=mounting(gravity_run)
    mapper.grid=CoverageGrid(); mapper.folder=gravity_run/'orb'
    first=mapper._gravity_pose(np.eye(4),0,4_000_000_000)
    assert first is not None and mapper.grid.gravity_aligned
    fixed=mapper.gravity_frames[0].copy()
    moved=np.eye(4); moved[:3,:3]=Rotation.from_euler('x',15,degrees=True).as_matrix()
    moved[:3,3]=[0.,-.3,1.]
    mapper.gravity_provider=lambda:[]
    second=mapper._gravity_pose(moved,0,9_000_000_000)
    np.testing.assert_allclose(second,fixed@moved)
    np.testing.assert_allclose(mapper.gravity_frames[0],fixed)
    assert mapper._gravity_pose(moved,7,9_000_000_000) is None


def test_gravity_coverage_uses_horizontal_xy_instead_of_camera_xz():
    from experiments.handheld_mapping.coverage import CoverageGrid
    pose=np.eye(4); pose[:3,3]=[1.,2.,9.]
    grid=CoverageGrid(gravity_aligned=True)
    grid.update_pose(0,pose)
    assert grid.snapshot()['path']==[(1.,2.)]
    assert grid.snapshot()['coordinate_system']=='gravity_z_up_XY'


def test_ramp_slope_survives_camera_and_rover_pitch_changes(gravity_run):
    """세 카메라 자세로 관측한 같은 15도 경사면이 평지로 펴지지 않는지 검사한다."""
    from scipy.spatial.transform import Rotation
    from experiments.handheld_mapping.terrain import GridSpec,TerrainGrid
    left_body=mounting(gravity_run)
    first_left=np.linalg.inv(left_body)
    first_left[:3,3]=[0.,0.,.5]
    slam_from_earth=np.linalg.inv(first_left)
    reference=alignment_matrix(np.eye(4),left_body,[0.,0.,1.])
    x,y=np.meshgrid(np.arange(.05,1.05,.05),np.arange(-.45,.5,.05))
    earth=np.c_[x.ravel(),y.ravel(),np.tan(np.deg2rad(15))*x.ravel()]
    grid=TerrainGrid(GridSpec((-1.,2.,-1.,1.),.05))
    for i,pitch in enumerate((0.,-10.,-15.)):
        earth_left=np.eye(4)
        earth_left[:3,:3]=Rotation.from_euler('y',pitch,degrees=True).as_matrix()@first_left[:3,:3]
        earth_left[:3,3]=[i*.1,0.,.5+i*.03]
        camera=(earth-earth_left[:3,3])@earth_left[:3,:3]
        slam_left=slam_from_earth@earth_left
        world_left=reference@slam_left
        points=camera@world_left[:3,:3].T+world_left[:3,3]
        grid.update(points,i+1)
    layers=grid.layers(2)
    assert np.count_nonzero(layers['valid'])>50
    assert np.nanmedian(layers['slope_deg'])==pytest.approx(15.,abs=.5)


def test_body_incline_events_have_hysteresis_and_do_not_switch_estimator():
    from scipy.spatial.transform import Rotation
    from experiments.handheld_mapping.gravity import BodyIncline
    monitor=BodyIncline()
    def pose(angle):
        result=np.eye(4); result[:3,:3]=Rotation.from_euler('y',angle,degrees=True).as_matrix()
        return result
    for stamp in (0,400_000_000,800_000_000):
        assert monitor.update(0,stamp,pose(10.))['body_tilted'] is False
    state=monitor.update(0,1_200_000_000,pose(10.))
    assert state['event']=='body_tilt_enter' and state['body_tilted']
    assert state['terrain_slope'] is False
    assert monitor.update(0,1_400_000_000,pose(4.))['body_tilted']
    for stamp in (1_600_000_000,2_000_000_000,2_400_000_000):
        assert monitor.update(0,stamp,pose(0.))['body_tilted']
    assert monitor.update(0,2_800_000_000,pose(0.))['event']=='body_tilt_exit'
    assert monitor.update(7,2_900_000_000,pose(0.))['body_tilted'] is False


def test_terrain_outputs_keep_gravity_provenance_and_preserve_earlier_revision(gravity_run):
    from experiments.handheld_mapping.gravity import process_maps,terrain_maps
    reference=process_maps(gravity_run,'orb')
    assert reference['status']=='complete'
    first=terrain_maps(gravity_run,'orb',reference)
    assert first['status']=='complete',first
    output=Path(first['maps'][0]['output_dir'])
    summary=json.loads((output/'summary.json').read_text())
    assert 'estimated gravity' in summary['coordinate_system']
    assert summary['alignment_metadata']['extrinsic_verified'] is False
    assert (output/'terrain.npz').is_file() and (output/'preview.png').is_file()
    original=(output/'terrain.npz').read_bytes()
    second=terrain_maps(gravity_run,'orb',reference)
    assert second['status']=='complete'
    assert second['maps'][0]['output_dir']!=str(output)
    assert (output/'terrain.npz').read_bytes()==original


def test_qt_gravity_is_automatic_for_real_capture(tmp_path, monkeypatch):
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping import app as mapping_app
    from experiments.handheld_mapping.controller import Session
    app = QApplication.instance() or QApplication([])
    session = Session()
    settings = []
    monkeypatch.setattr(session, 'start', settings.append)
    window = mapping_app.MappingWindow(session, output_root=tmp_path)
    try:
        window.start()
        assert settings[-1]['gravity_enabled'] is True
        assert settings[-1]['imu_enabled'] is True
        window.demo = True
        window.start()
        assert settings[-1]['gravity_enabled'] is False
        assert settings[-1]['imu_enabled'] is False
        app.processEvents()
    finally:
        window.close()


def test_failed_reference_is_not_reused_during_partial_reconstruction(gravity_run):
    from experiments.handheld_mapping.reconstruct import reconstruct
    derive_alignment(gravity_run,'orb',0)
    # 정지 자료가 바뀌어 기존 기준을 쓸 수 없는 경우에는 실패한 map에 적용하지 않는다.
    with (gravity_run/'imu.jsonl').open('a') as output:output.write('\n')
    with pytest.raises(ValueError):reconstruct(gravity_run)
    result=reconstruct(gravity_run,gravity_alignments={})
    assert result['maps'][0]['coordinate_system']=='initial_camera_optical'
