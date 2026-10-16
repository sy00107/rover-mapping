# ================================================================
# test_inertial.py
# 설명: 실험용 관성 설정·시간 재매핑·실제 초기화 판정을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_inertial.py -q
# ================================================================
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
import pytest


def test_camera_to_imu_uses_sensor_origin_and_already_body_axes(tmp_path: Path):
    from experiments.handheld_mapping.inertial import imu_from_camera
    urdf = tmp_path/'robot.urdf'
    urdf.write_text('''<robot><joint name="base_to_imu" type="fixed"><parent link="base_link"/>
      <child link="imu_link"/><origin xyz="-0.02 0 -0.03" rpy="0 0 0"/></joint></robot>''')
    camera_from_base = np.eye(4)
    camera_from_base[:3, 3] = [-.12, -.01, -.02]
    actual = imu_from_camera(urdf, camera_from_base)
    assert np.allclose(actual[:3, 3], [.14, .01, .05])
    assert np.allclose(actual[:3, :3], np.eye(3))


def test_non_body_calibration_cannot_be_silently_rotated_twice(tmp_path: Path):
    from experiments.handheld_mapping.inertial import imu_from_camera
    urdf=tmp_path/'robot.urdf'
    urdf.write_text('''<robot><joint name="base_to_imu" type="revolute"><parent link="base_link"/>
      <child link="imu_link"/><origin xyz="0 0 0"/></joint></robot>''')
    with pytest.raises(ValueError, match='fixed'):
        imu_from_camera(urdf, np.eye(4))


def test_camera_transform_uses_preserved_rectification_without_invented_probe_rate(tmp_path: Path):
    from experiments.handheld_mapping.inertial import camera_from_urdf
    urdf = tmp_path/'robot.urdf'
    urdf.write_text('''<robot><joint name="base_to_camera" type="fixed"><parent link="base_link"/>
      <child link="camera_link"/><origin xyz="1 0 0" rpy="0 0 0"/></joint>
      <joint name="camera_to_left_optical_frame" type="fixed"><parent link="camera_link"/>
      <child link="camera_left_optical_frame"/><origin xyz="0 2 0" rpy="0 0 0"/></joint></robot>''')
    rectification = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    stereo = {'source_camera_info': {'left': {'r': rectification.ravel().tolist()}}}
    actual = camera_from_urdf(urdf, stereo)
    assert np.allclose(actual[:3,:3], rectification)
    assert np.allclose(actual[:3,3], [2.,-1.,0.])


def test_keyframe_retiming_recovers_original_camera_stamp(tmp_path: Path):
    from experiments.handheld_mapping.inertial import restore_camera_stamps
    folder=tmp_path
    (folder/'timings.csv').write_text('stamp_ns,state,map_id,process_ms,imu_initialized\n1000000000,2,0,3.2,1\n')
    (folder/'map_0_keyframes.csv').write_text('map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n0,0,1.00000005,0,0,0,0,0,0,1\n')
    restore_camera_stamps(folder, {1000000000: 1792070000000000000})
    row=next(csv.DictReader((folder/'timings.csv').open()))
    frame=next(csv.DictReader((folder/'map_0_keyframes.csv').open()))
    assert int(row['stamp_ns']) == 1792070000000000000
    assert float(frame['source_stamp']) == pytest.approx(1792070000.)


def test_unknown_keyframe_stamp_is_rejected(tmp_path: Path):
    from experiments.handheld_mapping.inertial import restore_camera_stamps
    (tmp_path/'map_0_keyframes.csv').write_text('source_stamp\n1.5\n')
    with pytest.raises(ValueError, match='stamp'):
        restore_camera_stamps(tmp_path, {1000000000: 1792070000000000000})


@pytest.mark.parametrize('changed', ['calibration','urdf','axes'])
def test_changed_imu_calibration_or_mount_is_rejected_before_replay(tmp_path: Path, changed: str):
    from experiments.handheld_mapping.inertial import validate_imu_provenance
    calibration = {'version': 1, 'sensor': 'ISM330DHCX', 'frame': 'rover_body',
                   'acc_unit': 'g', 'gyro_unit': 'deg/s', 'chip_to_body': [[0,-1],[1,-1],[2,1]],
                   'acc_offset_g': [0.,0.,0.], 'acc_scale': [1.,1.,1.], 'gyro_bias_dps': [0.,0.,0.]}
    path=tmp_path/'imu_calibration.json'
    path.write_text(json.dumps(calibration))
    urdf=tmp_path/'robot.urdf'
    urdf.write_text('<robot/>')
    manifest={'calibration_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
              'chip_to_body_expected': calibration['chip_to_body'],
              'chip_to_body_actual': calibration['chip_to_body']}
    if changed=='calibration':
        calibration['gyro_bias_dps'][0]=1.
        path.write_text(json.dumps(calibration))
    elif changed=='urdf':
        urdf.write_text('<robot name="changed"/>')
    else:
        manifest['chip_to_body_actual']=[[0,1],[1,1],[2,1]]
    (tmp_path/'imu_manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='provenance'):
        validate_imu_provenance(tmp_path)


@pytest.mark.parametrize('flags,maps,expected', [([0,0,0],[],False), ([0,1,1],[0],True),
                                               ([1,1,0],[],False), ([0,0,0],[0],True)])
def test_fusion_claim_requires_actual_exported_map_initialization(tmp_path: Path, flags: list[int], maps: list[int], expected: bool):
    from experiments.handheld_mapping.inertial import fusion_evidence
    path=tmp_path/'timings.csv'
    path.write_text('stamp_ns,state,map_id,process_ms,imu_initialized,imu_samples\n'+
                   ''.join(f'{i},2,0,2.0,{flag},4\n' for i,flag in enumerate(flags)))
    result=fusion_evidence(path, {'exported_imu_initialized_maps': maps})
    assert result['orb_fused'] is expected
    assert result['last_tracked_frame_imu_initialized'] is bool(flags[-1])
    assert result['final_export_has_inertial_map'] is expected
    assert result['imu_initialized_frames']==sum(flags)


def test_replay_orchestration_finishes_from_actual_native_completion_contract(tmp_path: Path, monkeypatch):
    from experiments.handheld_mapping import inertial, inertial_timing
    run=tmp_path/'run'
    run.mkdir()
    cal={'version': 1, 'sensor': 'ISM330DHCX', 'frame': 'rover_body', 'acc_unit': 'g', 'gyro_unit': 'deg/s',
         'chip_to_body': [[0,-1],[1,-1],[2,1]], 'gyro_bias_dps': [0.,0.,0.],
         'acc_offset_g': [0.,0.,0.], 'acc_scale': [1.,1.,1.]}
    (run/'imu_calibration.json').write_text(json.dumps(cal))
    (run/'robot.urdf').write_text('''<robot>
      <joint name="base_to_imu" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0"/></joint>
      <joint name="base_to_camera" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="0 0 0"/></joint>
      <joint name="camera_to_left_optical_frame" type="fixed"><parent link="camera_link"/><child link="camera_left_optical_frame"/><origin xyz="0 0 0"/></joint></robot>''')
    manifest={'calibration_sha256': hashlib.sha256((run/'imu_calibration.json').read_bytes()).hexdigest(),
              'urdf_sha256': hashlib.sha256((run/'robot.urdf').read_bytes()).hexdigest(),
              'chip_to_body_actual': cal['chip_to_body'], 'chip_to_body_expected': cal['chip_to_body']}
    (run/'imu_manifest.json').write_text(json.dumps(manifest))
    info={'k': [100.,0.,320.,0.,100.,240.,0.,0.,1.], 'd': [0.]*5}
    stereo={'camera_profile':'640x480x15:Y8', 'left':info, 'right':info,
            'T_c1_c2': [[1.,0.,0.,.05],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]],
            'pose_output':'camera', 'source_camera_info':{'left': {'r': np.eye(3).ravel().tolist()}}}
    (run/'capture_calibration.json').write_text(json.dumps({'stereo':stereo}))
    origin=1792070000000000000
    camera={origin+i*50000000: i*.05 for i in range(35)}
    times=np.arange(-.1,2.,.02)
    data={'imu_times_s':times, 'gyro':np.zeros((len(times),3)), 'acc':np.tile([0.,0.,9.80665],(len(times),1)),
          'camera_origin_ns':origin, 'camera_times_s':camera,
          'report':{'input_sha256':{}, 'imu_time_correction_s':0., 'imu_time_source':'mcu_mean_sample_read'}}
    monkeypatch.setattr(inertial_timing,'aligned_inputs',lambda *args,**kwargs:data)
    monkeypatch.setattr(inertial,'iter_pairs',lambda run:((stamp,'left.png','right.png') for stamp in camera))
    monkeypatch.setattr(inertial,'vocabularies',lambda:(Path('ORBvoc'),Path('LSDvoc')))
    def native_boundary(command, **kwargs):
        folder=Path(command[5])
        assert len((folder/'imu.csv').read_text().splitlines())>50
        (folder/'runner_complete.json').write_text(json.dumps({'status':'complete','imu_requested':True,
                                                              'exported_imu_initialized_maps':[0]}))
        (folder/'timings.csv').write_text('stamp_ns,state,map_id,process_ms,imu_initialized,imu_samples\n'+
                                        f'{origin},2,0,2.0,1,4\n')
        (folder/'map_0_keyframes.csv').write_text('source_stamp\n'+format(origin*1e-9,'.17g')+'\n')
    monkeypatch.setattr(inertial.subprocess,'run',native_boundary)
    result=inertial.run_inertial(run)
    assert result['status']=='initialized'
    assert result['orb_fused'] is True
    assert result['imu_clock_verified'] is False
    assert json.loads((run/'orb_inertial/summary.json').read_text())['orb_fused'] is True


def test_automatic_replay_selects_one_boot_and_motion_window(tmp_path: Path, monkeypatch):
    from experiments.handheld_mapping import inertial, inertial_timing
    records=[]
    for segment,active in [(0,5),(1,25)]:
        for i in range(40):
            records.append({'acceptance':'accepted','clock':{'segment':segment},
                            'calibrated':{'gyro_dps':[10. if i<active else 0.,0.,0.]}})
    (tmp_path/'imu.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in records))
    def aligned(run, correction, **kwargs):
        assert correction==0 and kwargs['segment']==1
        return {'imu_times_s':np.arange(40.),'gyro':np.tile([.1,0.,0.],(40,1))}
    monkeypatch.setattr(inertial_timing,'aligned_inputs',aligned)
    captured={}
    def replay(run, **kwargs):
        captured.update(kwargs)
        (run/'orb_inertial').mkdir()
        return {'status':'initialized','orb_fused':True}
    monkeypatch.setattr(inertial,'run_inertial',replay)
    result=inertial.run_inertial_auto(tmp_path)
    assert captured['segment']==1
    assert captured['correlation_window_s']==(-1.,40.)
    assert result['automatic_selection']['selected_segment']==1


def test_automatic_replay_does_not_force_stationary_initialization(tmp_path: Path):
    from experiments.handheld_mapping.inertial import run_inertial_auto
    row={'acceptance':'accepted','clock':{'segment':0},'calibrated':{'gyro_dps':[0.,0.,0.]}}
    (tmp_path/'imu.jsonl').write_text((json.dumps(row)+'\n')*100)
    with pytest.raises(ValueError,match='motion'):
        run_inertial_auto(tmp_path)
    assert not (tmp_path/'orb_inertial').exists()
