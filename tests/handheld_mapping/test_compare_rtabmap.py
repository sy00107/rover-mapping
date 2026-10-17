import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.handheld_mapping.compare_rtabmap import orb_poses, tum_poses
from experiments.handheld_mapping.compare_rtabmap_maps import interpolate


def test_orb_keyframes_ordered_by_source_time(tmp_path):
    folder=tmp_path/'orb_offline'; folder.mkdir()
    (folder/'map_0_keyframes.csv').write_text(
        'source_stamp,x,y,z,qx,qy,qz,qw\n2,2,0,0,0,0,0,1\n1,1,0,0,0,0,0,1\n')
    stamps,poses=orb_poses(tmp_path)
    np.testing.assert_array_equal(stamps,[1,2])
    np.testing.assert_array_equal(poses[:,0,3],[1,2])


def test_tum_rejects_reversed_times(tmp_path):
    path=tmp_path/'poses.tum'
    path.write_text('2 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError,match='increase'): tum_poses(path)


def test_pose_interpolation_rotation_and_gap():
    poses=np.array([np.eye(4),np.eye(4)])
    poses[1,0,3]=2
    poses[1,:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    result=interpolate(10.5,np.array([10.,11.]),poses)
    np.testing.assert_allclose(result[:3,3],[1,0,0])
    np.testing.assert_allclose(result[:3,:3],Rotation.from_euler('z',45,degrees=True).as_matrix(),atol=1e-8)
    assert interpolate(9,np.array([10.,11.]),poses) is None
    assert interpolate(12,np.array([10.,11.]),poses) is None
    assert interpolate(11,np.array([10.,13.]),poses) is None


def rgbd_calibration():
    return {'depth':{'width':640,'height':480,'k':[300,0,320,0,300,240,0,0,1],
                     'T_left_depth':np.eye(4).tolist(),'d':[0]*5,'depth_scale_m':.001},
            'stereo':{'input_geometry':'rectified_confirmed',
                      'left':{'width':640,'height':480,'k':[300,0,320,0,300,240,0,0,1],'d':[0]*5}}}


def test_rgbd_requires_registered_depth():
    from experiments.handheld_mapping.rtabmap_rgbd import validate_calibration
    calibration=rgbd_calibration()
    assert validate_calibration(calibration)['depth_scale_m']==.001
    calibration['depth']['T_left_depth'][0][3]=.01
    with pytest.raises(ValueError,match='registered'): validate_calibration(calibration)
    calibration=rgbd_calibration();calibration['depth']['k'][0]=400
    with pytest.raises(ValueError,match='intrinsics'): validate_calibration(calibration)


def test_rgbd_preparation_time_association_and_preservation(tmp_path):
    import json
    from experiments.handheld_mapping.rtabmap_rgbd import prepare
    run=tmp_path/'source'
    for side in ('left','depth'):(run/'raw'/side).mkdir(parents=True)
    (run/'capture_calibration.json').write_text(json.dumps(rgbd_calibration()))
    for t in [1000000000,2000000000,3000000000]:(run/'raw/left'/f'{t}.png').touch()
    for t in [1001000000,2003000000,3000000000]:(run/'raw/depth'/f'{t}.png').touch()
    out=tmp_path/'result';prepare(run,out)
    summary=json.loads((out/'input_summary.json').read_text())
    assert summary['pairs']==2
    assert summary['unpaired_left']==1
    assert summary['max_depth_timestamp_gap_ns']==1000000
    assert len((out/'rgbd_pairs.tsv').read_text().splitlines())==2
    with pytest.raises(FileExistsError):prepare(run,out)
    assert len(list((run/'raw/left').glob('*.png')))==3
