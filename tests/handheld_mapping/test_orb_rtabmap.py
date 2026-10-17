import json
import sqlite3
import pytest

from experiments.handheld_mapping.orb_rtabmap import prepare,verify
from tests.handheld_mapping.test_compare_rtabmap import rgbd_calibration


def test_orb_external_pose_import_uses_sorted_final_keyframes_and_nearby_depth(tmp_path):
    run=tmp_path/'run'
    for name in ['raw/left','raw/depth','orb_offline']:(run/name).mkdir(parents=True)
    (run/'capture_calibration.json').write_text(json.dumps(rgbd_calibration()))
    (run/'orb_offline/map_0_keyframes.csv').write_text(
        'source_stamp,x,y,z,qx,qy,qz,qw\n2,1,0,0,0,0,0,1\n1,0,0,0,0,0,0,1\n')
    for t in [1000000000,2000000000]:
        for side in ['left','depth']:(run/'raw'/side/f'{t}.png').touch()
    out=tmp_path/'output';_,count=prepare(run,out)
    lines=(out/'orb_rgbd_frames.tsv').read_text().splitlines()
    assert count==2 and lines[0].startswith('1000000000\t')
    assert lines[1].split('\t')[3:6]==['1','0','0']
    summary=json.loads((out/'input_summary.json').read_text())
    assert summary['extra_loop_correction'] is False
    with pytest.raises(FileExistsError):prepare(run,out)


def write_graph(tmp_path,second_x=1,link_type=0):
    (tmp_path/'orb_imported.tum').write_text('1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n')
    (tmp_path/'rtab_optimized.tum').write_text(f'1 0 0 0 0 0 0 1\n2 {second_x} 0 0 0 0 0 1\n')
    c=sqlite3.connect(tmp_path/'rtabmap.db');c.execute('create table Link(type integer)')
    c.execute('insert into Link values(?)',(link_type,));c.commit();c.close()


def test_fusion_verification_rejects_pose_change(tmp_path):
    write_graph(tmp_path,second_x=1.1)
    with pytest.raises(ValueError,match='altered'):verify(tmp_path)


def test_fusion_verification_rejects_extra_closure(tmp_path):
    write_graph(tmp_path,link_type=1)
    with pytest.raises(ValueError,match='non-neighbor'):verify(tmp_path)


def test_fusion_verification_accepts_identical_orb_poses(tmp_path):
    write_graph(tmp_path)
    result=verify(tmp_path)
    assert result['max_position_change_m']==0
    assert result['retained_graph_nodes']==2


def test_display_trajectory_uses_saved_map_transform_and_checks_source(tmp_path):
    import numpy as np
    from experiments.handheld_mapping.rtabmap_surface import load_display_trajectory
    from experiments.handheld_mapping.gravity import fingerprint
    path=tmp_path/'rtab_optimized.tum'
    path.write_text('1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n')
    matrix=np.array([[0,-1,0,2],[1,0,0,3],[0,0,1,4],[0,0,0,1]])
    summary={'source_sha256':{str(path):fingerprint(path)},'gravity':{'world_from_slam':matrix.tolist()}}
    points=load_display_trajectory(summary)
    np.testing.assert_allclose(points,[[2,3,4],[2,4,4]])
    path.write_text(path.read_text()+'3 2 0 0 0 0 0 1\n')
    with pytest.raises(ValueError,match='changed'):load_display_trajectory(summary)
