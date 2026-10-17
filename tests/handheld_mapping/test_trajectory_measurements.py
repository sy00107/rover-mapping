"""Known motions test metres, yaw, map frames, in-place turns and camera-facing labels."""
import json
import numpy as np
import pytest
from experiments.handheld_mapping.trajectory_measurements import BillboardLabels, load_pose_groups, measurements


def poses(samples):
    result=[]
    for index,(x,z,yaw) in enumerate(samples):
        a=np.deg2rad(yaw)
        result.append({'stamp':float(index),'position':np.array([x,0.,z]),
                       'forward':np.array([-np.sin(a),0.,np.cos(a)]),'up':np.array([0.,-1.,0.])})
    return result


def test_two_straights_and_stationary_left_turn():
    rows=measurements(poses([(0,0,0),(0,1,0),(0,2,0),(0,2,45),(0,2,90),(-1,2,90),(-2,2,90)]))
    assert [r['value'] for r in rows if r['kind']=='distance']==[2.,2.]
    turn,=[r for r in rows if r['kind']=='turn']
    assert turn['text']=='L 90°'
    np.testing.assert_allclose(turn['anchor'],[0.,0.,2.])


def test_stationary_right_turn_has_no_distance_and_wrap_is_small():
    rows=measurements(poses([(0,0,0),(0,0,0),(0,0,-45),(0,0,-90),(0,0,-90)]))
    assert len(rows)==1 and rows[0]['text']=='R 90°'
    assert measurements(poses([(0,0,179),(0,0,-179),(0,0,-178)]))==[]


def test_pitch_does_not_count_as_turn_and_distance_is_3d():
    path=poses([(0,0,0),(0,1,0),(0,2,0)])
    path[1]['position'][1]=.5;path[2]['position'][1]=1.
    for row,angle in zip(path,[0,15,30]):
        a=np.deg2rad(angle);row['forward']=np.array([0.,-np.sin(a),np.cos(a)])
    distance,=measurements(path)
    assert distance['kind']=='distance' and distance['value']==pytest.approx(np.sqrt(5))


def test_stationary_jitter_does_not_accumulate_fake_metres():
    path=poses([(0,0,0)]*80)
    for index,row in enumerate(path):row['position']=np.array([.005*(-1)**index,0.,0.])
    assert measurements(path)==[]


def test_near_vertical_heading_omits_angles_but_retains_distance():
    path=poses([(0,0,0),(0,1,90),(0,2,180)])
    path[1]['forward']=np.array([0.,-1.,0.])
    rows=measurements(path)
    assert len(rows)==1 and rows[0]['kind']=='distance'


def test_merged_sources_time_gaps_and_gravity_transform_are_not_mixed(tmp_path):
    root=tmp_path/'run';dense=root/'dense_offline';dense.mkdir(parents=True)
    path=dense/'map_0_mesh.ply';path.touch()
    tf=np.array([[1,0,0,10],[0,0,1,20],[0,-1,0,30],[0,0,0,1]])
    (dense/'summary.json').write_text(json.dumps({'maps':[{'map_id':0,'coordinate_system':'gravity_z_up',
                                                       'world_from_slam':tf.tolist()}]}))
    source=root/'orb_offline';source.mkdir()
    (source/'map_0_keyframes.csv').write_text(
        'map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw,source_session\n'
        '0,3,3,0,0,3,0,0,0,1,B\n0,1,1,0,0,0,0,0,0,1,A\n'
        '0,4,12,0,0,20,0,0,0,1,A\n0,2,2,0,0,2,0,0,0,1,A\n')
    groups=load_pose_groups(path)
    assert sorted(map(len,groups))==[1,1,2]
    group=next(g for g in groups if len(g)==2)
    np.testing.assert_allclose(group[0]['position'],[10,20,30])
    np.testing.assert_allclose(group[0]['forward'],[0,1,0])
    np.testing.assert_allclose(group[0]['up'],[0,0,1])
    row,=measurements(group)
    assert row['value']==2.
    assert sum(len(measurements(g)) for g in groups)==1


def parameters():
    import open3d as o3d
    result=o3d.camera.PinholeCameraParameters()
    result.intrinsic=o3d.camera.PinholeCameraIntrinsic(800,600,500,500,400,300)
    result.extrinsic=np.eye(4)
    return result


def test_billboard_follows_camera_and_retains_pixel_height():
    labels=BillboardLabels([{'anchor':np.array([0.,0.,3.]),'kind':'distance','text':'1.25 m'}])
    p=parameters();assert labels.update(p) and labels.visible_count==1
    first=np.asarray(labels.geometry.points).copy()
    assert not labels.update(p)
    tf=np.eye(4);tf[2,3]=3;p.extrinsic=tf
    assert labels.update(p)
    world=np.asarray(labels.geometry.points)
    assert np.ptp(world[:-2,1])==pytest.approx(np.ptp(first[:-2,1]))
    camera=world@tf[:3,:3].T+tf[:3,3]
    screen=camera[:,:2]/camera[:,2,None]*500
    assert np.ptp(screen[:-2,1])==pytest.approx(16.)


def test_labels_behind_camera_are_hidden_and_crowding_is_bounded():
    entries=[{'anchor':np.array([0.,0.,3.]),'kind':'turn','text':'L 90°'} for _ in range(60)]
    labels=BillboardLabels(entries);p=parameters();labels.update(p)
    assert 0<labels.visible_count<=5
    tf=np.eye(4);tf[2,3]=-10;p.extrinsic=tf;labels.update(p)
    assert labels.visible_count==0 and len(labels.geometry.points)==0


def test_oblique_legacy_up_vector_matches_actual_orthogonal_camera():
    entry={'anchor':np.array([0.,0.,0.]),'kind':'distance','text':'1.25 m'}
    expected=BillboardLabels([entry]);actual=BillboardLabels([entry])
    p=parameters();tf=np.eye(4);tf[2,3]=3;p.extrinsic=tf
    expected.update(p)
    # ViewControl exports raw up, but LookAt renders its orthogonal projection.
    rotation=np.eye(3);rotation[1]=np.array([0.,1.,.4])/np.sqrt(1.16)
    tf[:3,:3]=rotation;tf[:3,3]=-rotation@np.array([0.,0.,-3.]);p.extrinsic=tf
    actual.update(p)
    assert actual.visible_count==expected.visible_count==1
    np.testing.assert_allclose(actual.geometry.points,expected.geometry.points,atol=1e-12)


@pytest.mark.parametrize('enabled,count',[(True,5),(False,4)])
def test_saved_viewer_adds_labels_without_changing_bounds_or_source(tmp_path,monkeypatch,enabled,count):
    import hashlib
    import open3d as o3d
    from types import SimpleNamespace
    from experiments.handheld_mapping.viewer import show_maps
    from test_viewer import maps,keyframes
    run=maps(tmp_path/'run')
    keyframes(run,rows='0,0,1,0,0,3,0,0,0,1\n0,1,2,0,1,3,0,0,0,1\n')
    path=run/'dense_offline/map_0_mesh.ply'
    o3d.io.write_triangle_mesh(str(path),o3d.geometry.TriangleMesh.create_box())
    before=hashlib.sha256(path.read_bytes()).hexdigest()
    added=[]
    class Viewer:
        def create_window(self,**kwargs):return True
        def add_geometry(self,geometry,**kwargs):added.append((geometry,kwargs));return True
        def update_geometry(self,geometry):pass
        def get_render_option(self):return SimpleNamespace()
        def get_view_control(self):
            methods={name:lambda value:None for name in ('set_lookat','set_up','set_front','set_zoom')}
            return SimpleNamespace(**methods,convert_to_pinhole_camera_parameters=parameters)
        def poll_events(self):return False
        def update_renderer(self):pass
        def destroy_window(self):pass
    monkeypatch.setattr(o3d.visualization,'Visualizer',Viewer)
    show_maps([('Map 0',path)],terrain_colors=False,path_measurements=enabled)
    assert len(added)==count
    if enabled:
        assert added[-1][1]=={'reset_bounding_box':False}
        assert isinstance(added[-1][0],o3d.geometry.LineSet)
    assert hashlib.sha256(path.read_bytes()).hexdigest()==before
