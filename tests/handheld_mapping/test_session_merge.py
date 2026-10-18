"""Known rigid registrations, disjoint visual validation and actual multi-session TSDF."""
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.handheld_mapping.gravity import fingerprint, source_paths
from experiments.handheld_mapping.session_merge import fuse_sessions, validated_sources
from experiments.handheld_mapping.session_registration import fit_visual_pairs
from test_core import make_run


def visual_evidence():
    rng=np.random.default_rng(17)
    matrix=np.eye(4);matrix[:3,:3]=Rotation.from_euler('z',2,degrees=True).as_matrix()
    matrix[:3,3]=[.2,-.1,.03]
    xyz=rng.uniform([-1.,-.6,3.],[1.,.6,5.],(80,3))
    k=np.array([[377.,0.,320.],[0.,377.,240.],[0.,0.,1.]])
    poses=np.tile(np.eye(4),(6,1,1));poses[:,0,3]=np.arange(6)*.2
    evidence=[]
    for i in range(6):
        pose=np.eye(4);pose[0,3]=i*.1
        points=xyz@matrix[:3,:3].T+matrix[:3,3]-pose[:3,3]
        uv=(points@k.T)[:,:2]/points[:,2,None]+rng.normal(0,.08,(80,2))
        evidence.append({'source_index':i,'target_index':i,'matches':80,'inliers':80,
                         'source_xyz':xyz,'target_uv':uv,'target_pose':pose,'transform':matrix.copy()})
    source=SimpleNamespace(run=Path('/source'),poses=poses)
    target=SimpleNamespace(run=Path('/reference'),k=k)
    return evidence,source,target,matrix


def test_image_fit_recovers_metric_transform_with_separate_held_out_views():
    pairs,source,target,truth=visual_evidence()
    result,report=fit_visual_pairs(pairs,source,target,{0,3})
    np.testing.assert_allclose(result,truth,atol=.002)
    assert report['held_out_used_for_consensus_or_fit'] is False
    assert report['held_out_source_views']==[0,3]
    assert {p['source_index'] for p in report['evidence_views']}.isdisjoint({0,3})
    assert report['held_out_reprojection_p50_p90_px'][1]<.3


def test_held_out_views_cannot_move_fit_to_hide_inconsistent_registration():
    pairs,source,target,_=visual_evidence()
    for pair in pairs:
        if pair['source_index'] in {0,3}:pair['target_uv']+=30
    with pytest.raises(ValueError,match='held-out'):
        fit_visual_pairs(pairs,source,target,{0,3})


def test_registration_rejects_insufficient_camera_baseline():
    pairs,source,target,_=visual_evidence();source.poses[:,:3,3]=0.
    with pytest.raises(ValueError,match='spatial support'):
        fit_visual_pairs(pairs,source,target,{0,3})


def test_visual_pose_frame_can_align_without_fabricating_imu_gravity():
    pairs,source,target,truth=visual_evidence()
    source.gravity_available=False
    truth[:3,:3]=Rotation.from_euler('x',15,degrees=True).as_matrix()
    for pair in pairs:
        points=pair['source_xyz']@truth[:3,:3].T+truth[:3,3]-pair['target_pose'][:3,3]
        pair['target_uv']=(points@target.k.T)[:,:2]/points[:,2,None]
        pair['transform']=truth.copy()
    result,report=fit_visual_pairs(pairs,source,target,{0,3})
    np.testing.assert_allclose(result,truth,atol=1e-5)
    assert report['source_frame']=='slam'
    assert 'source_gravity_to_reference_gravity' not in report


def test_appearance_retrieval_signature_is_stable_under_feature_order_changes():
    from experiments.handheld_mapping.session_registration import appearance_signature
    rng=np.random.default_rng(41)
    centers=rng.uniform(.02,.2,(16,128)).astype(np.float32)
    descriptors=centers[rng.integers(0,16,300)]+rng.normal(0,.005,(300,128)).astype(np.float32)
    query=appearance_signature(descriptors,centers)
    reordered=appearance_signature(descriptors[rng.permutation(len(descriptors))],centers)
    unrelated=appearance_signature(centers[rng.integers(0,16,300)]+rng.normal(0,.005,(300,128)),centers)
    assert query@reordered>.9999
    assert query@unrelated<.8


@pytest.fixture
def two_sessions(tmp_path):
    runs=[]
    for index in range(2):
        run=make_run(tmp_path/f'run{index}')
        (run/'orb').rename(run/'orb_offline')
        if index:
            p=run/'frames.jsonl';text=p.read_text().replace('1000000000','2000000000')
            p.write_text(text);(run/'raw/depth/1000000000.png').rename(run/'raw/depth/2000000000.png')
            p=run/'orb_offline/map_0_keyframes.csv';p.write_text(p.read_text().replace('1.0,1,0,0','2.0,1,0,0'))
        # Far cleanup requires two real reference frames, including in this fixture.
        frame=json.loads((run/'frames.jsonl').read_text().splitlines()[0])
        images=[];poses=['map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw']
        for j in range(3):
            stamp=(index+1)*1_000_000_000+j*100_000_000
            path=f'raw/depth/{stamp}.png'
            if path!=frame['path']:shutil.copy2(run/frame['path'],run/path)
            images.append({**frame,'stamp_ns':stamp,'path':path})
            poses.append(f'0,{j},{index+1+j*.1:.1f},1,0,0,0,0,0,1')
        (run/'frames.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in images))
        (run/'orb_offline/map_0_keyframes.csv').write_text('\n'.join(poses)+'\n')
        for name in ['robot.urdf','imu_calibration.json','imu.jsonl']:(run/name).write_text('{}')
        data={'run_id':run.name,'backend':'orb_offline','map_id':0,'world_from_slam':np.eye(4).tolist(),
              'source_sha256':{str(p.relative_to(run)):fingerprint(p) for p in source_paths(run,'orb_offline',0)}}
        path=run/'gravity/orb_offline/map_0.json';path.parent.mkdir(parents=True);path.write_text(json.dumps(data))
        runs.append(run)
    reference,source=runs
    matrix=np.eye(4);matrix[0,3]=.4
    report={'source':str(source),'reference':str(reference),
            'method':'mutual_RootSIFT_metric_depth_PnP_multiview_reprojection',
            'source_gravity_to_reference_gravity':matrix.tolist(),
            'held_out_reprojection_p50_p90_px':[.5,1.], 'held_out_used_for_consensus_or_fit':False,
            'distinct_source_views':4,'held_out_source_views':[0,3],'camera_support_span_m':1.,
            'source_sha256':{str(run/name):fingerprint(run/name) for run in runs for name in
              ['frames.jsonl','capture_calibration.json','orb_offline/map_0_keyframes.csv','gravity/orb_offline/map_0.json']}}
    return reference,source,report


def test_actual_depth_fusion_preserves_sources_and_exports_both_session_poses(two_sessions,tmp_path):
    import open3d as o3d
    from experiments.handheld_mapping.surface_colors import load_surface_layers
    reference,source,report=two_sessions
    before={p:p.read_bytes() for run in [reference,source] for p in run.rglob('*') if p.is_file()}
    output=tmp_path/'merged'
    result=fuse_sessions(reference,[report],output,depth_max=2.5,voxel_size=.025,progress=lambda _:None)
    assert result['status']=='complete'
    assert result['dense']['maps'][0]['integrated_frames']==6
    assert len(result['source_counts'])==2
    assert result['additional_observed_xy_5cm_cells']>0
    points=np.asarray(o3d.io.read_point_cloud(str(output/'dense_offline/map_0_cloud.ply')).points)
    assert np.median(points[:,2])==pytest.approx(2.,abs=.03)
    assert points[:,0].max()>2.5
    assert load_surface_layers(output/'dense_offline/map_0_mesh.ply') is not None
    from experiments.handheld_mapping.viewer import load_trajectory_groups
    groups=load_trajectory_groups(output/'dense_offline/map_0_mesh.ply')
    assert len(groups)==2 and all(len(group)==3 for group in groups)
    assert all(p.read_bytes()==data for p,data in before.items())
    with pytest.raises(FileExistsError):fuse_sessions(reference,[report],output)


@pytest.mark.parametrize('problem',['used_held_out','large_error','few_views','changed_input','nonrigid','duplicate'])
def test_bad_or_changed_registration_is_rejected_before_output(two_sessions,tmp_path,problem):
    reference,source,report=two_sessions
    reports=[report]
    if problem=='used_held_out':report['held_out_used_for_consensus_or_fit']=True
    elif problem=='large_error':report['held_out_reprojection_p50_p90_px']=[5.,12.]
    elif problem=='few_views':report['distinct_source_views']=3
    elif problem=='changed_input':(source/'frames.jsonl').write_text('changed')
    elif problem=='nonrigid':report['source_gravity_to_reference_gravity'][0][0]=2.
    elif problem=='duplicate':reports.append(report)
    output=tmp_path/'bad'
    with pytest.raises(ValueError):fuse_sessions(reference,reports,output)
    assert not output.exists()


def test_consensus_fusion_preserves_valid_surfaces_and_cleans_its_temporary_cache(two_sessions,tmp_path):
    reference,source,report=two_sessions
    output=tmp_path/'consensus'
    result=fuse_sessions(reference,[report],output,depth_max=2.5,voxel_size=.025,
                         progress=lambda _:None,cross_session_filter=True)
    assert result['dense']['maps'][0]['integrated_frames']==6
    assert result['cross_session_consensus']['vote_unit'].startswith('independent recording')
    assert sum(x['cross_session_stats']['confirmed_other_session_pixels'] for x in result['source_counts'])>0
    assert not list(output.glob('.depth_consensus_*'))


def test_merge_accepts_verified_visual_frame_without_source_imu_files(two_sessions):
    reference,source,report=two_sessions
    report['source_frame']='slam'
    (source/'gravity/orb_offline/map_0.json').unlink()
    report['source_sha256'].pop(str(source/'gravity/orb_offline/map_0.json'))
    data=validated_sources(reference,[report])
    assert data[1]['frame']=='slam'
    np.testing.assert_allclose(data[1]['world_from_slam'],report['source_gravity_to_reference_gravity'])


def test_two_fragments_of_one_capture_share_one_independent_vote(two_sessions,tmp_path):
    import copy
    reference,source,report=two_sessions
    frames=[json.loads(line) for line in (source/'frames.jsonl').read_text().splitlines()]
    more=[]
    for j,row in enumerate(frames):
        stamp=2_300_000_000+j*100_000_000;path=f'raw/depth/{stamp}.png'
        shutil.copy2(source/row['path'],source/path)
        more.append({**row,'stamp_ns':stamp,'path':path})
    (source/'frames.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in frames+more))
    keyfile=source/'orb_offline/map_7_keyframes.csv'
    keyfile.write_text('map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n'+
                       ''.join(f'7,{j},{2.3+j*.1:.1f},1,0,0,0,0,0,1\n' for j in range(3)))
    report['source_sha256'][str(source/'frames.jsonl')]=fingerprint(source/'frames.jsonl')
    second=copy.deepcopy(report);second['source_frame']='slam';second['source_map_id']=7
    second['source_sha256'][str(keyfile)]=fingerprint(keyfile)
    output=tmp_path/'fragments'
    result=fuse_sessions(reference,[report,second],output,depth_max=2.5,voxel_size=.025,
                         progress=lambda _:None,cross_session_filter=True)
    assert len(result['source_runs'])==2 and len(result['source_counts'])==3
    assert len(result['cross_session_consensus']['session_evidence'])==2
    assert result['dense']['maps'][0]['integrated_frames']==9


def test_persistent_single_recording_depth_spike_is_rejected_by_other_recording(two_sessions,tmp_path):
    import cv2
    reference,source,report=two_sessions
    # Both recordings see the same 2m wall. Only one repeatedly reports a false
    # 70cm foreground patch, which same-recording temporal agreement would keep.
    report['source_gravity_to_reference_gravity']=np.eye(4).tolist()
    for p in (reference/'raw/depth').glob('*.png'):
        depth=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
        depth[20:40,30:50]=700;cv2.imwrite(str(p),depth)
    output=tmp_path/'spike_filtered'
    result=fuse_sessions(reference,[report],output,depth_max=2.5,voxel_size=.025,
                         progress=lambda _:None,cross_session_filter=True)
    assert result['dense']['maps'][0]['integrated_frames']==6
    ref=next(x for x in result['source_counts'] if Path(x['source'])==reference)
    assert ref['cross_session_stats']['rejected_conflict_pixels']>50
    assert ref['cross_session_stats']['confirmed_other_session_pixels']>1000


def test_nonzero_reference_fragment_uses_its_pose_and_gravity(two_sessions):
    reference,source,report=two_sessions
    old=reference/'orb_offline/map_0_keyframes.csv';new=old.with_name('map_5_keyframes.csv')
    text=old.read_text().splitlines();new.write_text(text[0]+'\n'+'\n'.join('5,'+line.split(',',1)[1] for line in text[1:])+'\n');old.unlink()
    old=reference/'gravity/orb_offline/map_0.json';new=old.with_name('map_5.json')
    data=json.loads(old.read_text());data['map_id']=5
    # Alignment input provenance follows the renamed source keyframe snapshot.
    hashes=data.get('source_sha256',{})
    if 'orb_offline/map_0_keyframes.csv' in hashes:
        hashes.pop('orb_offline/map_0_keyframes.csv');hashes['orb_offline/map_5_keyframes.csv']=fingerprint(reference/'orb_offline/map_5_keyframes.csv')
    new.write_text(json.dumps(data));old.unlink()
    report['reference_map_id']=5
    report['source_sha256']={str(run/name):fingerprint(run/name) for run in [reference,source]
        for name in ['frames.jsonl','capture_calibration.json',
                     'orb_offline/map_5_keyframes.csv' if run==reference else 'orb_offline/map_0_keyframes.csv',
                     'gravity/orb_offline/map_5.json' if run==reference else 'gravity/orb_offline/map_0.json']}
    sources=validated_sources(reference,[report],reference_map_id=5)
    assert any(item['run']==reference.resolve() and item['map_id']==5 for item in sources)
    with pytest.raises(ValueError,match='reference'):
        validated_sources(reference,[report],reference_map_id=0)


def test_held_out_target_selection_uses_match_strength_not_fit_error():
    pairs,source,target,truth=visual_evidence()
    # A weak wrong retrieval candidate does not count as a second independent held-out view.
    for pair in list(pairs):
        if pair['source_index'] in {0,3}:
            wrong={**pair,'target_index':99,'inliers':15,'target_uv':pair['target_uv']+50}
            pairs.append(wrong)
    result,report=fit_visual_pairs(pairs,source,target,{0,3})
    np.testing.assert_allclose(result,truth,atol=.002)
    assert report['held_out_image_pairs']==2
    # A stronger wrong match MUST fail, even when a weaker candidate fits the answer.
    for pair in pairs:
        if pair['target_index']==99:pair['inliers']=81
    with pytest.raises(ValueError,match='held-out'):
        fit_visual_pairs(pairs,source,target,{0,3})
