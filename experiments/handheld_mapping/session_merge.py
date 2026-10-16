"""Fuse validated sessions' original depth into a separate gravity-aligned map."""
import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from experiments.handheld_mapping.depth_filter import DepthFilter, filtered_depths
from experiments.handheld_mapping.gravity import fingerprint, load_alignment
from experiments.handheld_mapping.ground import GroundGrid
from experiments.handheld_mapping.recording import ordered_keyframes
from experiments.handheld_mapping.reconstruct import pose_matrix, transform
from experiments.handheld_mapping.storage import atomic_json
from experiments.handheld_mapping.terrain import GridSpec
from experiments.handheld_mapping.terrain_replay import project_depth, save_grid
from experiments.handheld_mapping.costmap import RoverLimits


def validated_sources(reference: Path, reports: list[dict]) -> list[dict]:
    """Reject unverified/changed registrations before creating output geometry."""
    reference = Path(reference).resolve()
    sources = [{'run':reference, 'registration':np.eye(4), 'report':None}]
    seen = {(reference,0)}
    for report in reports:
        run = Path(report['source']).resolve()
        map_id = report.get('source_map_id',0)
        if (run,map_id) in seen or Path(report['reference']).resolve() != reference:
            raise ValueError('duplicate source or different registration reference')
        held = report.get('held_out_reprojection_p50_p90_px', [])
        if (report.get('method') != 'mutual_RootSIFT_metric_depth_PnP_multiview_reprojection'
                or report.get('held_out_used_for_consensus_or_fit') is not False
                or report.get('distinct_source_views', 0) < 4
                or len(report.get('held_out_source_views', [])) < 2
                or report.get('camera_support_span_m', 0) < .5
                or len(held) != 2 or not np.isfinite(held).all() or held[0] > 3. or held[1] > 6.):
            raise ValueError('registration did not pass held-out multiview quality checks')
        frame = report.get('source_frame','gravity')
        map_id = report.get('source_map_id',0)
        if frame not in ('gravity','slam') or type(map_id) is not int or map_id < 0:
            raise ValueError('invalid source registration frame/map')
        matrix = transform(report.get('source_to_reference',report.get('source_gravity_to_reference_gravity')))
        if frame == 'gravity' and matrix[2, 2] < np.cos(np.deg2rad(3.)):
            raise ValueError('source gravity incompatible with reference')
        required = {str(root/name) for root in [run, reference] for name in
                    ('frames.jsonl','capture_calibration.json')}
        required.update([str(reference/'orb_offline/map_0_keyframes.csv'),
                         str(reference/'gravity/orb_offline/map_0.json'),
                         str(run/f'orb_offline/map_{map_id}_keyframes.csv')])
        if frame == 'gravity':required.add(str(run/f'gravity/orb_offline/map_{map_id}.json'))
        hashes = report.get('source_sha256', {})
        if not required.issubset(hashes):
            raise ValueError('registration input provenance missing')
        for name in required:
            if fingerprint(Path(name)) != hashes[name]:
                raise ValueError(f'registration input changed: {name}')
        sources.append({'run':run,'registration':matrix,'report':report,'frame':frame,'map_id':map_id})
        seen.add((run,map_id))
    if len(sources) < 2:
        raise ValueError('at least two independently validated sessions required')
    for item in sources:
        run = item['run']
        map_id = item.get('map_id',0)
        gravity = load_alignment(run, 'orb_offline', map_id) if item.get('frame','gravity')=='gravity' else None
        if gravity is None and item.get('frame','gravity')=='gravity':
            raise ValueError('source gravity reference missing')
        item['map_id']=map_id
        item['world_from_slam'] = item['registration'] @ (transform(gravity['world_from_slam']) if gravity else np.eye(4))
        item['keyfile'] = run/f'orb_offline/map_{map_id}_keyframes.csv'
        item['keyframes'] = list(ordered_keyframes(item['keyfile'], map_id))
        item['calibration'] = json.loads((run/'capture_calibration.json').read_text())
        if not item['keyframes']:
            raise ValueError('source keyframes missing')
    sources.sort(key=lambda item:item['keyframes'][0][0])
    for a, b in zip(sources, sources[1:]):
        if a['keyframes'][-1][0] >= b['keyframes'][0][0]:
            raise ValueError('overlapping session epochs require separate observation clocks')
    return sources


def fuse_sessions(reference: Path, reports: list[dict], output: Path, depth_max: float = 2.,
                  voxel_size: float = .01, progress=print, cross_session_filter: bool = False) -> dict:
    import open3d as o3d
    reference, output = Path(reference).resolve(), Path(output).resolve()
    sources = validated_sources(reference, reports)
    if (not math.isfinite(depth_max) or not .3 < depth_max <= 10
            or not math.isfinite(voxel_size) or not .002 <= voxel_size <= .1):
        raise ValueError('invalid depth/voxel bounds')
    if any(output == source['run'] or output.is_relative_to(source['run']) for source in sources):
        raise ValueError('merged output must be outside original sessions')
    if type(cross_session_filter) is not bool:raise ValueError('cross_session_filter must be boolean')
    output.mkdir(parents=True, exist_ok=False)
    result = {'schema':1,'status':'building','method':'registered_sessions_original_depth_TSDF',
              'reference':str(reference),'depth_max_m':depth_max,'voxel_size_m':voxel_size,
              'coordinate_system':'reference session stationary IMU gravity frame',
              'source_runs':list(dict.fromkeys(str(s['run']) for s in sources)), 'registrations':reports,
              'raw_preserved':True,'accuracy_verified':False,'orb_fused':False}
    atomic_json(output/'merge.json', result)
    cache=None
    try:
        (output/'orb_offline').mkdir()
        (output/'dense_offline').mkdir()
        shutil.copy2(reference/'capture_calibration.json', output/'capture_calibration.json')
        positions = np.array([(s['world_from_slam'] @ pose_matrix(row))[:3, 3]
                              for s in sources for _, row in s['keyframes']])
        low, high = positions[:, :2].min(axis=0)-depth_max, positions[:, :2].max(axis=0)+depth_max
        resolution = max(.05, math.ceil(math.sqrt(float(np.prod(high-low))/200000)*100)/100)
        terrain = GroundGrid(GridSpec((float(low[0]),float(high[0]),float(low[1]),float(high[1])), resolution))
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=voxel_size, sdf_trunc=voxel_size*4,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
        filtering = DepthFilter(max_depth_m=depth_max, surface_cleanup=True)
        evidence=None;filter_metadata=filtering.metadata()
        if cross_session_filter:
            from experiments.handheld_mapping.session_consensus import build_consensus_cache
            cache=tempfile.TemporaryDirectory(prefix='.depth_consensus_',dir=output)
            progress('Building independent recording surface/free-space evidence')
            evidence=build_consensus_cache(sources,filtering,cache.name,progress)
            filter_metadata={**filter_metadata,'cross_session_consensus':evidence.metadata()}
        input_hashes={}
        for source in sources:
            paths=[source['run']/'frames.jsonl',source['run']/'capture_calibration.json',source['keyfile']]
            if source.get('frame','gravity')=='gravity':
                paths.append(source['run']/f'gravity/orb_offline/map_{source["map_id"]}.json')
            input_hashes.update({str(path):fingerprint(path) for path in paths})
        cameras, counts, observed = [], [], {}
        keypath = output/'orb_offline/map_0_keyframes.csv'
        with keypath.open('w') as stream:
            fields=['map_id','keyframe_id','source_stamp','x','y','z','qx','qy','qz','qw',
                    'source_session','source_keyframe_id']
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
            for source in sources:
                run, info = source['run'], source['calibration']['depth']
                k = np.asarray(info['k']).reshape(3, 3)
                intrinsic = o3d.camera.PinholeCameraIntrinsic(info['width'],info['height'],
                                                           k[0,0],k[1,1],k[0,2],k[1,2])
                left_depth = transform(info['T_left_depth'])
                integrated, total, max_gap, stats, cells = 0, 0, 0, {}, set()
                source_depth_hashes = {};consensus_stats={}
                if evidence is not None:
                    from experiments.handheld_mapping.session_consensus import cached_depths
                    frames=cached_depths(source,stats)
                else:
                    frames=filtered_depths(run,source['keyfile'],source['map_id'],info,
                                           source['world_from_slam'],filtering,stats)
                for row, frame, gap, depth in frames:
                    total += 1
                    if frame is None or depth is None or np.count_nonzero(depth) < 100:
                        continue
                    left_pose = source['world_from_slam'] @ pose_matrix(row)
                    depth_pose = left_pose @ left_depth
                    if evidence is not None:
                        depth,metrics=evidence.filter_depth(depth,info,depth_pose,source['consensus_index'])
                        for key,value in metrics.items():consensus_stats[key]=consensus_stats.get(key,0)+value
                        if np.count_nonzero(depth)<100:continue
                    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        o3d.geometry.Image(np.zeros((*depth.shape,3),np.uint8)),o3d.geometry.Image(depth),
                        depth_scale=1/info['depth_scale_m'],depth_trunc=depth_max,convert_rgb_to_intensity=False)
                    volume.integrate(rgbd,intrinsic,np.linalg.inv(depth_pose))
                    points = project_depth(depth,info,depth_pose,depth_max=depth_max)
                    terrain.update(points,frame['stamp_ns'],camera_position=depth_pose[:3,3])
                    xy = np.unique(np.floor(points[:,:2]/.05).astype(np.int64),axis=0)
                    cells.update(map(tuple,xy.tolist()))
                    cameras.append(depth_pose[:3,3]);integrated+=1;max_gap=max(max_gap,gap)
                    q=Rotation.from_matrix(left_pose[:3,:3]).as_quat()
                    writer.writerow(dict(zip(fields,[0,len(cameras)-1,row['source_stamp'],*left_pose[:3,3],
                        *q,run.name+f':map{source["map_id"]}',row['keyframe_id']])))
                    source_depth_hashes[frame['path']]=fingerprint(run/frame['path'])
                    if integrated % 100 == 0:
                        progress(f'{run.name}: {integrated} depth keyframes fused')
                if integrated == 0:
                    raise ValueError(f'no valid depth in source: {run.name}')
                observed.setdefault(str(run),set()).update(cells)
                counts.append({'source':str(run),'keyframes':total,'integrated_frames':integrated,
                               'source_map_id':source['map_id'],
                               'unintegrated_keyframes':total-integrated,'depth_filter_stats':stats,
                               'cross_session_stats':consensus_stats,
                               'max_timestamp_difference_ns':max_gap,'depth_sha256':source_depth_hashes})
        mesh = volume.extract_triangle_mesh();mesh.compute_vertex_normals()
        from experiments.handheld_mapping.surface_cleanup import clean_surface_mesh
        mesh, cleanup = clean_surface_mesh(mesh,cameras)
        if not len(mesh.triangles):
            raise ValueError('empty merged surface')
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
        cloud.normals = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_normals))
        for filename, geometry, write in [('map_0_mesh.ply',mesh,o3d.io.write_triangle_mesh),
                                          ('map_0_cloud.ply',cloud,o3d.io.write_point_cloud)]:
            if not write(str(output/'dense_offline'/filename),geometry):
                raise OSError('merged geometry write failed')
        total = sum(row['keyframes'] for row in counts)
        dense = {'status':'complete' if total==len(cameras) else 'partial',
                 'method':'multi_session_keyframe_TSDF','depth_max_m':depth_max,'voxel_size_m':voxel_size,
                 'depth_filter':filter_metadata,'coordinate_system':'gravity_z_up',
                 'maps':[{'map_id':0,'keyframes':total,'integrated_frames':len(cameras),
                          'unintegrated_keyframes':total-len(cameras),'vertices':len(mesh.vertices),
                          'triangles':len(mesh.triangles),'mesh':'dense_offline/map_0_mesh.ply',
                          'cloud':'dense_offline/map_0_cloud.ply','coordinate_system':'gravity_z_up',
                          'world_from_slam':np.eye(4).tolist(),'surface_cleanup':cleanup}],
                 'accuracy_verified_on_real_camera':False,'orb_fused':False}
        atomic_json(output/'dense_offline/summary.json',dense)
        terrain_result = save_grid(terrain,output/'terrain/orb_offline/map_0',RoverLimits(),2,
            {'status':dense['status'],'source':str(output),'backend':'orb_offline','map_id':0,
             'integrated_frames':len(cameras),'grid_from_world':np.eye(4).tolist(),
             'coordinate_system':'registered reference gravity frame; axis order y,x',
             'depth_max_m':depth_max,'depth_filter':filter_metadata,
             'source_sha256':{name:fingerprint(output/name) for name in
                             ('capture_calibration.json','orb_offline/map_0_keyframes.csv')},
             'alignment_metadata':{'method':'image_verified_multisession_gravity_registration',
                                   'imu_clock_verified':False,'extrinsic_verified':False,'orb_fused':False}})
        for name, expected in input_hashes.items():
            if fingerprint(Path(name)) != expected:
                raise ValueError(f'source changed during merge: {name}')
        union=set().union(*observed.values());base=observed[str(reference)]
        result.update(status='complete' if dense['status']=='complete' else 'partial',
                      cross_session_consensus=evidence.metadata() if evidence is not None else None,
                      dense=dense,terrain_status=terrain_result['status'],source_counts=counts,
                      input_sha256=input_hashes,
                      observed_xy_5cm_cells_reference=len(base),observed_xy_5cm_cells_merged=len(union),
                      additional_observed_xy_5cm_cells=len(union-base),
                      coverage_note='observed XY cells including obstacles; not validated free/ground area')
    except Exception as error:
        result.update(status='failed',error=str(error));atomic_json(output/'merge.json',result)
        raise
    finally:
        if cache is not None:cache.cleanup()
    atomic_json(output/'merge.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description='검증된 촬영들의 원본 깊이로 별도 통합 지도 생성')
    parser.add_argument('--reference',required=True,type=Path)
    parser.add_argument('--registration',required=True,type=Path,action='append')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--depth-max',type=float,default=2.)
    parser.add_argument('--cross-session-filter',action='store_true',
                        help='다른 촬영의 실제 표면/빈 공간 관측으로 충돌하는 깊이 제외')
    args=parser.parse_args()
    reports=[json.loads(path.read_text()) for path in args.registration]
    result=fuse_sessions(args.reference,reports,args.output,args.depth_max,
                         cross_session_filter=args.cross_session_filter)
    print(json.dumps({key:result[key] for key in ('status','source_runs','additional_observed_xy_5cm_cells')},indent=2))


if __name__=='__main__':main()
