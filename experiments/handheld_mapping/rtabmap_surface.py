"""Refine recorded RTAB RGB-D surfaces using the existing ORB depth/terrain/display pipeline."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np

from .compare_rtabmap import tum_poses
from .depth_filter import DepthFilter,filtered_depths
from .gravity import (alignment_matrix,fingerprint,left_receipts,mounting,
                      sample_from_record,stationary_anchor_reference,WINDOW_NS)
from .inertial import validate_imu_provenance
from .reconstruct import nearest
from .rtabmap_rgbd import validate_calibration
from .surface_colors import PALETTE,LEGEND,classify_surface_points


def derive_gravity(run,stamps,poses):
    validate_imu_provenance(run)
    frames=list(left_receipts(run));frame_stamps=[f['stamp_ns'] for f in frames]
    frame_lookup={f['stamp_ns']:f for f in frames}
    samples=[]
    with (run/'imu.jsonl').open() as stream:
        for line in stream:
            sample=sample_from_record(json.loads(line))
            if sample is not None:samples.append(sample)
    for stamp,pose in zip(stamps,poses):
        match=nearest(frame_stamps,round(stamp*1e9))
        if match is None:continue
        center=frame_lookup[match].get('pi_receipt_monotonic_ns')
        if type(center) is not int:continue
        window=[s for s in samples if abs(s['pi_ns']-center)<=WINDOW_NS]
        try:reference=stationary_anchor_reference(window,center)
        except ValueError:continue
        matrix=alignment_matrix(pose,mounting(run),reference['body_up'])
        return matrix,dict(method='stationary calibrated IMU + RTAB final pose + archived URDF',
                           anchor_stamp_ns=match,stationary_reference=reference,
                           imu_fused=False,extrinsic_verified=False,imu_clock_verified=False,
                           world_from_slam=matrix.tolist())
    raise ValueError('No verified stationary IMU interval matching RTAB graph; cannot align terrain')


def export_keyframes(path,stamps,poses):
    from scipy.spatial.transform import Rotation
    quats=Rotation.from_matrix(poses[:,:3,:3]).as_quat()
    with path.open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['map_id','keyframe_id','source_stamp','x','y','z','qx','qy','qz','qw'])
        for i,(t,p,q) in enumerate(zip(stamps,poses,quats)):
            writer.writerow([0,i,format(t,'.12f'),*p[:3,3],*q])


def build(source,output):
    import open3d as o3d
    from .ground import GroundGrid
    from .terrain import GridSpec
    from .terrain_replay import project_depth,save_grid
    from .costmap import RoverLimits
    from .ground_display import fill_small_ground_gaps
    from .obstacle_display import smooth_obstacle_display
    source=source.resolve();output=output.resolve()
    manifest=json.loads((source/'input_summary.json').read_text());run=Path(manifest['source'])
    info=validate_calibration(json.loads((run/'capture_calibration.json').read_text()))
    stamps,poses=tum_poses(source/'rtab_optimized.tum')
    alignment,gravity=derive_gravity(run,stamps,poses)
    output.mkdir(parents=True,exist_ok=False)
    keyfile=output/'rtab_keyframes.csv';export_keyframes(keyfile,stamps,poses)
    # Bounds use the existing map only to allocate the grid; classification uses replayed measurements.
    cloud=o3d.io.read_point_cloud(str(source/'rtab_map_cloud.ply'))
    xyz=np.asarray(cloud.points)@alignment[:3,:3].T+alignment[:3,3]
    lo=xyz[:,:2].min(axis=0)-.1;hi=xyz[:,:2].max(axis=0)+.1
    spec=GridSpec((float(lo[0]),float(hi[0]),float(lo[1]),float(hi[1])),.05)
    terrain=GroundGrid(spec);filtering=DepthFilter(max_depth_m=2.,surface_cleanup=True)
    volume=o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=.01,sdf_trunc=.04,color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
    k=info['k'];intrinsic=o3d.camera.PinholeCameraIntrinsic(info['width'],info['height'],k[0],k[4],k[2],k[5])
    color=o3d.geometry.Image(np.zeros((info['height'],info['width'],3),np.uint8))
    from .reconstruct import pose_matrix
    stats={};integrated=0
    for row,frame,gap,depth in filtered_depths(run,keyfile,0,info,alignment,filtering,stats):
        if depth is None or np.count_nonzero(depth)<100:continue
        world_depth=alignment@pose_matrix(row)@np.array(info['T_left_depth'])
        rgbd=o3d.geometry.RGBDImage.create_from_color_and_depth(color,o3d.geometry.Image(depth),depth_scale=1/info['depth_scale_m'],depth_trunc=2.,convert_rgb_to_intensity=False)
        volume.integrate(rgbd,intrinsic,np.linalg.inv(world_depth))
        points=project_depth(depth,info,world_depth,depth_max=2.)
        terrain.update(points,frame['stamp_ns'],camera_position=world_depth[:3,3])
        integrated+=1
        if integrated%25==0:print(f'{integrated}/{len(stamps)} filtered depth frames fused',flush=True)
    if integrated<2:raise ValueError('Insufficient depth after multiview checks')
    layers=terrain.layers(2)
    terrain_summary=save_grid(terrain,output/'terrain',RoverLimits(),2,
        dict(status='complete',source=str(run),backend='rtabmap_rgbd',map_id=0,
             integrated_frames=integrated,grid_from_world=alignment.tolist(),
             coordinate_system='estimated gravity Z-up; stationary IMU, provisional extrinsics',
             depth_filter=filtering.metadata(),depth_filter_stats=stats,alignment_metadata=gravity))
    classification=dict(layers=layers,bounds=np.array(spec.bounds),resolution=spec.resolution,
                        grid_from_geometry=np.eye(4),ground_tolerance_m=.02,obstacle_threshold_m=.06)
    mesh=volume.extract_triangle_mesh();mesh.compute_vertex_normals()
    if not len(mesh.triangles):raise ValueError('Empty refined mesh')
    # Keep components supported by at least 100 triangles; avoid detached single-pixel fragments.
    cluster,counts,_=mesh.cluster_connected_triangles()
    small=np.asarray(counts)[np.asarray(cluster)]<100
    removed=int(small.sum());mesh.remove_triangles_by_mask(small);mesh.remove_unreferenced_vertices()
    labels=classify_surface_points(np.asarray(mesh.vertices),classification)
    gaps=fill_small_ground_gaps(mesh,labels,prevent_nonmanifold=True)
    smoothing=smooth_obstacle_display(mesh,labels)
    gaps["scope"]="derived saved presentation mesh; source map unchanged"
    smoothing["scope"]="derived saved presentation mesh; source map unchanged"
    mesh.vertex_colors=o3d.utility.Vector3dVector(PALETTE[labels])
    cloud=o3d.geometry.PointCloud();cloud.points=mesh.vertices;cloud.normals=mesh.vertex_normals;cloud.colors=mesh.vertex_colors
    if not o3d.io.write_triangle_mesh(str(output/'rtab_surface_mesh.ply'),mesh):raise OSError('Mesh save failed')
    if not o3d.io.write_point_cloud(str(output/'rtab_surface_cloud.ply'),cloud):raise OSError('Cloud save failed')
    counts=np.bincount(labels,minlength=3)
    hashes={str(p):fingerprint(p) for p in [source/'rtab_optimized.tum',source/'input_summary.json',run/'capture_calibration.json',run/'imu.jsonl',run/'imu_manifest.json',run/'imu_calibration.json',run/'robot.urdf']}
    summary=dict(status='complete',display_title=manifest.get('display_title','RTAB 3D'),source=str(source),source_run=str(run),source_sha256=hashes,
                 gravity=gravity,coordinate_system='estimated gravity_z_up',legend=LEGEND,
                 mesh='rtab_surface_mesh.ply',cloud='rtab_surface_cloud.ply',integrated_frames=integrated,
                 vertices=len(mesh.vertices),triangles=len(mesh.triangles),removed_small_component_triangles=removed,
                 classification=dict(ground=int(counts[1]),obstacle=int(counts[2]),unknown=int(counts[0]),
                                     ground_tolerance_m=.02,obstacle_threshold_m=.06),
                 depth_filter=filtering.metadata(),depth_filter_stats=stats,
                 obstacle_smoothing=smoothing,ground_gap_fill=gaps,raw_preserved=True,
                 limitations=['Green is observed ground classification, not traversability.',
                              'Small enclosed ground-hole patches are inferred display surfaces.',
                              'Stationary IMU alignment uses provisional mounting extrinsics.',
                              'No global floor flattening; real ramps and steps retained.'])
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    preview(mesh,output/'preview.png')
    print(json.dumps(summary['classification'],indent=2),flush=True)
    return summary


def preview(mesh,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    xyz=np.asarray(mesh.vertices);colors=np.asarray(mesh.vertex_colors)
    stride=max(1,len(xyz)//120000);xyz=xyz[::stride];colors=colors[::stride]
    fig=plt.figure(figsize=(11,8));ax=fig.add_subplot(projection='3d')
    ax.scatter(*xyz.T,c=colors,s=.3,rasterized=True)
    ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title='RTAB: green ground / orange obstacles / grey unknown')
    ax.set_box_aspect(np.maximum(np.ptp(xyz,axis=0),.1));ax.view_init(35,-65)
    from matplotlib.ticker import MaxNLocator
    ax.zaxis.set_major_locator(MaxNLocator(4))
    fig.tight_layout();fig.savefig(path,dpi=170);plt.close(fig)


def load_display_trajectory(summary):
    """Read the exact pose source used by the mesh and apply its saved Z-up transform."""
    from .reconstruct import transform
    candidates=[(Path(name),digest) for name,digest in summary['source_sha256'].items()
                if Path(name).name=='rtab_optimized.tum']
    if len(candidates)!=1:
        raise ValueError('Exactly one final pose source is required for the trajectory')
    source,expected=candidates[0]
    if fingerprint(source)!=expected:
        raise ValueError('Trajectory source changed after surface generation')
    stamps,poses=tum_poses(source)
    alignment=transform(summary['gravity']['world_from_slam'])
    points=poses[:,:3,3]@alignment[:3,:3].T+alignment[:3,3]
    return points


def view(folder,cloud=False,snapshot=None,trajectory=False):
    import open3d as o3d
    summary=json.loads((folder/'summary.json').read_text())
    path=folder/summary['cloud' if cloud else 'mesh']
    geometry=o3d.io.read_point_cloud(str(path)) if cloud else o3d.io.read_triangle_mesh(str(path))
    if cloud:
        if not len(geometry.points):raise ValueError('Empty cloud')
    elif not len(geometry.triangles):raise ValueError('Empty mesh')
    viewer=o3d.visualization.Visualizer()
    if not viewer.create_window(window_name=summary.get('display_title','RTAB 3D')+' | Green: ground | Orange: obstacle | Grey: unknown',width=1200,height=850):
        raise RuntimeError('3D window initialization failed')
    try:
        viewer.add_geometry(geometry)
        if trajectory:
            from .viewer import trajectory_geometries
            points=load_display_trajectory(summary)
            overlays=trajectory_geometries(points.tolist())
            for overlay in overlays:viewer.add_geometry(overlay,reset_bounding_box=False)
            print(f'경로 {len(points)}개 | 파랑: 카메라 이동 경로 / 초록 구: 출발 / 빨강 구: 도착',flush=True)
        option=viewer.get_render_option();option.background_color=[.07,.09,.12];option.mesh_show_back_face=True
        option.mesh_color_option=o3d.visualization.MeshColorOption.Color
        option.point_color_option=o3d.visualization.PointColorOption.Color
        option.line_width=4.
        option.mesh_shade_option=o3d.visualization.MeshShadeOption(1)
        if summary.get('render'):
            import tempfile
            with tempfile.TemporaryDirectory(prefix='rtab-render-') as temp:
                config=Path(temp)/'render.json';option.save_to_json(str(config))
                settings=json.loads(config.read_text())
                settings['light_ambient_color']=[summary['render'].get('ambient_light',.35)]*3
                for index in range(4):
                    settings[f'light{index}_diffuse_power']=.35
                    settings[f'light{index}_specular_power']=.04
                config.write_text(json.dumps(settings));option.load_from_json(str(config))
        control=viewer.get_view_control();control.set_lookat(geometry.get_center());control.set_up([0,0,1]);control.set_front([1,-1,.7]);control.set_zoom(.7)
        print(LEGEND+' | 왼쪽 드래그: 회전 / 휠: 확대 / Shift+드래그: 이동',flush=True)
        print('VIEWER_READY',path,flush=True)
        if snapshot is not None:
            for _ in range(15):viewer.poll_events();viewer.update_renderer();time.sleep(.02)
            viewer.capture_screen_image(str(snapshot),do_render=True)
            return
        while viewer.poll_events():viewer.update_renderer();time.sleep(.02)
    finally:viewer.destroy_window()


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='mode',required=True)
    b=sub.add_parser('build');b.add_argument('source',type=Path);b.add_argument('output',type=Path)
    v=sub.add_parser('view');v.add_argument('folder',type=Path);v.add_argument('--cloud',action='store_true');v.add_argument('--snapshot',type=Path);v.add_argument('--trajectory',action='store_true')
    args=ap.parse_args()
    if args.mode=='build':build(args.source,args.output)
    else:view(args.folder,args.cloud,args.snapshot,args.trajectory)

if __name__=='__main__':main()
