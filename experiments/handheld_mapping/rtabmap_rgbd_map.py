"""Build a native-viewer mesh from optimized RTAB RGB-D graph nodes and preserved depth."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from .compare_rtabmap import tum_poses
from .depth_filter import DepthFilter,spatial_filter
from .reconstruct import nearest
from .rtabmap_rgbd import validate_calibration


def build(run,out):
    if (out/'rtab_map_summary.json').exists():
        raise FileExistsError('Map already complete; use a new output directory for regeneration')
    calibration=json.loads((run/'capture_calibration.json').read_text());info=validate_calibration(calibration)
    t,poses=tum_poses(out/'rtab_optimized.tum')
    if not len(t):raise ValueError('Empty optimized graph')
    depths=sorted(int(p.stem) for p in (run/'raw/depth').glob('*.png'))
    k=info['k'];scale=info['depth_scale_m']
    intrinsic=o3d.camera.PinholeCameraIntrinsic(info['width'],info['height'],k[0],k[4],k[2],k[5])
    volume=o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=.01,sdf_trunc=.04,color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
    filtering=DepthFilter(max_depth_m=2.,temporal_neighbours=0)
    integrated=0;skipped=[]
    color=o3d.geometry.Image(np.zeros((info['height'],info['width'],3),np.uint8))
    for index,(stamp,pose) in enumerate(zip(t,poses)):
        match=nearest(depths,round(stamp*1e9))
        if match is None:skipped.append(float(stamp));continue
        depth=cv2.imread(str(run/'raw/depth'/f'{match}.png'),cv2.IMREAD_UNCHANGED)
        if depth is None or depth.dtype!=np.uint16:raise ValueError(f'Invalid depth: {match}')
        depth,_=spatial_filter(depth,scale,filtering)
        if np.count_nonzero(depth)<100:skipped.append(float(stamp));continue
        rgbd=o3d.geometry.RGBDImage.create_from_color_and_depth(color,o3d.geometry.Image(depth),depth_scale=1/scale,depth_trunc=2.,convert_rgb_to_intensity=False)
        volume.integrate(rgbd,intrinsic,np.linalg.inv(pose@np.array(info['T_left_depth'])))
        integrated+=1
        if integrated%50==0:print(f'{integrated}/{len(t)} optimized graph nodes integrated',flush=True)
    if integrated<2:raise ValueError('Insufficient measured depth to build map')
    mesh=volume.extract_triangle_mesh();mesh.compute_vertex_normals();cloud=volume.extract_point_cloud()
    if not len(mesh.triangles):raise ValueError('Empty TSDF surface')
    for geometry,name,writer in [(mesh,'rtab_map_mesh.ply',o3d.io.write_triangle_mesh),(cloud,'rtab_map_cloud.ply',o3d.io.write_point_cloud)]:
        if not writer(str(out/name),geometry):raise OSError(f'Failed saving {name}')
    summary=dict(source=str(run),pose_source='RTAB RGB-D final optimized camera graph',
                 graph_nodes=len(t),integrated_depth_frames=integrated,skipped_timestamps=skipped,
                 voxel_m=.01,depth_max_m=2.,truncation_m=.04,depth_filter=filtering.metadata(),
                 mesh='rtab_map_mesh.ply',cloud='rtab_map_cloud.ply',vertices=len(mesh.vertices),
                 triangles=len(mesh.triangles),coordinate_frame='initial camera optical; no gravity alignment',
                 raw_preserved=True,ground_truth_accuracy_evaluated=False)
    (out/'rtab_map_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    # Diagnostic preview only; actual map is opened in the native Open3D viewer.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    xyz=np.asarray(cloud.points);xyz=xyz[::max(1,len(xyz)//80000)]
    fig=plt.figure(figsize=(10,8));ax=fig.add_subplot(projection='3d')
    ax.scatter(xyz[:,0],xyz[:,2],-xyz[:,1],s=.15,c=-xyz[:,1],cmap='terrain',rasterized=True)
    ax.set(xlabel='Optical X (m)',ylabel='Optical Z (m)',zlabel='-Optical Y (m)',title='RTAB-Map: measured-depth replay / 1 cm TSDF')
    span=np.ptp(xyz,axis=0);ax.set_box_aspect([span[0],span[2],span[1]]);ax.view_init(elev=28,azim=-65)
    fig.tight_layout();fig.savefig(out/'rtab_map_preview.png',dpi=170);plt.close(fig)
    return summary


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();print(json.dumps(build(args.run.resolve(),args.output.resolve()),indent=2))

if __name__=='__main__':main()
