"""Compare both SLAM maps using identical measured depth and timestamps."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation, Slerp
from plotly.subplots import make_subplots
import plotly.graph_objects as go

from .compare_rtabmap import orb_poses, tum_poses
from .depth_filter import DepthFilter, spatial_filter
from .reconstruct import nearest


def interpolate(t, stamps, poses):
    i = np.searchsorted(stamps, t)
    if i == 0 or i == len(stamps) or stamps[i]-stamps[i-1] > 2.:
        return None
    w = (t-stamps[i-1])/(stamps[i]-stamps[i-1])
    p = np.eye(4)
    p[:3,3] = (1-w)*poses[i-1,:3,3]+w*poses[i,:3,3]
    p[:3,:3] = Slerp(stamps[i-1:i+1]-t, Rotation.from_matrix(poses[i-1:i+1,:3,:3]))([0]).as_matrix()[0]
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run',type=Path); ap.add_argument('output',type=Path)
    a=ap.parse_args(); run=a.run; out=a.output
    ot,op=orb_poses(run); rt,rp=tum_poses(out/'rtab_optimized.tum')
    anchor=np.argmin(abs(rt-ot[0])); op=np.linalg.inv(op[0])@op; rp=np.linalg.inv(rp[anchor])@rp
    info=json.loads((run/'capture_calibration.json').read_text())['depth']
    k=np.array(info['k']).reshape(3,3); scale=info['depth_scale_m']
    intrinsic=o3d.camera.PinholeCameraIntrinsic(info['width'],info['height'],k[0,0],k[1,1],k[0,2],k[1,2])
    volumes=[o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=.02,sdf_trunc=.08,color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor) for _ in range(2)]
    depths=sorted(int(p.stem) for p in (run/'raw/depth').glob('*.png'))
    config=DepthFilter(max_depth_m=2.,temporal_neighbours=0)
    count=0; last=-np.inf; gaps=[]
    for t,pose in zip(ot,op):
        if t-last < 1.: continue
        other=interpolate(t,rt,rp)
        stamp=nearest(depths,round(t*1e9))
        if stamp is None or other is None: continue
        image=cv2.imread(str(run/'raw/depth'/f'{stamp}.png'),-1)
        image,_=spatial_filter(image,scale,config)
        rgbd=o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.zeros((*image.shape,3),np.uint8)),o3d.geometry.Image(image),depth_scale=1/scale,depth_trunc=2.,convert_rgb_to_intensity=False)
        for volume,p in zip(volumes,[pose,other]):
            volume.integrate(rgbd,intrinsic,np.linalg.inv(p@np.array(info['T_left_depth'])))
        count+=1; last=t; gaps.append(float(np.linalg.norm(pose[:3,3]-other[:3,3])))
    fig=make_subplots(rows=1,cols=2,specs=[[{'type':'scene'},{'type':'scene'}]],subplot_titles=['ORB-SLAM3','RTAB-Map'])
    allpoints=[]
    for col,(label,volume,poses) in enumerate(zip(['orb','rtab'],volumes,[op,rp]),1):
        cloud=volume.extract_point_cloud(); o3d.io.write_point_cloud(str(out/f'{label}_common_depth.ply'),cloud)
        mesh=volume.extract_triangle_mesh(); mesh.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(out/f'{label}_common_depth_mesh.ply'),mesh)
        xyz=np.asarray(cloud.points); allpoints.append(xyz)
        xyz=xyz[::max(1,len(xyz)//60000)]
        fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],mode='markers',marker=dict(size=1,color=xyz[:,1],colorscale='Viridis',showscale=False),name=label+' depth'),row=1,col=col)
        xyz=poses[:,:3,3]
        fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],mode='lines',line=dict(color='#ef553b',width=4),name=label+' trajectory'),row=1,col=col)
    bounds=np.vstack(allpoints+[op[:,:3,3],rp[:,:3,3]])
    scene=dict(aspectmode='data',xaxis=dict(title='X (m)',range=[bounds[:,0].min(),bounds[:,0].max()]),yaxis=dict(title='Y (m)',range=[bounds[:,1].min(),bounds[:,1].max()]),zaxis=dict(title='Z (m)',range=[bounds[:,2].min(),bounds[:,2].max()]),camera=dict(up=dict(x=0,y=-1,z=0),eye=dict(x=1.3,y=-1.8,z=-1.3)))
    fig.update_layout(scene=scene,scene2=scene,title=f'Same measured depth: {count} frames, 2 m range, 2 cm TSDF | camera optical axes',height=800)
    fig.write_html(str(out/'maps_comparison.html'),include_plotlyjs=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    f,axes=plt.subplots(1,2,figsize=(12,6),sharex=True,sharey=True)
    for ax,label,xyz,poses in zip(axes,['ORB-SLAM3','RTAB-Map'],allpoints,[op,rp]):
        xyz=xyz[::max(1,len(xyz)//100000)]
        ax.scatter(xyz[:,0],xyz[:,2],c=xyz[:,1],s=.1,cmap='viridis',vmin=bounds[:,1].min(),vmax=bounds[:,1].max(),rasterized=True)
        ax.plot(poses[:,0,3],poses[:,2,3],color='orangered',lw=1)
        ax.set(title=label,xlabel='X (m)',ylabel='Z (m)',aspect='equal');ax.grid(alpha=.2)
    f.suptitle(f'Identical measured depth ({count} frames) | 2 m range / 2 cm TSDF');f.tight_layout();f.savefig(out/'maps_comparison.png',dpi=180)
    (out/'map_comparison_summary.json').write_text(json.dumps(dict(common_depth_frames=count,voxel_m=.02,depth_max_m=2.,max_interpolation_gap_s=2.,median_position_disagreement_m=float(np.median(gaps)),p95_position_disagreement_m=float(np.percentile(gaps,95)),ground_truth_accuracy=False,alignment='first camera SE3; no scale fitting',note='Position disagreement between algorithms is not accuracy. RTAB optimized poses interpolated at ORB timestamps; identical spatial-only depth filtering.'),indent=2))
    print('common depth frames:',count)

if __name__=='__main__': main()
