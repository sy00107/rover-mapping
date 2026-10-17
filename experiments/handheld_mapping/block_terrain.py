"""Approximate classified terrain with a floor slab and rectangular obstacle solids."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import ConvexHull

from .gravity import fingerprint
from .surface_colors import PALETTE
from .stl_export import write_stl_verified


def floor_transform(plane):
    plane=np.asarray(plane,float);plane=plane/np.linalg.norm(plane[:3])
    if plane[2]<0:plane=-plane
    normal=plane[:3];x=np.array([1.,0,0]);x-=normal*np.dot(x,normal);x/=np.linalg.norm(x)
    matrix=np.eye(4);matrix[:3,:3]=np.stack([x,np.cross(normal,x),normal]);matrix[2,3]=plane[3]
    return matrix


def fit_rectangle(points):
    xy=np.asarray(points,float)[:,:2]
    hull=xy[ConvexHull(xy).vertices]
    edges=np.roll(hull,-1,axis=0)-hull
    angles=np.unique(np.mod(np.arctan2(edges[:,1],edges[:,0]),np.pi/2))
    best=None
    for angle in angles:
        c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s],[s,c]])
        local=xy@rotation
        lo,hi=np.percentile(local,[2,98],axis=0)
        size=np.maximum(hi-lo,.06);centre=(lo+hi)/2
        score=np.prod(size)
        if best is None or score<best[0]:best=(score,centre@rotation.T,size,float(angle))
    _,centre,size,angle=best
    return centre,size,angle


def box_mesh(obj):
    import open3d as o3d
    width,depth,height=obj['size_m'];yaw=obj['yaw_rad'];c,s=np.cos(yaw),np.sin(yaw)
    mesh=o3d.geometry.TriangleMesh.create_box(width,depth,height)
    mesh.translate([-width/2,-depth/2,0]);mesh.rotate(np.array([[c,-s,0],[s,c,0],[0,0,1]]),center=[0,0,0])
    mesh.translate(obj['bottom_center_m']);mesh.compute_vertex_normals()
    mesh.paint_uniform_color(obj['color']);return mesh


def clean_union(mesh):
    mesh.remove_duplicated_vertices();mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals();return mesh


def union_boxes(objects):
    import manifold3d as manifold
    import open3d as o3d
    solids=[]
    for obj in objects:
        width,depth,height=obj['size_m']
        solid=manifold.Manifold.cube([width,depth,height]).translate([-width/2,-depth/2,0])
        solid=solid.rotate([0,0,np.degrees(obj['yaw_rad'])]).translate(obj['bottom_center_m'])
        solids.append(solid)
    merged=manifold.Manifold.batch_boolean(solids,manifold.OpType.Add)
    result=merged.to_mesh()
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(result.vert_properties)[:,:3].astype(float)),
                                  o3d.utility.Vector3iVector(np.asarray(result.tri_verts).astype(np.int32)))
    return clean_union(mesh)


def build(source,output):
    import open3d as o3d
    source=source.resolve();output=output.resolve()
    metadata=json.loads((source/'summary.json').read_text());cloud_path=source/metadata['cloud']
    source_hash=fingerprint(cloud_path)
    cloud=o3d.io.read_point_cloud(str(cloud_path));xyz=np.asarray(cloud.points);colors=np.asarray(cloud.colors)
    labels=np.argmin(np.linalg.norm(colors[:,None]-PALETTE[None],axis=2),axis=1)
    ground=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[labels==1])).voxel_down_sample(.03)
    o3d.utility.random.seed(7)
    plane,inliers=ground.segment_plane(distance_threshold=.025,ransac_n=3,num_iterations=1000)
    if len(inliers)<.6*len(ground.points):raise ValueError('No dominant floor plane; manual floor selection required')
    matrix=floor_transform(plane)
    transformed=xyz@matrix[:3,:3].T+matrix[:3,3]
    obstacles=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(transformed[labels==2])).voxel_down_sample(.03)
    cluster=np.array(obstacles.cluster_dbscan(eps=.10,min_points=5));points=np.asarray(obstacles.points)
    objects=[];rejected=0
    for group in np.unique(cluster):
        if group<0:continue
        values=points[cluster==group]
        if len(values)<60:rejected+=1;continue
        # Fill unobserved sides as a rectangular solid, without identifying object semantics.
        centre,size,angle=fit_rectangle(values)
        top=float(np.percentile(values[:,2],97))
        if top<.07:rejected+=1;continue
        objects.append(dict(name=f'obstacle_{len(objects)+1:02d}',kind='rectangular_obstacle',
                            bottom_center_m=[float(centre[0]),float(centre[1]),-.025],
                            size_m=[float(size[0]),float(size[1]),max(.10,top+.025)],yaw_rad=angle,
                            support_points=len(values),color=[1.,.38,.10]))
    if not objects:raise ValueError('No supported obstacle groups')
    floor_points=transformed[labels==1]
    lo,hi=np.percentile(floor_points[:,:2],[.5,99.5],axis=0)
    for obj in objects:
        half=np.linalg.norm(obj['size_m'][:2])/2
        centre=np.array(obj['bottom_center_m'][:2]);lo=np.minimum(lo,centre-half-.05);hi=np.maximum(hi,centre+half+.05)
    floor=dict(name='floor',kind='floor_slab',bottom_center_m=[float((lo[0]+hi[0])/2),float((lo[1]+hi[1])/2),-.10],
               size_m=[float(hi[0]-lo[0]),float(hi[1]-lo[1]),.10],yaw_rad=0.,color=[.25,.65,.38])
    objects=[floor,*objects]
    output.mkdir(parents=True,exist_ok=False)
    (output/'parts').mkdir()
    for index,obj in enumerate(objects):
        part=box_mesh(obj)
        if not part.is_watertight():raise ValueError('Primitive must be watertight')
        write_stl_verified(output/'parts'/f'{obj["name"]}_m.stl',part)
    union=union_boxes(objects)
    if not union.is_watertight() or union.is_self_intersecting():
        raise ValueError('Combined STL is not a closed nonintersecting solid')
    components,counts,_=union.cluster_connected_triangles()
    if len(counts)!=1:raise ValueError('Combined STL must have a single connected shell')
    meter_stl=write_stl_verified(output/'terrain_m.stl',union)
    import copy
    mm=copy.deepcopy(union);mm.scale(1000.,center=[0,0,0])
    mm_stl=write_stl_verified(output/'terrain_mm.stl',mm)
    o3d.io.write_triangle_mesh(str(output/'terrain_union.ply'),union)
    if fingerprint(cloud_path)!=source_hash:raise RuntimeError('Source changed during fitting')
    summary=dict(source=str(source),source_cloud=str(cloud_path),source_sha256=source_hash,
                 floor_from_source=matrix.tolist(),objects=objects,obstacle_count=len(objects)-1,
                 floor_plane_source=np.asarray(plane).tolist(),floor_plane_support_fraction=len(inliers)/len(ground.points),
                 separate_ramps=0,rejected_small_groups=rejected,cluster_noise_points=int((cluster<0).sum()),
                 connected_shells=len(counts),watertight=True,self_intersections=False,
                 triangles=len(union.triangles),stl_m=meter_stl,stl_mm=mm_stl,
                 modelling='Single fitted floor slab plus filled rectangular obstacle bounds; unseen surfaces inferred.',
                 assumptions=['All retained obstacle clusters are rectangular solids extending to the floor.',
                              'No desk/leg semantics; unknown clutter excluded.',
                              'Small observed gaps and unobserved sides are intentionally simplified.',
                              'Approximate dimensions; STL stores numeric coordinates without unit metadata.',
                              'No independently supported large ramp extracted from this dominant-plane recording.'])
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    preview(output,summary)
    print(json.dumps({k:summary[k] for k in ['obstacle_count','connected_shells','watertight','triangles']},indent=2))
    return summary


def preview(output,summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(13,9));ax=fig.add_subplot(projection='3d');all_points=[]
    for obj in summary['objects']:
        mesh=box_mesh(obj);v=np.asarray(mesh.vertices);all_points.append(v)
        collection=Poly3DCollection(v[np.asarray(mesh.triangles)],facecolors=obj['color'],edgecolors=(.12,.15,.17,.5),linewidths=.3,alpha=.2 if obj['kind']=='floor_slab' else 1.)
        ax.add_collection3d(collection)
    xyz=np.vstack(all_points);lo=xyz.min(axis=0);hi=xyz.max(axis=0)
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]+.1),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title=f'Simplified terrain: floor + {summary["obstacle_count"]} rectangular obstacles')
    ax.set_box_aspect(np.maximum(hi-lo,.1));ax.view_init(35,-65)
    from matplotlib.ticker import MaxNLocator
    ax.zaxis.set_major_locator(MaxNLocator(4));fig.tight_layout();fig.savefig(output/'preview.png',dpi=170);plt.close(fig)


def view(folder,snapshot=None,trajectory=True):
    import open3d as o3d
    summary=json.loads((folder/'summary.json').read_text())
    parts=[box_mesh(obj) for obj in summary['objects']]
    source=o3d.io.read_point_cloud(summary['source_cloud']).voxel_down_sample(.035)
    source.transform(np.array(summary['floor_from_source']));source.paint_uniform_color([.7,.75,.8])
    viewer=o3d.visualization.VisualizerWithKeyCallback()
    if not viewer.create_window(window_name='Block terrain | O: source overlay | Green: floor | Orange: rectangular obstacle',width=1200,height=850):raise RuntimeError('3D window initialization failed')
    visible=False
    def toggle(_):
        nonlocal visible
        if visible:viewer.remove_geometry(source,reset_bounding_box=False)
        else:viewer.add_geometry(source,reset_bounding_box=False)
        visible=not visible;return False
    try:
        for part in parts:viewer.add_geometry(part)
        if trajectory:
            from .rtabmap_surface import load_display_trajectory
            from .viewer import trajectory_geometries
            source_summary=json.loads((Path(summary['source'])/'summary.json').read_text())
            points=load_display_trajectory(source_summary)
            matrix=np.array(summary['floor_from_source'])
            points=points@matrix[:3,:3].T+matrix[:3,3]
            overlays=trajectory_geometries(points.tolist())
            for geometry in overlays:viewer.add_geometry(geometry,reset_bounding_box=False)
            print('파랑: 이동 경로 / 초록 구: 출발 / 빨강 구: 도착 (카메라 추정 위치)',flush=True)
        viewer.register_key_callback(ord('O'),toggle)
        option=viewer.get_render_option();option.background_color=[.07,.09,.12];option.mesh_show_back_face=True
        option.mesh_shade_option=o3d.visualization.MeshShadeOption(0);option.line_width=4.
        centre=(np.asarray(summary['objects'][0]['bottom_center_m'])+np.array([0,0,.1]))
        control=viewer.get_view_control();control.set_lookat(centre);control.set_up([0,0,1]);control.set_front([1,-1,.85]);control.set_zoom(.65)
        print('VIEWER_READY | O: 원본 점군 겹치기 | 드래그: 회전 / 휠: 확대',flush=True)
        if snapshot:
            for _ in range(15):viewer.poll_events();viewer.update_renderer();time.sleep(.02)
            viewer.capture_screen_image(str(snapshot),do_render=True);return
        while viewer.poll_events():viewer.update_renderer();time.sleep(.02)
    finally:viewer.destroy_window()


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='mode',required=True)
    b=sub.add_parser('build');b.add_argument('source',type=Path);b.add_argument('output',type=Path)
    v=sub.add_parser('view');v.add_argument('folder',type=Path);v.add_argument('--snapshot',type=Path);v.add_argument('--no-trajectory',action='store_true')
    a=ap.parse_args()
    if a.mode=='build':build(a.source,a.output)
    else:view(a.folder,a.snapshot,not a.no_trajectory)

if __name__=='__main__':main()
