"""Preserve measured 2.5D terrain contours and connect only short, height-consistent gaps."""
import argparse
import copy
import json
from pathlib import Path
import time

import numpy as np
from scipy import ndimage as nd

from .block_terrain import floor_transform,clean_union
from .gravity import fingerprint
from .surface_colors import PALETTE
from .stl_export import write_stl_verified


def raster_quantile(points,lo,shape,res,quantile):
    ij=np.floor((points[:,:2]-lo)/res).astype(int)
    valid=(ij[:,0]>=0)&(ij[:,1]>=0)&(ij[:,0]<shape[1])&(ij[:,1]<shape[0])
    ij=ij[valid];values=points[valid,2];keys=ij[:,1]*shape[1]+ij[:,0]
    order=np.lexsort((values,keys));keys=keys[order];values=values[order]
    ids,start,counts=np.unique(keys,return_index=True,return_counts=True)
    positions=start+np.floor((counts-1)*quantile).astype(int)
    height=np.full(np.prod(shape),np.nan);height[ids]=values[positions]
    support=np.zeros(np.prod(shape),int);support[ids]=counts
    return height.reshape(shape),support.reshape(shape)


def join_mask(mask,heights,gap_m,res):
    radius=gap_m/2/res;n=int(np.ceil(radius));y,x=np.mgrid[-n:n+1,-n:n+1]
    kernel=x*x+y*y<=radius*radius+1e-8
    closed=nd.binary_closing(mask,structure=kernel)
    # Verify 3D agreement as well as XY proximity; do not connect a low and a high fragment.
    maximum=nd.maximum_filter(np.where(mask,heights,-np.inf),size=2*n+1)
    minimum=nd.minimum_filter(np.where(mask,heights,np.inf),size=2*n+1)
    local_span=maximum-minimum
    inferred=closed&~mask&(local_span<=gap_m)
    merged=mask|inferred
    components,number=nd.label(merged,np.ones((3,3)))
    sizes=np.bincount(components.ravel());keep=sizes>=4;keep[0]=False
    merged=keep[components];inferred&=merged
    return merged,inferred,dict(initial_components=nd.label(mask,np.ones((3,3)))[1],
                                merged_components=nd.label(merged,np.ones((3,3)))[1],
                                inferred_cells=int(inferred.sum()),removed_tiny_cells=int((mask&~merged).sum()),
                                gap_threshold_m=gap_m,maximum_nearby_height_span_m=gap_m)


def heightfield_mesh(height,lo,res,bottom):
    """One top graph, perimeter walls and a bottom fan: closed solid with no box fitting."""
    import open3d as o3d
    ny,nx=height.shape;y,x=np.mgrid[:ny,:nx]
    vertices=np.column_stack([lo[0]+x.ravel()*res,lo[1]+y.ravel()*res,height.ravel()])
    a=(np.arange(ny-1)[:,None]*nx+np.arange(nx-1)[None,:]).ravel()
    triangles=np.r_[np.c_[a,a+1,a+nx],np.c_[a+1,a+nx+1,a+nx]]
    ring=np.r_[np.arange(nx),np.arange(1,ny)*nx+nx-1,
               (ny-1)*nx+np.arange(nx-2,-1,-1),np.arange(ny-2,0,-1)*nx]
    base=vertices[ring].copy();base[:,2]=bottom
    ids=np.arange(len(ring))+len(vertices);next_ids=np.roll(ids,-1);next_ring=np.roll(ring,-1)
    centre=np.array([vertices[ring,0].mean(),vertices[ring,1].mean(),bottom])
    centre_id=len(vertices)+len(base)
    side=np.r_[np.c_[ring,ids,next_ids],np.c_[ring,next_ids,next_ring]]
    bottom_faces=np.c_[np.full(len(ring),centre_id),next_ids,ids]
    vertices=np.r_[vertices,base,centre[None]];triangles=np.r_[triangles,side,bottom_faces]
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),o3d.utility.Vector3iVector(triangles))


def build(source,out):
    import open3d as o3d
    import manifold3d as manifold
    metadata=json.loads((source/'summary.json').read_text());source_cloud=source/metadata['cloud'];digest=fingerprint(source_cloud)
    cloud=o3d.io.read_point_cloud(str(source_cloud));xyz=np.asarray(cloud.points)
    labels=np.argmin(np.linalg.norm(np.asarray(cloud.colors)[:,None]-PALETTE[None],axis=2),axis=1)
    ground=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[labels==1])).voxel_down_sample(.03)
    o3d.utility.random.seed(7);plane,_=ground.segment_plane(.025,3,1000)
    matrix=floor_transform(plane);xyz=xyz@matrix[:3,:3].T+matrix[:3,3]
    res=.02;gap=.08
    g=xyz[labels==1];obstacle=xyz[labels==2]
    glo,ghi=np.percentile(g[:,:2],[.5,99.5],axis=0)
    olo,ohi=np.percentile(obstacle[:,:2],[.2,99.8],axis=0) if len(obstacle) else (glo,ghi)
    lo=np.minimum(glo,olo)-.08;hi=np.maximum(ghi,ohi)+.08
    shape=tuple((np.ceil((hi-lo)/res).astype(int)+1)[::-1]);ground_z,gcount=raster_quantile(g,lo,shape,res,.5)
    distance,near=nd.distance_transform_edt(gcount==0,return_indices=True)
    ground_z=ground_z[tuple(near)]
    # Do not extend a small elevated ground fragment across unobserved base area.
    support_weight=np.clip((.12-distance*res)/.08,0.,1.)
    ground_z*=support_weight
    # Smooth centimetre-scale floor noise while retaining measured local slopes.
    ground_z=nd.gaussian_filter(ground_z,sigma=1.5)
    observed_top,count=raster_quantile(obstacle,lo,shape,res,.85)
    observed=(count>=2)&(observed_top-ground_z>=.045)
    mask,inferred,stats=join_mask(observed,observed_top,gap,res)
    nearest=nd.distance_transform_edt(~observed,return_distances=False,return_indices=True)
    top=observed_top[tuple(nearest)] if observed.any() else ground_z.copy()
    # Suppress two-centimetre comb-like notches without replacing the footprint by a box.
    top=np.where(mask,nd.median_filter(nd.grey_closing(top,size=(3,3)),size=3),top)
    # Height-aware local smoothing stays inside the footprint and preserves large height steps.
    for _ in range(2):
        numerator=top.copy()*2;denominator=np.full(shape,2.)
        padded=np.pad(top,1,mode='edge');padded_mask=np.pad(mask,1)
        for dy in [-1,0,1]:
            for dx in [-1,0,1]:
                if dx==dy==0:continue
                other=padded[1+dy:1+dy+shape[0],1+dx:1+dx+shape[1]]
                valid=padded_mask[1+dy:1+dy+shape[0],1+dx:1+dx+shape[1]]&mask&(abs(other-top)<.04)
                numerator+=np.where(valid,other,0);denominator+=valid
        top=np.where(mask,numerator/denominator,top)
    height=np.where(mask,np.maximum(ground_z+.045,top),ground_z)
    bottom=float(ground_z.min()-.08)
    mesh=heightfield_mesh(height,lo,res,bottom)
    # Error-bounded decimation retains the measured outlines instead of replacing them with boxes.
    solid=manifold.Manifold(manifold.Mesh(np.asarray(mesh.vertices,dtype=np.float32),np.asarray(mesh.triangles,dtype=np.uint32)))
    for tolerance in [.004,.002,.001,0.]:
        reduced=(solid.simplify(tolerance) if tolerance else solid).to_mesh()
        mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(reduced.vert_properties)[:,:3].astype(float)),o3d.utility.Vector3iVector(np.asarray(reduced.tri_verts).astype(np.int32)))
        mesh=clean_union(mesh)
        if mesh.is_watertight() and not mesh.is_self_intersecting():break
    else:raise ValueError('Joined terrain must be watertight and nonintersecting')
    _,parts,_=mesh.cluster_connected_triangles()
    if len(parts)!=1:raise ValueError('Joined terrain must be one connected shell')
    # Colours are for viewing. STL keeps only the solid geometry.
    v=np.asarray(mesh.vertices);ij=np.rint((v[:,:2]-lo)/res).astype(int)
    ix=np.clip(ij[:,0],0,shape[1]-1);iy=np.clip(ij[:,1],0,shape[0]-1)
    raised=(v[:,2]>ground_z[iy,ix]+.035)
    colors=np.tile([.25,.65,.38],(len(v),1));colors[raised]=[1.,.38,.10]
    if metadata.get('mapping_source'):
        from .surface_colors import load_surface_layers, surface_point_colors
        terrain=load_surface_layers(Path(metadata['mapping_source']))
        if terrain is not None:
            original_points=(v-matrix[:3,3])@matrix[:3,:3]
            colors=surface_point_colors(original_points,np.where(raised,2,1),terrain)
    else:
        from .surface_colors import SLOPE_COLOR, SLOPE_THRESHOLD_DEG
        gy,gx=np.gradient(ground_z,res)
        slope=np.degrees(np.arctan(np.hypot(gx[iy,ix],gy[iy,ix])))
        colors[(~raised)&(slope>=SLOPE_THRESHOLD_DEG)]=SLOPE_COLOR
    mesh.vertex_colors=o3d.utility.Vector3dVector(colors)
    out.mkdir(parents=True,exist_ok=False)
    stl_m=write_stl_verified(out/'terrain_m.stl',mesh);mm=copy.deepcopy(mesh);mm.scale(1000,center=[0,0,0]);stl_mm=write_stl_verified(out/'terrain_mm.stl',mm)
    o3d.io.write_triangle_mesh(str(out/'terrain.ply'),mesh)
    np.savez_compressed(out/'heightfield.npz',height=height,ground=ground_z,obstacle_mask=mask,inferred_connections=inferred,observed_mask=observed,origin=lo,resolution=res)
    if fingerprint(source_cloud)!=digest:raise RuntimeError('Source changed')
    summary=dict(source=str(source.resolve()),source_cloud=str(source_cloud.resolve()),source_sha256=digest,
                 floor_from_source=matrix.tolist(),mesh='terrain.ply',grid_resolution_m=res,join=stats,
                 simplify_tolerance_m=tolerance,triangles=len(mesh.triangles),watertight=True,self_intersections=False,
                 connected_shells=1,stl_m=stl_m,stl_mm=stl_mm,
                 assumptions=['2.5D top surface: overhangs and undercuts are filled down to ground.',
                              'Unobserved sides and short joined gaps are inferred, not measurements.',
                              'Closed rectangular base extends beneath the model.',
                              'Ground heights fade to the fitted plane beyond 4–12cm from measured floor support.',
                              'A 3x3-cell local closing/median removes small top-surface comb noise.',
                              'Small footprint groups under four 2cm cells are removed.'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(join=stats,triangles=len(mesh.triangles),watertight=True),indent=2),flush=True)
    return summary


def view(folder,snapshot=None):
    import open3d as o3d
    from .rtabmap_surface import load_display_trajectory
    from .viewer import trajectory_geometries
    summary=json.loads((folder/'summary.json').read_text());mesh=o3d.io.read_triangle_mesh(str(folder/summary['mesh']))
    mesh.compute_triangle_normals();matrix=np.array(summary['floor_from_source'])
    raw=o3d.io.read_point_cloud(summary['source_cloud']).voxel_down_sample(.03);raw.transform(matrix);raw.paint_uniform_color([.75,.8,.85])
    source_meta=json.loads((Path(summary['source'])/'summary.json').read_text())
    if 'mapping_source' in source_meta:
        from .viewer import load_trajectory
        points=np.asarray(load_trajectory(Path(source_meta['mapping_source'])),dtype=float).reshape(-1,3)
    else:
        points=load_display_trajectory(source_meta)
    points=points@matrix[:3,:3].T+matrix[:3,3]
    overlays=trajectory_geometries(points.tolist())
    viewer=o3d.visualization.VisualizerWithKeyCallback()
    if not viewer.create_window(window_name='Joined terrain | 8 cm local gaps | O: measured source overlay',width=1200,height=850):raise RuntimeError('3D initialization failed')
    visible=False
    def toggle(_):
        nonlocal visible
        if visible:viewer.remove_geometry(raw,reset_bounding_box=False)
        else:viewer.add_geometry(raw,reset_bounding_box=False)
        visible=not visible;return False
    try:
        viewer.add_geometry(mesh)
        for part in overlays:viewer.add_geometry(part,reset_bounding_box=False)
        viewer.register_key_callback(ord('O'),toggle)
        option=viewer.get_render_option();option.background_color=[.07,.09,.12];option.mesh_show_back_face=True
        option.mesh_shade_option=o3d.visualization.MeshShadeOption(0);option.line_width=4.
        control=viewer.get_view_control();control.set_up([0,0,1]);control.set_front([1,-1,.85]);control.set_zoom(.65)
        print('VIEWER_READY | 8cm 근접 틈 연결 / O: 원본 겹치기 / 파랑 경로·초록 출발·빨강 도착',flush=True)
        if snapshot:
            for _ in range(15):viewer.poll_events();viewer.update_renderer();time.sleep(.02)
            viewer.capture_screen_image(str(snapshot),do_render=True);return
        while viewer.poll_events():viewer.update_renderer();time.sleep(.02)
    finally:viewer.destroy_window()


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='mode',required=True)
    b=sub.add_parser('build');b.add_argument('source',type=Path);b.add_argument('output',type=Path)
    v=sub.add_parser('view');v.add_argument('folder',type=Path);v.add_argument('--snapshot',type=Path)
    args=ap.parse_args()
    if args.mode=='build':build(args.source,args.output)
    else:view(args.folder,args.snapshot)

if __name__=='__main__':main()
