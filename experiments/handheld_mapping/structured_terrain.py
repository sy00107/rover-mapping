"""Contour-constrained map export: planar terrain, structured obstacle patches, shared origin."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import shapely as sh
from shapely.geometry import Polygon, MultiPolygon, LineString
from shapely.ops import split


def polygons(geometry):
    if isinstance(geometry,Polygon):return [geometry] if not geometry.is_empty else []
    return [p for part in getattr(geometry,'geoms',[]) for p in polygons(part)]


def footprint(points,resolution=.025,closing=.04,simplify=.02,minimum_area=.012):
    """Union sampled support cells, bounded closing, topology-preserving contour reduction."""
    points=np.asarray(points)
    if not len(points):return MultiPolygon([])
    cells=np.unique(np.floor(points/resolution).astype(int),axis=0)
    lo=cells*resolution
    shape=sh.union_all(sh.box(lo[:,0],lo[:,1],lo[:,0]+resolution,lo[:,1]+resolution))
    if closing:
        shape=shape.buffer(closing,join_style='mitre').buffer(-closing,join_style='mitre')
    shape=sh.make_valid(shape.simplify(simplify,preserve_topology=True))
    result=[]
    for p in polygons(shape):
        if p.area<minimum_area:continue
        # Only small enclosed gaps are inferred. Large holes remain explicit.
        holes=[ring.coords for ring in p.interiors if Polygon(ring).area>.015]
        result.append(Polygon(p.exterior.coords,holes))
    return sh.union_all(result)


def mesh_from_triangles(vertices,faces):
    import open3d as o3d
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(vertices,float).reshape(-1,3)),
                                  o3d.utility.Vector3iVector(np.asarray(faces,np.int32).reshape(-1,3)))
    mesh.remove_duplicated_vertices();mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles();mesh.remove_unreferenced_vertices()
    return mesh


def ground_mesh(shape,planes):
    """Split contour at all plane junctions before constrained triangulation."""
    coefficients=np.array([p['coefficient'] for p in planes]);regions=polygons(shape)
    bounds=shape.bounds;extent=max(bounds[2]-bounds[0],bounds[3]-bounds[1])*4+1
    center=np.array([(bounds[0]+bounds[2])/2,(bounds[1]+bounds[3])/2])
    for i in range(len(coefficients)):
        for j in range(i):
            a,b,c=coefficients[i]-coefficients[j];normal=np.array([a,b]);length=np.linalg.norm(normal)
            if length<1e-8:continue
            anchor=center-normal*(normal@center+c)/(length*length)
            direction=np.array([-b,a])/length*extent
            line=LineString([anchor-direction,anchor+direction])
            regions=[piece for region in regions for piece in polygons(split(region,line))]
    vertices=[];faces=[]
    for region in regions:
        for triangle in sh.constrained_delaunay_triangles(region).geoms:
            xy=np.array(triangle.exterior.coords)[:3]
            cross=np.cross(xy[1]-xy[0],xy[2]-xy[0])
            if cross<0:xy=xy[::-1]
            z=np.max(np.c_[xy,np.ones(3)]@coefficients.T,axis=1)
            start=len(vertices);vertices.extend(np.c_[xy,z]);faces.append([start,start+1,start+2])
    # Shared XY vertices have one exact height; avoid nearly coplanar STL slivers.
    vertices=np.array(vertices);xy,inverse=np.unique(np.round(vertices[:,:2],8),axis=0,return_inverse=True)
    vertices=np.c_[xy,np.max(np.c_[xy,np.ones(len(xy))]@coefficients.T,axis=1)]
    return mesh_from_triangles(vertices,inverse[np.array(faces)])


def planar_obstacles(source_mesh,labels):
    """Replace well-supported local plane patches; retain simplified nonplanar remainder."""
    import open3d as o3d
    xyz=np.asarray(source_mesh.vertices);triangles=np.asarray(source_mesh.triangles)
    source_mesh.compute_vertex_normals();normals=np.asarray(source_mesh.vertex_normals)
    pcd=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[labels==2]))
    pcd.normals=o3d.utility.Vector3dVector(normals[labels==2]);pcd=pcd.voxel_down_sample(.015)
    points=np.asarray(pcd.points);all_normals=np.asarray(pcd.normals)
    groups=np.array(pcd.cluster_dbscan(eps=.075,min_points=10));patches=[];patch_info=[];regions=[]
    o3d.utility.random.seed(17)
    for group in np.unique(groups):
        if group<0:continue
        indices=np.flatnonzero(groups==group)
        if len(indices)<100:continue
        remaining=indices.copy()
        for _ in range(12):
            if len(remaining)<90:break
            cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[remaining]))
            model,inliers=cloud.segment_plane(.012,3,1200)
            if len(inliers)<90:break
            n=np.array(model[:3]);candidate=remaining[np.array(inliers)]
            compatible=np.abs(all_normals[candidate]@n)>.80
            ids=candidate[compatible]
            # Remove candidates even on rejection so a noisy plane does not repeat.
            remaining=np.setdiff1d(remaining,candidate)
            if len(ids)<70:continue
            values=points[ids];origin=values.mean(axis=0)
            _,_,vh=np.linalg.svd(values-origin,full_matrices=False);u,v,n=vh
            if np.dot(np.cross(u,v),n)<0:v=-v
            uv=(values-origin)@np.stack([u,v]).T
            shape=footprint(uv,.02,.025,.015,.012)
            if shape.is_empty or shape.area<.02:continue
            verts=[];faces=[]
            for polygon in polygons(shape):
                for tri in sh.constrained_delaunay_triangles(polygon).geoms:
                    xy=np.array(tri.exterior.coords)[:3]
                    if np.cross(xy[1]-xy[0],xy[2]-xy[0])<0:xy=xy[::-1]
                    start=len(verts);verts.extend(origin+xy[:,0,None]*u+xy[:,1,None]*v);faces.append([start,start+1,start+2])
            patch=mesh_from_triangles(verts,faces)
            patches.append(patch);regions.append((origin,u,v,n,shape))
            patch_info.append(dict(support=len(ids),area_m2=float(shape.area),
                plane_rms_m=float(np.sqrt(np.mean(((values-origin)@n)**2))),normal=n.tolist()))
    keep=(labels[triangles]==2).sum(axis=1)>=2
    centers=xyz[triangles].mean(axis=1)
    replaced=np.zeros(len(triangles),bool)
    for origin,u,v,n,shape in regions:
        delta=centers-origin;near=keep&(abs(delta@n)<.025)
        ids=np.flatnonzero(near)
        uv=np.c_[delta[ids]@u,delta[ids]@v]
        inside=sh.covers(shape.buffer(.012),sh.points(uv))
        replaced[ids[inside]]=True
    remainder=copy.deepcopy(source_mesh)
    remainder.triangles=o3d.utility.Vector3iVector(triangles[keep&~replaced]);remainder.remove_unreferenced_vertices()
    removed=0
    if len(remainder.triangles):
        clusters,counts,areas=remainder.cluster_connected_triangles()
        counts=np.array(counts);areas=np.array(areas)
        valid=(counts>=70)&(areas>=.008)
        remove=~valid[np.asarray(clusters)];removed=int(remove.sum())
        remainder.remove_triangles_by_mask(remove);remainder.remove_unreferenced_vertices()
        if len(remainder.triangles):
            # Preserve nonplanar objects; bounded simplification rather than box filling.
            remainder=remainder.simplify_quadric_decimation(max(150,int(len(remainder.triangles)*.25)),maximum_error=.000025)
    remainder_count=len(remainder.triangles)
    result=copy.deepcopy(remainder)
    for patch in patches:result+=patch
    result.remove_duplicated_vertices();result.remove_degenerate_triangles();result.compute_vertex_normals()
    return result,dict(planar_patches=patch_info,patch_count=len(patches),
        replaced_source_triangles=int(replaced.sum()),removed_small_fragment_triangles=removed,
        nonplanar_remainder_triangles=remainder_count,output_triangles=len(result.triangles),
        limitation='Obstacle back sides are not observed; reconstructed patches remain surfaces.')


def validate_stl(path):
    import open3d as o3d
    mesh=o3d.io.read_triangle_mesh(str(path));mesh.remove_duplicated_vertices()
    return dict(triangles=len(mesh.triangles),watertight=mesh.is_watertight(),
                self_intersecting=mesh.is_self_intersecting(),
                nonmanifold_edges=len(mesh.get_non_manifold_edges(allow_boundary_edges=True)))


def build(source,output):
    import open3d as o3d
    from .surface_colors import load_surface_layers,classify_surface_points,PALETTE,SLOPE_COLOR
    from .planar_terrain import fit_planes,ground_solid,start_ground_origin
    from .viewer import load_trajectory,map_frame
    from .stl_export import write_stl_verified
    from .gravity import fingerprint
    source=Path(source).resolve();output=Path(output).resolve();digest=fingerprint(source)
    terrain=load_surface_layers(source)
    if terrain is None or not map_frame(source)[1]:raise ValueError('Classified gravity-aligned source required')
    mesh=o3d.io.read_triangle_mesh(str(source));xyz=np.asarray(mesh.vertices)
    labels=classify_surface_points(xyz,terrain);ground=xyz[labels==1];planes=fit_planes(ground)
    coefficient=np.array([p['coefficient'] for p in planes])
    fitted=np.c_[ground[:,:2],np.ones(len(ground))]@coefficient.T
    supported=np.min(abs(fitted-ground[:,2,None]),axis=1)<=.035
    ground=ground[supported]
    predicted=np.max(np.c_[ground[:,:2],np.ones(len(ground))]@coefficient.T,axis=1)
    agreement=abs(predicted-ground[:,2])<=.04
    if agreement.mean()<.85:raise ValueError('Unsupported planar junction: explicit region segmentation needed')
    ground=ground[agreement]
    shape=footprint(ground[:,:2],.025,.055,.025,.04)
    observed_shape=footprint(ground[:,:2],.025,0,0,0)
    # Under a nearby observed object, ground extension is an explicit model inference.
    obstacles=xyz[labels==2];distance=cKDTree(ground[:,:2]).query(obstacles[:,:2])[0]
    near=obstacles[distance<.16]
    if len(near):
        underneath=footprint(near[:,:2],.025,.03,.02,.015)
        shape=sh.union_all([shape,underneath]).buffer(.025,join_style='mitre').buffer(-.025,join_style='mitre').simplify(.02,preserve_topology=True)
    top=ground_mesh(shape,planes)
    top.compute_triangle_normals();top.compute_vertex_normals()
    points=np.asarray(top.vertices);idx=np.argmax(np.c_[points[:,:2],np.ones(len(points))]@coefficient.T,axis=1)
    slopes=np.degrees(np.arctan(np.linalg.norm(coefficient[:,:2],axis=1)))
    colors=np.tile(PALETTE[1],(len(points),1));colors[slopes[idx]>=5]=SLOPE_COLOR
    top.vertex_colors=o3d.utility.Vector3dVector(colors)
    obstacle,obstacle_stats=planar_obstacles(mesh,labels);obstacle.paint_uniform_color(PALETTE[2])
    trajectory=np.asarray(load_trajectory(source))
    if not len(trajectory):raise ValueError('Final trajectory required for start-ground origin')
    origin=start_ground_origin(trajectory[0],planes)
    top.translate(-origin);obstacle.translate(-origin)
    slab=ground_solid(top)
    # Keep ground and obstacle ownership distinct. Do not infer unseen obstacle volumes.
    t=np.asarray(top.triangles);v=np.asarray(top.vertices)
    face_points=v[t].reshape(-1,3)
    # Split display vertices at plane boundaries for sharp normals and non-blended colours.
    display=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(face_points),
        o3d.utility.Vector3iVector(np.arange(len(face_points)).reshape(-1,3)))
    centers=v[t].mean(axis=1)+origin
    face_plane=np.argmax(np.c_[centers[:,:2],np.ones(len(centers))]@coefficient.T,axis=1)
    face_colors=np.tile(PALETTE[1],(len(t),1));face_colors[slopes[face_plane]>=5]=SLOPE_COLOR
    display.vertex_colors=o3d.utility.Vector3dVector(np.repeat(face_colors,3,axis=0))
    display.compute_vertex_normals()
    side=copy.deepcopy(slab)
    side.triangles=o3d.utility.Vector3iVector(np.asarray(slab.triangles)[len(top.triangles):])
    side.remove_unreferenced_vertices();side.paint_uniform_color([.10,.24,.17]);side.compute_vertex_normals()
    scene=display+side+obstacle;map_model=slab+obstacle
    output.mkdir(parents=True,exist_ok=False)
    for name,obj in [('ground_planar',top),('obstacles_structured',obstacle),('scene_colored',scene)]:
        o3d.io.write_triangle_mesh(str(output/(name+'.ply')),obj)
    write_stl_verified(output/'ground_solid_m.stl',slab)
    mm=copy.deepcopy(slab);mm.scale(1000,center=[0,0,0]);write_stl_verified(output/'ground_solid_mm.stl',mm)
    write_stl_verified(output/'map_m.stl',map_model)
    mm=copy.deepcopy(map_model);mm.scale(1000,center=[0,0,0]);write_stl_verified(output/'map_mm.stl',mm)
    validations={name:validate_stl(output/name) for name in ['ground_solid_m.stl','ground_solid_mm.stl','map_m.stl','map_mm.stl']}
    np.savetxt(output/'trajectory_m.csv',trajectory-origin,delimiter=',',header='x_m,y_m,z_m',comments='')
    transform=np.eye(4);transform[:3,3]=-origin
    (output/'ground_footprint_source.geojson').write_text(sh.to_geojson(shape))
    report=dict(source=str(source),source_sha256=digest,origin_in_source_m=origin.tolist(),export_from_source=transform.tolist(),
        gravity_rotation_applied=False,planes=[dict(coefficient=p['coefficient'].tolist(),slope_deg=float(s),rms_m=p['rms_m']) for p,s in zip(planes,slopes)],
        ground=dict(area_m2=float(shape.area),inferred_xy_area_m2=float(shape.difference(observed_shape).area),
            removed_xy_area_m2=float(observed_shape.difference(shape).area),boundary_simplify_m=.025,
            closing_radius_m=.055,under_object_extension_limit_m=.16,triangles=len(top.triangles),
            polygon_count=len(polygons(shape)),boundary_vertices=sum(len(p.exterior.coords) for p in polygons(shape))),
        obstacles=obstacle_stats,stl_reload=validations,assumptions=[
            'Planar ground is the supported convex envelope of fitted planes.',
            'Contour regularization, small hole filling and ground beneath nearby obstacles are inferred.',
            'Ground solid has 8cm vertical thickness; map contains open obstacle surfaces, not a printable closed assembly.',
            'Obstacle plane patches preserve measured orientation; small fragments are discarded; unseen backs remain unknown.'])
    if fingerprint(source)!=digest:raise RuntimeError('Source changed')
    (output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    return report


def view(folder):
    import time
    import open3d as o3d
    from .viewer import trajectory_geometries
    folder=Path(folder).resolve();summary=json.loads((folder/'summary.json').read_text())
    model=o3d.io.read_triangle_mesh(str(folder/'scene_colored.ply'))
    model.compute_vertex_normals()
    raw=o3d.io.read_triangle_mesh(summary['source']);raw.translate(-np.array(summary['origin_in_source_m']))
    raw.compute_vertex_normals();raw.paint_uniform_color([.55,.62,.69])
    viewer=o3d.visualization.VisualizerWithKeyCallback()
    if not viewer.create_window(window_name='Structured map | Pink: planar slope | O: original overlay',width=1200,height=850):
        raise RuntimeError('3D window initialization failed')
    visible=False
    def toggle(_):
        nonlocal visible
        if visible:viewer.remove_geometry(raw,reset_bounding_box=False)
        else:viewer.add_geometry(raw,reset_bounding_box=False)
        visible=not visible
        return False
    try:
        viewer.add_geometry(model)
        for part in trajectory_geometries(np.loadtxt(folder/'trajectory_m.csv',delimiter=',',skiprows=1).tolist()):
            viewer.add_geometry(part,reset_bounding_box=False)
        viewer.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=.16),reset_bounding_box=False)
        viewer.register_key_callback(ord('O'),toggle)
        options=viewer.get_render_option();options.background_color=[.07,.09,.12];options.mesh_show_back_face=True
        control=viewer.get_view_control();control.set_up([0,0,1]);control.set_front([.35,-1,.75]);control.set_zoom(.64)
        for _ in range(20):viewer.poll_events();viewer.update_renderer();time.sleep(.03)
        viewer.capture_screen_image(str(folder/'preview.png'),do_render=True)
        print('VIEWER_READY | O: original overlay',flush=True)
        while viewer.poll_events():viewer.update_renderer();time.sleep(.02)
    finally:viewer.destroy_window()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source',type=Path);parser.add_argument('output',type=Path,nargs='?');parser.add_argument('--view',action='store_true')
    args=parser.parse_args()
    if args.view:view(args.source)
    elif args.output is None:parser.error('output folder required for build')
    else:build(args.source,args.output)


if __name__=='__main__':main()
