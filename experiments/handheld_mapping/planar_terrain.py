"""Piecewise planar ground in the saved gravity frame, with separate observed obstacles."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from scipy import ndimage as nd


def fit_planes(points, threshold=.015, minimum_fraction=.12, maximum=6):
    import open3d as o3d
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).voxel_down_sample(.025)
    minimum=max(80,int(len(cloud.points)*minimum_fraction))
    planes=[]; o3d.utility.random.seed(7)
    while len(cloud.points)>=minimum and len(planes)<maximum:
        model,ids=cloud.segment_plane(threshold,3,2500)
        if len(ids)<minimum:break
        values=np.asarray(cloud.points)[ids]
        # Refit z=ax+by+c. Never rotate the gravity frame.
        coefficient=np.linalg.lstsq(np.c_[values[:,:2],np.ones(len(values))],values[:,2],rcond=None)[0]
        if np.degrees(np.arctan(np.linalg.norm(coefficient[:2])))<=30:
            if not any(np.linalg.norm(coefficient-p['coefficient'])<.04 for p in planes):
                residual=values[:,2]-np.c_[values[:,:2],np.ones(len(values))]@coefficient
                planes.append(dict(coefficient=coefficient,support=len(ids),rms_m=float(np.sqrt(np.mean(residual**2)))))
        cloud=cloud.select_by_index(ids,invert=True)
    if not planes:raise ValueError('No supported planar ground')
    return planes


def clip_polygon(polygon, difference):
    """Clip XY polygon against one linear plane-height inequality."""
    result=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        da=np.dot(difference[:2],a)+difference[2];db=np.dot(difference[:2],b)+difference[2]
        if da>=-1e-10:result.append(a)
        if (da>0 and db<0) or (da<0 and db>0):
            result.append(a+(b-a)*(da/(da-db)))
    return result


def planar_mesh(points, planes, resolution=.025):
    """Raster support only, close small enclosed holes; exact planar faces and crease splits."""
    import open3d as o3d
    coefficients=np.asarray([p['coefficient'] for p in planes])
    predicted=np.c_[points[:,:2],np.ones(len(points))]@coefficients.T
    residual=np.min(abs(predicted-points[:,2,None]),axis=1)
    support=points[residual<=.035]
    # A convex ramp/flat junction uses the upper envelope. Reject this model if it
    # contradicts most fitted observations rather than silently forcing a valley flat.
    envelope=np.max(np.c_[support[:,:2],np.ones(len(support))]@coefficients.T,axis=1)
    agreement=abs(envelope-support[:,2])<=.04
    if agreement.mean()<.85:
        raise ValueError('These planes do not form a supported convex ramp junction; split regions explicitly')
    support=support[agreement]
    lo=support[:,:2].min(axis=0)-resolution
    ij=np.floor((support[:,:2]-lo)/resolution).astype(int)
    shape=tuple((ij.max(axis=0)+3)[::-1]);mask=np.zeros(shape,bool);mask[ij[:,1],ij[:,0]]=True
    # Close holes enclosed by observations, never extend the exterior footprint.
    holes=nd.binary_fill_holes(mask)&~mask
    components,count=nd.label(holes)
    inferred=np.zeros(shape,bool)
    for identity in range(1,count+1):
        yy,xx=np.where(components==identity)
        if max(np.ptp(xx)+1,np.ptp(yy)+1)*resolution<=.08:
            inferred[yy,xx]=True
    mask|=inferred
    # Four observed neighbouring cells support one quad; large holes remain open.
    quads=mask[:-1,:-1]&mask[1:,:-1]&mask[:-1,1:]&mask[1:,1:]
    vertices=[];faces=[]
    for y,x in zip(*np.where(quads)):
        xy=lo+np.array([[x,y],[x+1,y],[x+1,y+1],[x,y+1]])*resolution
        for triangle in [xy[[0,1,2]],xy[[0,2,3]]]:
            for identity,coefficient in enumerate(coefficients):
                polygon=list(triangle)
                for other in range(len(coefficients)):
                    if other!=identity:
                        polygon=clip_polygon(polygon,coefficient-coefficients[other])
                        if len(polygon)<3:break
                if len(polygon)<3:continue
                polygon=np.asarray(polygon);z=polygon@coefficient[:2]+coefficient[2]
                start=len(vertices);vertices.extend(np.c_[polygon,z])
                faces.extend([[start,start+i,start+i+1] for i in range(1,len(polygon)-1)])
    if not faces:raise ValueError('Insufficient continuous ground support')
    vertices=np.asarray(vertices)
    xy,inverse=np.unique(np.round(vertices[:,:2],9),axis=0,return_inverse=True)
    unique=np.c_[xy,np.max(np.c_[xy,np.ones(len(xy))]@coefficients.T,axis=1)]
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(unique),o3d.utility.Vector3iVector(inverse[np.asarray(faces)]))
    mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_unreferenced_vertices()
    return mesh,dict(grid_resolution_m=resolution,observed_cells=int((mask&~inferred).sum()),
                     inferred_hole_cells=int(inferred.sum()),envelope_agreement_fraction=float(agreement.mean()),
                     maximum_fit_distance_m=.035,maximum_closed_hole_m=.08)


def ground_solid(top,thickness=.08):
    """Make a vertical-thickness shell without extending ground to a nominal plane."""
    import open3d as o3d
    vertices=np.asarray(top.vertices);triangles=np.asarray(top.triangles);n=len(vertices)
    directed=np.concatenate([triangles[:,[0,1]],triangles[:,[1,2]],triangles[:,[2,0]]])
    _,first,counts=np.unique(np.sort(directed,axis=1),axis=0,return_index=True,return_counts=True)
    boundary=directed[first[counts==1]]
    a,b=boundary.T
    sides=np.r_[np.c_[a,a+n,b+n],np.c_[a,b+n,b]]
    lower=vertices.copy();lower[:,2]-=thickness
    solid=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.r_[vertices,lower]),
             o3d.utility.Vector3iVector(np.r_[triangles,triangles[:,::-1]+n,sides]))
    return solid


def start_ground_origin(start_position, planes):
    """Vertical projection of the first final camera pose onto the fitted ground."""
    start=np.asarray(start_position,dtype=float)
    coefficients=np.asarray([p['coefficient'] for p in planes],dtype=float)
    if start.shape!=(3,) or not np.isfinite(start).all():
        raise ValueError('A finite starting camera position is required')
    origin=start.copy()
    origin[2]=np.max(coefficients[:,:2]@start[:2]+coefficients[:,2])
    return origin


def build(source,output):
    import open3d as o3d
    from .surface_colors import load_surface_layers,classify_surface_points,PALETTE,SLOPE_COLOR
    from .stl_export import write_stl_verified
    from .gravity import fingerprint
    source=Path(source).resolve();output=Path(output).resolve();digest=fingerprint(source)
    terrain=load_surface_layers(source)
    if terrain is None:raise ValueError('Same-map ground classification required')
    mesh=o3d.io.read_triangle_mesh(str(source));xyz=np.asarray(mesh.vertices);triangles=np.asarray(mesh.triangles)
    labels=classify_surface_points(xyz,terrain)
    # Input must already be in the gravity frame: no implicit floor alignment.
    from .viewer import map_frame
    _,aligned=map_frame(source)
    if not aligned:raise ValueError('Gravity-aligned input required')
    planes=fit_planes(xyz[labels==1]);top,stats=planar_mesh(xyz[labels==1],planes)
    top.compute_triangle_normals();top.compute_vertex_normals()
    coefficient=np.asarray([p['coefficient'] for p in planes]);v=np.asarray(top.vertices)
    index=np.argmin(abs(np.c_[v[:,:2],np.ones(len(v))]@coefficient.T-v[:,2,None]),axis=1)
    slopes=np.degrees(np.arctan(np.linalg.norm(coefficient[:,:2],axis=1)))
    colors=np.tile(PALETTE[1],(len(v),1));colors[slopes[index]>=5]=SLOPE_COLOR
    top.vertex_colors=o3d.utility.Vector3dVector(colors)
    obstacle=copy.deepcopy(mesh)
    keep=(labels[triangles]==2).sum(axis=1)>=2
    obstacle.triangles=o3d.utility.Vector3iVector(triangles[keep]);obstacle.remove_unreferenced_vertices()
    obstacle.paint_uniform_color(PALETTE[2]);obstacle.compute_vertex_normals()
    scene=top+obstacle;solid=ground_solid(top)
    if not solid.is_watertight() or solid.is_self_intersecting():
        raise ValueError('Planar ground shell failed topology validation')
    from .viewer import load_trajectory
    trajectory=load_trajectory(source)
    if not trajectory:raise ValueError('출발점 원점 설정에는 최종 카메라 경로가 필요합니다.')
    origin=start_ground_origin(trajectory[0],planes)
    export_from_source=np.eye(4);export_from_source[:3,3]=-origin
    # One translation for every exported asset; axes and all relative distances stay fixed.
    for geometry in (top,obstacle,scene,solid):geometry.translate(-origin)
    nearest_support=float(np.min(np.linalg.norm(xyz[labels==1,:2]-origin[:2],axis=1)))
    output.mkdir(parents=True,exist_ok=False)
    np.savetxt(output/'trajectory_m.csv',np.asarray(trajectory)-origin,delimiter=',',
               header='x_m,y_m,z_m',comments='')
    o3d.io.write_triangle_mesh(str(output/'ground_planar.ply'),top)
    o3d.io.write_triangle_mesh(str(output/'obstacles_observed.ply'),obstacle)
    o3d.io.write_triangle_mesh(str(output/'scene_colored.ply'),scene)
    write_stl_verified(output/'ground_solid_m.stl',solid)
    mm=copy.deepcopy(solid);mm.scale(1000,center=[0,0,0]);write_stl_verified(output/'ground_solid_mm.stl',mm)
    write_stl_verified(output/'scene_surface_m.stl',scene)
    if fingerprint(source)!=digest:raise RuntimeError('Source changed')
    report=dict(source=str(source),source_sha256=digest,gravity_rotation_applied=False,
        origin_definition='vertical projection of first final camera pose onto fitted ground envelope',
        origin_in_source_m=origin.tolist(),export_from_source=export_from_source.tolist(),
        source_from_export=np.linalg.inv(export_from_source).tolist(),
        start_camera_in_export_m=(np.asarray(trajectory[0])-origin).tolist(),
        origin_ground_is_fitted=True,origin_nearest_observed_ground_xy_distance_m=nearest_support,
        origin_ground_requires_extrapolation=nearest_support>.08,
        trajectory='trajectory_m.csv',
        planes_in_export=[dict(coefficient=[float(p['coefficient'][0]),float(p['coefficient'][1]),
            float(p['coefficient'][2]+p['coefficient'][:2]@origin[:2]-origin[2])]) for p in planes],
        planes=[dict(coefficient=p['coefficient'].tolist(),support=p['support'],rms_m=p['rms_m'],slope_deg=float(s)) for p,s in zip(planes,slopes)],
        ground=stats,ground_triangles=len(top.triangles),obstacle_triangles=len(obstacle.triangles),
        ground_solid_watertight=solid.is_watertight(),ground_solid_self_intersecting=solid.is_self_intersecting(),
        scene_surface_is_closed=False,assumptions=['User states terrain comprises planar surfaces.',
        'Ground uses supported convex piecewise-plane envelope; more complex junctions require region segmentation.',
        'Small enclosed holes are inferred; exterior and large unobserved gaps stay open.',
        'Ground solid is an inferred 8cm vertical thickness; obstacle surface remains observed and open.'])
    (output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();build(args.source,args.output)


if __name__=='__main__':main()
