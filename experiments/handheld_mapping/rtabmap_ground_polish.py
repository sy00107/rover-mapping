"""Reduce small observed-ground holes and unstable ground shading in derived RTAB meshes."""
import argparse
import json
from pathlib import Path

import numpy as np

from .surface_colors import GROUND,PALETTE,classify_surface_points
from .ground_display import fill_small_ground_gaps
from .stl_export import supported_ground_patch,smooth_export_surface,prevent_face_damage,mesh_edges
from .gravity import fingerprint


def load_terrain(folder):
    info=json.loads((folder/'terrain/summary.json').read_text())
    with np.load(folder/'terrain/terrain.npz',allow_pickle=False) as data:
        layers={key:data[key] for key in data.files}
    return dict(layers=layers,bounds=np.array(info['grid']['bounds']),resolution=info['grid']['resolution'],
                grid_from_geometry=np.eye(4),ground_tolerance_m=.02,obstacle_threshold_m=.06)


def stable_ground_normals(points,normals,labels,terrain):
    """Shade observed ground using local terrain slope; positions and labels stay fixed."""
    normals=normals.copy();points=np.asarray(points)
    bounds=terrain['bounds'];res=terrain['resolution'];layers=terrain['layers']
    x=np.floor((points[:,0]-bounds[0])/res).astype(int);y=np.floor((points[:,1]-bounds[2])/res).astype(int)
    active=(labels==GROUND)&(x>=0)&(y>=0)&(x<layers['ground'].shape[1])&(y<layers['ground'].shape[0])
    ids=np.flatnonzero(active)
    gx=layers['reference_ground_gradient_x'][y[ids],x[ids]];gy=layers['reference_ground_gradient_y'][y[ids],x[ids]]
    valid=np.isfinite(gx)&np.isfinite(gy);ids=ids[valid]
    expected=np.column_stack([-gx[valid],-gy[valid],np.ones(valid.sum())])
    expected/=np.linalg.norm(expected,axis=1)[:,None]
    normals[ids]=expected
    return normals,dict(vertices=len(ids),method='terrain slope normals for ground lighting only',changes_positions=False)


def reconnect_ground_sheet(mesh,labels,terrain,maximum_edge_m=.06):
    """Retessellate only observed 2.5D green ground; never connect long/unknown gaps."""
    import open3d as o3d
    from scipy.spatial import Delaunay
    points=np.asarray(mesh.vertices);faces=np.asarray(mesh.triangles)
    ground_ids=np.flatnonzero(labels==GROUND)
    # Collapse near-duplicate XY sheets to one measured representative per 5 mm cell.
    cells=np.floor(points[ground_ids,:2]/.005).astype(np.int64)
    _,inverse=np.unique(cells,axis=0,return_inverse=True)
    order=np.lexsort((points[ground_ids,2],inverse));sorted_group=inverse[order]
    starts=np.r_[0,np.flatnonzero(np.diff(sorted_group))+1];ends=np.r_[starts[1:],len(order)]
    selected=ground_ids[order[(starts+ends)//2]]
    # Keep original mixed-class boundary vertices so obstacle seams are not displaced.
    mixed=np.any(labels[faces]!=GROUND,axis=1)&np.any(labels[faces]==GROUND,axis=1)
    seam=np.unique(faces[mixed]);seam=seam[labels[seam]==GROUND]
    selected=np.unique(np.r_[selected,seam])
    triangles=selected[Delaunay(points[selected,:2]).simplices]
    xyz=points[triangles]
    lengths=np.linalg.norm(xyz-np.roll(xyz,1,axis=1),axis=2)
    candidate=lengths.max(axis=1)<=maximum_edge_m
    normal=np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0]);norm=np.linalg.norm(normal,axis=1)
    # Exclude folded/near-vertical triangles even if their vertices were coloured green.
    candidate &= (norm>1e-10)&(abs(normal[:,2])>=.6*norm)
    triangles=triangles[candidate]
    supported=[]
    for start in range(0,len(triangles),10000):
        chunk=triangles[start:start+10000];xyz=points[chunk]
        samples=np.concatenate([xyz.mean(axis=1)[:,None,:],(xyz+np.roll(xyz,1,axis=1))*.5],axis=1)
        known=(classify_surface_points(samples.reshape(-1,3),terrain).reshape(-1,4)==GROUND).all(axis=1)
        supported.append(chunk[known])
    triangles=np.concatenate(supported)
    xyz=points[triangles];down=np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0])[:,2]<0
    triangles[down]=triangles[down][:,[0,2,1]]
    retained=faces[~np.all(labels[faces]==GROUND,axis=1)]
    # Do not add a third incident face at an existing class seam.
    all_faces=np.r_[retained,triangles]
    edges,inverse,counts=mesh_edges(all_faces)
    per_face_counts=counts[inverse].reshape(3,len(all_faces)).T
    allowed=(per_face_counts[len(retained):]<=2).all(axis=1)
    rejected=int((~allowed).sum());triangles=triangles[allowed]
    mesh.triangles=o3d.utility.Vector3iVector(np.r_[retained,triangles])
    mesh.compute_vertex_normals()
    return dict(method='local XY Delaunay of measured green points',maximum_edge_m=maximum_edge_m,
                representative_cell_m=.005,removed_ground_faces=len(faces)-len(retained),
                reconstructed_ground_faces=len(triangles),non_ground_faces_preserved=True,
                topology_rejected_faces=rejected,
                requires_observed_ground_at_centres_and_edge_midpoints=True,
                scope='derived 2.5D ground presentation; inferred connectivity')


def build(source,output):
    import open3d as o3d
    from .rtabmap_surface import preview
    source=source.resolve();output=output.resolve()
    metadata=json.loads((source/'summary.json').read_text());terrain=load_terrain(source)
    source_mesh=source/metadata['mesh'];before_hash=fingerprint(source_mesh)
    mesh=o3d.io.read_triangle_mesh(str(source_mesh))
    original=np.asarray(mesh.vertices).copy();colors=np.asarray(mesh.vertex_colors).copy()
    labels=np.argmin(np.linalg.norm(colors[:,None,:]-PALETTE[None,:,:],axis=2),axis=1)
    original_faces=len(mesh.triangles)
    # Existing bounded normal-only smoother, applied to ground alone; obstacle and unknown fixed.
    ground_only=np.where(labels==GROUND,GROUND,0)
    smooth=smooth_export_surface(mesh,ground_only,iterations=12)
    smooth.update(prevent_face_damage(mesh,original,labels,terrain))
    holes=fill_small_ground_gaps(mesh,labels,maximum_diameter_m=.08,maximum_edges=40,
                               maximum_plane_residual_m=.008,
                               candidate_validator=lambda faces:supported_ground_patch(faces,terrain),
                               prevent_nonmanifold=True)
    reconnection=reconnect_ground_sheet(mesh,labels,terrain)
    holes['scope']='derived presentation mesh; terrain-supported inferred patches'
    holes['maximum_plane_residual_m']=.008
    _,_,edge_counts=mesh_edges(np.asarray(mesh.triangles))
    if np.any(edge_counts>2):raise ValueError("Ground reconnection produced nonmanifold edges")
    normals,normal_stats=stable_ground_normals(np.asarray(mesh.vertices),np.asarray(mesh.vertex_normals),labels,terrain)
    mesh.vertex_normals=o3d.utility.Vector3dVector(normals)
    assert np.array_equal(np.asarray(mesh.vertices)[labels!=GROUND],original[labels!=GROUND])
    assert np.array_equal(mesh.vertex_colors,colors)
    output.mkdir(parents=True,exist_ok=False)
    if not o3d.io.write_triangle_mesh(str(output/'rtab_surface_mesh.ply'),mesh):raise OSError('Mesh save failed')
    cloud=o3d.geometry.PointCloud();cloud.points=mesh.vertices;cloud.colors=mesh.vertex_colors;cloud.normals=mesh.vertex_normals
    if not o3d.io.write_point_cloud(str(output/'rtab_surface_cloud.ply'),cloud):raise OSError('Cloud save failed')
    inferred=np.r_[np.zeros(len(mesh.triangles)-reconnection["reconstructed_ground_faces"],bool),np.ones(reconnection["reconstructed_ground_faces"],bool)]
    np.save(output/'additional_inferred_faces.npy',inferred,allow_pickle=False)
    metadata.update(source=str(source),mesh='rtab_surface_mesh.ply',cloud='rtab_surface_cloud.ply',
                    source_mesh_sha256=before_hash,triangles=len(mesh.triangles),
                    ground_polish=dict(smoothing=smooth,additional_holes=holes,ground_reconnection=reconnection,shading_normals=normal_stats,
                                       remaining_boundary_edges=int((edge_counts==1).sum()),
                                       remaining_nonmanifold_edges=int((edge_counts>2).sum()),
                                       original_vertices_unchanged_for_obstacle_and_unknown=True,
                                       note='Small ground patches and slope normals are presentation refinements, not measurements.'),
                    render=dict(ambient_light=.35),terrain_source=str(source/'terrain'))
    assert fingerprint(source_mesh)==before_hash
    (output/'summary.json').write_text(json.dumps(metadata,indent=2)+'\n')
    preview(mesh,output/'preview.png')
    print(json.dumps(metadata['ground_polish'],indent=2))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('source',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();build(args.source,args.output)

if __name__=='__main__':main()
