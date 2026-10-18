"""Portable viewer: python3 view_maps.py [RUN_ID]. Requires numpy and open3d only."""
from pathlib import Path
import json
import sys
import numpy as np
import open3d as o3d

root=Path(__file__).resolve().parent
runs=sorted(p for p in (root/'derived').iterdir() if (p/'latest.json').is_file())
if len(sys.argv)>1:runs=[p for p in runs if p.name==sys.argv[1]]
found=False
for run in runs:
    pointer=json.loads((run/'latest.json').read_text());folder=(root/pointer['folder']).resolve()
    if not folder.is_relative_to(root):raise ValueError('Invalid preview path')
    report=json.loads((folder/'report.json').read_text())
    for item in report['maps']:
        found=True
        mesh=o3d.io.read_triangle_mesh(str(folder/item['mesh']));mesh.compute_vertex_normals()
        geometries=[mesh]
        points=np.loadtxt(folder/item['trajectory'],delimiter=',',skiprows=1).reshape(-1,3)
        if len(points)>1:
            line=o3d.geometry.LineSet(o3d.utility.Vector3dVector(points),
                o3d.utility.Vector2iVector(np.c_[np.arange(len(points)-1),np.arange(1,len(points))]))
            line.paint_uniform_color([.1,.5,1]);geometries.append(line)
        o3d.visualization.draw_geometries(geometries,window_name=f'{run.name} | Map {item["map_id"]}',
            mesh_show_back_face=True,up=[0,0,1] if item['coordinate_system']=='gravity_z_up' else [0,-1,0])
if not found:print('No saved maps. Check derived/*/*/report.json for capture/reconstruction failures.')
