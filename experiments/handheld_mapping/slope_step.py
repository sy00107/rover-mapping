"""User-confirmed ramp junctions: inferred orange risers in a fixed map frame.

Only the explicitly selected 10/30-degree patches of an assumed layout are used.
These display meshes do not modify measurements, terrain costs or SLAM poses.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import label
from shapely.geometry import MultiPoint, LineString

from .surface_colors import load_surface_layers, PALETTE, OBSTACLE


def slope_patch(mesh_path, angle_limits):
    terrain = load_surface_layers(mesh_path)
    if terrain is None:
        raise ValueError('slope patch requires terrain in the same map frame')
    with np.load(Path(terrain['output_dir'])/'terrain.npz') as arrays:
        slope = arrays['slope_deg']
        mask = arrays['ground'] & np.isfinite(slope)
        mask &= (slope >= angle_limits[0]) & (slope <= angle_limits[1])
        groups, _ = label(mask, np.ones((3, 3)))
        sizes = np.bincount(groups.ravel()); sizes[0] = 0
        if sizes.max() < 20:
            raise ValueError('insufficient connected slope support')
        y, x = np.where(groups == sizes.argmax())
        bounds, res = terrain['bounds'], terrain['resolution']
        points = np.c_[bounds[0]+(x+.5)*res, bounds[2]+(y+.5)*res,
                       arrays['height_m'][y, x]]
    matrix = terrain['grid_from_geometry']
    return (points-matrix[:3, 3]) @ matrix[:3, :3]


def plane(points):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('finite XYZ plane support required')
    design = np.c_[points[:, :2], np.ones(len(points))]
    fit, _, rank, _ = np.linalg.lstsq(design, points[:, 2], rcond=None)
    if rank < 3:
        raise ValueError('plane support is collinear')
    if np.sqrt(np.mean((design@fit-points[:, 2])**2)) > .025:
        raise ValueError('ramp plane fit exceeds 2.5 cm RMS')
    return fit


def step_geometry(first, second, *, spacing=.02, maximum_gap=.35):
    """Fit facing ramp edges, extend to a common seam and add a vertical riser.

    The caller selects a known physical junction. This is deliberately not a
    generic hole filler. The overlap span is bounded by both observed hulls.
    """
    if spacing <= 0 or maximum_gap <= 0:
        raise ValueError('positive spacing and gap limits required')
    first, second = np.asarray(first, float), np.asarray(second, float)
    fits = [plane(first), plane(second)]
    gradient = fits[1][:2]
    if np.linalg.norm(gradient) < .01:
        raise ValueError('second ramp has no stable uphill direction')
    along = gradient/np.linalg.norm(gradient)
    across = np.array([-along[1], along[0]])
    if (second[:, :2].mean(0)-first[:, :2].mean(0))@across < 0:
        across = -across
    basis = np.column_stack((along, across))
    hulls = [MultiPoint(p[:, :2]@basis).convex_hull for p in (first, second)]
    lo = max(h.bounds[0] for h in hulls)+1e-7
    hi = min(h.bounds[2] for h in hulls)-1e-7
    if hi-lo < spacing:
        raise ValueError('ramps have no shared junction span')
    stations = np.linspace(lo, hi, max(2, int(np.ceil((hi-lo)/spacing))+1))
    profiles = []
    for station in stations:
        edges = [h.intersection(LineString([(station, -10000), (station, 10000)])) for h in hulls]
        a, b = edges[0].bounds[3], edges[1].bounds[1]
        gap = b-a
        if gap < -1e-6 or gap > maximum_gap:
            profiles.append(None)
            continue
        seam = (a+b)/2
        xy = np.array([[station, a], [station, seam], [station, b]])@basis.T
        za = np.c_[xy, np.ones(3)]@fits[0]
        zb = np.c_[xy, np.ones(3)]@fits[1]
        # Two short ramp extensions meet an actual vertical face at the seam.
        profiles.append(np.array([np.r_[xy[0], za[0]], np.r_[xy[1], za[1]],
                                  np.r_[xy[1], zb[1]], np.r_[xy[2], zb[2]]]))
    vertices, faces = [], []
    accepted = []
    for i in range(len(profiles)-1):
        a, b = profiles[i:i+2]
        if a is None or b is None:
            continue  # No spanning unsupported/wide gaps.
        start = len(vertices); vertices.extend(a); vertices.extend(b)
        for j in range(3):
            faces.extend([[start+j, start+j+1, start+j+4],
                          [start+j+1, start+j+5, start+j+4]])
        accepted.extend([a, b])
    if not faces:
        raise ValueError('no bounded facing ramp edges for a step')
    profiles = np.array(accepted)
    heights = np.abs(profiles[:, 1, 2]-profiles[:, 2, 2])
    return np.array(vertices), np.array(faces, np.int32), dict(
        method='fixed_ramp_junction_vertical_riser_v1', semantic='ramp_step',
        inferred=True, display_only=True, maximum_edge_gap_m=maximum_gap,
        step_height_range_m=[float(heights.min()), float(heights.max())],
        first_plane=fits[0].tolist(), second_plane=fits[1].tolist(),
        shared_uphill_span_m=[float(lo), float(hi)],
        panels='first ramp extension, vertical riser, second ramp extension')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_layout_steps(root):
    """Add separate artifacts for each existing assumed-layout hypothesis."""
    import open3d as o3d
    from .storage import atomic_json
    from .slope_polish import polish_ramps
    root = Path(root).resolve()
    report = json.loads((root/'alignment.json').read_text())
    reference = Path(report['reference'])/f'dense_offline/map_{report["reference_map_id"]}_mesh.ply'
    first = slope_patch(reference, (5, 20))
    entries = []
    for row in report['hypotheses']:
        source = Path(report['source'])/f'dense_offline/map_{row["map_id"]}_mesh.ply'
        transform = np.array(row['world_from_source'])
        second = slope_patch(source, (25, 35))
        second = second@transform[:3, :3].T+transform[:3, 3]
        xyz, faces, info = step_geometry(first, second)
        step = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(xyz), o3d.utility.Vector3iVector(faces))
        step.remove_degenerate_triangles(); step.remove_unreferenced_vertices()
        step.paint_uniform_color(PALETTE[OBSTACLE]); step.compute_vertex_normals()
        rank = row['rank']; prefix = f'hypothesis_{rank}'
        step_name = prefix+'_step_riser.ply'
        if not o3d.io.write_triangle_mesh(str(root/step_name), step):
            raise OSError('step mesh write failed')
        inputs = {}
        polish_stats = {}
        for suffix in ('mesh', 'full_context'):
            original = root/f'{prefix}_{suffix}.ply'
            inputs[original.name] = sha256(original)
            observed = o3d.io.read_triangle_mesh(str(original))
            combined = observed+step
            target = root/f'{prefix}_{suffix}_with_step.ply'
            if not o3d.io.write_triangle_mesh(str(target), combined):
                raise OSError('combined step mesh write failed')
            polished, polish_stats[suffix] = polish_ramps(observed, (first, second))
            polished += step
            polished.compute_vertex_normals()
            target = root/f'{prefix}_{suffix}_polished_with_step.ply'
            if not o3d.io.write_triangle_mesh(str(target), polished):
                raise OSError('polished ramp mesh write failed')
        entries.append(dict(rank=rank, **info, vertices=len(step.vertices), triangles=len(step.triangles),
                            step_mesh=step_name, input_sha256=inputs,
                            source_mesh_sha256={str(reference): sha256(reference), str(source): sha256(source)},
                            world_from_source=transform.tolist(), ramp_polish=polish_stats))
    atomic_json(root/'ramp_steps.json', dict(
        semantic='ramp_step', color='orange', inferred=True, display_only=True,
        registration_verified=report.get('registration_verified', False),
        alignment_sha256=sha256(root/'alignment.json'), layouts=entries))
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('layout_dir', type=Path)
    args = parser.parse_args()
    print(json.dumps(build_layout_steps(args.layout_dir), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
