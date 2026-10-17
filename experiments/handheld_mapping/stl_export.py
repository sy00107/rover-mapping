"""Separate, bounded ground/obstacle refinement for STL exports; mapping inputs stay immutable."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from experiments.handheld_mapping.surface_colors import (
    GROUND, OBSTACLE, PALETTE, classify_surface_points, load_surface_layers, surface_point_colors)
from experiments.handheld_mapping.viewer import select_maps


def mesh_edges(triangles):
    directed = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    return np.unique(np.sort(directed, axis=1), axis=0, return_inverse=True, return_counts=True)


def smooth_export_surface(mesh, labels, iterations=16):
    """Normal-only Taubin filtering; pin seams, sharp edges, boundaries and unknowns."""
    import open3d as o3d
    from scipy.sparse import coo_matrix
    original = np.asarray(mesh.vertices).copy()
    triangles = np.asarray(mesh.triangles)
    labels = np.asarray(labels)
    if labels.shape != (len(original),):
        raise ValueError('surface labels do not match vertices')
    mesh.compute_vertex_normals()
    normals = np.asarray(mesh.vertex_normals).copy()
    face_normals = np.asarray(mesh.triangle_normals)
    edges, inverse, counts = mesh_edges(triangles)
    first, last = np.full(len(edges), len(triangles)), np.zeros(len(edges), int)
    face_ids = np.tile(np.arange(len(triangles)), 3)
    np.minimum.at(first, inverse, face_ids)
    np.maximum.at(last, inverse, face_ids)
    sharp = np.einsum('ij,ij->i', face_normals[first], face_normals[last]) < np.cos(np.deg2rad(45.))
    length = np.linalg.norm(original[edges[:, 0]]-original[edges[:, 1]], axis=1)
    seams = labels[edges[:, 0]] != labels[edges[:, 1]]
    fixed = np.zeros(len(original), bool)
    fixed[edges[(counts != 2) | sharp | seams | (length > .04)].ravel()] = True
    active = np.isin(labels, [GROUND, OBSTACLE]) & ~fixed
    same = ~seams & ~sharp & (length <= .04)
    pairs = edges[same]
    weights = 1./np.maximum(length[same], .003)
    adjacency = coo_matrix((np.r_[weights, weights],
                            (np.r_[pairs[:, 0], pairs[:, 1]], np.r_[pairs[:, 1], pairs[:, 0]])),
                           shape=(len(original), len(original))).tocsr()
    total = np.asarray(adjacency.sum(axis=1)).ravel()
    active &= total > 0
    normalization = 1./np.maximum(total, 1e-12)
    caps = np.where(labels == GROUND, .008, .015)
    points = original.copy()
    for _ in range(iterations):
        for strength in (.5, -.53):
            delta = adjacency.dot(points)*normalization[:, None]-points
            delta = np.einsum('ij,ij->i', delta, normals)[:, None]*normals
            displacement = points+strength*delta-original
            displacement *= np.minimum(1., caps/np.maximum(np.linalg.norm(displacement, axis=1), 1e-12))[:, None]
            points[active] = (original+displacement)[active]
    mesh.vertices = o3d.utility.Vector3dVector(points)
    return {'iterations': iterations, 'ground_shift_limit_m': .008, 'obstacle_shift_limit_m': .015,
            'protected_sharp_angle_deg': 45., 'source_boundary_edges': int((counts == 1).sum()),
            'source_nonmanifold_edges': int((counts > 2).sum()),
            'fixed_vertices': int(fixed.sum()), 'moves_only_along_original_normals': True}


def prevent_face_damage(mesh, original, labels, terrain):
    """Undo any category change, face inversion or collapse introduced by smoothing."""
    import open3d as o3d
    points = np.asarray(mesh.vertices).copy()
    faces = np.asarray(mesh.triangles)
    changed_category = classify_surface_points(points, terrain) != labels
    points[changed_category] = original[changed_category]
    before = np.cross(original[faces[:, 1]]-original[faces[:, 0]],
                      original[faces[:, 2]]-original[faces[:, 0]])
    norm2 = np.einsum('ij,ij->i', before, before)
    reverted = changed_category.copy()
    for _ in range(20):
        after = np.cross(points[faces[:, 1]]-points[faces[:, 0]], points[faces[:, 2]]-points[faces[:, 0]])
        bad = (np.einsum('ij,ij->i', before, after) < .25*norm2)
        if not bad.any():
            break
        indices = np.unique(faces[bad])
        points[indices] = original[indices]
        reverted[indices] = True
    else:
        # Conservative fallback: an unresolved local interaction never escapes to STL.
        points = original.copy()
        reverted[:] = True
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.compute_vertex_normals()
    shifts = np.linalg.norm(points-original, axis=1)
    return {'reverted_vertices': int(reverted.sum()), 'moved_vertices': int((shifts > 1e-9).sum()),
            'ground_max_shift_m': float(shifts[labels == GROUND].max(initial=0.)),
            'obstacle_max_shift_m': float(shifts[labels == OBSTACLE].max(initial=0.)),
            'category_changes_after_guard': int((classify_surface_points(points, terrain) != labels).sum())}


def supported_ground_patch(triangles, terrain):
    """Require terrain evidence at patch vertices, edge midpoints and face centres."""
    samples = np.concatenate([triangles.reshape(-1, 3), triangles.mean(axis=1),
                              (triangles+np.roll(triangles, 1, axis=1)).reshape(-1, 3)*.5])
    return bool(np.all(classify_surface_points(samples, terrain) == GROUND))


def write_stl_verified(path, mesh):
    import open3d as o3d
    mesh.compute_triangle_normals()
    if not o3d.io.write_triangle_mesh(str(path), mesh, write_ascii=False):
        raise RuntimeError(f'STL write failed: {path}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    records = np.fromfile(path, dtype=dtype, offset=84)
    with path.open('rb') as stream:
        stream.seek(80)
        declared_count = int.from_bytes(stream.read(4), 'little')
    expected = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
    if (declared_count != len(expected) or path.stat().st_size != 84+50*len(expected) or len(records) != len(expected)
            or not np.array_equal(records['vertices'], expected) or not np.isfinite(records['normal']).all()):
        raise RuntimeError('STL coordinate verification failed')
    return {'bytes': path.stat().st_size, 'triangles': len(expected), 'coordinates_verified': True,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def export_stl(source, output):
    import open3d as o3d
    from experiments.handheld_mapping.ground_display import fill_small_ground_gaps
    source, output = Path(source).resolve(), Path(output).resolve()
    terrain = load_surface_layers(source)
    if terrain is None:
        raise ValueError('지면·장애물 층이 필요합니다. 먼저 같은 지도의 지형 결과를 생성하세요.')
    inputs = [source, Path(terrain['output_dir'])/'summary.json', Path(terrain['output_dir'])/'terrain.npz']
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    mesh = o3d.io.read_triangle_mesh(str(source))
    vertices, faces = np.asarray(mesh.vertices).copy(), np.asarray(mesh.triangles).copy()
    if not len(faces) or not np.isfinite(vertices).all():
        raise ValueError('유효한 삼각형 메시가 필요합니다.')
    area2 = np.linalg.norm(np.cross(vertices[faces[:, 1]]-vertices[faces[:, 0]],
                                     vertices[faces[:, 2]]-vertices[faces[:, 0]]), axis=1)
    faces = faces[area2 >= 1e-12]
    _, unique = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    mesh.triangles = o3d.utility.Vector3iVector(faces[np.sort(unique)])
    labels = classify_surface_points(vertices, terrain)
    # A fresh output is mandatory; no original or previous export may be overwritten.
    output.mkdir(parents=True, exist_ok=False)
    raw = write_stl_verified(output/'map_raw.stl', mesh)
    smooth = smooth_export_surface(mesh, labels)
    smooth.update(prevent_face_damage(mesh, vertices, labels, terrain))
    original_faces = len(mesh.triangles)
    holes = fill_small_ground_gaps(mesh, labels, maximum_diameter_m=.08, maximum_edges=40,
                                  maximum_plane_residual_m=.004,
                                  candidate_validator=lambda triangles: supported_ground_patch(triangles, terrain),
                                  prevent_nonmanifold=True)
    holes['scope'] = 'STL export only; terrain-supported inferred patches'
    holes['maximum_plane_residual_m'] = .004
    _, _, counts = mesh_edges(np.asarray(mesh.triangles))
    if int((counts > 2).sum()) > smooth['source_nonmanifold_edges']:
        raise RuntimeError('STL refinement created nonmanifold edges; export rejected')
    refined = write_stl_verified(output/'map_refined.stl', mesh)
    inferred = np.arange(len(mesh.triangles)) >= original_faces
    np.save(output/'inferred_faces.npy', inferred, allow_pickle=False)
    mesh.vertex_colors = o3d.utility.Vector3dVector(surface_point_colors(np.asarray(mesh.vertices), labels, terrain))
    if not o3d.io.write_triangle_mesh(str(output/'map_refined.ply'), mesh):
        raise RuntimeError('coloured preview write failed')
    unchanged = all(hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p)] for p in inputs)
    if not unchanged:
        raise RuntimeError('source changed during STL export; rerun after mapping finishes')
    report = {'status': 'complete', 'source': str(source), 'input_sha256': hashes, 'source_unchanged': unchanged,
              'raw': raw, 'refined': refined, 'smoothing': smooth, 'ground_holes': holes,
              'removed_degenerate_or_duplicate_triangles': len(area2)-original_faces,
              'remaining_boundary_edges': int((counts == 1).sum()),
              'remaining_nonmanifold_edges': int((counts > 2).sum()),
              'units': 'coordinate values are metres; STL has no unit metadata',
              'not_watertight': bool((counts == 1).any()), 'nav2_ready': False,
              'collision_validated': False, 'self_intersections_checked': False,
              'raw_capture_and_mapping_pipeline_changed': False,
              'inferred_faces': 'inferred_faces.npy; triangle-order mask for raw vs inferred surfaces',
              'limitations': ['large holes and unknown space remain open',
                              'ground patches are inferred, not new measurements',
                              'boundary, class and 45-degree feature edges are fixed',
                              'no component deletion, decimation, global flattening or watertight reconstruction']}
    (output/'quality.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--backend', default='auto', choices=['auto', 'orb_offline', 'orb', 'orbline', 'orb_inertial'])
    parser.add_argument('--map-id', type=int, default=0)
    args = parser.parse_args()
    source = select_maps(args.run, backend=args.backend, map_id=args.map_id)[0][1]
    report = export_stl(source, args.output)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
