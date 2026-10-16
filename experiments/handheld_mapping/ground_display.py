"""초록 지면 안의 작은 닫힌 구멍만 표시용 삼각형으로 메운다."""
import numpy as np


def _triangulate_polygon(points):
    """단순 오목 다각형을 기존 꼭짓점만 사용해 ear clipping으로 삼각화한다."""
    cross = lambda a, b: a[0]*b[1]-a[1]*b[0]
    for i in range(len(points)):
        a, b = points[i], points[(i+1) % len(points)]
        for j in range(i+1, len(points)):
            if j == i+1 or (i == 0 and j == len(points)-1):
                continue
            c, d = points[j], points[(j+1) % len(points)]
            if (cross(b-a, c-a)*cross(b-a, d-a) <= 0
                    and cross(d-c, a-c)*cross(d-c, b-c) <= 0
                    and np.all(np.maximum(np.minimum(a, b), np.minimum(c, d)) <=
                               np.minimum(np.maximum(a, b), np.maximum(c, d))+1e-12)):
                return []
    orientation = np.sign(sum(cross(a, b) for a, b in zip(points, np.roll(points, -1, axis=0))))
    if not orientation:
        return []
    remaining, result = list(range(len(points))), []
    while len(remaining) > 3:
        for position, current in enumerate(remaining):
            previous, following = remaining[position-1], remaining[(position+1) % len(remaining)]
            a, b, c = points[[previous, current, following]]
            if orientation*cross(b-a, c-b) <= 1e-12:
                continue
            others = [i for i in remaining if i not in (previous, current, following)]
            if any(min(orientation*cross(b-a, points[i]-a),
                       orientation*cross(c-b, points[i]-b),
                       orientation*cross(a-c, points[i]-c)) >= -1e-12 for i in others):
                continue
            result.append([previous, current, following])
            remaining.pop(position)
            break
        else:
            return []
    a, b, c = points[remaining]
    if orientation*cross(b-a, c-b) <= 1e-12:
        return []
    result.append(remaining)
    return result


def fill_small_ground_gaps(mesh, labels):
    """지름5cm·20변 이하이며 초록으로 둘러싸인 평면 내부 구멍만 메운다."""
    import open3d as o3d
    from experiments.handheld_mapping.surface_colors import GROUND

    vertices, triangles = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    labels = np.asarray(labels)
    if labels.shape != (len(vertices),):
        raise ValueError('ground labels do not match mesh vertices')
    stats = {'method': 'enclosed_green_ground_holes_v1', 'filled_holes': 0,
             'added_triangles': 0, 'maximum_hole_diameter_m': .05,
             'new_vertices': False, 'scope': 'viewer memory only; inferred display patches'}
    if not len(triangles) or not np.any(labels == GROUND):
        return stats
    directed = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    _, first, counts = np.unique(np.sort(directed, axis=1), axis=0, return_index=True, return_counts=True)
    boundary = directed[first[counts == 1]]
    face_ids = np.tile(np.arange(len(triangles)), 3)[first[counts == 1]]
    mesh.compute_triangle_normals()
    normals = np.asarray(mesh.triangle_normals)
    outgoing, incoming = {}, {}
    for i, (a, b) in enumerate(boundary):
        outgoing.setdefault(int(a), []).append(i)
        incoming.setdefault(int(b), []).append(i)
    visited, additions = set(), []
    for start in range(len(boundary)):
        if start in visited:
            continue
        chain, current, closed = [], start, False
        while current not in visited:
            visited.add(current)
            chain.append(current)
            a, b = boundary[current]
            if len(outgoing[int(a)]) != 1 or len(incoming.get(int(a), [])) != 1:
                break
            choices = outgoing.get(int(b), [])
            if len(choices) != 1:
                break
            current = choices[0]
            if current == start:
                closed = True
                break
        if not closed or not 3 <= len(chain) <= 20:
            continue
        indices = boundary[chain, 0]
        if not np.all(labels[indices] == GROUND):
            continue
        points = vertices[indices]
        distance = np.linalg.norm(points[:, None]-points[None, :], axis=2)
        if distance.max() > .05:
            continue
        centered = points-points.mean(axis=0)
        _, singular, basis = np.linalg.svd(centered, full_matrices=False)
        if singular[1] < 1e-5 or np.max(np.abs(centered@basis[2])) > .008:
            continue
        reference = normals[face_ids[chain]].mean(axis=0)
        area = np.cross(centered, np.roll(centered, -1, axis=0)).sum(axis=0)
        # An interior hole winds opposite to its surrounding faces. A small outer island
        # winds the same way and must not get a duplicate cap or grow into the unknown.
        if np.dot(area, reference) >= -1e-10:
            continue
        indices = indices[::-1]
        projected = (vertices[indices]-points.mean(axis=0))@basis[:2].T
        faces = _triangulate_polygon(projected)
        if len(faces) != len(indices)-2:
            continue
        additions.extend(indices[np.asarray(faces)].tolist())
        stats['filled_holes'] += 1
    if additions:
        mesh.triangles = o3d.utility.Vector3iVector(np.r_[triangles, additions])
        mesh.compute_vertex_normals()
    stats['added_triangles'] = len(additions)
    return stats
