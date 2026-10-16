"""장애물의 화면 표시용 메시·법선만 보정하고 저장 지도와 분류는 유지한다."""
import numpy as np


def smooth_obstacle_display(mesh, labels):
    """주황 영역 내부에만 제한된 Taubin 보정; 경계·모서리·연결은 고정한다."""
    import open3d as o3d
    from scipy.sparse import coo_matrix
    from experiments.handheld_mapping.surface_colors import OBSTACLE

    original = np.asarray(mesh.vertices).copy()
    triangles = np.asarray(mesh.triangles)
    labels = np.asarray(labels)
    if labels.shape != (len(original),):
        raise ValueError('obstacle labels do not match mesh vertices')
    obstacle = labels == OBSTACLE
    stats = {'method': 'masked_normal_taubin_display_v1', 'obstacle_vertices': int(obstacle.sum()),
             'smoothed_vertices': 0, 'max_vertex_shift_m': 0., 'iterations': 8,
             'new_faces_or_bridges': False, 'scope': 'viewer memory only'}
    if not obstacle.any() or not len(triangles):
        return stats
    mesh.compute_vertex_normals()
    normals = np.asarray(mesh.vertex_normals).copy()
    face_normals = np.asarray(mesh.triangle_normals)
    edges, inverse, counts = np.unique(np.sort(np.concatenate([
        triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]]), axis=1),
        axis=0, return_inverse=True, return_counts=True)
    face_indices = np.tile(np.arange(len(triangles)), 3)
    first, last = np.full(len(edges), len(triangles)), np.zeros(len(edges), int)
    np.minimum.at(first, inverse, face_indices)
    np.maximum.at(last, inverse, face_indices)
    sharp = np.einsum('ij,ij->i', face_normals[first], face_normals[last]) < np.cos(np.deg2rad(60.))
    length = np.linalg.norm(original[edges[:, 0]]-original[edges[:, 1]], axis=1)
    mixed = labels[edges[:, 0]] != labels[edges[:, 1]]
    fixed = np.zeros(len(original), bool)
    fixed[edges[(counts != 2) | sharp | mixed | (length > .05)].ravel()] = True
    movable = obstacle & ~fixed
    if not movable.any():
        return stats
    same = obstacle[edges[:, 0]] & obstacle[edges[:, 1]] & ~sharp & (length <= .05)
    edges, length = edges[same], length[same]
    weights = 1./np.maximum(length, .003)
    row, column = np.r_[edges[:, 0], edges[:, 1]], np.r_[edges[:, 1], edges[:, 0]]
    adjacency = coo_matrix((np.r_[weights, weights], (row, column)),
                           shape=(len(original), len(original))).tocsr()
    total = np.asarray(adjacency.sum(axis=1)).ravel()
    movable &= total > 0
    normalization = 1./np.maximum(total, 1e-12)
    points = original.copy()
    for _ in range(8):
        for strength in (.5, -.53):
            delta = adjacency.dot(points)*normalization[:, None]-points
            # Normal-only movement avoids pulling the obstacle outline sideways.
            delta = np.einsum('ij,ij->i', delta, normals)[:, None]*normals
            candidate = points+strength*delta
            displacement = candidate-original
            displacement *= np.minimum(1., .01/np.maximum(np.linalg.norm(displacement, axis=1), 1e-12))[:, None]
            points[movable] = (original+displacement)[movable]
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.compute_vertex_normals()
    shaded = np.asarray(mesh.vertex_normals).copy()
    # Smooth lighting across the same obstacle surface, without blending category colours.
    for _ in range(3):
        mean = adjacency.dot(shaded)*normalization[:, None]
        mean /= np.maximum(np.linalg.norm(mean, axis=1), 1e-12)[:, None]
        aligned = np.einsum('ij,ij->i', mean, shaded) > .5
        active = movable & aligned
        shaded[active] = .5*(shaded[active]+mean[active])
        shaded[active] /= np.maximum(np.linalg.norm(shaded[active], axis=1), 1e-12)[:, None]
    mesh.vertex_normals = o3d.utility.Vector3dVector(shaded)
    shift = np.linalg.norm(points-original, axis=1)
    stats.update(smoothed_vertices=int(np.count_nonzero(shift > 1e-9)),
                 max_vertex_shift_m=float(shift.max(initial=0.)))
    return stats
