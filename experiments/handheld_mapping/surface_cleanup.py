"""원거리 depth의 경계를 보존하는 평활화와 작은 빈틈 후보 생성."""
import numpy as np


NEAR_M = 1.0
FAR_M = 1.3
MAX_AGREEMENT_M = .045


def smooth_far_depth(depth: np.ndarray, scale: float, minimum: float, maximum: float) -> np.ndarray:
    """5x5 inverse-depth bilateral; 1m 이내·깊이 경계·원본의 빈 픽셀은 유지한다."""
    z = depth.astype(np.float32)*scale
    valid = (z >= minimum) & (z <= maximum)
    inverse = np.divide(1., z, out=np.zeros_like(z), where=valid)
    padded_z = np.pad(z, 2, constant_values=0.)
    padded_inverse = np.pad(inverse, 2, constant_values=0.)
    numerator = np.zeros_like(z)
    denominator = np.zeros_like(z)
    tolerance = np.minimum(.02+.015*z*z, MAX_AGREEMENT_M)
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            other_z = padded_z[2+dy:2+dy+z.shape[0], 2+dx:2+dx+z.shape[1]]
            other_inverse = padded_inverse[2+dy:2+dy+z.shape[0], 2+dx:2+dx+z.shape[1]]
            same_surface = (valid & (other_z >= minimum) & (other_z <= maximum)
                            & (np.abs(other_z-z) <= tolerance))
            weight = np.exp(-.5*((other_inverse-inverse)/.012)**2
                            -.5*(dx*dx+dy*dy)/(1.2**2))*same_surface
            numerator += weight*other_inverse
            denominator += weight
    mean_inverse = np.divide(numerator, denominator, out=inverse.copy(), where=denominator > 0)
    smoothed_z = np.divide(1., mean_inverse, out=z.copy(), where=mean_inverse > 0)
    blend = np.clip((z-NEAR_M)/(FAR_M-NEAR_M), 0., 1.)
    blend = blend*blend*(3.-2.*blend)
    result = depth.copy()
    changed = valid & (z > NEAR_M)
    result[changed] = np.rint(((1.-blend)*z+blend*smoothed_z)[changed]/scale).astype(np.uint16)
    return result


def supported_hole_candidates(depth: np.ndarray, scale: float, maximum: float) -> tuple[np.ndarray, np.ndarray]:
    """원거리의 한 픽셀 빈틈 중 8이웃 6개 이상이 같은 면인 후보만 생성한다.

    후보는 반드시 다른 두 프레임의 실측값 확인을 거쳐야 한다. 재귀 채움은 하지 않는다.
    """
    z = depth.astype(np.float32)*scale
    padded = np.pad(z, 1, constant_values=0.)
    neighbours = np.stack([padded[1+dy:1+dy+z.shape[0], 1+dx:1+dx+z.shape[1]]
                           for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy])
    valid = (neighbours > NEAR_M) & (neighbours <= maximum)
    count = valid.sum(axis=0)
    low = np.where(valid, neighbours, np.inf).min(axis=0)
    high = np.where(valid, neighbours, -np.inf).max(axis=0)
    # Opposing neighbours prevent extending the outside silhouette into unobserved space.
    enclosed = ((z == 0) & (padded[1:-1, :-2] > 0) & (padded[1:-1, 2:] > 0)
                & (padded[:-2, 1:-1] > 0) & (padded[2:, 1:-1] > 0))
    no_conflicting_surface = ((neighbours == 0) | valid).all(axis=0)
    candidate = enclosed & no_conflicting_surface & (count >= 6) & ((high-low) <= .03)
    mean = np.divide(np.where(valid, neighbours, 0.).sum(axis=0), count,
                     out=np.zeros_like(z), where=count > 0)
    result = depth.copy()
    result[candidate] = np.rint(mean[candidate]/scale).astype(np.uint16)
    return result, candidate


def clean_surface_mesh(mesh, camera_positions):
    """표시용: 경로에서 먼 작은 조각을 정리하고 기존 연결만 약하게 평활화한다."""
    from scipy.spatial import cKDTree
    import open3d as o3d
    points = np.asarray(mesh.vertices)
    distances = cKDTree(np.asarray(camera_positions)).query(points)[0]
    labels, counts, areas = mesh.cluster_connected_triangles()
    labels, counts, areas = np.asarray(labels), np.asarray(counts), np.asarray(areas)
    triangles = np.asarray(mesh.triangles)
    near_component = np.zeros(len(counts), bool)
    np.logical_or.at(near_component, labels, np.any(distances[triangles] <= NEAR_M, axis=1))
    tiny = (counts < 100) & (areas < .005) & ~near_component
    removed = int(counts[tiny].sum())
    if removed:
        mesh.remove_triangles_by_mask(tiny[labels])
        mesh.remove_unreferenced_vertices()
    stats = {'method': 'far_tiny_components_and_bounded_taubin_v1',
             'removed_triangles': removed, 'removed_components': int(tiny.sum()),
             'max_vertex_shift_m': 0., 'near_camera_radius_preserved_m': NEAR_M,
             'new_faces_or_bridges': False, 'scope': 'display mesh/cloud only; not terrain/costmap'}
    if not len(mesh.vertices):
        return mesh, stats
    original = np.asarray(mesh.vertices).copy()
    distance = cKDTree(np.asarray(camera_positions)).query(original)[0]
    smoothed = mesh.filter_smooth_taubin(number_of_iterations=2,
                                        filter_scope=o3d.geometry.FilterScope.Vertex)
    delta = np.asarray(smoothed.vertices)-original
    # Blend only far vertices and cap displacement at 5mm. No new faces or long bridges.
    blend = np.clip((distance-NEAR_M)/(FAR_M-NEAR_M), 0., 1.)
    length = np.linalg.norm(delta, axis=1)
    delta *= np.minimum(1., .005/np.maximum(length, 1e-12))[:, None]*blend[:, None]
    mesh.vertices = o3d.utility.Vector3dVector(original+delta)
    mesh.compute_vertex_normals()
    stats['max_vertex_shift_m'] = float(np.linalg.norm(delta, axis=1).max(initial=0.))
    return mesh, stats
