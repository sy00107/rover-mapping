"""Display-only planar replacement of the two explicitly selected fixed ramps."""
import numpy as np
from scipy.spatial import ConvexHull

from .slope_step import plane
from .surface_colors import PALETTE, OBSTACLE, SLOPE_COLOR


def polish_ramps(mesh, patches, tolerance=.06):
    """Replace noisy ground in observed convex footprints; preserve orange objects.

    These are inferred complete ramp surfaces, including unobserved interior holes.
    No extension beyond the selected support hulls or change to the input mesh.
    """
    import open3d as o3d
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    colors = np.asarray(mesh.vertex_colors)
    if colors.shape != vertices.shape:
        raise ValueError('classified vertex colours required')
    orange = np.linalg.norm(colors-PALETTE[OBSTACLE], axis=1) < .05
    remove = np.zeros(len(faces), bool)
    surfaces = o3d.geometry.TriangleMesh()
    stats = []
    for points in patches:
        fit = plane(points)
        hull = ConvexHull(points[:, :2])
        # All three corners must be inside and close to this plane. This keeps
        # neighbouring ground, elevated structures and mixed boundary faces.
        inside = np.all(vertices[:, :2] @ hull.equations[:, :2].T
                        + hull.equations[:, 2] <= 1e-8, axis=1)
        residual = vertices[:, 2]-(vertices[:, :2]@fit[:2]+fit[2])
        selected = inside & (np.abs(residual) <= tolerance) & ~orange
        covered = selected[faces].all(axis=1)
        remove |= covered
        xy = points[hull.vertices, :2]
        # Convex fan: exact footprint coverage with no holes, duplicate faces
        # or rough surface normals. Heights use the same plane as the riser.
        xyz = np.c_[xy, xy@fit[:2]+fit[2]]
        triangles = np.array([[0, i, i+1] for i in range(1, len(xy)-1)], np.int32)
        surface = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(xyz),
                                          o3d.utility.Vector3iVector(triangles))
        surface.paint_uniform_color(SLOPE_COLOR)
        surfaces += surface
        stats.append(dict(plane=fit.tolist(), footprint_area_m2=float(hull.volume),
                          removed_triangles=int(covered.sum()),
                          replacement_triangles=len(triangles)))
    result = o3d.geometry.TriangleMesh(mesh)
    result.remove_triangles_by_mask(remove)
    result.remove_unreferenced_vertices()
    result += surfaces
    result.compute_vertex_normals()
    return result, dict(method='selected_ramp_convex_plane_display_v1', inferred=True,
                        display_only=True, maximum_replacement_residual_m=tolerance,
                        footprint='convex hull of selected observed ramp cell centres',
                        ramps=stats, removed_triangles=int(remove.sum()))
