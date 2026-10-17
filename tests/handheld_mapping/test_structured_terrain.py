import numpy as np
import shapely as sh
from shapely.geometry import Polygon
from experiments.handheld_mapping.structured_terrain import footprint, ground_mesh, planar_obstacles


def test_constrained_ground_keeps_hole_and_exact_planar_crease():
    shape=Polygon([(0,0),(2,0),(2,1),(1.8,1),(1.8,.8),(0,.8)],
                  holes=[[(.3,.2),(.6,.2),(.6,.5),(.3,.5)]])
    coefficients=np.array([[0.,0.,0.],[.2,0.,-.2]])
    mesh=ground_mesh(shape,[{'coefficient':c} for c in coefficients])
    v=np.asarray(mesh.vertices);t=np.asarray(mesh.triangles)
    residual=abs(np.c_[v[:,:2],np.ones(len(v))]@coefficients.T-v[:,2,None])
    assert np.min(np.max(residual[t],axis=1),axis=1).max()<1e-8
    areas=[]
    for tri in v[t]:
        polygon=Polygon(tri[:,:2]);assert shape.covers(polygon)
        areas.append(polygon.area)
    assert abs(sum(areas)-shape.area)<1e-8
    assert len(t)<30  # Contour triangulation, no interior raster tessellation.


def test_footprint_does_not_fill_large_unknown_hole():
    x,y=np.meshgrid(np.arange(0,1.01,.02),np.arange(0,1.01,.02))
    keep=~((x>.3)&(x<.7)&(y>.3)&(y<.7))
    shape=footprint(np.c_[x[keep],y[keep]],closing=.03,simplify=.015)
    assert not shape.covers(sh.Point(.5,.5))
    assert shape.is_valid


def test_obstacle_plane_reconstruction_preserves_tilt_without_box_fill():
    import open3d as o
    x,y=np.meshgrid(np.arange(0,.61,.02),np.arange(0,.51,.02))
    points=np.c_[x.ravel(),y.ravel(),(.4+.2*x).ravel()]
    ny,nx=x.shape
    a=(np.arange(ny-1)[:,None]*nx+np.arange(nx-1)).ravel()
    faces=np.r_[np.c_[a,a+1,a+nx],np.c_[a+1,a+nx+1,a+nx]]
    source=o.geometry.TriangleMesh(o.utility.Vector3dVector(points),o.utility.Vector3iVector(faces))
    mesh,stats=planar_obstacles(source,np.full(len(points),2))
    assert stats['patch_count']>=1
    assert len(mesh.triangles)<len(faces)/4
    v=np.asarray(mesh.vertices)
    assert np.max(abs(v[:,2]-(.4+.2*v[:,0])))<1e-7
    assert not mesh.is_watertight()  # No invented sides, underside or cube.
