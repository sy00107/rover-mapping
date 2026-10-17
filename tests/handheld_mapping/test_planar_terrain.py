import numpy as np
from experiments.handheld_mapping.planar_terrain import fit_planes, planar_mesh, ground_solid


def test_noisy_flat_ramp_become_exact_planes_in_gravity_frame():
    rng=np.random.default_rng(12)
    x,y=np.meshgrid(np.arange(0,2.01,.02),np.arange(-.4,.41,.02))
    true=np.maximum(0.,np.tan(np.deg2rad(10))*(x-1))
    points=np.c_[x.ravel(),y.ravel(),(true+rng.normal(0,.002,true.shape)).ravel()]
    planes=fit_planes(points,threshold=.007)
    angles=sorted(np.degrees(np.arctan(np.linalg.norm(p['coefficient'][:2]))) for p in planes)
    assert len(angles)==2
    assert angles[0]<.5 and abs(angles[1]-10)<.5
    mesh,stats=planar_mesh(points,planes)
    v=np.asarray(mesh.vertices);t=np.asarray(mesh.triangles)
    coefficients=np.asarray([p['coefficient'] for p in planes])
    residual=abs(np.c_[v[:,:2],np.ones(len(v))]@coefficients.T-v[:,2,None])
    assert np.max(np.min(residual,axis=1))<1e-10
    # Every triangle belongs to one plane, including the junction split.
    assert np.max(np.min(np.max(residual[t],axis=1),axis=1))<1e-8
    assert len(ground_solid(mesh).get_non_manifold_edges(allow_boundary_edges=False))==0


def test_large_unobserved_hole_stays_open():
    x,y=np.meshgrid(np.arange(0,1.01,.02),np.arange(0,1.01,.02))
    keep=~((x>.3)&(x<.7)&(y>.3)&(y<.7))
    points=np.c_[x[keep],y[keep],np.zeros(keep.sum())]
    mesh,_=planar_mesh(points,[{'coefficient':np.zeros(3)}])
    centers=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].mean(axis=1)
    assert not np.any((centers[:,0]>.38)&(centers[:,0]<.62)&(centers[:,1]>.38)&(centers[:,1]<.62))


def test_start_ground_origin_uses_vertical_projection_without_rotation():
    from experiments.handheld_mapping.planar_terrain import start_ground_origin
    planes=[{'coefficient':np.array([0.,0.,.3])},
            {'coefficient':np.array([.2,0.,-.1])}]
    start=np.array([3.,2.,1.5])
    origin=start_ground_origin(start,planes)
    np.testing.assert_allclose(origin,[3.,2.,.5])
    np.testing.assert_allclose(start-origin,[0.,0.,1.])
    xyz=np.array([[3.,2.,.5],[4.,2.,.7]])
    shifted=xyz-origin
    np.testing.assert_allclose(shifted[0],0.,atol=1e-12)
    np.testing.assert_allclose(shifted[1]-shifted[0],xyz[1]-xyz[0])
