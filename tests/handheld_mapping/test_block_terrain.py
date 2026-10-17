import numpy as np

from experiments.handheld_mapping.block_terrain import floor_transform,fit_rectangle,union_boxes


def test_floor_alignment_is_rigid_and_places_fitted_plane_at_zero():
    plane=np.array([-.1,.2,1.,.3]);matrix=floor_transform(plane)
    xy=np.array([[0.,0.],[1.,0.],[0.,2.]])
    z=-(xy@plane[:2]+plane[3])/plane[2]
    points=np.column_stack([xy,z]);aligned=points@matrix[:3,:3].T+matrix[:3,3]
    np.testing.assert_allclose(aligned[:,2],0,atol=1e-10)
    np.testing.assert_allclose(matrix[:3,:3]@matrix[:3,:3].T,np.eye(3),atol=1e-10)
    assert np.linalg.det(matrix[:3,:3])>0


def test_rectangle_retains_long_bar_orientation():
    x,y=np.meshgrid(np.linspace(-1,1,30),np.linspace(-.08,.08,8))
    points=np.c_[x.ravel(),y.ravel()];angle=.4
    rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    points=points@rotation.T+[2,3]
    centre,size,yaw=fit_rectangle(points)
    np.testing.assert_allclose(centre,[2,3],atol=.02)
    assert max(size)>1.8 and min(size)<.2
    assert abs(np.sin(2*(yaw-angle)))<.05


def test_floor_and_overlapping_rotated_boxes_form_one_closed_solid():
    import pytest
    pytest.importorskip("manifold3d")
    floor=dict(size_m=[3,3,.1],bottom_center_m=[0,0,-.1],yaw_rad=0.)
    box=dict(size_m=[.3,1,.5],bottom_center_m=[0,0,-.025],yaw_rad=.3)
    another=dict(size_m=[.5,.5,.4],bottom_center_m=[.2,.2,-.025],yaw_rad=.1)
    mesh=union_boxes([floor,box,another])
    assert mesh.is_watertight()
    assert not mesh.is_self_intersecting()
    _,counts,_=mesh.cluster_connected_triangles()
    assert len(counts)==1
    assert len(mesh.triangles)<500
