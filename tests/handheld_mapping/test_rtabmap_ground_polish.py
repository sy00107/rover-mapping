import numpy as np

from experiments.handheld_mapping.rtabmap_ground_polish import reconnect_ground_sheet,stable_ground_normals
from experiments.handheld_mapping.surface_colors import GROUND,OBSTACLE
from tests.handheld_mapping.test_ground_display import ground_with_hole


def flat_terrain():
    shape=(40,40)
    return dict(bounds=np.array([-.1,.3,-.1,.3]),resolution=.01,grid_from_geometry=np.eye(4),
                ground_tolerance_m=.02,obstacle_threshold_m=.06,
                layers={'ground':np.ones(shape,bool),'surface_observed':np.ones(shape,bool),
                        'obstacle_observed':np.ones(shape,bool),'reference_ground_height_m':np.zeros(shape),
                        'reference_ground_gradient_x':np.zeros(shape),'reference_ground_gradient_y':np.zeros(shape)})


def test_ground_normals_do_not_change_obstacles_or_input_arrays():
    terrain=flat_terrain();terrain['layers']['reference_ground_gradient_x'][:]=.25
    points=np.array([[0.,0.,0.],[.01,.01,.1]])
    normals=np.array([[0.,0.,-1.],[1.,0.,0.]])
    result,stats=stable_ground_normals(points,normals,np.array([GROUND,OBSTACLE]),terrain)
    np.testing.assert_allclose(result[0],np.array([-.25,0,1])/np.sqrt(1.0625))
    np.testing.assert_array_equal(result[1],normals[1])
    np.testing.assert_array_equal(normals[0],[0,0,-1])
    assert stats['vertices']==1


def test_reconnected_ground_respects_gap_limit_and_observed_mask():
    mesh,labels=ground_with_hole(width=3,slope=0)
    terrain=flat_terrain()
    # A region with missing terrain evidence must not acquire inferred faces.
    terrain['layers']['ground'][18:22,18:22]=False
    original=np.asarray(mesh.vertices).copy()
    stats=reconnect_ground_sheet(mesh,labels,terrain)
    faces=np.asarray(mesh.triangles);xyz=np.asarray(mesh.vertices)[faces]
    assert len(faces)>0 and stats['maximum_edge_m']==.06
    assert np.linalg.norm(xyz-np.roll(xyz,1,axis=1),axis=2).max()<=.06+1e-9
    np.testing.assert_array_equal(mesh.vertices,original)
    centres=xyz.mean(axis=1);mask=(centres[:,0]>=.08)&(centres[:,0]<.12)&(centres[:,1]>=.08)&(centres[:,1]<.12)
    assert not mask.any()
    from experiments.handheld_mapping.stl_export import mesh_edges
    assert not (mesh_edges(faces)[2]>2).any()
