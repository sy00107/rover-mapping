import numpy as np

from experiments.handheld_mapping.joined_terrain import join_mask,heightfield_mesh


def gap_example(gap_cells,height_difference=0):
    mask=np.zeros((20,30),bool)
    mask[4:16,4:9]=True
    mask[4:16,9+gap_cells:14+gap_cells]=True
    height=np.full(mask.shape,np.nan)
    height[4:16,4:9]=.25
    height[4:16,9+gap_cells:14+gap_cells]=.25+height_difference
    return mask,height


def test_short_gap_is_joined_but_long_gap_stays_open():
    mask,height=gap_example(3)
    merged,inferred,stats=join_mask(mask,height,.08,.02)
    assert inferred[10,10] and merged[10,10]
    assert stats['merged_components']==1
    mask,height=gap_example(7)
    merged,inferred,stats=join_mask(mask,height,.08,.02)
    assert not merged[10,12]
    assert stats['merged_components']==2


def test_height_mismatch_prevents_short_gap_join():
    mask,height=gap_example(3,.20)
    merged,inferred,stats=join_mask(mask,height,.08,.02)
    assert not inferred[10,10]
    assert stats['merged_components']==2
    assert np.all(merged[mask])


def test_ramp_and_column_heightfield_form_closed_connected_solid():
    x,y=np.meshgrid(np.arange(12)*.02,np.arange(10)*.02)
    height=.2*x+.1*y;height[3:6,4:8]+=.2
    mesh=heightfield_mesh(height,np.array([1.,2.]),.02,-.1)
    assert mesh.is_watertight()
    assert not mesh.is_self_intersecting()
    _,parts,_=mesh.cluster_connected_triangles()
    assert len(parts)==1
    np.testing.assert_allclose(np.asarray(mesh.vertices)[:height.size,2],height.ravel())
