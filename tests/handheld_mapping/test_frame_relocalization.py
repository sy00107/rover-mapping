import numpy as np
from experiments.handheld_mapping.frame_relocalization import solve_view


def fixture():
    rng=np.random.default_rng(14)
    world=rng.uniform([-1,-.8,1.5],[1,.8,3.5],(150,3))
    k=np.array([[500.,0,320],[0,500,240],[0,0,1]])
    uv=(world@k.T)[:,:2]/world[:,2,None]
    return world,uv,k,np.arange(150)


def test_independent_features_recover_metric_pose():
    world,uv,k,ids=fixture()
    result=solve_view(world,uv,k,ids)
    assert result['passed']
    assert result['held_out_used_for_fit'] is False
    np.testing.assert_allclose(result['pose'],np.eye(4),atol=1e-5)


def test_held_out_corruption_rejected_without_affecting_fit():
    world,uv,k,ids=fixture()
    uv[ids%3==0]+=30
    result=solve_view(world,uv,k,ids)
    assert not result['passed']
    np.testing.assert_allclose(result['pose'],np.eye(4),atol=1e-5)


def test_too_few_independent_features_rejected():
    world,uv,k,ids=fixture()
    assert solve_view(world[:30],uv[:30],k,ids[:30]) is None
