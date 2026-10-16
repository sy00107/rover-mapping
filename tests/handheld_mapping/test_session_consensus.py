"""Separate-recording agreement, actual free space and occlusion protection."""
import numpy as np
import pytest

from experiments.handheld_mapping.session_consensus import SessionConsensus


def grid(count=2):
    return SessionConsensus([-1.,-1.,0.],[1.,1.,2.5],count,resolution=.02)


def add(evidence,index,surface,free=(),frames=3):
    evidence.start(index)
    for _ in range(frames):
        evidence.add_points(np.asarray(surface).reshape(-1,3),np.asarray(free).reshape(-1,3))
    evidence.finish()


def test_other_recording_free_space_rejects_unsupported_foreground():
    evidence=grid()
    add(evidence,0,[(0,0,1.)])
    add(evidence,1,[(0,0,1.5)],[(0,0,1.)])
    reject,confirmed=evidence.classify(np.array([[0,0,1.],[.8,0,1.]]),0)
    assert reject.tolist()==[True,False]
    assert not confirmed.any()  # Own-session surface is not independent evidence.


def test_nearby_real_surface_protects_rounding_and_agreement():
    evidence=grid()
    add(evidence,0,[(0,0,1.)])
    add(evidence,1,[(0,0,1.01)],[(0,0,1.)])
    reject,confirmed=evidence.classify(np.array([[0,0,1.]]),0)
    assert not reject[0] and confirmed[0]


def test_another_supporting_session_protects_against_a_conflicting_session():
    evidence=grid(3)
    add(evidence,0,[(0,0,1.)]);add(evidence,1,[(0,0,1.)])
    add(evidence,2,[(0,0,1.5)],[(0,0,1.)])
    reject,confirmed=evidence.classify(np.array([[0,0,1.]]),0)
    assert confirmed[0] and not reject[0]


def test_single_unstable_frame_cannot_vote_as_clear_space():
    evidence=grid();add(evidence,0,[(0,0,1.)])
    add(evidence,1,[(0,0,1.5)],[(0,0,1.)],frames=1)
    reject,confirmed=evidence.classify(np.array([[0,0,1.]]),0)
    assert not reject[0] and not confirmed[0]


def test_actual_depth_rays_do_not_label_occluded_background_as_free():
    info={'width':80,'height':60,'k':[70.,0.,39.5,0.,70.,29.5,0.,0.,1.],
          'depth_scale_m':.001,'T_left_depth':np.eye(4).tolist()}
    evidence=grid()
    for index,z in [(0,2000),(1,1000)]:
        evidence.start(index)
        for _ in range(3):evidence.add_depth(np.full((60,80),z,np.uint16),info,np.eye(4),2.5)
        evidence.finish()
    # The foreground camera only sees through space BEFORE its 1m surface.
    # Use an actually sampled ray/step; missing sparse evidence must stay unknown.
    free_point=[.5*.61/70,2.5*.61/70,.61]
    reject,confirmed=evidence.classify(np.array([[0.,0.,2.],free_point]),0)
    assert not reject[0] and not confirmed[0]
    assert reject[1]


def test_classification_requires_completed_independent_recordings():
    evidence=grid();add(evidence,0,[(0,0,1.)])
    with pytest.raises(ValueError,match='incomplete'):evidence.classify(np.array([[0,0,1.]]),0)


def test_grid_limit_is_enforced_before_large_allocation():
    evidence=SessionConsensus([0,0,0],[10,10,4],2,max_cells=10000)
    assert len(evidence.surface)<=10000
