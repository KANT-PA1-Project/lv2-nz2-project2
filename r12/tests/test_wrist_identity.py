import copy
import numpy as np
from arm_practice.wrist_identity import WristIdentity


def sample():
    return dict(id=0, color='R', shape='cylinder', center_map_m=[1., 2., .3],
                pose_camera=dict(center_camera_m=[.1, .2, .4], confidence=.9, translation_valid=True),
                depth_surface_m=.4, depth_consistent=True)


def observe(gate, stamp, candidate=None, **override):
    args=dict(camera='arm_camera', camera_frame='arm_camera_left_optical', run_id='run1',
              stamp_ns=stamp, fresh=True, view_ready=True, stationary=True, arm_velocity=[0.]*5)
    args.update(override)
    return gate.observe(sample() if candidate is None else candidate, **args)


def test_top_camera_never_confirms_and_wrist_requires_three_distinct_frames():
    g=WristIdentity()
    for i in range(10):assert observe(g, i*50_000_000, camera='r07_camera1') is None
    assert observe(g, 1_000_000_000) is None
    assert observe(g, 1_050_000_000) is None
    assert observe(g, 1_050_000_000) is None
    result=observe(g, 1_100_000_000)
    assert result['wrist_consistent_frames']==3 and result['camera']=='arm_camera'
    assert not result['grasp_pose_accepted']


def test_motion_stale_or_unready_resets_partial_confirmation():
    for override in [dict(stationary=False),dict(fresh=False),dict(view_ready=False),dict(arm_velocity=[.03]*5)]:
        g=WristIdentity();observe(g,0);observe(g,50_000_000)
        assert observe(g,100_000_000,**override) is None
        assert observe(g,150_000_000) is None


def test_conflicting_position_or_class_restarts_evidence():
    for change in [dict(center_map_m=[1.03,2.,.3]),dict(shape='cuboid'),dict(color='G')]:
        g=WristIdentity();observe(g,0);observe(g,50_000_000);s=sample();s.update(change)
        assert observe(g,100_000_000,s) is None
        assert g.pending[0]['count']==1


def test_invalid_pose_depth_id_or_duplicates_are_rejected():
    bad=[]
    for key,value in [('id',12),('id',True),('center_map_m',[1.,2.,np.nan]),('depth_consistent',False),('depth_surface_m',-1.)]:
        s=sample();s[key]=value;bad.append(s)
    for key,value in [('translation_valid',False),('confidence',np.nan),('center_camera_m',[0,0,-1])]:
        s=sample();s['pose_camera'][key]=value;bad.append(s)
    for s in bad:
        g=WristIdentity()
        for i in range(4):assert observe(g,i*50_000_000,s) is None
    assert observe(WristIdentity(),0,unique_in_frame=False) is None


def test_run_and_frame_gaps_cannot_reuse_confirmation():
    for override,stamp in [(dict(run_id='run2'),100_000_000),({},500_000_000)]:
        g=WristIdentity();observe(g,0);observe(g,50_000_000)
        assert observe(g,stamp,**override) is None


def test_result_preserves_input_and_is_independent_of_future_mutation():
    g=WristIdentity();s=sample();original=copy.deepcopy(s)
    observe(g,0,s);observe(g,50_000_000,s);r=observe(g,100_000_000,s)
    assert s==original
    s['pose_camera']['confidence']=0
    assert r['pose_camera']['confidence']==.9


def test_conflicting_current_wrist_evidence_revokes_old_inventory_entry():
    g=WristIdentity();observe(g,0);observe(g,50_000_000)
    assert observe(g,100_000_000) is not None
    moved=sample();moved['center_map_m'][0]+=.05
    assert observe(g,150_000_000,moved) is None
    assert g.invalidated_ids=={0}
