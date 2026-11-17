import dataclasses,pytest
from test_pick_place_cycle import Harness
from arm_practice.pick_place_cycle import PickPlaceCycle

def at_phase(phase):
    h=Harness()
    for _ in range(800):
        if h.c.phase==phase:return h
        h.step()
    raise AssertionError('phase not reached')
def feed(h,obs):
    return h.c.step(h.t,h.c.reference,[0]*5,max(h.c.width,.030),0.,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=obs)
@pytest.mark.parametrize('phase',['lift','hold','place','release'])
@pytest.mark.parametrize('loss',['missing','stale','no_depth','no_identity','wrong_target','wrong_run'])
def test_visual_loss_faults_and_cannot_resume(phase,loss):
    h=at_phase(phase);o=h.obs();before=h.c.reference.copy()
    if loss=='missing':o=None
    elif loss=='stale':o=dataclasses.replace(o,capture_ns=0,encoder_capture_ns=0)
    elif loss=='no_depth':o=dataclasses.replace(o,depth_valid=False)
    elif loss=='no_identity':o=dataclasses.replace(o,identity_verified=False)
    elif loss=='wrong_target':o=dataclasses.replace(o,target_id=4)
    else:o=dataclasses.replace(o,run_id='new_run')
    r=feed(h,o)
    assert r['phase']=='failed' and not r['cycle_success'] and not r['release_verified']
    assert r['arm_target'] is None and r['cancel_previous_arm_target'] and r['requires_fresh_local_hold']
    assert (h.c.reference==before).all()
    if phase in ('lift','hold'):assert not r['grasp_verified']
    # Historical grasp at place/release is allowed, but never current success.
    assert feed(h,h.obs())==r

def test_completed_cycle_does_not_seed_new_target():
    old=Harness();assert old.run()['cycle_success']
    h=Harness();h.c=PickPlaceCycle(4,h.c.waypoints,h.c.lo,h.c.hi,h.clearance,placement_base_m=[.3,0,.5])
    c=h.c.command()
    assert c['phase']=='observe' and not c['grasp_verified'] and not c['release_verified'] and not c['cycle_success']
    assert h.c.hold_samples==[] and h.c.release_samples==[] and h.c.run_id is None
    h.step(old.obs())
    assert h.c.phase=='observe' and not h.c.retained and not h.c.released

def test_new_instance_same_target_new_run_does_not_inherit_success():
    old=Harness();old.run();new=Harness()
    new.step(dataclasses.replace(new.obs(),run_id='new_run'))
    assert new.c.phase=='observe' and new.c.run_id=='new_run'
    assert not new.c.retained and not new.c.released and new.c.hold_samples==[]

def test_failed_cycle_requires_new_instance_not_implicit_reset():
    h=at_phase('lift');r=feed(h,None);fresh=dataclasses.replace(h.obs(),run_id='fresh_run')
    assert feed(h,fresh)==r
    assert not hasattr(h.c,'reset')
