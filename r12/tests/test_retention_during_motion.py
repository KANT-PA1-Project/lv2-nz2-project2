import pytest
from test_pick_place_cycle import Harness

def at_phase(phase):
    h=Harness()
    for _ in range(800):
        if h.c.phase==phase:return h
        h.step()
    raise AssertionError('phase not reached')
def measured_step(h,aperture):
    # Keep the visual object-to-TCP relation unchanged: the fresh aperture
    # observation, not invented GT/slip/contact, must trigger this loss veto.
    return h.c.step(h.t,h.c.reference,[.01,0,0,0,0],aperture,0.,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs())
@pytest.mark.parametrize('phase',['lift','place'])
@pytest.mark.parametrize('aperture',[.027,.034,.075])
def test_loss_during_motion_stops_before_visual_slip(phase,aperture):
    h=at_phase(phase);before=h.c.reference.copy();r=measured_step(h,aperture)
    assert r['phase']=='failed' and r['reason']=='grasp_aperture_lost'
    assert r['cancel_previous_arm_target'] and r['requires_fresh_local_hold']
    assert r['gripper_hold_mode']=='retain_existing_clamp'
    assert (h.c.reference==before).all()
    assert not r['cycle_success'] and not r['release_verified']
    if phase=='lift':assert not r['grasp_verified']
@pytest.mark.parametrize('aperture',[.028,.030,.033])
def test_existing_width_range_not_tightened(aperture):
    h=at_phase('lift');r=measured_step(h,aperture)
    assert r['phase']=='lift' and not r['grasp_verified']
def test_complete_sensor_trace_still_requires_lift_and_release():
    h=Harness();r=h.run()
    assert r['cycle_success'] and r['grasp_verified'] and r['release_verified']
    assert 'mission_success' not in r
