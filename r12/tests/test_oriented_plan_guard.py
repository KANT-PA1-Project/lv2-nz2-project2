import copy
import math
from arm_practice.oriented_plan_guard import accept_oriented_plan


def fixture():
    request=dict(request_id='case-1',goal_index=4,map_revision='map-hash',
                 path_revision='path-hash',footprint_revision='cad-hash',
                 created_monotonic_s=10.,sensor_pose=[1.,2.,.5])
    plan=dict(ok=True,preparation_origin=[1.,2.],position_margin_m=.01,
              yaw_error_rad=math.radians(2))
    return plan,request


def run(plan,request,active=None,pose=None,**kwargs):
    return accept_oriented_plan(plan,request,active or copy.deepcopy(request),
        [1.,2.,.5] if pose is None else pose,
        now_s=kwargs.get('now_s',12.),sensor_stamp_s=kwargs.get('sensor_stamp_s',11.95))


def test_current_result_is_not_motion_authorization():
    p,r=fixture();result=run(p,r)
    assert result['accepted'] and not result['motion_authorized']


def test_every_changed_revision_rejects_old_result():
    p,r=fixture()
    for key in ('request_id','goal_index','map_revision','path_revision','footprint_revision'):
        active=copy.deepcopy(r);active[key]='changed'
        assert run(p,r,active)['reason']=='superseded_request'


def test_expired_future_and_nonfinite_clocks_fail_closed():
    p,r=fixture()
    for now,stamp in [(16.,15.95),(12.,10.),(9.,8.95),(12.,12.1),(float('nan'),11.95)]:
        assert not run(p,r,now_s=now,sensor_stamp_s=stamp)['accepted']


def test_motion_since_request_and_wrong_origin_rejected():
    p,r=fixture()
    assert run(p,r,pose=[1.02,2.,.5])['reason']=='preparation_origin_drift'
    assert run(p,r,pose=[1.,2.,.6])['reason']=='preparation_heading_drift'
    p['preparation_origin']=[1.02,2.]
    assert run(p,r)['reason']=='plan_origin_mismatch'


def test_malformed_missing_and_unsafe_margins_fail_closed():
    p,r=fixture()
    for bad in [[],[1,2],['bad',2,3],[float('nan'),2,3]]:
        assert not run(p,r,pose=bad)['accepted']
    for key in ('request_id','map_revision'):
        rr=copy.deepcopy(r);del rr[key]
        assert not run(p,rr)['accepted']
    for margin in [0,-1,float('nan')]:
        pp=copy.deepcopy(p);pp['position_margin_m']=margin
        assert not run(pp,r)['accepted']
