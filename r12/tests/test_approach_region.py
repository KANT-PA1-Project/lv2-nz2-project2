from arm_practice.approach_region import ApproachRegion,region_arrival_ready
from arm_practice.goal_stop_hold import GoalStopHold,goal_arrival_ready


def test_coarse_region_only_accepts_settled_and_fresh_base():
    h=ApproachRegion(.1)
    h.update(1000000000,0,.08,.1,False,True)
    assert not region_arrival_ready(.08,.1,h)
    h.update(1050000000,0,.08,.01,True,True)
    assert region_arrival_ready(.08,.01,h)
    h.update(1100000000,0,.08,.01,True,False)
    assert not region_arrival_ready(.08,.01,h)


def test_legacy_arrival_is_unchanged():
    assert GoalStopHold.radius_m==.02
    assert not goal_arrival_ready(.08,.01)


def test_drift_outside_region_requires_reapproach():
    h=ApproachRegion(.1)
    h.update(1000000000,0,.08,.01,True,True)
    h.update(1050000000,0,.11,.01,True,True)
    assert not h.active
