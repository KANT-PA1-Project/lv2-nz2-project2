import numpy as np
import pytest
from test_pick_place_cycle import Harness
from arm_practice.pick_place_cycle import PickPlaceCycle

def test_reference_rate_matches_slow_lqr_contract():
 h=Harness();h.c.waypoints['approach'][:]=.2
 prior=None
 for _ in range(80):
  r=h.step();q=r['arm_target']
  if q is not None:
   if prior is not None:assert np.max(np.abs(np.asarray(q)-prior))<=.005*.05+1e-12
   prior=np.array(q)
 assert h.c.phase=='approach'
 assert np.max(prior)>0

@pytest.mark.parametrize('rate',[0,-.1,.005001,float('nan'),float('inf'),True])
def test_rate_cannot_exceed_validated_controller_contract(rate):
 h=Harness()
 with pytest.raises(ValueError,match='arm_reference_rate'):
  PickPlaceCycle(3,h.c.waypoints,[-1]*5,[1]*5,h.clearance,placement_base_m=[.3,0,.5],arm_reference_rate=rate)


def test_impossible_round_trip_rejected_before_motion():
 h=Harness();h.c.waypoints['approach'][:]=.5
 assert h.c.reference_travel_lower_bound(np.zeros(5))>h.c.timeout
 r=h.step()
 assert r['phase']=='failed'
 assert r['reason']=='plan_travel_exceeds_cycle_timeout'
 assert r['arm_target'] is None
 assert not r['grasp_verified']


def test_travel_bound_uses_parallel_joint_motion_and_every_segment():
 h=Harness()
 for phase,x in zip(('approach','lift','place','retreat','refold'),(.1,.2,.1,.05,0.)):
  h.c.waypoints[phase][:]=x
 assert h.c.reference_travel_lower_bound(np.zeros(5))==pytest.approx(80.)
