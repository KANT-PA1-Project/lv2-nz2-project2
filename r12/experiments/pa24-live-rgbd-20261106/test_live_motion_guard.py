"""Dispatch boundaries on synthetic state; no simulator control authority."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parent/'source'
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/sensor_runtime')]
from live_motion_guard import LiveMotionGuard
from arm_practice.native_rgbd.live_view import LiveViewCandidate,_CANDIDATE_MINTER
import test_live_view as lifecycle


class FakeRobot:
    def __init__(self,q):self.q=q
    def get_joint_positions(self):return self.q


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.now=1_000_000_000;self.q=np.array([0.,.25,-.3298,-.3298,0.,.027,-.027])
        self.guard=LiveMotionGuard.__new__(LiveMotionGuard)
        g=self.guard;g.run_id='unit';g.issuer_id='issuer';g.clock=lambda:self.now;g.epoch=lambda:(500,0.)
        g.arm=list(range(5));g.fingers=[5,6];g.robot=FakeRobot(self.q)
        g.anchor=self.q[:5].copy();g.half_width=.002;g.tracking=.001;g.max_step=.0004
        g.joint_lower=np.full(5,-.33);g.joint_upper=np.full(5,.33)
        g.lower=np.maximum(g.anchor-.003,g.joint_lower);g.upper=np.minimum(g.anchor+.003,g.joint_upper)
        g.finger_anchor=self.q[5:].copy();g.fingerprint='fixed';g._environment=lambda:([],'fixed');g.consumed=False
        self.data=dict(q=g.anchor+np.array([0.,.0005,0.,0.,0.]),q_start=g.anchor.copy(),
                       capture_step=500,capture_lower_ns=self.now,run_id='unit',issuer_id='issuer')
    def candidate(self):return LiveViewCandidate(self.data,_mint=_CANDIDATE_MINTER)
    def reject(self,reason):
        with self.assertRaisesRegex(ValueError,reason):self.guard.admit(self.candidate())
    def test_current_candidate_admitted_once(self):
        goal=self.guard.admit(self.candidate())
        self.assertAlmostEqual(float(np.max(abs(goal-self.data['q_start']))),.0004)
        self.assertGreater(goal[1],self.data['q_start'][1])
        self.reject('already_consumed')
    def test_dictionary_replay_rejected(self):
        with self.assertRaisesRegex(ValueError,'serialized_or_untrusted'):
            self.guard.admit(self.data)
    def test_wrong_run_rejected(self):self.data['run_id']='other';self.reject('run_mismatch')
    def test_wrong_issuer_rejected(self):self.data['issuer_id']='other';self.reject('issuer_mismatch')
    def test_stale_rejected(self):self.now+=500_000_001;self.reject('stale_at_dispatch')
    def test_future_clock_rejected(self):self.now-=1;self.reject('stale_at_dispatch')
    def test_changed_epoch_rejected(self):self.data['capture_step']=499;self.reject('epoch_not_current')
    def test_native_start_drift_rejected(self):self.q[1]+=.000001;self.reject('native_start_changed')
    def test_outside_region_rejected(self):self.data['q'][1]+=.003;self.reject('outside_certified_region')
    def test_changed_environment_rejected(self):self.guard._environment=lambda:([],'changed');self.reject('geometry_changed')
    def test_velocity_veto(self):
        qd=np.zeros(7);qd[1]=.02
        with self.assertRaisesRegex(ValueError,'velocity_abort'):self.guard.check_state(self.q,qd,np.zeros(7),[])
    def test_same_step_FD_veto(self):
        fd=np.zeros(7);fd[1]=.02
        with self.assertRaisesRegex(ValueError,'velocity_abort'):self.guard.check_state(self.q,np.zeros(7),fd,[])
    def test_contact_veto(self):
        with self.assertRaisesRegex(ValueError,'unexpected_contact'):self.guard.check_state(self.q,np.zeros(7),np.zeros(7),[{}])
    def test_joint_region_veto(self):
        self.q[1]+=.004
        with self.assertRaisesRegex(ValueError,'region_exit'):self.guard.check_state(self.q,np.zeros(7),np.zeros(7),[])


class LiveConsumptionTests(unittest.TestCase):
    def setUp(self):
        lifecycle.LifecycleTests.setUp(self)
        self.body.update(run_id='synthetic-unit-test',issuer_id='unit')
        self.body['encoder_history']['capture_settling']=dict(settled_interval=True,continuous_steps=500)
    def confirm(self):
        self.planner.propose(self.capture)
        self.step+=1;self.body['capture_physics_step']=self.step;self.body['current_physics_step']=self.step
        self.planner.issuer.last_verified=self.step
        self.planner.propose(self.capture)
    def test_unconfirmed_identity_cannot_dispatch(self):
        self.planner.propose(self.capture)
        with self.assertRaisesRegex(ValueError,'no_live_confirmed'):self.planner.consume_live_candidate()
    def test_consumption_is_once_only(self):
        self.confirm()
        self.assertIsInstance(self.planner.consume_live_candidate(),LiveViewCandidate)
        with self.assertRaisesRegex(ValueError,'no_live_confirmed'):self.planner.consume_live_candidate()
    def test_stale_after_confirmation_cannot_dispatch(self):
        self.confirm();self.now+=500_000_001
        with self.assertRaisesRegex(ValueError,'stale_before'):self.planner.consume_live_candidate()
    def test_unsettled_capture_cannot_dispatch(self):
        self.confirm();self.body['encoder_history']['capture_settling']['settled_interval']=False
        with self.assertRaisesRegex(ValueError,'capture_not_settled'):self.planner.consume_live_candidate()


if __name__=='__main__':unittest.main(verbosity=2)
