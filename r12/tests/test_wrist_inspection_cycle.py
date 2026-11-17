"""Synthetic command/feedback tests; not physical clearance or ID validation."""
import unittest
import numpy as np
from arm_practice.wrist_inspection_cycle import WristInspectionCycle
from arm_practice.wrist_identity import WristIdentity


def candidate():
    return dict(id=2, color='R', shape='cylinder', center_map_m=[1., 2., .1],
                depth_surface_m=.3, depth_consistent=True,
                pose_camera=dict(center_camera_m=[0., 0., .3],
                                 translation_valid=True, confidence=.9))


class Tests(unittest.TestCase):
    def make(self, checker=lambda q, target, direction: True):
        return WristInspectionCycle([.15, -.10, 0, 0, 0], [-1]*5, [1]*5,
                                    clearance_check=checker, exposure_seconds=.6)

    def step(self, cycle, i, q, **kw):
        args=dict(velocity=[0.]*5, pose=[0.]*3, stopped=True, valid=True, run_id='test')
        args.update(kw)
        return cycle.step(i*.05, q, **args)

    def simulate(self, cycle, *, identity=False):
        q=np.zeros(5); gate=WristIdentity(); states=[]; commands=[]
        for i in range(300):
            evidence=[]
            if identity and cycle.view_ready:
                result=gate.observe(candidate(), camera='arm_camera',
                    camera_frame='arm_camera_left_optical', run_id='test',
                    stamp_ns=i*50_000_000, fresh=True, view_ready=True,
                    stationary=True, arm_velocity=[0.]*5)
                if result is not None:evidence=[result]
            target=self.step(cycle, i, q, confirmations=evidence)
            states.append(cycle.state)
            if target is not None:
                self.assertLessEqual(float(np.max(np.abs(target-q))), .005000001)
                q=target; commands.append(target.copy())
            if cycle.done:break
        return q, states, commands

    def test_full_cycle_needs_identity_and_measured_fold_settle(self):
        c=self.make();q,states,commands=self.simulate(c,identity=True)
        self.assertEqual(c.state,'complete');self.assertTrue(c.inspection_passed)
        self.assertEqual(c.status()['confirmed_ids'],[2]);np.testing.assert_allclose(q,0,atol=1e-9)
        self.assertIn('moving',states);self.assertIn('observing',states);self.assertIn('returning',states)
        self.assertFalse(c.status()['grasp_started'])

    def test_no_identity_still_refolds_without_passing_inspection(self):
        c=self.make();q,_,_=self.simulate(c)
        self.assertEqual(c.state,'complete');self.assertFalse(c.inspection_passed)
        self.assertEqual(c.reason,'refolded_without_identity');np.testing.assert_allclose(q,0,atol=1e-9)

    def test_missing_clearance_never_issues_any_arm_target(self):
        for checker in (None,lambda *a:False,lambda *a:{'safe':True}):
            c=self.make(checker);q,_,commands=self.simulate(c)
            self.assertEqual(commands,[]);self.assertEqual(c.reason,'clearance_unavailable')

    def test_exception_and_lost_clearance_stop_commands(self):
        for raises in (False,True):
            def check(q,target,direction):
                if target[0]>.03:
                    if raises:raise RuntimeError('depth unavailable')
                    return False
                return True
            c=self.make(check);q,_,_=self.simulate(c)
            self.assertEqual(c.reason,'clearance_lost');self.assertLessEqual(q[0],.03+1e-9)

    def test_return_requires_its_own_clearance(self):
        directions=[]
        def check(q,target,direction):
            directions.append(direction);return direction!='return'
        c=self.make(check);q,_,_=self.simulate(c,identity=True)
        self.assertEqual(c.reason,'clearance_lost');self.assertIn('return',directions)
        self.assertFalse(c.inspection_passed)

    def test_motion_sensor_and_pose_faults_fail_after_start(self):
        for kwargs,reason in [(dict(stopped=False),'base_moved'),
                              (dict(valid=False),'sensor_invalid'),
                              (dict(pose=[.02,0,0]),'base_pose_drift'),
                              (dict(run_id='different'),'capture_run_changed')]:
            c=self.make();q=np.zeros(5)
            for i in range(12):
                t=self.step(c,i,q)
                if t is not None:q=t
            self.assertIsNone(self.step(c,12,q,**kwargs));self.assertEqual(c.reason,reason)

    def test_tracking_error_and_clock_gap(self):
        for fault,reason in [('tracking','arm_tracking_error'),('clock','clock_gap')]:
            c=self.make();q=np.zeros(5)
            for i in range(12):
                t=self.step(c,i,q)
                if t is not None:q=t
            if fault=='tracking':q[0]+=.03
            self.assertIsNone(self.step(c,20 if fault=='clock' else 12,q))
            self.assertEqual(c.reason,reason)

    def test_stale_or_other_run_identity_does_not_count(self):
        for run,delta in [('different',0),('test',-50_000_000)]:
            c=self.make();q=np.zeros(5)
            for i in range(300):
                proof=dict(id=1,camera='arm_camera',camera_frame='arm_camera_left_optical',
                    evidence_role='wrist_confirmed_identity_with_pose_estimate',
                    wrist_consistent_frames=3,stamp_ns=i*50_000_000+delta,run_id=run)
                target=self.step(c,i,q,confirmations=[proof])
                if target is not None:q=target
                if c.done:break
            self.assertEqual(c.confirmed,{})

    def test_unsettled_arm_cannot_become_observing(self):
        c=self.make();q=np.zeros(5)
        for i in range(100):
            target=self.step(c,i,q,velocity=[.03]*5 if i>10 else [0.]*5)
            if target is not None:q=target
            self.assertFalse(c.view_ready)

    def test_conflicting_evidence_can_revoke_cycle_identity_before_refold(self):
        c=self.make();q=np.zeros(5)
        for i in range(300):
            c.confirmed[2]={'id':2}  # An earlier accepted result is revoked below.
            target=self.step(c,i,q,invalidated_ids=[2])
            if target is not None:q=target
            self.assertEqual(c.confirmed,{})
            if c.done:break
        self.assertFalse(c.inspection_passed)

    def test_invalid_goal_is_rejected(self):
        for goal in ([2]*5,[0]*4,[float('nan')]*5):
            with self.assertRaises(ValueError):WristInspectionCycle(goal,[-1]*5,[1]*5)


if __name__=='__main__':unittest.main()
