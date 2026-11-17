import copy
import unittest
from arm_practice.wrist_perception_request import wrist_schedule_feedback
from arm_practice.perception_schedule import select_perception_work


class Tests(unittest.TestCase):
    def fixture(self):
        return dict(stamp_ns=1_000_000_000, frame_capture_monotonic_ns=20_000_000_000,
            capture_run_id='test', arm_measured_velocity=[0.]*5, arm_feedback_valid=True,
            arm_feedback_stamp_ns=1_000_000_000, base_stopped=True, command=[0.]*3,
            observation_state='observing', observation_sensor_checks={'fresh':True},
            stationary_wheel_hold_enabled=True, base_sensor_feedback=dict(run_id='test',
                stamp_ns=1_000_000_000, capture_monotonic_ns=20_000_000_000,
                valid=True, twist=[0.]*3))

    def work(self,response,**kw):
        args=dict(run_id='test',stamp_ns=1_050_000_000,now_ns=20_300_000_000);args.update(kw)
        return select_perception_work(1,**wrist_schedule_feedback(response,**args))

    def test_ready_response_schedules_wrist_frame(self):
        self.assertEqual(self.work(self.fixture())['mode'],'wrist_confirmation')

    def test_stale_run_or_missing_frame_keeps_survey(self):
        for kw in [dict(run_id='different'),dict(stamp_ns=1_100_000_000),
                   dict(now_ns=20_500_000_001),dict(now_ns=19_999_999_999)]:
            self.assertEqual(self.work(self.fixture(),**kw)['mode'],'survey')

    def test_feedback_gates_and_pose_phase_cannot_be_bypassed(self):
        for key,value in [('arm_measured_velocity',[.03]*5),('command',[.01,0,0]),
                          ('base_stopped',False),('observation_state','returning'),
                          ('arm_feedback_stamp_ns',999),('stationary_wheel_hold_enabled',False)]:
            r=self.fixture();r[key]=value;self.assertEqual(self.work(r)['mode'],'survey')
        for key,value in [('valid',False),('run_id','other'),('twist',[0,0,.03])]:
            r=self.fixture();r['base_sensor_feedback'][key]=value
            self.assertEqual(self.work(r)['mode'],'survey')

    def test_missing_and_malformed_payloads_are_rejected(self):
        for response in [{},None,{'stamp_ns':'bad'}]:
            self.assertEqual(self.work(response)['mode'],'survey')
        r=self.fixture();r['arm_measured_velocity']=[float('nan')]*5
        self.assertEqual(self.work(r)['mode'],'survey')

    def test_input_unchanged(self):
        r=self.fixture();prior=copy.deepcopy(r);self.work(r);self.assertEqual(r,prior)


if __name__=='__main__':unittest.main()
