import unittest
from arm_practice.signal_observation import checked_signal_color

class SignalTests(unittest.TestCase):
    def setUp(self):
        self.p=dict(capture_run_id='a',stamp_ns=1000000000,frame_capture_monotonic_ns=100,
            signal_color='G',signal_observation=dict(run_id='a',stamp_ns=1000000000,
            capture_monotonic_ns=100,camera='r07_camera1',clock_domain='synthetic_packet_20hz',color='OFF',roi_source=dict(run_id='a',stamp_ns=1000000000,capture_monotonic_ns=100,camera='r07_camera1')))
    def check(self, now=130):return checked_signal_color(self.p,now_ns=now,max_age_ns=30)
    def test_boundary_and_untrusted_legacy_color(self):self.assertEqual(self.check(),'OFF')
    def test_expired_or_future(self):
        for now in (99,131):self.assertEqual(self.check(now),'UNKNOWN')
    def test_identity_mismatch(self):
        for key,value in [('run_id','b'),('stamp_ns',1050000000),('capture_monotonic_ns',99),('camera','arm_camera'),('clock_domain','other')]:
            self.setUp();self.p['signal_observation'][key]=value
            self.assertEqual(self.check(),'UNKNOWN')
    def test_invalid_metadata(self):
        for key,value in [('stamp_ns',True),('frame_capture_monotonic_ns',100.0),('capture_run_id','')]:
            self.setUp();self.p[key]=value
            self.assertEqual(self.check(),'UNKNOWN')
    def test_missing_or_unknown(self):
        self.p['signal_observation']['color']='bad';self.assertEqual(self.check(),'UNKNOWN')
        del self.p['signal_observation'];self.assertEqual(self.check(),'UNKNOWN')

    def test_roi_expired_while_current_image_fresh(self):
        self.p['stamp_ns']=1050000000
        self.p['frame_capture_monotonic_ns']=120
        self.p['signal_observation'].update(stamp_ns=1050000000,capture_monotonic_ns=120)
        self.assertEqual(self.check(130),'OFF')
        self.assertEqual(self.check(131),'UNKNOWN')
    def test_bad_roi_source(self):
        for key,value in [('run_id','b'),('camera','arm_camera'),('stamp_ns',1050000000),('capture_monotonic_ns',101),('capture_monotonic_ns',True)]:
            self.setUp();self.p['signal_observation']['roi_source'][key]=value
            self.assertEqual(self.check(),'UNKNOWN')
    def test_missing_roi(self):
        self.p['signal_observation']['roi_source']=None
        self.assertEqual(self.check(),'UNKNOWN')
