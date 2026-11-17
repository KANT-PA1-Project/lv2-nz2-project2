import unittest
from arm_practice.perception_response_identity import check_packet_perception

class PacketTests(unittest.TestCase):
    def setUp(self):
        self.packet=dict(stamp_ns=1000000000,capture_run_id='a',frame_capture_monotonic_ns=100,
            perception=dict(run_id='a',clock_domain='synthetic_packet_20hz',camera='r07_camera1',
            frame_index=0,stamp_ns=1000000000,capture_monotonic_ns=100,
            inference_started_monotonic_ns=110,inference_completed_monotonic_ns=120))
    def check(self, now=130):
        return check_packet_perception(self.packet,now_ns=now,max_age_ns=30)['matched']
    def test_current_packet(self):self.assertTrue(self.check())
    def test_transit_expiry(self):self.assertFalse(self.check(131))
    def test_sender_verdict_not_trusted(self):
        self.packet['perception']['provenance_check']={'matched':True}
        self.assertFalse(self.check(131))
    def test_old_frame_rejected(self):
        self.packet['stamp_ns']+=50000000
        self.assertFalse(self.check())
    def test_wrong_run_capture_camera(self):
        for key,value in [('run_id','b'),('capture_monotonic_ns',99),('camera','arm_camera')]:
            self.setUp();self.packet['perception'][key]=value
            self.assertFalse(self.check())
    def test_invalid_packet(self):
        for stamp in [True,1000000000.0,1000000001,-1]:
            self.packet['stamp_ns']=stamp
            self.assertFalse(self.check())
    def test_missing_identity(self):
        del self.packet['capture_run_id']
        self.assertFalse(self.check())
