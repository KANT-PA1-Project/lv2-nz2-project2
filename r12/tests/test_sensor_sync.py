import unittest
from arm_practice.sensor_sync import wait_visual_pair

class PairTests(unittest.TestCase):
    def run_wait(self, events):
        now=[0.];status={};odom={};calls=[]
        def spin(dt):
            now[0]+=dt;calls.append(dt)
            for when,kind,stamp in events:
                if now[0]>=when:(status if kind=='status' else odom)[stamp]=1
        result=wait_visual_pair(10,status,odom,spin,timeout=.02,clock=lambda:now[0])
        return result,calls
    def test_status_first_waits_for_odom(self):
        r,_=self.run_wait([(.005,'status',10),(.015,'odom',10)])
        self.assertFalse(r['timed_out']);self.assertGreaterEqual(r['wait_seconds'],.015)
    def test_odom_first_waits_for_status(self):
        r,_=self.run_wait([(.005,'odom',10),(.015,'status',10)])
        self.assertFalse(r['timed_out'])
    def test_wrong_stamp_times_out(self):
        r,_=self.run_wait([(.005,'status',9),(.005,'odom',9)])
        self.assertTrue(r['timed_out']);self.assertAlmostEqual(r['wait_seconds'],.02)
    def test_missing_odom_bounded(self):
        r,_=self.run_wait([(.005,'status',10)])
        self.assertTrue(r['status_received']);self.assertFalse(r['odom_received']);self.assertLessEqual(r['wait_seconds'],.02)
