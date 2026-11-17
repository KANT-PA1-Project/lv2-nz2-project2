import unittest
import numpy as np
from test_capture_bundle_identity import CaptureBundleTest
from arm_practice.capture_bundle_identity import CaptureSessionLedger

class LedgerTest(unittest.TestCase):
 def setUp(self):
  fixture=CaptureBundleTest();fixture.setUp();self.d=fixture.d;self.p=fixture.p;self.ledger=CaptureSessionLedger('run-a','g1')
 def check(self,generation='g1'):
  return self.ledger.inspect(self.d,self.p,generation=generation)
 def advance(self,frame=3,capture=124):
  self.d['frame_index']=np.int64(frame);self.d['stamp_ns']=np.int64(1000000000+frame*50000000);self.d['capture_monotonic_ns']=np.int64(capture);self.p.update(stamp_ns=int(self.d['stamp_ns']),frame_capture_monotonic_ns=capture)
 def test_sequence_with_gap(self):
  self.assertTrue(self.check()['sequence_eligible']);self.advance(6,125);self.assertTrue(self.check()['sequence_eligible'])
 def test_duplicate_latches(self):
  self.check();self.assertFalse(self.check()['sequence_eligible']);self.advance();self.assertEqual(self.check()['reason'],'session_faulted_requires_new_instance')
 def test_regressing_frame(self):
  self.check();self.advance(1,124);self.assertFalse(self.check()['sequence_eligible'])
 def test_regressing_capture(self):
  self.check();self.advance(3,122);self.assertFalse(self.check()['sequence_eligible'])
 def test_new_run_same_stamp(self):
  self.check();self.d['run_id']=np.array('run-b');self.p['capture_run_id']='run-b';self.assertEqual(self.check()['reason'],'capture_run_changed')
 def test_explicit_new_instance_rejects_old_callback(self):
  self.check();self.check();self.ledger=CaptureSessionLedger('run-a','g2');self.assertEqual(self.check()['reason'],'old_callback_generation');self.assertFalse(self.ledger.faulted);self.assertTrue(self.check('g2')['sequence_eligible'])
 def test_invalid_bundle_latches(self):
  del self.d['run_id'];self.assertFalse(self.check()['sequence_eligible']);self.assertTrue(self.ledger.faulted)
