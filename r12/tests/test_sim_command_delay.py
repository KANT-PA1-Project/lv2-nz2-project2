import unittest
from arm_practice.sim_command_delay import SimCommandDelay
class DelayTests(unittest.TestCase):
 def queue(self,**kw):return SimCommandDelay('run',delay_ns=kw.get('delay',100),max_age_ns=kw.get('age',300),capacity=kw.get('capacity',4))
 def put(self,q,seq=0,capture=0,now=0,v=(.1,0,0),run='run'):return q.submit(run_id=run,sequence=seq,capture_ns=capture,now_ns=now,velocity=v)
 def test_physics_time_release_and_original_capture(self):
  q=self.queue();self.assertTrue(self.put(q));self.assertEqual(q.sample(99)['velocity'],(0,0,0));x=q.sample(100);self.assertEqual(x['velocity'],(.1,0,0));self.assertEqual(x['capture_ns'],0);self.assertEqual(x['age_ns'],100)
 def test_expiry_boundary_and_no_refresh(self):
  q=self.queue();self.put(q);self.assertEqual(q.sample(300)['reason'],'delayed_command');self.assertEqual(q.sample(301)['reason'],'expired_at_application');self.assertEqual(q.sample(302)['velocity'],(0,0,0))
 def test_delay_larger_than_age(self):
  q=self.queue(delay=400);self.put(q);self.assertEqual(q.sample(400)['reason'],'expired_at_application')
 def test_latest_due_wins_and_no_old_replay(self):
  q=self.queue();self.put(q);self.put(q,1,20,20,(.2,0,0));self.assertEqual(q.sample(120)['sequence'],1);self.assertEqual(q.sample(150)['sequence'],1)
 def test_stop_discards_pending_and_active(self):
  q=self.queue();self.put(q);q.sample(100);self.put(q,1,100,100);self.assertEqual(q.sample(101,safety_stop=True)['velocity'],(0,0,0));self.assertEqual(q.sample(201)['reason'],'await_command');self.put(q,2,202,202);self.assertEqual(q.sample(302)['sequence'],2)
 def test_invalid_run_latches_stop(self):
  q=self.queue();self.put(q);self.assertFalse(self.put(q,1,0,0,run='other'));self.assertEqual(q.sample(100)['reason'],'wrong_run');self.assertFalse(self.put(q,2,100,100))
 def test_duplicate_and_reversed_capture(self):
  for seq,cap in [(0,10),(1,9)]:
   q=self.queue();self.put(q,0,10,10);self.assertFalse(self.put(q,seq,cap,10));self.assertEqual(q.sample(110)['velocity'],(0,0,0))
 def test_clock_reversal_latches(self):
  q=self.queue();self.put(q);q.sample(100);self.assertEqual(q.sample(99)['reason'],'invalid_or_reversed_clock');self.assertEqual(q.sample(200)['velocity'],(0,0,0))
 def test_overflow_stops(self):
  q=self.queue(capacity=1);self.put(q);self.assertFalse(self.put(q,1,1,1));self.assertEqual(q.sample(101)['reason'],'queue_overflow')
 def test_bad_inputs(self):
  for cap,now,v in [(1,0,(0,0,0)),(0,301,(0,0,0)),(0,0,(float('nan'),0,0)),(0,0,(0,0))]:
   q=self.queue();self.assertFalse(self.put(q,capture=cap,now=now,v=v));self.assertEqual(q.sample(400)['velocity'],(0,0,0))
 def test_input_copy_and_zero_delay(self):
  q=self.queue(delay=0);v=[.1,0,0];self.put(q,v=v);v[0]=99;self.assertEqual(q.sample(0)['velocity'],(.1,0,0))
if __name__=='__main__':unittest.main()
