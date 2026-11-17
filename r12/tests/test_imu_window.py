import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from imu_window import reduce_window

class ImuWindowTests(unittest.TestCase):
 def window(self):return [{'time':i/120,'valid':True,'gyro':[0,(-1)**i*.8,0],'accel':[0,0,9.81]} for i in range(6)]
 def test_alternating_vibration_has_zero_integrated_rotation(self):
  r=reduce_window(self.window());np.testing.assert_allclose(r['gyro'],0);self.assertEqual(r['native_sample_count'],6)
 def test_constant_turn_preserved(self):
  s=self.window()
  for x in s:x['gyro']=[.1,.2,.3]
  np.testing.assert_allclose(reduce_window(s)['gyro'],[.1,.2,.3])
 def test_missing_duplicate_invalid_fail_closed(self):
  s=self.window()
  with self.assertRaises(ValueError):reduce_window(s[:-1])
  s[2]['time']=s[1]['time']
  with self.assertRaises(ValueError):reduce_window(s)
  s=self.window();s[2]['gyro'][0]=float('nan')
  with self.assertRaises(ValueError):reduce_window(s)

 def test_float32_native_timestamps_after_long_run(self):
  s=self.window()
  for i,x in enumerate(s):x['time']=float(np.float32(1800+i/120))
  self.assertTrue(reduce_window(s)['valid'])
  s[3]['time']=s[2]['time']
  with self.assertRaises(ValueError):reduce_window(s)
