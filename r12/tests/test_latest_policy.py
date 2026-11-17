import unittest,tempfile,json
from pathlib import Path
import numpy as np
import torch
from arm_practice.latest_policy import LatestPolicy,publish_completed,allocate,dc_effort
from arm_practice.swerve_highspeed_v2_rl import Policy
class LatestTests(unittest.TestCase):
 def test_atomic_version_and_stationary_swap(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);p=root/'checkpoint.pt';torch.save({'model':Policy().state_dict(),'iteration':3},p)
   with self.assertRaises(ValueError):publish_completed(p,root/'published',4)
   first=publish_completed(p,root/'published',3);reader=LatestPolicy(root/'published');reader.refresh(True);self.assertEqual(reader.version,first['sha256'])
   model=Policy();torch.save({'model':model.state_dict(),'iteration':4},p);second=publish_completed(p,root/'published',4);reader.refresh(False);self.assertEqual(reader.version,first['sha256']);reader.refresh(True);self.assertEqual(reader.version,second['sha256'])
   (root/'published'/second['file']).write_bytes(b'corrupt');reader.version=None;reader.refresh(True);self.assertIn('checksum',reader.error)
 def test_zero_command_and_motor_limit(self):
  a,w=allocate([0,0,0],np.zeros(4),None,np.ones(4));np.testing.assert_array_equal(w,0)
  a,w=allocate([1,1,2],np.zeros(4),None,np.ones(4));self.assertLessEqual(np.max(np.abs(w)),29.321531434)
  np.testing.assert_allclose(dc_effort(np.ones(4)*100,0,np.ones(4)*29.321531434),0,atol=1e-9)
  self.assertLessEqual(np.max(np.abs(dc_effort(np.ones(4)*100,0,np.zeros(4)))),.108)
