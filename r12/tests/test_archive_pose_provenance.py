import json,tempfile,unittest
from pathlib import Path
import numpy as np
from arm_practice.observed_frame_archive import archive_observed_frame
class TestArchiveProvenance(unittest.TestCase):
 def test_explicit_and_missing_provenance(self):
  for provenance in (None,dict(schema='r12_raw_visual_pose_v1',field_representation='received_unadapted',source_session_verified=False)):
   with tempfile.TemporaryDirectory() as d:
    p=Path(d);np.savez(p/'frame.npz',frame_index=0,stamp_ns=1000000000,clock_domain='synthetic_packet_20hz',run_id='test')
    packet=dict(stamp_ns=1000000000,arm_feedback_stamp_ns=1000000000,arm_feedback_valid=True,capture_run_id='test',arm_measured_position=[0.]*5,visual_pose={'covariance':[0.]*36})
    if provenance is not None:packet['visual_pose_provenance']=provenance
    archive_observed_frame(p/'frame.npz',p/'archive',packet,0)
    saved=json.loads((p/'archive/000000.json').read_text());self.assertEqual(saved['visual_pose_provenance'],provenance);self.assertEqual(saved['visual_pose'],packet['visual_pose'])
