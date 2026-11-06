from pathlib import Path
import numpy as np
from arm_practice.native_rgbd.camera_geometry import transforms,wrist_transform
ROOT=Path(__file__).resolve().parents[1]

def test_requested_camera_matches_full_rig_at_distinct_joint_states():
    for q in ([0]*5,[-.3,.15,-.3,.15,0],[.2,-.1,.3,-.2,.1]):
        joints={f'arm_joint{i+1}':v for i,v in enumerate(q)}
        np.testing.assert_array_equal(wrist_transform(ROOT,joints),transforms(ROOT,joints)['arm_camera_left_optical'])
