import numpy as np
from arm_practice.native_rgbd.puck_fixture import setup_pose
def test_fixture_projects_at_optical_center_without_sensor_claim():
    base=np.eye(4);base[:3,3]=[.7,.9,.008];cam=np.eye(4);cam[:3,3]=[.3,.02,.56]
    p=setup_pose(base,cam);back=np.linalg.inv(base@cam)@np.r_[p,1]
    assert np.allclose(back,[0,0,.25,1])
