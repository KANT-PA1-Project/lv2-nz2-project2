import numpy as np
import pytest
from arm_practice.native_rgbd.observe_setup import preserve_arm_reference

def test_preserves_gravity_support_error_without_modifying_inputs():
 measured=np.array([4.,-.3,-.02,-.018,-.006,0.,7.]);setup=np.array([0.,-.3,0.,0.,0.,0.,0.]);old=measured.copy()
 result=preserve_arm_reference(measured,setup,[1,2,3,4,5])
 assert np.array_equal(result,[4.,-.3,0.,0.,0.,0.,7.]);assert np.array_equal(measured,old)
 assert 80*(result[2]-measured[2])==pytest.approx(1.6)
@pytest.mark.parametrize('indices',[[1,2,3,4,4],[1,2,3,4,8],[1,2,3,4,True]])
def test_invalid_mapping(indices):
 with pytest.raises(ValueError):preserve_arm_reference(np.zeros(7),np.zeros(7),indices)
def test_nonfinite():
 with pytest.raises(ValueError):preserve_arm_reference(np.zeros(5),[0,0,float('nan'),0,0],[0,1,2,3,4])
