import math
import numpy as np
import pytest
from arm_practice.pose_displacement_bound import rigid_surface_displacement,remaining_external_clearance

def test_rotation_extremal_point_attains_chord_bound():
 r=.03354101966249685
 for theta in (0,.01,.1,math.pi):
  actual=np.linalg.norm(np.array([r*math.cos(theta)-r,r*math.sin(theta),0]))
  assert rigid_surface_displacement(0,theta,r)==pytest.approx(actual)

def test_nominal_three_cm_is_not_robust_three_cm():
 assert remaining_external_clearance(.03219,.005,0,.03355)<.03
 assert remaining_external_clearance(.033118,0,math.radians(.5),.03355)>.03

@pytest.mark.parametrize('bad',[-1,True,float('nan'),float('inf')])
def test_invalid_error_bound_rejected(bad):
 for args in ((bad,0,1),(0,bad,1),(0,0,bad)):
  with pytest.raises(ValueError):rigid_surface_displacement(*args)

def test_nonprincipal_rotation_rejected():
 with pytest.raises(ValueError):rigid_surface_displacement(0,4,1)
