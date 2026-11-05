import pytest
from arm_practice.native_rgbd.pick_sensor_adapter import pose_error_bounds

def budget():
 return dict(validated=True,scope='simulation_sensor_pose',report_sha256='0'*64,translation_bound_m=.001,rotation_bound_rad=.02)

def test_position_only_evidence_cannot_admit_pick():
 b=budget();del b['rotation_bound_rad']
 with pytest.raises(ValueError,match='rotation_bound_rad'):pose_error_bounds(b)

@pytest.mark.parametrize('value',[True,-.001,.10001,float('nan'),float('inf'),None])
def test_invalid_orientation_bound_rejected(value):
 b=budget();b['rotation_bound_rad']=value
 with pytest.raises(ValueError,match='rotation_bound_rad'):pose_error_bounds(b)

def test_both_validated_bounds_returned():
 assert pose_error_bounds(budget())==(.001,.02)

def test_development_measurements_not_validation():
 b=budget();b['validated']=False
 with pytest.raises(ValueError,match='missing'):pose_error_bounds(b)
