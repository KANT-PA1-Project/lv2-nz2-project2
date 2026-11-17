import pytest
from arm_practice.native_rgbd.pick_sensor_adapter import require_pose_confidence

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-float('inf'),True,False,None,'0.9',.2499,1.0001])
def test_invalid_confidence_cannot_admit_pick(value):
    with pytest.raises(ValueError,match='pose_confidence_unverified'):
        require_pose_confidence({'confidence':value})

@pytest.mark.parametrize('value',[.25,.9,1])
def test_finite_probability_at_threshold_admitted(value):
    assert require_pose_confidence({'confidence':value}) == value

@pytest.mark.parametrize('value',[float('nan'),float('inf'),True,'0.9',.2499,1.0001])
def test_native_observe_rejects_score_before_geometry_admission(value):
    # Reuse the complete timestamp/FK packet fixture, but exercise the native
    # adapter explicitly; the legacy script adapter is a separate implementation.
    from test_pick_sensor_adapter import Adapter, root
    from arm_practice.native_rgbd.pick_sensor_adapter import PickSensorAdapter
    fixture=Adapter()
    fixture.setUp()
    try:
        fixture.a=PickSensorAdapter(root,pose_error_budget=dict(
            validated=True,scope='simulation_sensor_pose',report_sha256='0'*64,
            translation_bound_m=.001,rotation_bound_rad=.02))
        fixture.packet['perception']['detections'][0]['pose']['confidence']=value
        assert fixture.observe() is None
        assert fixture.a.reason=='pose_confidence_unverified'
    finally:
        fixture.doCleanups()
