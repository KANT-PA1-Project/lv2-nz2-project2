import numpy as np
import pytest
from arm_practice.native_rgbd.hold_reference import preserve_arm_reference


def test_camera_transition_preserves_gravity_support_without_retargeting_other_joints():
    arm = [2, 4, 5, 7, 8]
    target = np.zeros(10)
    measured = np.arange(10, dtype=float) / 100
    measured[arm] = [-.3, -.02066, -.01811, 0, 0]
    target[arm] = [-.3, 0, 0, 0, 0]
    before = measured.copy()
    result = preserve_arm_reference(measured, target, arm)
    # v105 regression: retargeting measured q2 would erase ~1.65 Nm of Kp80 support.
    assert 80 * (result[4] - measured[4]) == pytest.approx(1.6528)
    other = [0, 1, 3, 6, 9]
    np.testing.assert_array_equal(result[other], measured[other])
    result[4] = 1
    np.testing.assert_array_equal(measured, before)
    assert target[4] == 0


@pytest.mark.parametrize('indices', [[0, 1, 2, 3, 3], [0, 1, 2, 3, 10],
                                      [False, 1, 2, 3, 4], [0, 1, 2, 3]])
def test_ambiguous_mapping_rejected(indices):
    with pytest.raises(ValueError, match='arm mapping'):
        preserve_arm_reference(np.zeros(10), np.zeros(10), indices)


def test_nonfinite_command_rejected():
    with pytest.raises(ValueError, match='hold reference'):
        preserve_arm_reference(np.zeros(5), [0, 0, float('nan'), 0, 0], range(5))
