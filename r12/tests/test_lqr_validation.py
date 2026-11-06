import numpy as np
import pytest
from arm_practice.lqr_validation import affine_rollout_errors


def test_exact_affine_plant_with_nonzero_offset():
    # x'=2x+3u+1, analytically computed independent samples.
    result = affine_rollout_errors([[2]], [[3]], [1], [[0], [4], [6]],
                                  [[1], [-1]], [0, 8, 16], stride=8)
    assert result['rollout_rmse'] == [0]
    assert result['one_step_rmse'] == [0]
    assert not result['motion_authorized']


def test_small_one_step_bias_accumulates_without_measurement_reset():
    result = affine_rollout_errors([[1]], [[1]], [.01], np.zeros((101, 1)),
                                  np.zeros((100, 1)), list(range(101)), stride=1)
    assert result['one_step_rmse'][0] == pytest.approx(.01)
    assert result['terminal_error'][0] == pytest.approx(1)
    assert result['rollout_rmse'][0] > .5


@pytest.mark.parametrize('indices', [[0, 8, 24], [0, 8, 0], [0, True, 16]])
def test_gaps_resets_and_bool_ticks_rejected(indices):
    with pytest.raises(ValueError):
        affine_rollout_errors([[1]], [[1]], [0], [[0], [0], [0]],
                              [[0], [0]], indices, stride=8)


def test_finite_inputs_that_overflow_prediction_rejected():
    with pytest.raises(ValueError, match='nonfinite model rollout'):
        affine_rollout_errors([[1e308]], [[1]], [0], [[1e308], [0]],
                              [[0]], [0, 8], stride=8)


def test_finite_prediction_that_overflows_error_score_rejected():
    with pytest.raises(ValueError, match='nonfinite model rollout'):
        affine_rollout_errors([[1]], [[1]], [0], [[1e200], [0]],
                              [[0]], [0, 8], stride=8)
