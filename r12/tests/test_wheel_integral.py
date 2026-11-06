import unittest
import numpy as np
from arm_practice.wheel_integral import WheelIntegral
from arm_practice.latest_policy import dc_effort


class WheelIntegralTest(unittest.TestCase):
    def test_static_error_builds_bounded_additional_torque(self):
        control = WheelIntegral()
        target = np.array([2., -2., 2., -2.])
        for _ in range(200):
            extra = control.step(target, np.zeros(4), .05, .02)
            self.assertLessEqual(np.max(np.abs(extra)), .04)
        np.testing.assert_allclose(extra, [.04, -.04, .04, -.04])
        np.testing.assert_allclose(dc_effort(target, 0, np.zeros(4), wheel_feedforward=extra),
                                   [.08, -.08, .08, -.08])

    def test_saturated_error_does_not_wind_up(self):
        control = WheelIntegral()
        for _ in range(200):
            extra = control.step([10., -10., 10., -10.], np.zeros(4), .05, .02)
        np.testing.assert_array_equal(extra, np.zeros(4))

    def test_stop_disabled_and_invalid_feedback_reset(self):
        for trigger in ('stop', 'disabled', 'nan', 'bad_dt'):
            control = WheelIntegral()
            control.step(np.ones(4), np.zeros(4), .05, .02)
            target = np.zeros(4) if trigger == 'stop' else np.ones(4)
            measured = np.full(4, np.nan) if trigger == 'nan' else np.zeros(4)
            extra = control.step(target, measured, .2 if trigger == 'bad_dt' else .05,
                                 .02, enabled=trigger != 'disabled')
            np.testing.assert_array_equal(extra, np.zeros(4))

    def test_integral_cannot_override_total_torque_or_back_emf_caps(self):
        velocities = np.array([0., 0., 29.321531434, -29.321531434])
        torque = dc_effort([100., -100., 100., -100.], 0, velocities,
                           wheel_feedforward=[.04, -.04, .04, -.04])
        np.testing.assert_allclose(torque, [.108, -.108, 0., 0.], atol=1e-9)


if __name__ == '__main__':
    unittest.main()
