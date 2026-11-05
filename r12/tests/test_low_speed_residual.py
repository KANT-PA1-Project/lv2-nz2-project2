import unittest
import numpy as np

from arm_practice.latest_policy import allocate, dc_effort
from arm_practice.low_speed_residual import limit_low_speed_residual


class LowSpeedResidualTests(unittest.TestCase):
    def test_low_speed_targets_keep_nominal_direction_and_bound(self):
        rng = np.random.default_rng(412)
        for _ in range(300):
            command = np.r_[rng.uniform(-.015, .015, 2), rng.uniform(-.08, .08)]
            current = rng.uniform(-np.pi/2, np.pi/2, 4)
            action = rng.uniform(-1, 1, 4)
            target0, wheels0 = allocate(command, current, current, np.zeros(4))
            target, wheels = allocate(command, current, current, action, low_speed_residual_fraction=.25)
            np.testing.assert_array_equal(target, target0)
            self.assertTrue(np.all(np.abs(wheels - wheels0) <= .25*np.abs(wheels0) + 1e-12))
            np.testing.assert_array_equal(np.sign(wheels), np.sign(wheels0))

    def test_cruise_and_fast_pure_yaw_retain_legacy_output(self):
        current = np.array([-.8, .3, .9, -1.1])
        action = np.array([-.9, .8, -.7, .6])
        for command in ([.12, 0, 0], [0, -.12, .03], [0, 0, 1.]):
            old = allocate(command, current, current, action)
            new = allocate(command, current, current, action, low_speed_residual_fraction=.25)
            for a, b in zip(old, new):
                np.testing.assert_array_equal(a, b)

    def test_stop_has_no_residual_target_or_steering_motion(self):
        current = np.array([-.8, .3, .9, -1.1])
        target, wheels = allocate([0, 0, 0], current, current, np.ones(4), low_speed_residual_fraction=.25)
        np.testing.assert_array_equal(target, current)
        np.testing.assert_array_equal(wheels, np.zeros(4))

    def test_zero_nominal_wheel_receives_no_low_speed_correction(self):
        nominal = np.array([0., 1., -1., .2])
        correction = limit_low_speed_residual(nominal, np.array([6., -6., 6., -6.]), .03, .25)
        self.assertEqual(correction[0], 0)
        np.testing.assert_array_equal(np.sign(nominal + correction), np.sign(nominal))

    def test_blend_is_continuous_and_within_endpoints(self):
        nominal = np.array([.2, -.5, .8, 0.])
        residual = np.array([-6., 6., -6., 6.])
        for boundary in (.04, .08):
            before = limit_low_speed_residual(nominal, residual, boundary-1e-8, .25)
            after = limit_low_speed_residual(nominal, residual, boundary+1e-8, .25)
            np.testing.assert_allclose(before, after, atol=1e-10, rtol=0)
        bounded = limit_low_speed_residual(nominal, residual, .04, .25)
        for speed in np.linspace(.04, .08, 20):
            correction = limit_low_speed_residual(nominal, residual, speed, .25)
            self.assertTrue(np.all(correction >= np.minimum(bounded, residual)-1e-12))
            self.assertTrue(np.all(correction <= np.maximum(bounded, residual)+1e-12))

    def test_invalid_configuration_rejected(self):
        for fraction in (-.1, 1., float('nan')):
            with self.assertRaises(ValueError):
                limit_low_speed_residual(np.ones(4), np.ones(4), .02, fraction)

    def test_motor_limits_still_apply_to_limited_targets(self):
        _, target = allocate([.02, -.01, .02], np.zeros(4), None, np.ones(4), low_speed_residual_fraction=.25)
        for velocity in (-29.321531434, 0., 29.321531434):
            torque = dc_effort(target, np.zeros(4), np.full(4, velocity), wheel_feedforward=.04)
            self.assertTrue(np.all(np.abs(torque) <= .108 + 1e-12))
            if velocity > 0:
                self.assertTrue(np.all(torque <= 1e-12))
            if velocity < 0:
                self.assertTrue(np.all(torque >= -1e-12))


if __name__ == '__main__':
    unittest.main()
