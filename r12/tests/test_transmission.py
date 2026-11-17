import unittest
import numpy as np
from arm_practice.transmission import *
class TransmissionTests(unittest.TestCase):
    def test_position_velocity_and_power(self):
        self.assertAlmostEqual(float(motor_to_wheel(2*np.pi)),8*np.pi)
        np.testing.assert_allclose(wheel_to_motor(motor_to_wheel([-2,0,3])),[-2,0,3])
        torque=float(motor_torque_to_wheel(2.7,.8))
        self.assertAlmostEqual(torque*float(motor_to_wheel(5)),2.7*5*.8)
    def test_nominal_speed_and_register(self):
        speed=float(motor_to_wheel(70*2*np.pi/60))*WHEEL_RADIUS_M
        self.assertAlmostEqual(speed,.8997956670006765)
        self.assertEqual(int(wheel_velocity_to_register(100)),305)
        self.assertEqual(int(wheel_velocity_to_register(-100)),-305)
        self.assertLessEqual(abs(float(register_to_wheel_velocity(-305))),float(motor_to_wheel(70*2*np.pi/60)))
    def test_invalid_data(self):
        with self.assertRaises(ValueError):wheel_velocity_to_register(float('nan'))
        with self.assertRaises(ValueError):motor_torque_to_wheel(1,1.1)
