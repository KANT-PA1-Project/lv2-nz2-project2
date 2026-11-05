import unittest
import numpy as np
from arm_practice.motion_envelope import MotionEnvelope

class EnvelopeTests(unittest.TestCase):
    def test_all_directions_reverse_and_stop(self):
        for direction in ([1,0,0],[0,1,0],[1,1,0],[0,0,1],[1,1,1]):
            e=MotionEnvelope();previous=np.zeros(3)
            for target in [np.array(direction)*.2,-np.array(direction)*.2,np.zeros(3)]:
                for _ in range(150):
                    v,_=e.step(target,[0,0,0],[0,0,9.81],.05)
                    self.assertLessEqual(np.linalg.norm(v[:2]),.120001)
                    self.assertLessEqual(abs(v[2]),.200001)
                    self.assertLessEqual(np.linalg.norm((v-previous)[:2])/.05,.200001)
                    self.assertLessEqual(abs(v[2]-previous[2])/.05,.400001)
                    previous=v
            np.testing.assert_allclose(previous,0,atol=1e-10)
    def test_fault_stops_immediately(self):
        e=MotionEnvelope()
        for _ in range(20):e.step([.12,0,0],[0,0,0],[0,0,9.81],.05)
        v,state=e.step([.12,0,0],[float('nan'),0,0],[0,0,9.81],.05)
        np.testing.assert_array_equal(v,0);self.assertIn('invalid',state['reason'])
    def test_tilt_latches(self):
        e=MotionEnvelope();a=np.deg2rad(10)
        v,state=e.step([.12,0,0],[0,0,0],[0,9.81*np.sin(a),9.81*np.cos(a)],.05)
        self.assertTrue(state['latched']);np.testing.assert_array_equal(v,0)
        v,state=e.step([.12,0,0],[0,0,0],[0,0,9.81],.05)
        self.assertTrue(state['latched']);np.testing.assert_array_equal(v,0)

    def test_continuous_direction_changes_respect_vector_speed(self):
        e=MotionEnvelope()
        for angle in np.linspace(0,20*np.pi,2000):
            v,_=e.step([.2*np.cos(angle),.2*np.sin(angle),.2],[0,0,0],[0,0,9.81],.05)
            self.assertLessEqual(np.linalg.norm(v[:2]),.120001)

class WheelSlewTests(unittest.TestCase):
    def test_ppo_jumps_and_reversal(self):
        from arm_practice.motion_envelope import WheelSlew
        e=WheelSlew();last=np.zeros(4)
        for desired in ([10]*4,[-10]*4,[0]*4):
            for _ in range(60):
                v=e.step(desired)
                self.assertLessEqual(np.max(np.abs(v-last)),e.acceleration*.05+1e-9);last=v
        np.testing.assert_allclose(last,0)
        e.step([10]*4);np.testing.assert_array_equal(e.step([10]*4,interlock=True),0)

class VisualAttitudeTests(unittest.TestCase):
    def test_stereo_bounds_sustained_gyro_drift(self):
        e=MotionEnvelope(speed=.9)
        for _ in range(200):
            _,state=e.step([.8,0,0],[0,.10,0],[0,0,9.81],.05,visual_tilt=[0,0])
            self.assertFalse(state['latched'])
            self.assertLess(abs(state['tilt_deg'][1]),2)
    def test_real_visual_tilt_latches_without_smoothing_delay(self):
        e=MotionEnvelope()
        _,state=e.step([.1,0,0],[0,0,0],[0,0,9.81],.05,visual_tilt=[0,np.deg2rad(9)])
        self.assertTrue(state['latched'])
    def test_disagreement_and_invalid_camera_stop(self):
        for visual in ([0,np.deg2rad(6)],[float('nan'),0]):
            e=MotionEnvelope()
            v,state=e.step([.1,0,0],[0,0,0],[0,0,9.81],.05,visual_tilt=visual)
            np.testing.assert_array_equal(v,0)
            self.assertIn(state['reason'],['attitude_disagreement','invalid_visual_attitude'])
