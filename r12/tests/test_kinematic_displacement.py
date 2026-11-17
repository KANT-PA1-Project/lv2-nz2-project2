import unittest
import numpy as np
from arm_practice.kinematic_displacement import collider_joint_radii, point_displacement_bound


def transform(xyz):
    value = np.eye(4)
    value[:3, 3] = xyz
    return value


class KinematicDisplacementTests(unittest.TestCase):
    def test_single_joint_chord_bounded_for_small_and_large_angles(self):
        joints = [('j1', 'base_link', 'link', np.eye(4), np.eye(4), 'Z', False)]
        radii = collider_joint_radii(joints, 'link', transform([1, 0, 0]), [.2, .2, .2], ['j1'], {})
        for angle in [.0001, .5, np.pi, 7.]:
            point = np.array([1.1, .1, .1])
            c, s = np.cos(angle), np.sin(angle)
            rotated = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ point
            self.assertLessEqual(np.linalg.norm(rotated-point), point_displacement_bound(radii, [angle]))

    def test_child_frames_and_fixed_gripper_translation_included(self):
        joints = [('j1', 'base_link', 'a', transform([0, 0, .3]), transform([.1, 0, 0]), 'X', False),
                  ('j2', 'a', 'b', transform([0, .2, 0]), transform([0, 0, .05]), 'Y', False),
                  ('finger', 'b', 'tip', np.eye(4), np.eye(4), 'Y', True)]
        radius = collider_joint_radii(joints, 'tip', np.eye(4), [.02, .02, .02], ['j1', 'j2'], {'finger': -.03})
        self.assertAlmostEqual(radius[1], np.sqrt(3)*.01+.03+.05)
        self.assertAlmostEqual(radius[0], radius[1]+.2+.1)
        self.assertAlmostEqual(point_displacement_bound(radius, [.005, .003]), radius[0]*.005+radius[1]*.003)

    def test_non_simultaneous_motion_requires_full_joint_excursions(self):
        # One joint can finish before the other starts: nearest-endpoint half-step
        # is not assumed, while the full excursion sum remains conservative.
        lengths = [.5, .3]
        radii = [.8, .3]
        q0, q1 = np.array([0., 0.]), np.array([.2, -.3])
        def position(q):
            return lengths[0]*np.array([np.cos(q[0]), np.sin(q[0])]) + lengths[1]*np.array([np.cos(q.sum()), np.sin(q.sum())])
        bound = point_displacement_bound(radii, np.abs(q1-q0))
        for t in np.linspace(0, 1, 101):
            q = q0+(q1-q0)*np.clip([2*t, 2*t-1], 0, 1)
            self.assertLessEqual(np.linalg.norm(position(q)-position(q0)), bound+1e-12)

    def test_fixed_base_has_zero_joint_motion_bound(self):
        radius = collider_joint_radii([], 'base_link', np.eye(4), [1, 1, 1], ['j1'], {})
        self.assertEqual(point_displacement_bound(radius, [1]), 0)

    def test_missing_joint_nonrigid_transform_and_cycle_rejected(self):
        joint = ('j1', 'base_link', 'a', np.eye(4), np.eye(4), 'X', False)
        with self.assertRaises(ValueError):
            collider_joint_radii([joint], 'a', np.eye(4), [1, 1, 1], ['other'], {})
        scaled = np.diag([2, 1, 1, 1])
        with self.assertRaises(ValueError):
            collider_joint_radii([joint], 'a', scaled, [1, 1, 1], ['j1'], {})
        cycle = ('j1', 'a', 'a', np.eye(4), np.eye(4), 'X', False)
        with self.assertRaises(ValueError):
            collider_joint_radii([cycle], 'a', np.eye(4), [1, 1, 1], ['j1'], {})

    def test_invalid_excursions_and_unfixed_prismatic_rejected(self):
        for value in [[-.1], [float('nan')], [1, 2]]:
            with self.assertRaises(ValueError):
                point_displacement_bound([1], value)
        joint = ('finger', 'base_link', 'tip', np.eye(4), np.eye(4), 'X', True)
        with self.assertRaises(ValueError):
            collider_joint_radii([joint], 'tip', np.eye(4), [1, 1, 1], ['j1'], {})


if __name__ == '__main__':
    unittest.main()
