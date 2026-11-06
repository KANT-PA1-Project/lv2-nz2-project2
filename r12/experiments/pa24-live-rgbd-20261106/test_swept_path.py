"""Geometric boundary tests: endpoint-only checks cannot prove a path."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from audit_swept_path import gap


class SeparatingAxisTests(unittest.TestCase):
    def test_axis_aligned_gap_in_metres(self):
        A = np.eye(4); B = np.eye(4); B[0,3] = 3
        self.assertAlmostEqual(gap(A,np.ones(3),B,np.ones(3)),1.)

    def test_overlap_has_no_positive_bound(self):
        self.assertEqual(gap(np.eye(4),np.ones(3),np.eye(4),np.ones(3)),0.)

    def test_thin_boxes_cross_at_midpoint(self):
        A = np.eye(4); B0 = np.eye(4); B1 = np.eye(4)
        B0[1,3] = -1.; B1[1,3] = 1.
        half = np.array([1.,.01,.01])
        self.assertGreater(gap(A,half,B0,half),0.)
        self.assertGreater(gap(A,half,B1,half),0.)
        # The displacement bound vetoes a crossing despite clear endpoints.
        self.assertLessEqual(gap(A,half,B0,half)-np.linalg.norm(B1[:3,3]-B0[:3,3]),0.)
        self.assertEqual(gap(A,half,np.eye(4),half),0.)

    def test_rotated_thin_boxes_overlap(self):
        A = np.eye(4); B = np.eye(4)
        B[:3,:3] = Rotation.from_euler("z",45,degrees=True).as_matrix()
        self.assertEqual(gap(A,np.array([1.,.01,.01]),B,np.array([1.,.01,.01])),0.)

    def test_rotation_arc_bound_covers_all_intermediate_vertices(self):
        radius = 1.3; angle = 1.1
        point = np.array([radius,0.,0.]); bound = radius*angle
        for a in np.linspace(-angle,angle,301):
            moved = Rotation.from_euler("z",a).apply(point)
            self.assertLessEqual(np.linalg.norm(moved-point),bound+1e-12)

    def test_margin_deducts_both_body_displacements(self):
        A = np.eye(4); B = np.eye(4); B[0,3] = 2.04
        initial = gap(A,np.ones(3),B,np.ones(3))
        self.assertGreater(initial,.03)
        self.assertLess(initial-.006-.006,.03)


if __name__ == "__main__":
    unittest.main(verbosity=2)
