"""Mathematical synthetic tests, not physical calibration evidence."""
import unittest
import numpy as np
from fit_sim_camera import fit_handeye,matrix
from audit_swept_path import audit
from pathlib import Path


class HandEyeFitTests(unittest.TestCase):
    def dataset(self):
        X=matrix(np.array([-.8,.4,-.7,.04,.025,.06]))
        Y=matrix(np.array([.2,-.4,.3,.5,.02,.42]))
        angles=[(-.3,0),(.3,0),(0,-.4),(0,.4),(-.2,.3),(.2,-.3),(.1,.2),(-.1,-.2)]
        pairs=[]
        for a,b in angles:
            B=matrix(np.array([a,0,b,.3+.03*a,.02*b,.4+.02*b]))
            C=np.linalg.inv(X)@np.linalg.inv(B)@Y
            pairs.append((B,C))
        return pairs[:6],pairs[6:],X

    def test_recovers_transform_and_checks_held_out_poses(self):
        train,holdout,X=self.dataset()
        report=fit_handeye(train,holdout)
        np.testing.assert_allclose(report['T_link_camera_candidate'],X,atol=1e-7,rtol=0)
        self.assertLess(report['holdout_residuals']['translation_max_m'],1e-8)
        self.assertLess(report['holdout_residuals']['rotation_max_rad'],1e-8)

    def test_repeated_stationary_pose_rejected(self):
        train,holdout,_=self.dataset()
        with self.assertRaisesRegex(ValueError,'nonparallel_rotation'):
            fit_handeye([train[0]]*6,holdout)

    def test_missing_holdout_rejected(self):
        train,_,_=self.dataset()
        with self.assertRaisesRegex(ValueError,'insufficient_train_or_holdout'):
            fit_handeye(train,[])

    def test_nonfinite_pose_rejected(self):
        train,holdout,_=self.dataset();train[0][0][0,0]=np.nan
        with self.assertRaisesRegex(ValueError,'invalid_pose'):
            fit_handeye(train,holdout)


class StaticAdmissionTests(unittest.TestCase):
    source=Path(__file__).resolve().parent/'source'
    q=[0.,.2476936668,-.3298198833,-.3298198824,0.]

    def test_missing_environment_is_not_certified(self):
        report=audit(self.source,self.q,self.q)
        self.assertTrue(report['all_self_bounds_proven'])
        self.assertFalse(report['environment_bounds_proven'])

    def test_close_obstacle_vetoes_admission(self):
        report=audit(self.source,self.q,self.q,obstacle_spheres=[([0,0,0],.5)])
        self.assertFalse(report['environment_bounds_proven'])

    def test_far_obstacle_uses_all_447_colliders(self):
        report=audit(self.source,self.q,self.q,obstacle_spheres=[([100,0,0],.1)])
        self.assertTrue(report['environment_bounds_proven'])
        self.assertEqual(report['collider_count'],447)
        self.assertEqual(report['unmapped_colliders'],[])
        self.assertFalse(report['motion_authorized'])


if __name__=='__main__':unittest.main(verbosity=2)
