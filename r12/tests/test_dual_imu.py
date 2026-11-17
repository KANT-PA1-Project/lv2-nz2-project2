import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from dual_imu import DualImuFusion,Sample,rotation

class DualImuTests(unittest.TestCase):
    def pair(self,t,omega,alpha,force):
        samples=[]
        for r,q in zip([[.057,0,.16597],[-.057,0,.16597]],[[0,0,0,1],[0,0,1,0]]):
            R=rotation(q);measured=force+np.cross(alpha,r)+np.cross(omega,np.cross(omega,r))
            samples.append(Sample(t,R.T@omega,R.T@measured,np.eye(3)*.0004,np.eye(3)*.04))
        return samples
    def fusion(self):return DualImuFusion([[.057,0,.16597],[-.057,0,.16597]],[[0,0,0,1],[0,0,1,0]],alpha_tau=0)
    def test_mount_rotation_and_lever_arm(self):
        f=self.fusion();w=np.array([.2,.1,.3]);alpha=np.array([.1,-.2,.3]);force=np.array([.1,.2,9.81])
        self.assertIsNone(f.update(self.pair(1,w,alpha,force)))
        result=f.update(self.pair(1.05,w+.05*alpha,alpha,force))
        np.testing.assert_allclose(result['accel'],force,atol=1e-10)
        np.testing.assert_allclose(result['gyro'],w+.05*alpha,atol=1e-10)
    def test_missing_disagreeing_nonfinite_pair_rejected(self):
        p=self.pair(1,np.zeros(3),np.zeros(3),[0,0,9.81])
        with self.assertRaises(ValueError):self.fusion().update(p[:1])
        p[1].gyro[2]=2
        with self.assertRaises(ValueError):self.fusion().update(p)
        p[1].gyro[2]=np.nan
        with self.assertRaises(ValueError):self.fusion().update(p)
    def test_repeated_timestamp_and_gap(self):
        f=self.fusion();p=lambda t:self.pair(t,np.zeros(3),np.zeros(3),[0,0,9.81])
        f.update(p(1))
        with self.assertRaises(ValueError):f.update(p(1))
        self.assertIsNone(f.update(p(2)))
        self.assertIsNotNone(f.update(p(2.05)))
