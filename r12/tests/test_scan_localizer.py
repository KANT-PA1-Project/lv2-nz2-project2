import unittest
import numpy as np
from arm_practice.scan_localizer import ScanLocalizer

class ScanLocalizerTests(unittest.TestCase):
    def fixture(self):
        grid=np.zeros((100,100),dtype=np.uint8);grid[[0,-1],:]=1;grid[:,[0,-1]]=1
        transforms={};scans={}
        for i,yaw in enumerate([0,np.pi/2,np.pi,3*np.pi/2]):
            c,s=np.cos(yaw),np.sin(yaw);t=np.eye(4);t[:2,:2]=[[c,-s],[s,c]];transforms[str(i)]=t
            angles=np.linspace(-np.pi/2,np.pi/2,90)+yaw
            dirs=np.column_stack([np.cos(angles),np.sin(angles)])
            # Exact wall intersections, independent of the distance-field fit.
            distances=np.where(dirs>0,.98,-1.)/np.where(abs(dirs)>1e-9,dirs,1e-9)
            distances[distances<=0]=1e9;scans[str(i)]=distances.min(axis=1)
        return ScanLocalizer({'resolution':.02,'occupied':grid},transforms),scans
    def test_recovers_small_pose_drift_from_all_lidars(self):
        localizer,scans=self.fixture();fit=localizer.fit(scans,[1.08,.94,.025])
        self.assertIsNotNone(fit)
        np.testing.assert_allclose(fit['pose'],[1.,1.,0.],atol=.025)
    def test_missing_or_corrupt_sensor_rejected(self):
        localizer,scans=self.fixture();scans.pop('3')
        self.assertIsNone(localizer.fit(scans,[1,1,0]))
        localizer,scans=self.fixture();scans['0'][0]=np.nan
        self.assertIsNone(localizer.fit(scans,[1,1,0]))
