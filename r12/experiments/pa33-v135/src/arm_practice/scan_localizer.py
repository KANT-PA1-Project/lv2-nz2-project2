"""Four-lidar scan-to-known-map correction; no simulator pose input."""
import numpy as np
from scipy.ndimage import distance_transform_edt,map_coordinates
from scipy.optimize import least_squares

class ScanLocalizer:
    def __init__(self,grid,extrinsics):
        self.res=float(grid['resolution'])
        self.padding=20
        self.distance=distance_transform_edt(1-np.pad(grid['occupied'],self.padding))*self.res
        self.extrinsics=extrinsics
    def fit(self,scans,prior):
        points=[]
        if np.asarray(prior).shape!=(3,) or not np.isfinite(prior).all():return None
        if set(scans)!=set(self.extrinsics):return None
        for name,ranges in scans.items():
            r=np.asarray(ranges);a=np.linspace(-np.pi/2,np.pi/2,len(r))
            if r.shape!=(90,) or not np.isfinite(r).all():return None
            good=np.isfinite(r)&(r>.04)&(r<5.9)
            local=np.column_stack([r*np.cos(a),r*np.sin(a),np.zeros(len(r))])
            t=np.asarray(self.extrinsics[name]);points.extend((local@t[:3,:3].T+t[:3,3])[good][::2,:2])
        if len(points)<25:return None
        points=np.asarray(points);prior=np.asarray(prior)
        def residual(p,regularize=True):
            c,s=np.cos(p[2]),np.sin(p[2]);world=points@np.array([[c,s],[-s,c]])+p[:2]
            d=map_coordinates(self.distance,(world/self.res+self.padding).T[::-1],order=1,mode='constant',cval=1.)
            return np.r_[d,(p-prior)*[.03,.03,.1]] if regularize else d
        fit=least_squares(residual,prior,bounds=(prior-[.25,.25,.1],prior+[.25,.25,.1]),loss='soft_l1',f_scale=.025,diff_step=1e-3,max_nfev=30)
        errors=residual(fit.x,False);inliers=errors<.06
        if inliers.mean()<.7:return None
        # Reject poorly constrained geometry, e.g. a single straight wall.
        eigen=np.linalg.eigvalsh(fit.jac.T@fit.jac)
        if eigen[0]<.03:return None
        return {'pose':fit.x,'inlier_fraction':float(inliers.mean()),'rmse':float(np.sqrt(np.mean(errors[inliers]**2)))}
