"""RGB-D-only observation candidates and bounded wrist-view IK, offline only.
Colour components are hypotheses, never puck identities or grasp confirmations.
"""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform


def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 data=np.load(a.run/'frame.npz');cfg=json.loads((root/'configs/r12_robot.json').read_text());rig=json.loads((root/'configs/sensors/camera_rig.json').read_text())
 names=cfg['joint_names'];urdf=root/'models/r12/r12_arm.urdf';home=np.array([0,.33,.33,.33,0.])
 local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',rig['arm_camera']['optical_rpy_rad']).as_matrix();local[:3,3]=rig['arm_camera']['optical_center_xyz_m'];local[1,3]+=rig['arm_camera']['stereo_baseline_m']/2
 def camera(q):return forward_transform(urdf,'arm_link5',dict(zip(names,q)))@local
 candidates=[]
 for name in rig['cuvslam_rigid_rig']:
  rgb=data['rgb'][0] if name=='r07_camera1' else data[name+'_left_optical'];depth=data['depth'] if name=='r07_camera1' else data[name+'_depth']
  hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
  mask=(((hsv[:,:,0]<12)|(hsv[:,:,0]>170)|((hsv[:,:,0]>35)&(hsv[:,:,0]<85)))&(hsv[:,:,1]>110)&(hsv[:,:,2]>60)).astype(np.uint8)
  count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
  T=forward_transform(urdf,name+'_left_optical',dict(zip(names,home)))
  for i in range(1,count):
   x,y,w,h,area=stats[i]
   if area<12 or w<3 or h<3:continue
   v,u=np.where(labels==i);z=depth[v,u];valid=np.isfinite(z)&(z>.05)&(z<4)
   if valid.sum()<10:continue
   z=z[valid];u=u[valid];v=v[valid]
   points=np.stack([(u-320)*z/400,(v-240)*z/400,z],axis=1)
   target=T[:3,:3]@np.median(points,axis=0)+T[:3,3]
   span=np.ptp(points,axis=0)
   if max(span)>.12:continue
   # Desired20cm stand-off gives40px over a20mm texture at fx400.
   def residual(q):
    C=camera(q);rel=C[:3,:3].T@(target-C[:3,3]);return rel-np.array([0,0,.20])
   results=[least_squares(residual,start,bounds=(cfg['lower'],cfg['upper']),max_nfev=120) for start in [home,np.zeros(5),np.array([-.9,-.2,-.2,-.2,0]),np.array([.9,-.2,-.2,-.2,0])]]
   fit=min(results,key=lambda r:np.linalg.norm(r.fun));C=camera(fit.x);rel=C[:3,:3].T@(target-C[:3,3])
   candidates.append(dict(camera=name,bbox=[int(x),int(y),int(w),int(h)],pixels=int(area),surface_base_m=target.tolist(),observed_span_camera_m=span.tolist(),candidate_joints_rad=fit.x.tolist(),view_residual_m=float(np.linalg.norm(fit.fun)),target_in_wrist_optical_m=rel.tolist(),nominal20mm_texture_px=float(400*.02/rel[2]) if rel[2]>0 else None))
 # Separate base translation from vertical reach: moving on the floor cannot
 # change the height of the optical-axis point at the desired stand-off.
 starts=np.random.default_rng(12).uniform(cfg['lower'],cfg['upper'],size=(12,5))
 height_fits=[]
 for start in starts:
  fit=least_squares(lambda q: np.array([(camera(q)@np.array([0,0,.20,1]))[2]-.11]), start,bounds=(cfg['lower'],cfg['upper']),max_nfev=100)
  height_fits.append(fit)
 best=min(height_fits,key=lambda f: abs(f.fun[0]));C=camera(best.x);aim=(C@np.array([0,0,.20,1]))[:3]
 vertical=dict(target_height_m=.11,optical_axis_point_m=aim.tolist(),height_error_m=float(abs(best.fun[0])),candidate_joints_rad=best.x.tolist(),seed_count=len(starts),scope='Height-only local IK with free base XY; no collision or visibility validation, not a global infeasibility proof')
 result=dict(run=a.run.name,candidates=candidates,height_only_view_search=vertical,scope='Offline RGB-D colour hypotheses only; joint limits preserved. Uses commanded folded posture for fixed-camera FK only. No identity, collision check, execution or grasp success. IK is a bounded local search, not an impossibility proof.')
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
