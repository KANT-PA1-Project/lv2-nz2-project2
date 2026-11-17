"""Offline actual-USD self-collision and optical-FOV view search."""
import json,itertools
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12';model=CollisionModel();home=np.array([0,.33,.33,.33,0])
v=json.loads((out/'v26_wrist_view_candidates.json').read_text())['candidates'][2];old=json.loads((root/'reports/local/diagnostic_s6_observed_goal_v26/ros_results.json').read_text())[-1]['estimated_map_pose'];new=json.loads((root/'reports/local/diagnostic_wrist_hold_v30/ros_results.json').read_text())[-1]['estimated_map_pose']
def T(p):
 m=np.eye(4);m[:3,:3]=Rotation.from_euler('z',p[2]).as_matrix();m[:2,3]=p[:2];return m
point=(np.linalg.inv(T(new))@T(old)@np.r_[v['surface_base_m'],1])[:3];cfg=json.loads((root/'configs/sensors/camera_rig.json').read_text())['arm_camera'];local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',cfg['optical_rpy_rad']).as_matrix();local[:3,3]=cfg['optical_center_xyz_m'];local[1,3]+=.025
valid=[];tested=0
for q1,q2,q3,q4,q5 in itertools.product([-np.pi/2,-1.4,-1.2],[-.35,-.25,-.15],[-.35,-.2,0],[-.35,-.2,0],[-.35,0,.35]):
 q=np.array([q1,q2,q3,q4,q5]);tested+=1;C=model.transforms(q)['arm_link5']@local;rel=C[:3,:3].T@(point-C[:3,3])
 if not .12<rel[2]<.5:continue
 uv=400*rel[:2]/rel[2]+[320,240]
 if not (40<uv[0]<600 and 40<uv[1]<440) or model.hits(q):continue
 # Each joint follows its own bounded speed, matching ObservationPosture.
 way=home.copy();way[0]=q1
 ok=True;count=0
 for a,b in [(home,way),(way,q)]:
  duration=np.max(np.abs(b-a))/.1
  for t in np.arange(0,duration+.101,.1):
   sample=a+np.clip(b-a,-.1*t,.1*t);count+=1
   if model.hits(sample):ok=False;break
  if not ok:break
 if ok:valid.append({'q':q.tolist(),'waypoints':[way.tolist(),q.tolist()],'pixel':uv.tolist(),'nominal_tag_px':float(8/rel[2]),'samples':count,'score':float(np.linalg.norm((uv-[320,240])/[320,240]))})
valid.sort(key=lambda x:x['score']);result={'tested':tested,'valid_count':len(valid),'candidates':valid[:5],'target_base_m':point.tolist(),'scope':'Anonymous V26 RGB-D target transformed using estimated poses. USD cubes+open gripper and rate-limited two-stage path, sampled only. No environment/visibility/uncertainty proof; needs fresh target reacquisition.'};(out/'v30_alternative_views.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
