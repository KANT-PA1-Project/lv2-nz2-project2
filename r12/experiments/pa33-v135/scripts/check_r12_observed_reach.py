"""Offline bounded position-only reach search from observed metric camera pose.
No execution authorization: orientation, environment and swept path remain unverified.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('inference',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
assert (a.run/'completion.json').exists()
row=json.loads((a.run/'ros_results.json').read_text())[-1];q=np.array(row['arm_measured_position']);data=json.loads(a.inference.read_text());cfg=json.loads((root/'configs/r12_robot.json').read_text());rig=json.loads((root/'configs/sensors/camera_rig.json').read_text())['arm_camera'];model=CollisionModel()
C=np.eye(4);C[:3,:3]=Rotation.from_euler('xyz',rig['optical_rpy_rad']).as_matrix();C[:3,3]=rig['optical_center_xyz_m'];C[1,3]+=rig['stereo_baseline_m']/2;C=model.transforms(q)['arm_link5']@C
rng=np.random.default_rng(32);starts=[np.clip(q,cfg['lower'],cfg['upper']),np.zeros(5),*rng.uniform(cfg['lower'],cfg['upper'],size=(30,5))];results=[]
for obj in data['detections']:
 if not obj.get('depth_consistent') or len(obj.get('tag_ids',[]))!=1:continue
 target=(C@np.r_[obj['pose']['center_camera_m'],1])[:3]
 def tcp(joints):return (model.transforms(joints)['arm_link5']@np.array([.12475,0,0,1]))[:3]
 fits=[least_squares(lambda x:tcp(x)-target,start,bounds=(cfg['lower'],cfg['upper']),max_nfev=200) for start in starts];fit=min(fits,key=lambda x:np.linalg.norm(x.fun));results.append({'observed_id':obj['tag_ids'][0],'target_base_m':target.tolist(),'candidate_q':fit.x.tolist(),'tcp_base_m':tcp(fit.x).tolist(),'position_residual_m':float(np.linalg.norm(fit.fun)),'endpoint_self_collision_pairs':model.hits(fit.x),'seeds':len(starts)})
report={'run':a.run.name,'results':results,'scope':'Offline observed CenterPose RGB-D-associated identity; measured joint FK and CAD TCP. Position-only bounded local IK; not global infeasibility proof, grasp, orientation, environmental or swept path verification.'};a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
