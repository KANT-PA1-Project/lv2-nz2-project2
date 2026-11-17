"""Unrestricted geometry examples only; not trained IK or motion commands."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from arm_practice.learned_ik import ArmFK
from arm_practice.kinematics import forward_transform
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12';cfg=json.loads((root/'configs/r12_robot.json').read_text());urdf=root/'models/r12/r12_arm.urdf';fk=ArmFK(urdf,cfg['joint_names'],[.12475,0,0]);original=np.array(json.loads((out/'v171_link_length_only.json').read_text())['observed_target_base_m']);yaw=np.arctan2(original[1],original[0]);results=[]
for label,radius,vertical in [('recorded_distance_position_only',np.linalg.norm(original[:2]),False),('closer_300mm_top_down',.300,True)]:
 target=np.array([radius*np.cos(yaw),radius*np.sin(yaw),original[2]])
 def full(x):return np.r_[yaw,x,0.]
 def residual(x):
  pos=fk(full(x))-target
  return np.r_[pos,(sum(x)+np.pi/2)*.2] if vertical else pos
 fit=least_squares(residual,[-1.,.2,-.7],max_nfev=2000,ftol=1e-13,xtol=1e-13,gtol=1e-13)
 q=full(fit.x);T=forward_transform(urdf,'arm_link5',dict(zip(cfg['joint_names'],q)));direction=T[:3,0]
 results.append(dict(case=label,target_base_m=target.tolist(),geometric_q_rad=q.tolist(),endpoint_error_mm=float(np.linalg.norm(fk(q)-target)*1000),approach_axis_base=direction.tolist(),downward_orientation_error_deg=float(np.degrees(np.arccos(np.clip(direction@[0,0,-1],-1,1)))) if vertical else None))
assert all(r['endpoint_error_mm']<.001 for r in results)
assert results[1]['downward_orientation_error_deg']<.001
report=dict(complete=True,results=results,scope='Unrestricted-angle kinematic examples to verify link lengths only. These are solver-generated geometric witnesses, not learned IK, executable poses, collision-free trajectories, load validation or physical joint limits. Active configs and model unchanged.',execution_authorized=False)
(out/'v172_link_length_fk_check.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
