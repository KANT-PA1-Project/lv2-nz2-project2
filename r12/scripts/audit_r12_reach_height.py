"""Analytic lower TCP height for source arm geometry/limits; compare USD FK."""
import json,sys,itertools,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform
from audit_r12_usd_arm_path import CollisionModel
m=CollisionModel();cfg=json.loads((root/'configs/r12_robot.json').read_text());lo=np.array(cfg['lower']);hi=np.array(cfg['upper'])
assert np.allclose(lo[1:4],-.35) and np.allclose(hi[1:4],.35)
def z(q):
 a,b,c=q[1:4];return .3345+.024*np.sin(a)+.1854*np.cos(a)+.149*np.sin(a+b)+.1911*np.sin(a+b+c)
derivative_lower=[.024*math.cos(.35)-.1854*math.sin(.35)+.149*math.cos(.7)+.1911*math.cos(1.05),.149*math.cos(.7)+.1911*math.cos(1.05),.1911*math.cos(1.05)]
assert min(derivative_lower)>0
minimum=float(z(lo));errors=[]
for q in np.random.default_rng(33).uniform(lo,hi,(1000,5)):
 errors.append(abs(z(q)-(m.transforms(q)['arm_link5']@np.array([.12475,0,0,1]))[2]))
# Bound all finger-pad points relative to TCP by rotation-invariant radius.
T=m.transforms(np.zeros(5));radii={}
for name,body,local,_ in m.boxes:
 if 'pad' not in name:continue
 M=np.linalg.inv(T['arm_link5'])@T[body]@local
 corners=np.array(list(itertools.product([-1,1],repeat=3)))*m.sizes[name]/2
 pts=corners@M[:3,:3].T+M[:3,3]
 radii[name]=float(np.linalg.norm(pts-[.12475,0,0],axis=1).max())
result={'original_limits_rad':{'lower':lo.tolist(),'upper':hi.tolist()},'tcp_z_formula_m':'.3345+.024*sin(q2)+.1854*cos(q2)+.149*sin(q2+q3)+.1911*sin(q2+q3+q4)','positive_derivative_lower_bounds_m_per_rad':derivative_lower,'analytic_min_tcp_z_base_m':minimum,'minimum_at_q2_q3_q4':[-.35]*3,'usd_fk_comparison_samples':1000,'max_usd_formula_error_m':max(errors),'open_gripper_pad_radius_from_tcp_m':radii,'conservative_open_pad_z_lower_bound_m':minimum-max(radii.values()) if radii else None,'scope':'Analytic bound for source geometry and original limits with level base; excludes self/environment collision, which only restrict reach. Base XY/yaw cannot lower height. Not hardware certification. Open-pad radius bound only; closed fingers require separate geometry verification.'}
out=root.parent.parent/'outputs/r12/v33_reach_height_audit.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
