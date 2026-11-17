"""Full aperture pad-radius bound with original arm limits; offline geometry only."""
import itertools,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_r12_usd_arm_path import CollisionModel

root=Path(__file__).resolve().parents[1];m=CollisionModel();cfg=json.loads((root/'configs/r12_gripper.json').read_text());previous=json.loads((root.parent.parent/'outputs/r12/v33_reach_height_audit.json').read_text());limits=cfg['simulation_finger_displacement_m']
def transforms(displacement):
    values={'gripper_left_joint':displacement,'gripper_right_joint':-displacement};out={'base_link':np.eye(4)};pending=list(m.joints)
    while pending:
        progress=False
        for j in pending[:]:
            name,b0,b1,L0,L1,axis,prism=j
            if b0 not in out:continue
            motion=np.eye(4);ax=np.eye(3)['XYZ'.index(axis)];v=values.get(name,0.)
            if prism:motion[:3,3]=ax*v
            else:motion[:3,:3]=Rotation.from_rotvec(ax*v).as_matrix()
            out[b1]=out[b0]@L0@motion@np.linalg.inv(L1);pending.remove(j);progress=True
        if not progress:raise ValueError('Disconnected chain')
    return out
radii=[]
for displacement in limits:
    T=transforms(displacement)
    for name,body,local,_ in m.boxes:
        if 'pad' not in name:continue
        M=np.linalg.inv(T['arm_link5'])@T[body]@local
        corners=np.array(list(itertools.product([-1,1],repeat=3)))*m.sizes[name]/2
        pts=corners@M[:3,:3].T+M[:3,3]
        radii.append({'displacement_m':displacement,'pad':name,'radius_m':float(np.linalg.norm(pts-[.12475,0,0],axis=1).max())})
old=m.transforms(np.zeros(5));new=transforms(.02768613)
assert all(np.allclose(old[k],new[k],atol=1e-10) for k in old)
radius=max(r['radius_m'] for r in radii);lower=previous['analytic_min_tcp_z_base_m']-radius
result={'finger_displacement_range_m':limits,'pad_endpoint_radii':radii,'all_apertures_pad_radius_upper_m':radius,'level_base_pad_height_lower_m':lower,'observed_target_center_base_m':.10560810677985444,'pad_to_observed_center_height_gap_lower_m':lower-.10560810677985444,'proof_scope':'Norm of each pad corner relative to TCP is convex in linear finger displacement; maximum over each closed displacement interval occurs at an endpoint. Rotations preserve radius. Combined with prior analytic TCP minimum under original arm limits. Applies to modeled pad cube geometry, level base; no environment, finger force, object extent or uncertainty certification. Base translation/yaw cannot change this height bound. Tilt/lifting base are not included.','open_transform_regression_passed':True}
out=root.parent.parent/'outputs/r12/v123_gripper_reach_interval.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
