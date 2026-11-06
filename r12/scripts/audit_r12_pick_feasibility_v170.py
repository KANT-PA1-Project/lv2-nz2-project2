"""Decision record for one-puck attempt under unchanged model limits."""
import json,hashlib
from pathlib import Path
import numpy as np
from arm_practice.learned_ik import ArmFK,LearnedIK
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12'
cfg=json.loads((root/'configs/r12_robot.json').read_text());fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0]);model=LearnedIK(out/'learned_ik_v1/model.pt',fk)
observed=json.loads((out/'v31_observed_reach.json').read_text())['results'][0];target=observed['target_base_m'];pad=json.loads((out/'v123_gripper_reach_interval.json').read_text());surface=json.loads((out/'v125_observed_surface_height.json').read_text())
q=np.array(cfg['lower']);a,b,c=q[1:4]
assert np.allclose(q[1:4],[-.35]*3)
z=.3345+.024*np.sin(a)+.1854*np.cos(a)+.149*np.sin(a+b)+.1911*np.sin(a+b+c)
assert abs(fk(q)[2]-z)<1e-8
lower=z-pad['all_apertures_pad_radius_upper_m'];top=surface['surface_sets']['all_valid_detection_bbox']['height_min_max_m'][1]
r={'complete':True,'target_observed_id':observed['observed_id'],'target_base_m':target,'observation_source':'V31 wrist RGB-D/CenterPose; archived, not fresh for execution','current_joint_limits_rad':{'lower':cfg['lower'],'upper':cfg['upper']},'learned_ik_result':model.solve(target),'tcp_height_lower_mm':float(z*1000),'pad_height_lower_mm':float(lower*1000),'observed_surface_max_height_mm':float(top*1000),'pad_lower_minus_observed_surface_mm':float((lower-top)*1000),'prior_bounded_solver_endpoint_error_mm':observed['position_residual_m']*1000,'configuration_sha256':hashlib.sha256((root/'configs/r12_robot.json').read_bytes()).hexdigest(),'pick_attempt_started':False,'success':False,'execution_authorized':False,'decision':'Current model limits cannot bring pads to the recorded visible target surface; target is outside supported neural training cloud. Resolve physical joint-limit provenance before changing model/training or attempting a grasp. Base XY/yaw changes cannot remove this level-base height gap.','limitations':'Conditional model/visible-surface result, not full-object or hardware impossibility proof. No joint limits, target height, TCP, fixture or acceptance thresholds changed.'}
(out/'v170_pick_feasibility.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
