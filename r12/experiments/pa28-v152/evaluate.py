from pathlib import Path
import json,sys
import numpy as np
root=Path(__file__).parent
exec(compile((root/'audit.py').read_text().split('rows=[]')[0],'audit','exec'))
rundir=Path(sys.argv[1]) if len(sys.argv)>1 else root/'lift';rows=json.loads((rundir/'per_step.json').read_text());contacts=json.loads((rundir/'contacts.json').read_text());failure=json.loads((rundir/'failure.json').read_text()) if (rundir/'failure.json').exists() else None
close=[r for r in rows if r['phase']=='contact_hold'];liftrows=[r for r in rows if r['phase']=='lift'];hold=[r for r in rows if r['phase']=='retention'];start=close[-1] if close else rows[0];z0=start['puck_position_evaluator'][2]
def relative(r):
 t=tf(r['arm_q'])['arm_link5'];return (np.linalg.inv(t)@np.r_[np.array(r['puck_position_evaluator'])-[0,0,.008],1])[:3]
rel0=relative(start);relerrors=[float(np.linalg.norm(relative(r)-rel0))for r in hold];hc=[c for c in contacts if c['phase']=='retention'];support=[c for c in hc if '/World/Support'in c['paths']];left=[c for c in hc if any('gripper_left/pad_collision' in x for x in c['paths'])];right=[c for c in hc if any('gripper_right/pad_collision'in x for x in c['paths'])]
res=dict(scope='Independent physical unit only; fixture GT setup once after world.reset; encoder fixedtrajectory control; all object poses/contact data evaluator only',failure=failure,steps=len(rows),max_arm_qd=max(max(map(abs,r['arm_qd']))for r in rows),max_arm_fd=max(max(map(abs,r['arm_fd']))for r in rows),max_lift_height_m=max(r['puck_position_evaluator'][2]-z0 for r in rows),hold_samples=len(hold),hold_duration_s=len(hold)/960,hold_height_min_m=min([r['puck_position_evaluator'][2]-z0 for r in hold],default=None),hold_relative_error_max_m=max(relerrors,default=None),hold_support_events=len(support),hold_left_pad_events=len(left),hold_right_pad_events=len(right),final_aperture_m=rows[-1]['aperture_m'],final_force_command_N=rows[-1]['command_force_N'],mission_success=False,placement_attempted=any(r["phase"]=="release" for r in rows),environment_evaluator_interval_s=.1,continuous_clearance_certificate=False)
res['unit_lift_retention_criteria_met']=bool(not failure and len(hold)>=1920 and res['hold_height_min_m']>=.025 and res['hold_relative_error_max_m']<=.002 and not support and left and right)
(rundir/'evaluation.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
