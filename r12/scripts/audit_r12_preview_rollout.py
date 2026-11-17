"""Replay observed command sequences to falsify ideal actuator tracking.

Future recorded commands are evaluation inputs only, never planner observations.
Initial state comes from encoders; later predicted states equal references.
"""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.transition_reference import moving_reference


def audit(run):
    rows=json.loads((run/'sim_results.json').read_text());out={}
    for steps in [1,5,10,20]:
        errors=[]
        for i in range(1,len(rows)-steps,5):
            block=rows[i:i+steps]
            if not all(r.get('transport_posture_ok') and not r.get('finished') and np.linalg.norm(r['command'])>=1e-6 and r.get('motion_envelope',{}).get('reason')=='limited' for r in block):continue
            t=rows[i]['drive_telemetry'];angle=np.array(t['steer_position_rad']);wheel=np.array(t['wheel_velocity_rad_s']);ref=np.array(t['steer_target_rad']);wref=np.array(t['wheel_target_rad_s'])
            for j in range(steps):
                ref,wref,_=moving_reference(rows[i+j]['command'],angle,wheel,ref,wref)
                angle=ref.copy();wheel=wref.copy()
            actual=rows[i+steps]['drive_telemetry']
            errors.append({'start_frame':rows[i]['frame'],'steering_rmse_rad':float(np.sqrt(np.mean((angle-actual['steer_position_rad'])**2))),'wheel_rmse_rad_s':float(np.sqrt(np.mean((wheel-actual['wheel_velocity_rad_s'])**2)))})
        out[str(steps*.05)]={'samples':len(errors),'steering_rmse_rad':float(np.sqrt(np.mean([e['steering_rmse_rad']**2 for e in errors]))),'wheel_rmse_rad_s':float(np.sqrt(np.mean([e['wheel_rmse_rad_s']**2 for e in errors]))),'worst':sorted(errors,key=lambda e:e['steering_rmse_rad'],reverse=True)[:3]}
    return {'horizons_s':out,'scope':'Recorded future commands only for model validation; not a causal planner evaluation. Actual encoder state is used only at initialization and for scoring. Ideal reference-as-state prediction omits PI, torque and steering/contact dynamics. These are joint prediction errors, not body tracking acceptance metrics.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('output');a=p.parse_args();r=audit(Path(a.run));Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
