"""Offline diagnostic ablations; observed future decisions are never control inputs."""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.latest_policy import allocate
from arm_practice.steering_transition import brake_before_large_turn
from audit_r12_actuator_lag import valid


def run(rows,model,mode,steps=10):
    output=[]
    for i in range(1,len(rows)-steps,5):
        if not all(valid(x) for x in rows[i:i+steps]):continue
        t=rows[i]['drive_telemetry'];angle=np.array(t['steer_position_rad']);wheel=np.array(t['wheel_velocity_rad_s']);ref=np.array(t['steer_target_rad']);wref=np.array(t['wheel_target_rad_s']);mismatches=[];reference_divergence=None
        for j in range(steps):
            actual=rows[i+j+1]['drive_telemetry']
            a,w=allocate(rows[i+j]['command'],angle,ref,np.zeros(4))
            held,braked,mask=brake_before_large_turn(a,angle,w,wheel)
            observed=np.array(actual['transition_braking'])
            if not np.array_equal(mask,observed):mismatches.append(rows[i+j+1]['frame'])
            if mode=='observed_brake_mask':held=np.where(observed,angle,a);braked=np.where(observed,0.,w)
            ref=held;wref=wref+np.clip(braked-wref,-24.45*.05,24.45*.05)
            if mode=='observed_reference':ref=np.array(actual['steer_target_rad']);wref=np.array(actual['wheel_target_rad_s'])
            if reference_divergence is None and np.max(abs(ref-actual['steer_target_rad']))>.3:reference_divergence=rows[i+j+1]['frame']
            angle+=model['steering_alpha']*(ref-angle);wheel+=model['wheel_alpha']*(wref-wheel)
        actual=rows[i+steps]['drive_telemetry']
        output.append({'start_frame':rows[i]['frame'],'steering_mse':float(np.mean((angle-actual['steer_position_rad'])**2)),'wheel_mse':float(np.mean((wheel-actual['wheel_velocity_rad_s'])**2)),'brake_mismatch_frames':mismatches,'first_reference_error_over_0_3_frame':reference_divergence})
    return {'windows':len(output),'steering_rmse_rad':float(np.sqrt(np.mean([r['steering_mse'] for r in output]))),'wheel_rmse_rad_s':float(np.sqrt(np.mean([r['wheel_mse'] for r in output]))),'windows_with_brake_mismatch':sum(bool(x['brake_mismatch_frames']) for x in output),'worst':sorted(output,key=lambda x:x['steering_mse'],reverse=True)[:5]}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('model');p.add_argument('output');a=p.parse_args()
    rows=json.loads((Path(a.run)/'sim_results.json').read_text());model=json.loads(Path(a.model).read_text())['model']
    result={m:run(rows,model,m) for m in ['free','observed_brake_mask','observed_reference']}
    result['scope']='0.5s recorded-command replay. Observed future brake masks/references are diagnostic oracle ablations, NOT deployable predictors or controller observations. Coefficients unchanged from V97. Results do not establish physical causality or full future-state prediction.'
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:({x:y for x,y in v.items() if x!='worst'} if isinstance(v,dict) else v) for k,v in result.items()},indent=2))
