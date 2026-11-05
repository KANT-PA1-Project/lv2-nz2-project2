"""Separate first brake-predicate mismatch from later rollout divergence."""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.latest_policy import allocate
from arm_practice.transition_reference import moving_reference
from audit_r12_actuator_lag import valid


def predicates(command,angle,wheel,reference):
    a,w=allocate(command,angle,reference,np.zeros(4))
    margins=np.column_stack([abs(a-angle)-.3,abs(wheel)-2.,.2*abs(wheel)-abs(w)])
    return margins,margins>0


def audit(rows,model):
    first=[];reconstruction_errors=0;windows=0
    for i in range(1,len(rows)-10,5):
        if not all(valid(x) for x in rows[i:i+10]):continue
        windows+=1;t=rows[i]['drive_telemetry'];angle=np.array(t['steer_position_rad']);wheel=np.array(t['wheel_velocity_rad_s']);ref=np.array(t['steer_target_rad']);wr=np.array(t['wheel_target_rad_s'])
        for j in range(10):
            cmd=rows[i+j]['command'];actual=rows[i+j]['drive_telemetry'];next_actual=rows[i+j+1]['drive_telemetry']
            pm,pp=predicates(cmd,angle,wheel,ref)
            am,ap=predicates(cmd,np.array(actual['steer_position_rad']),np.array(actual['wheel_velocity_rad_s']),np.array(actual['steer_target_rad']))
            observed=np.array(next_actual['transition_braking']);reconstruction_errors+=int(not np.array_equal(ap.all(axis=1),observed))
            differing=np.flatnonzero(pp.all(axis=1)!=observed)
            if len(differing):
                first.append({'start_frame':rows[i]['frame'],'first_mismatch_frame':rows[i+j+1]['frame'],'modules':[{'index':int(m),'changed_predicates':[name for k,name in enumerate(['steering_gap','wheel_speed','drive_ratio']) if pp[m,k]!=ap[m,k]],'predicted_margins':pm[m].tolist(),'reconstructed_actual_margins':am[m].tolist(),'predicted_brake':bool(pp[m].all()),'actual_brake':bool(observed[m])} for m in differing]});break
            ref,wr,_=moving_reference(cmd,angle,wheel,ref,wr);angle+=model['steering_alpha']*(ref-angle);wheel+=model['wheel_alpha']*(wr-wheel)
    counts={name:sum(name in m['changed_predicates'] for x in first for m in x['modules']) for name in ['steering_gap','wheel_speed','drive_ratio']}
    return {'windows':windows,'first_mismatch_windows':len(first),'actual_mask_reconstruction_errors':reconstruction_errors,'changed_predicate_counts':counts,'first_mismatches':first,'scope':'First discrepancy per overlapping .5s window, not independent physical events. Reconstructed actual predicates use quantized encoder logs, command i applies at i+1. Future recorded decisions only for diagnostic comparison. Positive margins mean condition true; units differ by column.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('model');p.add_argument('output');a=p.parse_args();rows=json.loads((Path(a.run)/'sim_results.json').read_text());model=json.loads(Path(a.model).read_text())['model'];r=audit(rows,model);Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
