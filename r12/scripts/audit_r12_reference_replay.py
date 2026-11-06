"""One-step logged-state replay; command at frame i applies in frame i+1."""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.transition_reference import moving_reference


def audit(run):
    rows=json.loads((run/'sim_results.json').read_text());errors=[]
    for prev,row in zip(rows,rows[1:]):
        if not prev.get('transport_posture_ok') or prev.get('finished') or np.linalg.norm(prev['command'])<1e-6:continue
        if prev.get('motion_envelope',{}).get('reason')!='limited' or prev.get('lidar_guard')=='invalid_sensor':continue
        p=prev['drive_telemetry'];t=row['drive_telemetry']
        a,w,b=moving_reference(prev['command'],p['steer_position_rad'],p['wheel_velocity_rad_s'],p['steer_target_rad'],p['wheel_target_rad_s'])
        errors.append({'frame':row['frame'],'steer_max_error_rad':float(max(abs(a-t['steer_target_rad']))),'wheel_max_error_rad_s':float(max(abs(w-t['wheel_target_rad_s']))),'brake_mask_match':bool(np.array_equal(b,t['transition_braking']))})
    return {'samples':len(errors),'steer_error_max_rad':max(x['steer_max_error_rad'] for x in errors),'wheel_error_max_rad_s':max(x['wheel_max_error_rad_s'] for x in errors),'brake_mask_mismatches':sum(not x['brake_mask_match'] for x in errors),'worst':sorted(errors,key=lambda x:x['wheel_max_error_rad_s'],reverse=True)[:10],'scope':'One-step reference reproduction from logged quantized encoders; runtime allocator uses unquantized steering, so exact parity is not expected. Moving zero-residual mode only. No multi-step plant, torque/PI, collision or physical tracking prediction.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('output');a=p.parse_args();r=audit(Path(a.run));Path(a.output).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
