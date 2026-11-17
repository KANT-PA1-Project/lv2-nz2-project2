"""Offline yaw-error/steering correlation, never used as control observation."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('runs',nargs='+');p.add_argument('--output',required=True);a=p.parse_args();result={}
for name in a.runs:
 rows=json.loads((Path(name)/'sim_results.json').read_text())
 yaw=np.unwrap(np.radians([r['actual_rpy_deg'][2] for r in rows]));cmd=np.array([r['command'] for r in rows]);dt=np.diff([r['frame'] for r in rows])*.05
 error=np.diff(yaw)/dt-cmd[:-1,2];i=int(np.argmax(abs(error)))
 steer=np.array([r['drive_telemetry']['steer_position_rad'] for r in rows]);rates=np.diff(steer,axis=0)/dt[:,None]
 mask=np.array([r.get('transport_posture_ok',False) and not r.get('finished',False) for r in rows[:-1]])
 fast=np.max(abs(rates),axis=1)>2
 def stats(m):return {'samples':int(m.sum()),'yaw_rmse':float(np.sqrt(np.mean(error[m]**2))) if m.any() else None,'yaw_peak':float(np.max(abs(error[m]))) if m.any() else None}
 result[Path(name).name]={'fast_steer_over2rad_s':stats(mask&fast),'other':stats(mask&~fast),'worst_frame':i,'worst_yaw_error':float(error[i]),'window':[{'frame':r['frame'],'command':r['command'],'steer_position':r['drive_telemetry']['steer_position_rad'],'steer_target':r['drive_telemetry']['steer_target_rad'],'wheel_velocity':r['drive_telemetry']['wheel_velocity_rad_s'],'wheel_target':r['drive_telemetry']['wheel_target_rad_s'],'transition_braking':r['drive_telemetry']['transition_braking']} for r in rows[max(0,i-3):i+5]],'limitation':'Correlation only; closed-loop trajectories differ. Slower slew does not enforce compatible wheel rolling velocities.'}
Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:{a:b for a,b in v.items() if a!='window'} for k,v in result.items()},indent=2))
