"""Offline-only metrics for standalone transition runs; never control feedback."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args();results={}
for name in ['baseline','heading','smooth']:
 file=a.run/(name+'.json')
 if not file.exists():continue
 d=json.loads(file.read_text());r=d['rows'];yaw=np.unwrap([x['actual_rpy'][2] for x in r]);wy=np.diff(yaw)/.05;v=np.diff(np.array([x['actual_xyz'][:2] for x in r]),axis=0)/.05;c=np.array([x['command_world'][:2] for x in r[1:]])
 yaw_command=np.array([x['command'][2] for x in r[1:]]);yaw_error=wy-yaw_command
 if d.get('command_frame')=='body':
  # Offline reference only: average pose yaw projects body-frame commands into world.
  mid=(yaw[1:]+yaw[:-1])/2;body=np.array([x['command'][:2] for x in r[1:]])
  c=np.column_stack([np.cos(mid)*body[:,0]-np.sin(mid)*body[:,1],np.sin(mid)*body[:,0]+np.cos(mid)*body[:,1]])
 # In this harness command at frame i drives physics before that same row.
 windows={}
 window_specs=[('transition',slice(219,260)),('steady_before',slice(190,219)),('steady_after',slice(270,299)),('all_motion',slice(139,300))]
 if d.get('ramp_start_stop'):
  window_specs=[('acceleration',slice(139,179)),('steady_before',slice(179,219)),('transition',slice(219,259)),('deceleration',slice(259,299)),('post_stop',slice(299,None)),('all_motion',slice(139,None))]
 if d.get('ramp_start_stop') and d.get('motion_stop_frame',300)!=300:
  stop=d['motion_stop_frame']
  window_specs=[(name,sl) for name,sl in window_specs if name not in ['deceleration','post_stop']]
  window_specs += [('deceleration',slice(stop-41,stop-1)),('post_stop',slice(stop-1,None)),('extended_cruise',slice(279,stop-41))]
 if d.get('yaw_zero_dwell'):window_specs += [('yaw_zero_dwell',slice(229,269)),('yaw_return',slice(269,279))]
 for label,sl in window_specs:
  windows[label]={'world_xy_rmse':float(np.sqrt(np.mean(np.sum((v[sl]-c[sl])**2,axis=1)))),'yaw_rms':float(np.sqrt(np.mean(yaw_error[sl]**2))),'peak_abs_yaw_error':float(abs(yaw_error[sl]).max()),'peak_abs_yaw':float(abs(wy[sl]).max()),'max_speed':float(np.linalg.norm(v[sl],axis=1).max()),'min_speed':float(np.linalg.norm(v[sl],axis=1).min())}
 results[name]={'frames':len(r),'heading_deg':float(np.degrees(d['planned_heading'])),'max_encoder_rad_s':float(np.max(np.abs([x['wheel_encoder'] for x in r]))),'max_tilt_deg':float(np.degrees(max(np.linalg.norm(x['actual_rpy'][:2]) for x in r))),'pre_motion_arm_error':r[139]['arm_error_rad'],'windows':windows}
report={'scope':'Isolated deterministic flat-floor audit, no sensor-stack or mission acceptance; requested speed is not measured performance. Includes initialization tilt.','results':results};a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report))
