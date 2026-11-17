"""Offline actual-pose evaluation only; never supplies truth to control."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args();rows=json.loads((a.run/'sim_results.json').read_text());xyz=np.array([r['actual_base'] for r in rows]);yaw=np.unwrap(np.radians([r['actual_rpy_deg'][2] for r in rows]));dt=np.diff([r['frame'] for r in rows])*.05
# Physics steps precede current response; previous response drove this interval.
commands=np.array([r['command'] for r in rows[:-1]]);velocity=np.diff(xyz[:,:2],axis=0)/dt[:,None];mid=(yaw[1:]+yaw[:-1])/2;body=np.stack([np.cos(mid)*velocity[:,0]+np.sin(mid)*velocity[:,1],-np.sin(mid)*velocity[:,0]+np.cos(mid)*velocity[:,1]],axis=1);wy=np.diff(yaw)/dt;speed=np.linalg.norm(body,axis=1)
mask=np.array([r.get('transport_posture_ok',False) and not r.get('navigation_finished',r.get('finished',False)) for r in rows[:-1]])
active=np.flatnonzero(mask & (np.linalg.norm(commands[:,:2],axis=1)>.02));windows=[]
if len(active):
 first,last=active[0],active[-1];slow=(speed<.02)&mask;start=None
 for i in range(first,last+2):
  if i<=last and slow[i]:
   if start is None:start=i
  elif start is not None:
   seconds=float(dt[start:i].sum())
   if seconds>=.25:windows.append({'first_frame':rows[start]['frame'],'duration_s':seconds,'command_speed_peak_m_s':float(np.linalg.norm(commands[start:i,:2],axis=1).max()),'lidar_reasons':sorted({str(r.get('lidar_guard')) for r in rows[start:i]}),'path_indices':sorted({r['path_follower']['path_index'] for r in rows[start:i] if r.get('path_follower')})})
   start=None
result={'run':a.run.name,'frames':len(rows),'trace_finished':bool(rows[-1].get('finished',False)),'peak_abs_yaw_error_rad_s':float(np.max(np.abs(wy[mask]-commands[mask,2]))) if mask.any() else None,'max_tilt_deg':float(np.max(np.linalg.norm(np.array([r['actual_rpy_deg'][:2] for r in rows]),axis=1))),'invalid_imu_windows':sum(not a['valid'] for r in rows for a in r.get('imu_window',[])),'navigation_samples':int(mask.sum()),'achieved_peak_speed_m_s':float(speed[mask].max()) if mask.any() else None,'xy_vector_rmse_m_s':float(np.sqrt(np.mean(np.sum((body[mask]-commands[mask,:2])**2,axis=1)))) if mask.any() else None,'yaw_rmse_rad_s':float(np.sqrt(np.mean((wy[mask]-commands[mask,2])**2))) if mask.any() else None,'low_speed_windows':windows,'scope':'Evaluator-only finite differences of actual pose at20Hz against previous command; navigation interval only. Low-speed windows are candidates, not automatically unwanted stops or corner classifications. Not full mission or fixed-suite acceptance.'};a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
