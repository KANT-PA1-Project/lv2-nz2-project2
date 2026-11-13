import json,sys
from pathlib import Path
import numpy as np
for arg in sys.argv[1:]:
 p=Path(arg);cfg=json.loads((p/'inputs.json').read_text())['config'];r=json.loads((p/'result.json').read_text());rows=[json.loads(l) for l in (p/'frames.jsonl').open()]
 q0=np.array(cfg['barcode_arm_q_rad']);qerr=max(np.abs(np.array(x['joint_q'])-q0).max() for x in rows)
 target_error=0.;motion_count=0
 for line in (p/'physics.jsonl').open():
  x=json.loads(line);target_error=max(target_error,float(np.max(np.abs(np.array(x['target'])-q0))))
  motion_count+=x['phase']=='move_base'
 route=r['signal_route'];events=route['events'];preps=[x for x in events if x['event']=='prepare_started'];ready=next(x['t_s'] for x in events if x['event']=='ready')
 moved=r.get('base_moved',False);check=dict(all_signal_frames_use_mast=all(x['signal_camera']=='mast' for x in rows),arm_targets_unchanged=target_error<1e-9,measured_arm_stays_within_01_rad=qerr<.01,no_arm_motion_phases=not any(x['phase'] in ['move','move_transport'] for x in rows),fresh_off_after_relocation=(not moved or len(preps)==2 and preps[1]['t_s']>=r['base_motion_end_s']+.49 and ready>=preps[1]['t_s']+1.),base_reaches_goal=(not moved or np.linalg.norm(np.array(rows[-1]['base_xy'])-cfg['signal_fallback']['base_xyz_yaw'][:2])<.005),route_consistent=(moved==route['fallback_used']))
 check={k:bool(v) for k,v in check.items()}
 audit=dict(checks=check,all_checks_passed=all(check.values()),max_measured_arm_error_rad=float(qerr),max_arm_target_error_rad=target_error,move_base_physics_samples=motion_count,initial_base_xy=rows[0]['base_xy'],final_base_xy=rows[-1]['base_xy'],ready_at_s=ready)
 (p/'mast_relocation_audit.json').write_text(json.dumps(audit,indent=2)+'\n');print(p.name,json.dumps(audit))
