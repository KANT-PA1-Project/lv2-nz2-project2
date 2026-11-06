"""Audit IMU aggregation and attitude guards against evaluator-only PhysX pose."""
import argparse
import json
from collections import Counter
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('--run',required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
rows=json.loads((root/'reports/local'/a.run/'sim_results.json').read_text())
actual=np.array([r['actual_rpy_deg'][:2] for r in rows])
estimated=np.array([r['motion_envelope']['tilt_deg'] for r in rows])
error=(estimated-actual+180)%360-180
windows=[s for r in rows for s in r.get('imu_window',[])]
valid=[s for s in windows if s.get('valid')]
stops=[r['frame'] for r in rows if r['motion_envelope']['reason']=='tilt_stop']
xy=np.array([r['actual_base'][:2] for r in rows])
summary={
 'run':a.run,'frames':len(rows),'simulation_seconds':len(rows)*.05,
 'actual_max_tilt_deg':float(np.abs(actual).max()),
 'estimated_max_tilt_deg':float(np.abs(estimated).max()),
 'tilt_error_rmse_deg':np.sqrt(np.mean(error**2,axis=0)).tolist(),
 'tilt_error_max_deg':float(np.abs(error).max()),
 'tilt_stop_first_frame':stops[0] if stops else None,
 'valid_native_windows':len(valid),'invalid_native_windows':len(windows)-len(valid),
 'native_samples_per_window':sorted(set(s['native_sample_count'] for s in valid)),
 'peak_actual_xy_speed_m_s':float(np.linalg.norm(np.diff(xy,axis=0)/.05,axis=1).max()) if len(xy)>1 else 0,
 'contact_reports':rows[-1]['contact_count'],
 'completed_stations':len(rows[-1]['completed']),
 'recognized_puck_ids':rows[-1]['recognized_puck_ids'],
 'policy_version':rows[-1]['policy_version'],
 'execution_scope':rows[-1]['execution_scope'],
 'tracking_fraction':float(np.mean([r.get('tracking')==1 for r in rows])),
 'last_motion_envelope':rows[-1]['motion_envelope'],
 'motion_reason_counts':dict(Counter(r['motion_envelope']['reason'] for r in rows)),
 'visual_attitude_frames':sum(r.get('visual_tilt_deg') is not None for r in rows),
 'last_visual_tilt_deg':rows[-1].get('visual_tilt_deg'),
 'barcode':rows[-1].get('barcode'),'signal_complete':rows[-1].get('signal_complete'),
 'limitations':['Snapshot of a simulation run; not a completed mission or hardware certification.',
 'PhysX pose is used only for this audit, never as an input to the attitude guard.']}
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
