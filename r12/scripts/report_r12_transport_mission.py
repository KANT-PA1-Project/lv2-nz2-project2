"""Audit start ordering and motion constraints from a real integration run."""
import argparse,json
from pathlib import Path
from collections import Counter
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args();root=Path(__file__).resolve().parents[1]
rows=json.loads((root/'reports/local'/args.run/'sim_results.json').read_text());last=rows[-1]
def first(predicate):return next((r['frame'] for r in rows if predicate(r)),None)
bad=[r['frame'] for r in rows if np.linalg.norm(r['command'])>1e-6 and (not r.get('transport_posture_ok') or not r.get('signal_complete') or r.get('signal_error'))]
positions=np.array([r['actual_base'][:2] for r in rows]);speed=np.linalg.norm(np.diff(positions,axis=0)/.05,axis=1)
report={'run':args.run,'frames':len(rows),'first_signal_complete_frame':first(lambda r:r.get('signal_complete')),'first_transport_ready_frame':first(lambda r:r.get('transport_posture_ok')),'first_nonzero_base_command_frame':first(lambda r:np.linalg.norm(r['command'])>1e-6),'premature_motion_frames':bad,'transport_state':last.get('transport_state'),'signal_error':last.get('signal_error'),'signal_sequence':last.get('signal_sequence'),'barcode':last.get('barcode'),'last_path_follower':last.get('path_follower'),'lidar_guard_reasons':dict(Counter(str(r.get('lidar_guard')) for r in rows)),'max_actual_speed_m_s':float(speed.max()) if len(speed) else 0,'max_actual_tilt_deg':max(max(abs(x) for x in r['actual_rpy_deg'][:2]) for r in rows),'contacts':last['contact_count'],'completed_stations':len(last['completed']),'total_stations':last['total_goals'],'recognized_puck_ids':last['recognized_puck_ids'],'scope':last['execution_scope'],'policy_version':last['policy_version'],'latest_base_command':last['command'],'mission_finished':last['finished'],'mission_optimizer_trained':False,'limitations':['Current survey follower still has fixed yaw. Mission-aware holonomic heading optimization and twelve-puck pick/place remain unfinished.','Simulation uses declared provisional masses/friction/torques. Not physical robot certification.']}
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
