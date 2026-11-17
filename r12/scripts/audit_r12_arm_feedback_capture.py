"""Validate completed encoder telemetry capture; no hardware/IK performance claim."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
assert (a.run/'completion.json').exists()
r=json.loads((a.run/'ros_results.json').read_text());sim=json.loads((a.run/'sim_results.json').read_text());checks={}
checks['nonempty_and_matching_count']=len(r)>1 and len(r)==len(sim)
checks['same_packet_timestamp']=all(x['arm_feedback_stamp_ns']==x['stamp_ns'] for x in r)
checks['monotonic_50ms']=bool(np.all(np.diff([x['arm_feedback_stamp_ns'] for x in r])==50_000_000))
checks['first_velocity_invalid']=not r[0]['arm_feedback_valid']
checks['subsequent_valid']=all(x['arm_feedback_valid'] for x in r[1:])
q=np.array([x['arm_measured_position'] for x in r]);v=np.array([x['arm_measured_velocity'] for x in r]);derived=((np.diff(q,axis=0)+np.pi)%(2*np.pi)-np.pi)/.05
checks['finite_5_joint_feedback']=q.shape==(len(r),5) and np.isfinite(q).all() and np.isfinite(v).all()
checks['velocity_matches_position_difference']=bool(np.allclose(v[1:],derived,atol=1e-9))
checks['position_on_encoder_grid']=bool(np.allclose(q/(2*np.pi/4096),np.rint(q/(2*np.pi/4096)),atol=1e-5))
checks['no_base_commands']=all(np.linalg.norm(x['command'])==0 for x in r)
checks['no_false_live_freshness_claim']=all(x['arm_feedback_live_freshness_verified'] is False for x in r)
checks['response_preserved_in_sim_log']=all(x['arm_feedback_stamp_ns']==y['arm_feedback_stamp_ns'] and x['arm_measured_position']==y['arm_measured_position'] for x,y in zip(r,sim))
checks={k:bool(v) for k,v in checks.items()}
result={'run':a.run.name,'frames':len(r),'checks':checks,'passed':all(checks.values()),'last_position_rad':q[-1].tolist(),'last_velocity_rad_s':v[-1].tolist(),'max_target_measured_difference_rad':float(np.max(np.abs(np.array([x['arm_target'] for x in r])-q))),'scope':'Completed stationary sensor diagnostic. Packet-clock consistency and finite-difference telemetry only; no independent wall-clock freshness guarantee, no learned IK motion or grasp execution.'}
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['passed']
