"""Offline sensor-command scenario audit; future logged inputs are NOT control inputs."""
import argparse,json,time
from pathlib import Path
import numpy as np
from arm_practice.heading_feasibility import choose_twist_heading,choose_transition_heading,heading_transition_margin,twist_steering_margin
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('output');a=p.parse_args()
rows=json.loads((Path(a.run)/'sim_results.json').read_text());samples=[]
for i in range(0,len(rows)-20,5):
 r=rows[i];pose=r.get('estimated_map_pose')
 if pose is None or not r.get('transport_posture_ok') or np.linalg.norm(r['command'][:2])<.05:continue
 velocities=[]
 for x in rows[i:i+21:5]:
  h=x['estimated_map_pose'][2];c,s=np.cos(h),np.sin(h)
  velocities.append(np.array([[c,-s],[s,c]])@np.array(x['command'][:2]))
 h=pose[2];t=time.perf_counter();end,score=choose_twist_heading(velocities,h);proposal_ms=(time.perf_counter()-t)*1000
 transition=heading_transition_margin(velocities,h,end)
 connected,_=choose_transition_heading(velocities,h);connected_margin=heading_transition_margin(velocities,h,connected)
 samples.append({'frame':i,'current_heading':h,'connected_candidate':connected,'connected_transition_margin':connected_margin,'candidate':end,'endpoint_margin':score,'transition_margin':transition,'current_margin':twist_steering_margin(velocities,h,[-.2,0,.2]),'proposal_ms':proposal_ms})
result={'samples':len(samples),'endpoint_above_0_1_but_transition_below_0_06':sum(x['endpoint_margin']>.1 and x['transition_margin']<.06 for x in samples),'connected_margin_violations':sum(x['connected_transition_margin']<min(.06,x['current_margin'])-1e-9 for x in samples),'connected_holds':sum(abs(x['connected_candidate']-x['current_heading'])<1e-9 for x in samples),'proposal_latency_p95_ms':float(np.percentile([x['proposal_ms'] for x in samples],95)),'rows':samples,'scope':'Offline future sensor-command scenarios in m/s; not recorded paths, not control inputs, no physics or collision certification.'}
Path(a.output).write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='rows'})
