"""Offline replay of sensor-estimated poses; no simulation truth in veto."""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent/'sensor_runtime'))
from swept_footprint import motion_clear
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--output',required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[1];run=Path(a.run)
rows=json.loads((run/'ros_results.json').read_text())
grid=np.load(root/'models/navigation/maze_grid.npz')
hull=json.loads((root/'configs/sensors/transport_footprint.json').read_text())['vertices']
results=[];durations=[]
for i,r in enumerate(rows):
 pose=r.get('estimated_map_pose');cmd=r['command']
 if pose is None or not r.get('transport_posture_ok'):continue
 t=time.perf_counter();clear=motion_clear(pose[:2],pose[2],cmd,hull,grid['occupied'],float(grid['resolution']));durations.append((time.perf_counter()-t)*1000)
 results.append({'frame':i,'clear':bool(clear),'moving':bool(np.linalg.norm(cmd)>1e-6),'pose':pose,'command':cmd})
blocked=[r for r in results if not r['clear'] and r['moving']]
s={'scope':'Offline sensor-pose/command replay, static occupancy from runtime maze_grid.npz; not counterfactual trajectory or dynamic acceptance','samples':len(results),'blocked_moving':len(blocked),'first_blocked_moving':blocked[0] if blocked else None,'latency_ms':{'median':float(np.median(durations)),'p95':float(np.percentile(durations,95)),'max':max(durations)},'precontact_740_764':[r for r in results if 740<=r['frame']<=764]}
Path(a.output).write_text(json.dumps(s,indent=2));print(json.dumps({k:v for k,v in s.items() if k!='precontact_740_764'},indent=2))
