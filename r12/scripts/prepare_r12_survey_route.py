import sys,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.navigation import Grid
grid=Grid(root/'models/navigation/maze_grid.npz');cfg=json.loads((root/'models/navigation/mission_navigation.json').read_text())
points={'START':cfg['start']}
for name,center in list(zip([f'S{i+1}' for i in range(6)],cfg['starts']))+list(zip(['L1','L2','L3'],cfg['destinations'])):
 # Staging pose only, not a proven manipulator reach/grasp pose.
 options=[np.array(center)+.55*np.array([np.cos(a),np.sin(a)]) for a in np.linspace(0,2*np.pi,16,endpoint=False)]
 for p in options:
  if grid.valid(p):
   try:grid.path(cfg['start'],p)
   except ValueError:continue
   points[name]=p.tolist();break
 if name not in points:raise RuntimeError(('no staging point',name))
route_order=['START','S1','S2','S3','S4','S5','S6','L1','L2','L3','START']
routes=[]
for a,b in zip(route_order,route_order[1:]):routes.append({'from':a,'to':b,'path':grid.path(points[a],points[b]).tolist()})
(root/'models/navigation/survey_route.json').write_text(json.dumps({'points':points,'routes':routes,'scope':'Visit all six source staging poses and three destination staging poses, return. Not puck transport schedule.'},indent=2))
