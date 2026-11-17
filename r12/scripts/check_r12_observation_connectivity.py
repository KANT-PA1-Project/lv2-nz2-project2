"""Conservative fixed-yaw padded CAD connectivity on recorded Nav2 costmap."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_erosion,label
from shapely.geometry import Polygon
from shapely import intersects_xy
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('candidates',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
g=np.load(a.run/'costmap.npz');res=float(g['resolution']);origin=g['origin'];occ=(g['data']>=100)|(g['data']<0)
f=json.loads((root/'configs/sensors/transport_footprint.json').read_text());shape=Polygon(f['vertices']).buffer(res/np.sqrt(2),join_style=2);n=int(np.ceil(np.abs(np.array(shape.exterior.coords)).max()/res));xx,yy=np.meshgrid(np.arange(-n,n+1)*res,np.arange(-n,n+1)*res);kernel=intersects_xy(shape,-xx,-yy)
free=binary_erosion(~occ,structure=kernel,border_value=0);components,_=label(free)
pose=json.loads((a.run/'ros_results.json').read_text())[-1]['estimated_map_pose'];start=np.floor((np.array(pose[:2])-origin)/res).astype(int);start_id=int(components[start[1],start[0]])
results=[]
for i,c in enumerate(json.loads(a.candidates.read_text())['candidates']):
 cell=np.floor((np.array(c['base_xy'])-origin)/res).astype(int);cid=int(components[cell[1],cell[0]]) if ((cell>=0).all() and cell[0]<free.shape[1] and cell[1]<free.shape[0]) else 0
 results.append({'candidate_index':i,'component':cid,'connected':bool(cid and cid==start_id),'base_xy':c['base_xy']})
result=dict(start_component=start_id,free_cells=int(free.sum()),results=results,scope='Fixed yawpi, folded4cmCAD plus cell-extent guard,4-connected grid. Recorded global costmap includes configured static map; not sensor-only map. No dynamic/arm path proof.')
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
