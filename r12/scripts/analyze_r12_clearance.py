"""Conservative fixed-heading connectivity sweep; not a dynamic clearance certificate."""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_erosion, label
from shapely.geometry import Polygon
from shapely import intersects_xy
root=Path(__file__).resolve().parents[1]
f=json.loads((root/'reports/local/clearance_optimization/original_footprint.json').read_text())
g=np.load(root/'models/navigation/maze_grid.npz');occ=g['occupied'].astype(bool);res=float(g['resolution'])
points=json.loads((root/'models/navigation/routes.json').read_text())['points']
base=Polygon(f['vertices']);out=[]
for margin in np.arange(.03,.201,.005):
 # Start from existing3cm padded CAD hull; conservatively cover cell extent.
 shape=base.buffer(float(margin)-f['padding_m']+res/np.sqrt(2),join_style=2)
 coords=np.asarray(shape.exterior.coords);n=int(np.ceil(np.abs(coords).max()/res));xx,yy=np.meshgrid(np.arange(-n,n+1)*res,np.arange(-n,n+1)*res)
 # Current runtime keeps yaw pi. Inverse rotate grid offsets into body coordinates.
 kernel=intersects_xy(shape,-xx,-yy)
 free=binary_erosion(~occ,structure=kernel,border_value=0)
 components,count=label(free) #4-connected paths avoid diagonal corner cutting
 ids={k:int(components[int(v[1]/res),int(v[0]/res)]) for k,v in points.items()}
 connected=ids['START']!=0 and len(set(ids.values()))==1
 out.append({'margin_m':round(float(margin),3),'all_stations_connected':connected,'station_components':ids,'free_cells':int(free.sum()),'component_count':int(count)})
report={'resolution_m':res,'heading_rad':float(np.pi),'cell_extent_guard_m':res/np.sqrt(2),'source_padding_m':f['padding_m'],'results':out,'limitations':'Static2D grid, fixed folded CAD posture and yaw pi; all9 named station centers must connect. No guarantee for unmodeled height obstacles, localization error, steering dynamics or arbitrary yaw.'}
p=root/'reports/local/clearance_optimization/sweep.json';p.write_text(json.dumps(report,indent=2));print(json.dumps(out))
