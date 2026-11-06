"""Offline observed-lidar candidate docking region, not a grasp feasibility test."""
import json,argparse
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('output');args=p.parse_args();root=Path(__file__).resolve().parents[1];report=Path(args.report)
rows=json.loads((report/'sim_results.json').read_text());row=next(r for r in reversed(rows) if r.get('diagnostic_lidar_scans'));pose=np.array(row['estimated_map_pose']);goal=np.array(json.loads((report/'route.json').read_text())['points']['S6']);vertices=np.array(json.loads((root/'configs/sensors/transport_footprint.json').read_text())['vertices']);ex=json.loads((report/'lidar_extrinsics.json').read_text());points=[]
for name,ranges in row['diagnostic_lidar_scans'].items():
 r=np.array(ranges);a=np.linspace(-np.pi/2,np.pi/2,90);t=np.array(ex[name]);pts=np.c_[r*np.cos(a),r*np.sin(a),np.zeros(90)]@t[:3,:3].T+t[:3,3];points.extend(pts[r<5.95,:2])
c,s=np.cos(pose[2]),np.sin(pose[2]);world=np.array(points)@np.array([[c,s],[-s,c]])+pose[:2]
mapdata=np.load(root/'models/navigation/maze_grid.npz');occupied=mapdata['occupied'].astype(bool);res=float(mapdata['resolution']);hull=ConvexHull(vertices);gx,gy=np.meshgrid(np.arange(vertices[:,0].min(),vertices[:,0].max()+res/2,res/2),np.arange(vertices[:,1].min(),vertices[:,1].max()+res/2,res/2));samples=np.c_[gx.ravel(),gy.ravel()];samples=samples[(samples@hull.equations[:,:2].T+hull.equations[:,2]<=1e-9).all(1)];samples=np.vstack([samples,vertices]);candidates=[]
for dx in np.arange(-.15,.151,.005):
 for dy in np.arange(-.15,.151,.005):
  xy=goal+[dx,dy];body=-(world-xy);dist=np.linalg.norm(body,axis=1);normal=body/np.maximum(dist[:,None],1e-9);gap=dist-np.max(normal@vertices.T,axis=1)
  if gap.min()<.015:continue
  cells=np.floor((xy-samples)/res).astype(int)
  if (cells<0).any() or (cells[:,0]>=occupied.shape[1]).any() or (cells[:,1]>=occupied.shape[0]).any() or occupied[cells[:,1],cells[:,0]].any():continue
  candidates.append({'xy':xy.tolist(),'offset_m':float(np.hypot(dx,dy)),'min_observed_support_gap_m':float(gap.min())})
candidates.sort(key=lambda a:a['offset_m']);out={'frame':row['frame'],'station':'S6','original_goal':goal.tolist(),'candidate_count':len(candidates),'nearest_candidates':candidates[:5],'scope':'Offline snapshot only, 4cm padded CAD unchanged plus15mm radial guard reserve; static map footprint sampled. No approach path, arm reach, multi-view persistence or full mission validation yet.'};Path(args.output).write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
