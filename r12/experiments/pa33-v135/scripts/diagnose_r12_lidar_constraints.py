"""Offline sensor-only reconstruction of radial lidar guard constraints."""
import argparse,json
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon,Point
p=argparse.ArgumentParser();p.add_argument('report');p.add_argument('output');args=p.parse_args()
root=Path(__file__).resolve().parents[1];report=Path(args.report)
rows=json.loads((report/'sim_results.json').read_text());row=next(r for r in reversed(rows) if r.get('diagnostic_lidar_scans'))
extrinsics=json.loads((report/'lidar_extrinsics.json').read_text());vertices=np.array(json.loads((root/'configs/sensors/transport_footprint.json').read_text())['vertices']);poly=Polygon(vertices);points=[];sources=[]
for name,ranges in row['diagnostic_lidar_scans'].items():
 r=np.array(ranges);angles=np.linspace(-np.pi/2,np.pi/2,len(r));local=np.column_stack([r*np.cos(angles),r*np.sin(angles),np.zeros(len(r))]);t=np.array(extrinsics[name]);world=local@t[:3,:3].T+t[:3,3]
 for i in np.where(r<5.95)[0]:points.append(world[i,:2]);sources.append([name,int(i),float(r[i])])
points=np.array(points);distance=np.linalg.norm(points,axis=1);normal=points/distance[:,None];support=np.max(normal@vertices.T,axis=1);gap=distance-support
order=np.argsort(gap)[:12];out={'frame':row['frame'],'estimated_pose':row.get('estimated_map_pose'),'command':row.get('command'),'guard':row.get('lidar_guard'),'smallest_radial_constraints':[{'sensor':sources[i][0],'beam':sources[i][1],'range_m':sources[i][2],'point_body':points[i].tolist(),'radial_support_gap_m':float(gap[i]),'distance_to_padded_polygon_m':float(poly.distance(Point(points[i]))),'inside_padded_polygon':bool(poly.covers(Point(points[i])))} for i in order],'caveat':'Observed lidar points only; padded polygon distance differs from radial support approximation. No obstacle labels or fixture truth. This does not authorize bypassing continuous swept collision checks.'};Path(args.output).write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
