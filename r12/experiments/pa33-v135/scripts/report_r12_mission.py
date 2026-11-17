"""Evaluate independent simulator truth against sensor-only mission control."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--run',default='sensors');p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rows=json.loads((root/'reports/local'/args.run/'sim_results.json').read_text())
valid=[r for r in rows if r['estimated_odom_pose'] is not None]
actual=np.array([r['actual_base'][:2] for r in valid]);estimated=np.array([r.get('estimated_map_pose',[.734-r['estimated_odom_pose'][0],.950-r['estimated_odom_pose'][1]])[:2] for r in valid])
error=np.linalg.norm(actual-estimated,axis=1)
contacts=json.loads((root/'reports/local'/args.run/'contacts.json').read_text())
last=rows[-1]
summary={'run':args.run,'actuator_policy':last.get('actuator_policy','bc_baseline'),'frames':len(rows),'simulation_seconds':len(rows)*.05,'total_goals':last.get('total_goals',10),'completed':last['completed'],'mission_finished':last['finished'],'robot_environment_contact_reports':len(contacts),'tracking_success_fraction':float(np.mean([r['tracking']==1 for r in rows])),'dual_imu_valid_fraction_after_warmup':float(np.mean([r['imu_fused'] for r in rows[1:]])),'position_error_rmse_m':float(np.sqrt(np.mean(error**2))),'position_error_max_m':float(error.max()),'final_actual_xy':actual[-1].tolist(),'final_estimated_xy':estimated[-1].tolist(),'nvblox_known_cells':last['nvblox'].get('known_cells',0),'pass_navigation':bool(last['finished'] and not contacts and error.max()<.15),'limitations':['Navigation waypoint audit only; no signal interpretation, grasp, puck sorting, or physical robot validation.','Bambu carton dimensions and map/PDF registration are provisional.','Ideal rendered depth; simulator instance IDs remove robot self-occlusion only.','Known CAD occupancy prior plus live nvBlox and four lidar obstacle layers.','Two simulated OpenCR IMUs use nominal extrinsics, not measured hardware calibration.']}
route=json.loads((root/last.get('route_file','models/navigation/routes.json')).read_text())
goal_errors=[]
for item in last['completed']:
    index=int(round((item['stamp_ns']-1_000_000_000)/50_000_000))
    if 0<=index<len(rows):
        goal_errors.append({'station':item['name'],'actual_error_m':float(np.linalg.norm(np.array(rows[index]['actual_base'][:2])-route['points'][item['name']])),'actual_yaw_error_deg':None if 'actual_rpy_deg' not in rows[index] else float(abs((rows[index]['actual_rpy_deg'][2]-180+180)%360-180))})
summary['goal_actual_errors']=goal_errors
summary['max_goal_actual_error_m']=max((x['actual_error_m'] for x in goal_errors),default=None)
tilt=[max(abs(x) for x in r['actual_rpy_deg'][:2]) for r in rows if 'actual_rpy_deg' in r]
summary['max_tilt_deg']=max(tilt) if tilt else None
summary['precision_and_tilt_pass']=bool(last['finished'] and len(goal_errors)==last.get('total_goals',10) and not contacts and summary['max_goal_actual_error_m']<=.03 and tilt and max(tilt)<4 and all(x['actual_yaw_error_deg'] is not None and x['actual_yaw_error_deg']<3 for x in goal_errors))
return_rows=rows[-61:]
contained=[]
for r in return_rows:
    if 'actual_rpy_deg' not in r:continue
    yaw=np.deg2rad(r['actual_rpy_deg'][2]);c,s=np.cos(yaw),np.sin(yaw)
    corners=np.array([[-.2,-.2],[-.2,.2],[.2,-.2],[.2,.2]])@np.array([[c,s],[-s,c]])+r['actual_base'][:2]
    contained.append(bool((corners[:,0]>=.434).all() and (corners[:,0]<=1.034).all() and (corners[:,1]>=.650).all() and (corners[:,1]<=1.250).all()))
summary['return_3s_chassis_inside_provisional_start_region']=all(contained) if len(contained)==61 and last['finished'] else None
summary['precision_criteria']={'actual_goal_yaw_error_deg':3,'actual_goal_error_m':.03,'max_tilt_deg':4,'environment_contacts':0,'scope':'Staging navigation only, no grasp TCP/payload certification'}
(args.output/'R12_mission_validation.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
d=np.load(root/'models/navigation/maze_grid.npz');res=float(d['resolution']);occ=d['occupied']
fig,ax=plt.subplots(figsize=(7,9));ax.imshow(occ,origin='lower',extent=[0,occ.shape[1]*res,0,occ.shape[0]*res],cmap='Greys',alpha=.65);ax.plot(actual[:,0],actual[:,1],label='PhysX evaluator');ax.plot(estimated[:,0],estimated[:,1],'--',label='Camera + 2 IMUs + 8 encoders + 4 lidars');ax.scatter([.734],[.950],marker='*',s=100,label='Start');ax.set(xlabel='x (m)',ylabel='y (m)',title=f"R12 navigation: {len(last['completed'])}/{last.get('total_goals',10)} waypoints; contacts={len(contacts)}");ax.legend();ax.set_aspect('equal');fig.tight_layout();fig.savefig(args.output/'R12_mission_trajectory.png',dpi=180);plt.close(fig)
print(json.dumps(summary,ensure_ascii=False))
