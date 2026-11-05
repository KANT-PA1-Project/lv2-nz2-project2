"""Loopback zero-command receiver only. No ROS, estimator, decoder or mission admission."""
import socket,json,os,time
from pathlib import Path
out=Path(os.environ['FLEET_RUN_DIR']);rows=[]
with socket.socket()as s:
 s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('127.0.0.1',20033));s.listen(1);s.settimeout(240)
 (out/'sink_ready.json').write_text(json.dumps({'port':20033,'pid':os.getpid(),'ROS_started':False,'perception_started':False}))
 conn,_=s.accept()
 with conn,conn.makefile('r')as f:
  for line in f:
   packet=json.loads(line);native=packet['native_capture_diagnostic'];assert packet['legacy_image_publication_disabled']and native['observation_admitted']is False
   assert packet['perception']=={} and not any(packet['perception_capabilities'].values())
   timeline=packet['native_packet_timeline']
   assert packet['clock_domain']=='native_physics_960hz' and packet['stamp_ns']==timeline['current_physics_step']*10**9//960
   assert timeline['capture_physics_step']+16==timeline['current_physics_step']
   assert not timeline['observation_admitted'] and not timeline['motion_authorized']
   assert all(x['native_samples'][-1]['physics_step']==timeline['current_physics_step'] for x in packet['imu'])
   rows.append({'native_packet_timeline':timeline,'all_IMU_samples':packet['imu'],'current_feedback':native['current_feedback'],'received_ns':time.monotonic_ns(),'run_id':packet['capture_run_id'],'native_capture':native,'native_packet_stamp_ns':packet['stamp_ns'],'IMU_valid':[x['valid']for x in packet['imu']],'lidar_count':len(packet['scans']),'encoder_joint_count':len(packet['names']),'velocity_valid':packet['velocity_valid'],'perception_not_attempted':True})
   conn.sendall((json.dumps({'command':[0,0,0],'wheel_action':[0,0,0,0],'wheel_scale':1,'diagnostic_receiver':True})+'\n').encode())
   (out/'stationary_received_packets.json').write_text(json.dumps(rows,indent=2))
(out/'sink_exit.json').write_text(json.dumps({'exit':0,'received_packets':len(rows),'observation_admitted':False,'motion_command':[0,0,0]}))
