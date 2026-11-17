"""Short encoder/dual-IMU/lidar guarded drive; not a Nav2 mission acceptance.

No model/world pose subscription. Gazebo force actuator owns command expiry.
The gripper is never commanded; its open target belongs to the loaded scene.
"""
import argparse, json, sys, threading, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu, LaserScan
from tf2_ros import Buffer, TransformListener
from gz.transport import Node
from gz.msgs.double_pb2 import Double
from gz.msgs.double_v_pb2 import Double_V

repo=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(repo/'src'),'/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/turtlebot_swerve']
from arm_practice.transport_posture import TransportPosture
from arm_practice.latest_policy import allocate
from arm_practice.swerve import wheel_odometry
from arm_practice.motion_envelope import MotionEnvelope
from arm_practice.lidar_guard import constrain_velocity
from arm_practice.kinematics import forward_transform
from dual_imu import DualImuFusion,Sample

def stamp(m):return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--seconds',type=float,default=180);ap.add_argument('--fold-only',action='store_true');args=ap.parse_args()
 out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 rclpy.init();node=rclpy.create_node('r12_gazebo_guarded_drive');lock=threading.Lock();latest={};imus=[{},{}];subs=[]
 def store(key,m):
  with lock:latest[key]=(m,time.monotonic())
 def imu_cb(i,m):
  with lock:
   imus[i][stamp(m)]=(m,time.monotonic())
   while len(imus[i])>20:del imus[i][min(imus[i])]
 subs.append(node.create_subscription(JointState,'/r12/joint_states',lambda m:store('joint',m),qos_profile_sensor_data))
 lids=['front_right','rear_left','low_front_left','low_rear_right']
 for name in lids:subs.append(node.create_subscription(LaserScan,'/r12/lidar/'+name,lambda m,n=name:store(n,m),qos_profile_sensor_data))
 for i in range(2):subs.append(node.create_subscription(Imu,f'/opencr_{i+1}/imu',lambda m,i=i:imu_cb(i,m),qos_profile_sensor_data))
 tfbuf=Buffer();listener=TransformListener(tfbuf,node)
 executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node);thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
 gz=Node();drive=gz.advertise('/r12/gz/drive_targets',Double_V)
 arms=[gz.advertise(f'/r12/gz/command/arm_joint{i}',Double) for i in range(1,6)]
 config=json.loads((repo/'configs/r12_transport.json').read_text());posture=TransportPosture(config)
 fusion=DualImuFusion([[.057,0,.16597],[-.057,0,.16597]],[[0,0,0,1],[0,0,1,0]])
 envelope=MotionEnvelope(speed=.05,acceleration=.05,jerk=.2)
 extrinsics={name:forward_transform(repo/'models/r12/r12_arm.urdf','lidar_'+name,{}) for name in lids}
 footprint=json.loads((repo/'configs/sensors/transport_footprint.json').read_text())['vertices']
 phase='await_sensors';reason=None;previous=None;last_stamp=None;last_live=time.monotonic();start=time.monotonic();origin=None;odom=np.zeros(3);drive_since=None;stop_since=None;events=[];last_note=None;max_tilt=0.;max_speed=0.;rows=0;tf_verified=False
 def publish(steer,wheels):
  msg=Double_V();msg.data.extend([*map(float,steer),*map(float,wheels)]);drive.publish(msg)
 def zero():publish(np.zeros(4) if previous is None else previous,np.zeros(4))
 try:
  with (out/'samples.jsonl').open('w') as log:
   while time.monotonic()-start<args.seconds and not (out/'STOP').exists():
    with lock:
     snapshot=dict(latest);common=set(imus[0])&set(imus[1]);ns=max(common) if common else None
     pair=[imus[i][ns] for i in range(2)] if ns is not None else []
    if 'joint' not in snapshot or any(n not in snapshot for n in lids) or ns is None:
     zero();time.sleep(.02);continue
    if last_stamp==ns:
     if time.monotonic()-last_live>.5:zero()
     time.sleep(.005);continue
    dt=.01 if last_stamp is None else (ns-last_stamp)*1e-9;last_stamp=ns
    if not 0<dt<=.1:reason='imu_time_gap';zero();break
    last_live=time.monotonic();jm,jwall=snapshot['joint'];positions=dict(zip(jm.name,jm.position));velocities=dict(zip(jm.name,jm.velocity))
    an=[f'arm_joint{i}' for i in range(1,6)];modules=['lh1','rh1','lh2','rh2'];sn=[x+'_steer_joint' for x in modules];wn=[x+'_wheel_joint' for x in modules]
    if not all(n in positions and n in velocities for n in an+sn+wn):reason='missing_joint';zero();break
    q=np.array([positions[n] for n in an]);v=np.array([velocities[n] for n in an]);steer=np.array([positions[n] for n in sn]);wheel=np.array([velocities[n] for n in wn])
    if previous is None:previous=steer.copy()
    fresh=(time.monotonic()-jwall<.5 and abs(stamp(jm)-ns)<100_000_000 and all(time.monotonic()-t<.5 for _,t in pair))
    scans={}
    for name in lids:
     scan,wall=snapshot[name];ranges=np.array(scan.ranges);valid=not np.isnan(ranges).any() and not (ranges<0).any()
     fresh &= valid and time.monotonic()-wall<2.5 and abs(stamp(scan)-ns)<200_000_000
     # Actual360-degree Gazebo rays are resampled into the guard's front90 rays.
     angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
     scans[name]=np.interp(np.linspace(-np.pi/2,np.pi/2,90),angles,np.minimum(ranges,6.))
    if not fresh:zero();reason='stale_or_invalid_sensor';break
    samples=[]
    for im,_ in pair:
     samples.append(Sample(ns*1e-9,np.array([im.angular_velocity.x,im.angular_velocity.y,im.angular_velocity.z]),np.array([im.linear_acceleration.x,im.linear_acceleration.y,im.linear_acceleration.z]),np.eye(3)*.0004,np.eye(3)*.04))
    fused=fusion.update(samples)
    if fused is None:zero();continue
    twist,residual=wheel_odometry(steer,wheel)
    yawrate=fused['gyro'][2];yaw=odom[2];odom[:2]+=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])@twist[:2]*dt;odom[2]+=yawrate*dt
    stopped=np.max(np.abs(wheel))<.2 and abs(yawrate)<.05
    fold=posture.step(ns*1e-9,q,v,stopped,valid=True)
    if fold is not None:
     for pub,value in zip(arms,fold):pub.publish(Double(data=float(value)))
    # Recheck velocity even after TransportPosture has latched readiness.
    transport_ok=posture.ready and np.max(np.abs(v))<.02 and np.max(np.abs(q-posture.goal))<=posture.tolerance
    if not tf_verified:
     try:
      evidence={}
      for name in ['gripper_left','gripper_right']:
       tr=tfbuf.lookup_transform('base_link',name,rclpy.time.Time());t=tr.transform.translation;rot=tr.transform.rotation
       actual=np.array([t.x,t.y,t.z]);fk=forward_transform(repo/'models/r12/r12_mobile_gripper.urdf',name,positions)
       error=float(np.linalg.norm(actual-fk[:3,3]))
       if error>.002:raise ValueError('TF/FK translation mismatch')
       evidence[name]={'parent_chain':'base_link -> arm -> arm_link5 -> '+name,'translation_error_m':error,'quaternion_xyzw':[rot.x,rot.y,rot.z,rot.w]}
      (out/'gripper_tf_check.json').write_text(json.dumps(evidence,indent=2));tf_verified=True
     except Exception:pass
    target=np.zeros(3)
    if phase=='await_sensors':phase='fold_arm'
    if phase=='fold_arm' and transport_ok:phase='fold_complete' if args.fold_only else 'align_steering'
    if phase=='fold_complete':reason='fold_and_settle_verified';zero();break
    if phase in ['align_steering','drive'] and not transport_ok:reason='transport_posture_lost';zero();break
    if phase=='align_steering':
     previous,_=allocate([.05,0,0],steer,previous,np.zeros(4),dt=dt,steer_rate=.5)
     publish(previous,np.zeros(4))
     if np.max(np.abs(steer-np.array([np.pi/2,np.pi/2,0,0])))<.05 and np.max(np.abs([velocities[n] for n in sn]))<.05:
      phase='drive';drive_since=ns;origin=odom.copy()
    if phase=='drive':
     if np.linalg.norm(odom[:2]-origin[:2])>=.08 or (ns-drive_since)*1e-9>=3.:
      phase='brake';stop_since=ns
     else:target=[.05,0,0]
    command,motion=envelope.step(target,fused['gyro'],fused['accel'],dt,valid=True)
    tilt=float(np.max(np.abs(motion['tilt_deg'])));max_tilt=max(max_tilt,tilt)
    if tilt>=4 or residual>.03 or abs(yawrate)>.10:reason='tilt_encoder_or_yaw_safety_stop';zero();break
    limited,guard=constrain_velocity(command,scans,extrinsics,footprint=footprint)
    if phase=='drive' and np.linalg.norm(limited-command)>.005:reason='lidar_clearance_stop';zero();break
    if phase=='drive':
     previous,ws=allocate(limited,steer,previous,np.zeros(4),dt=dt,steer_rate=.5)
     publish(previous,ws)
    elif phase!='align_steering':zero()
    if phase=='brake' and stopped and (ns-stop_since)*1e-9>=.5:reason='short_drive_and_stop_complete';zero();break
    max_speed=max(max_speed,float(np.linalg.norm(twist[:2])))
    row=dict(stamp_ns=ns,phase=phase,arm_position=q.tolist(),arm_velocity=v.tolist(),transport_state=posture.state,transport_ready=transport_ok,steer=steer.tolist(),wheel=wheel.tolist(),body_twist_encoder=twist.tolist(),odom_encoder_gyro=odom.tolist(),tilt_deg=tilt,gyro_z=float(yawrate),command=list(command),guard=guard)
    log.write(json.dumps(row)+'\n');rows+=1
    if phase!=last_note:
     print(json.dumps(row),flush=True);events.append({'phase':phase,'stamp_ns':ns});last_note=phase
    if rows%30==0:log.flush();(out/'progress.json').write_text(json.dumps(row,indent=2))
   else:reason='wall_timeout_or_user_stop'
 except Exception as error:
  reason=type(error).__name__+': '+str(error)
  raise
 finally:
  for _ in range(15):zero();time.sleep(.02)
  executor.shutdown(timeout_sec=2);thread.join(timeout=2);node.destroy_node();rclpy.shutdown()
  result=dict(reason=reason,phase=phase,wall_s=time.monotonic()-start,samples=rows,phase_events=events,estimated_travel_m=0. if origin is None else float(np.linalg.norm(odom[:2]-origin[:2])),max_encoder_speed_m_s=max_speed,max_imu_tilt_deg=max_tilt,gripper_tf_verified=tf_verified,controller='bounded deterministic allocator; no PPO replacement',localization='wheel encoders + dualIMU; no simulator pose input',cuVSLAM_Nav2_nvblox_connected=False,mission_success=False,performance_accepted=False)
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)

if __name__=='__main__':main()
