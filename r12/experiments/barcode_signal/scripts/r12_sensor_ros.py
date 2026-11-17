#!/usr/bin/env python3
"""Sensor transport audit: real rendered images to cuVSLAM, all encoder and dual IMU fusion."""
import json,time,socket,sys,os,base64,numpy as np
from rclpy.serialization import serialize_message
from copy import deepcopy
from functools import wraps
from pathlib import Path
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,Imu,JointState,LaserScan,PointCloud2,PointField
from std_msgs.msg import String,Float64
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from nvblox_msgs.msg import DistanceMapSlice
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
def atomic_json(path,data):
 temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(json.dumps(data,separators=(',',':')));temporary.replace(path)
root=Path(__file__).resolve().parents[1];out=root/'reports/local'/os.getenv('R12_SENSOR_RUN','sensors');sys.path.insert(0,str(root/'src'));sys.path.insert(0,str(root/'scripts/sensor_runtime'))
from command_send_freshness import encode_base_response
from dual_imu import DualImuFusion,Sample
from colored_cloud import project as project_cloud,pack_rgb
from arm_practice.isaac_pixel_geometry import camera_matrix,projection_matrix,require_archive_geometry
from arm_practice.swerve import wheel_odometry
from arm_practice.sensor_sync import wait_visual_pair
from arm_practice.async_path_smoother import AsyncPathSmoother
from arm_practice.publication_diagnostics import PublicationDiagnostics
from arm_practice.result_journal import ResultJournal
result_journal=ResultJournal(out/'ros_results.json') if os.getenv('R12_INCREMENTAL_RESULT_LOGS','0')=='1' else None
stationary_capture=os.getenv("R12_STATIONARY_CAPTURE","0")=="1"
if stationary_capture and any(os.getenv(k,"0")=="1" for k in ("R12_MISSION","R12_SENSOR_MOTION")):raise ValueError("Stationary capture forbids mission/motion")
publication_diagnostics=PublicationDiagnostics(out,buffered=os.getenv("R12_BUFFER_PUBLICATION_DIAGNOSTICS","0")=="1") if os.getenv("R12_PUBLICATION_DIAGNOSTICS","0")=="1" else None
from arm_practice.perception_response_identity import check_packet_perception
from arm_practice.signal_observation import checked_signal_color
from arm_practice.tower_signal import tower_signal_color,signal_frame_fresh
from arm_practice.arm_feedback_record import arm_feedback_record
from arm_practice.base_feedback_record import base_feedback_record
from arm_practice.motion_envelope import MotionEnvelope
from arm_practice.lidar_guard import constrain_velocity
from arm_practice.scan_localizer import ScanLocalizer
from arm_practice.swerve_rl import ResidualController
from arm_practice.latest_policy import LatestPolicy
from arm_practice.arm_command_gate import ArmCommandGate
from arm_practice.observation_posture import ObservationPosture
from arm_practice.wheel_position_hold import WheelPositionHold
from arm_practice.planner_stop_hold import planner_stop_hold_allowed
from arm_practice.mission import SignalRecorder,plan
from path_follower import follow
from swept_footprint import motion_clear
from path_smoothing import smooth_path
from arm_practice.transport_posture import TransportPosture
from arm_practice.stationary_observation_dwell import StationaryObservationDwell
from arm_practice.goal_stop_hold import GoalStopHold,goal_arrival_ready
from arm_practice.wrist_identity import WristIdentity
rclpy.init();node=Node('r12_sensor_audit',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
tf=node.create_publisher(TFMessage,'/tf_static',qos);map_tf=node.create_publisher(TFMessage,'/tf',10);clock=node.create_publisher(Clock,'/clock',10);joint_pub=node.create_publisher(JointState,'/r12/joint_states',10)
imu_pub=[node.create_publisher(Imu,f'/opencr_{i+1}/imu',10) for i in range(2)];fused_pub=node.create_publisher(Imu,'/imu/base/data_raw',10);odom_pub=node.create_publisher(Odometry,'/wheel/odometry',10)
images={side:node.create_publisher(Image,f'/r12/stereo/{side}/image',10) for side in ['left','right']};infos={side:node.create_publisher(CameraInfo,f'/r12/stereo/{side}/camera_info',10) for side in images};depth_pub=node.create_publisher(Image,'/r12/stereo/left/depth',10)
extra_publishers={}
for camera in ['r07_camera2','r07_camera3','arm_camera']:
 for side in ['left','right']:
  key=camera+'_'+side+'_optical';base='/r12/'+camera+'/'+side;extra_publishers[key]=(node.create_publisher(Image,base+'/image',3),node.create_publisher(CameraInfo,base+'/camera_info',3),node.create_publisher(Image,base+'/depth',3) if side=='left' else None)
cloud_pub=node.create_publisher(PointCloud2,'/r12/points',2);rgb_cloud_pub=node.create_publisher(PointCloud2,'/r12/points_rgb',2)
cloud_publication_audit=[];cloud_audit_enabled=os.getenv('R12_CLOUD_PUBLICATION_AUDIT','0')=='1'
map_pub=node.create_publisher(OccupancyGrid,'/map',qos)
map_data=np.load(root/'models/navigation/maze_grid.npz');grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=float(map_data['resolution']);grid.info.height,grid.info.width=map_data['occupied'].shape;grid.info.origin.orientation.w=1.;grid.data=(map_data['occupied'].astype(int)*100).ravel().tolist();map_pub.publish(grid)
scan_pubs={n:node.create_publisher(LaserScan,'/r12/lidar/'+n,10) for n in ['front_right','rear_left','low_front_left','low_rear_right']}
client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
route_file=os.getenv('R12_ROUTE_FILE','models/navigation/routes.json');route_data=json.loads((root/route_file).read_text())
goal_metadata=route_data.get('steps',[{'station':r['to'],'phase':'coverage_approach'} for r in route_data.get('routes',[])])
goals=[(r['station'],route_data['points'][r['station']]) for r in goal_metadata]
perception_required=os.getenv('R12_MISSION_PERCEPTION','1')=='1'
perception_max_age_ns=int(float(os.getenv('R12_PERCEPTION_MAX_AGE_S','0.5'))*1e9)
if perception_max_age_ns<=0:raise ValueError('Invalid perception maximum age')
# tower_signal 판정은 프레임 영상(data)이 필요한데, 수신 즉시 기록(R12_SIGNAL_AT_RECEIVE=1) 시점에는 영상이 아직 없음.
# 생산자는 이제 색을 UNKNOWN 으로 보내므로 그 경로를 켜면 기록기가 오류로 멈춤 → 조용히 우회하지 않고 시작 시 거부
if os.getenv('R12_SIGNAL_AT_RECEIVE','0')=='1':raise SystemExit('R12_SIGNAL_AT_RECEIVE=1 은 tower_signal 신호등 판정과 함께 쓸 수 없음 (수신 시점에 영상이 없음). 0 으로 실행')
recorder=SignalRecorder();signal_armed=False;barcode=None;signal_error=None;inventory={}
wrist_identity=WristIdentity()
goal_observation_dwell_s=float(os.getenv('R12_GOAL_OBSERVATION_DWELL_S','0'))
goal_dwell=None if goal_observation_dwell_s==0 else StationaryObservationDwell(goal_observation_dwell_s)
goal_stop=GoalStopHold(capture_radius_m=float(os.getenv('R12_GOAL_CAPTURE_RADIUS_M','.02'))) if os.getenv('R12_GOAL_STOP_HOLD','0')=='1' else None
continuous_mode=os.getenv('R12_CONTINUOUS_PATH','0')=='1'
if continuous_mode:
 from arm_practice.approach_region import ApproachRegion,region_arrival_ready
 from continuous_path import ContinuousPath,safe_reference
 goal_stop=ApproachRegion(float(os.getenv('R12_APPROACH_RADIUS_M','.10')))
arm_cfg=json.loads((root/'configs/r12_robot.json').read_text());arm_gate=ArmCommandGate(arm_cfg['joint_names'],arm_cfg['lower'],arm_cfg['upper'],arm_cfg['velocity']);arm_velocity=[0.]*5;gripper_request={'width':.075,'stamp':None}
transport_cfg=json.loads((root/'configs/r12_transport.json').read_text())
if not transport_cfg['physics_validated']:raise RuntimeError('Transport posture requires physics validation')
transport=TransportPosture(transport_cfg,require_clearance=True);base_stopped=False;auto_fold_started=False
wheel_hold=WheelPositionHold()
observation_cfg=route_data.get('observation_posture')
observation=None if observation_cfg is None else ObservationPosture(observation_cfg['arm_q'],arm_cfg['lower'],arm_cfg['upper'],observation_cfg.get('waypoints'))
wrist_surface_guard=None
if observation_cfg is not None and observation_cfg.get('cycle')=='live_surface_guarded_inspection':
 from arm_practice.wrist_inspection_cycle import WristInspectionCycle
 from wrist_surface_guard import WristSurfaceGuard
 wrist_surface_guard=WristSurfaceGuard()
 observation=WristInspectionCycle(observation_cfg['arm_q'],arm_cfg['lower'],arm_cfg['upper'],clearance_check=wrist_surface_guard,exposure_seconds=observation_cfg['exposure_seconds'])
def gripper_command(msg):
 if np.isfinite(msg.data) and .020<=msg.data<=.075:gripper_request.update(width=float(msg.data),stamp=node.get_clock().now().nanoseconds)
if not stationary_capture:node.create_subscription(Float64,'/r12/gripper/width_command',gripper_command,10)
def select_arm(msg):
 global transport
 try:
  if not base_stopped:raise ValueError('Arm mode changes require stopped base')
  arm_gate.select(msg.data,arm_velocity)
  if msg.data=='transport':transport=TransportPosture(transport_cfg,require_clearance=True)
 except ValueError as e:node.get_logger().warning(str(e))
if not stationary_capture:node.create_subscription(String,'/r12/arm_mode',select_arm,10)
for mode,topic in [('planner','/r12/joint_commands'),('servo','/r12/servo_joint_commands'),('policy','/r12/policy_joint_commands')]:
 if not stationary_capture:node.create_subscription(JointState,topic,lambda m,mode=mode:arm_gate.submit(mode,m.name,m.position,(m.header.stamp.sec*10**9+m.header.stamp.nanosec)/1e9),10)
mission={'index':0,'path':[],'busy':False,'last_plan':-10**12,'plan_errors':[],'completed':[],'finished':False,'arrival':None}
callback_duration_rows=[]
def timed_ros_callback(name):
 def decorate(fn):
  @wraps(fn)
  def wrapped(*args,**kwargs):
   start=time.monotonic_ns()
   try:return fn(*args,**kwargs)
   finally:
    if os.getenv('R12_CALLBACK_DURATION_TIMING','0')=='1' and len(callback_duration_rows)<20000:
     callback_duration_rows.append(dict(name=name,start_ns=start,duration_ns=time.monotonic_ns()-start,packet_stamp_ns=globals().get('ns')))
  return wrapped
 return decorate
path_worker=None;path_result_audit=[]
oriented_session=None
oriented_mode=os.getenv('R12_ORIENTED_LIVE_PATH','0')=='1'
if oriented_mode and (not continuous_mode or os.getenv('R12_MISSION','0')!='1'):raise ValueError('Oriented live paths require continuous mission mode')
@timed_ros_callback('planned')
def planned(f,goal_index,nav_request=None):
 path_received_monotonic_s=time.monotonic()
 if goal_index!=mission['index']:return
 if oriented_session is not None and not oriented_session.path_request_current(nav_request):return
 try:
  result=f.result()
  if result.status!=4:mission['busy']=False;mission['plan_errors'].append(result.status);return
  raw_path=[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]
  if oriented_session is not None:
   verdict=oriented_session.queue_path(result.result.path,goal_index,nav_request,received_monotonic_s=path_received_monotonic_s)
   if not oriented_session.path_request_in_flight():mission['busy']=False
   path_result_audit.append(dict(oriented_callback=verdict))
  elif path_worker is not None:
   path_worker.submit(raw_path,goal_index)
  else:
   mission['path']=smooth_path(raw_path,map_data['occupied'],map_data['clearance'],float(map_data['resolution']),transport_footprint)
   mission['busy']=False
 except Exception as e:mission['busy']=False;mission['plan_errors'].append(str(e))
def accepted(f,goal_index,nav_request=None):
 if goal_index!=mission['index']:return
 if oriented_session is not None and not oriented_session.path_request_current(nav_request):return
 try:
  handle=f.result()
  if not handle.accepted:mission['busy']=False;mission['plan_errors'].append('rejected');return
  handle.get_result_async().add_done_callback(lambda result:planned(result,goal_index,nav_request))
 except Exception as e:mission['busy']=False;mission['plan_errors'].append(str(e))
status={};odom={};filtered={};slice_state={} 
@timed_ros_callback('save_costmap')
def save_costmap(m):
 np.savez_compressed(out/'costmap.npz',data=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width),resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y])
node.create_subscription(OccupancyGrid,'/global_costmap/costmap',save_costmap,10)

visual_pub=node.create_publisher(Odometry,'/r12/visual_odometry',10)
node.create_subscription(Odometry,'/odometry/filtered',lambda m:filtered.update({'message':m}),100)
slice_callback_timings=[]
@timed_ros_callback('receive_slice')
def receive_slice(m):
 start=time.monotonic_ns()
 if oriented_session is not None:oriented_session.on_slice(m)
 known_cells=int(np.count_nonzero(np.asarray(m.data,dtype=np.float64)!=m.unknown_value)) if os.getenv('R12_FAST_SLICE_COUNT','0')=='1' else sum(v!=m.unknown_value for v in m.data)
 slice_state.update({'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'known_cells':known_cells})
 end=time.monotonic_ns()
 if os.getenv('R12_SLICE_CALLBACK_TIMING','0')=='1' and len(slice_callback_timings)<200:
  slice_callback_timings.append(dict(start_ns=start,end_ns=end,duration_ns=end-start,cells=len(m.data),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec))
node.create_subscription(DistanceMapSlice,'/nvblox_node/static_map_slice',receive_slice,10)
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
visual_timeouts={}
visual_response_timings={}
callback_audit_enabled=os.getenv('R12_CALLBACK_AUDIT','0')=='1'
callback_audit_count=0;capture_links={}
def note_visual_arrival(m,kind,info=None):
 receipt_ns=time.monotonic_ns()
 key=stamp(m)
 # Diagnostics only: retain processing and middleware timing separately from
 # callback entry. These fields never authorize or relax sensor/control gates.
 def metadata(name):return info.get(name) if isinstance(info,dict) else getattr(info,name,None)
 timing=dict(callback_monotonic_ns=receipt_ns,callback_wall_ns=time.time_ns(),middleware_source_timestamp_ns=metadata('source_timestamp'),middleware_received_timestamp_ns=metadata('received_timestamp'))
 if kind=='status':timing.update(node_callback_execution_time_s=m.node_callback_execution_time,track_execution_time_s=m.track_execution_time)
 visual_response_timings.setdefault(key,{})[kind]=timing
 while len(visual_response_timings)>64:visual_response_timings.pop(next(iter(visual_response_timings)))
 global callback_audit_count
 if callback_audit_enabled and callback_audit_count<400:
  entry=dict(kind=kind,stamp_ns=key,receipt_monotonic_ns=receipt_ns,current_packet_stamp_ns=globals().get('ns'),capture=capture_links.get(key),cdr_base64=base64.b64encode(serialize_message(m)).decode(),publisher_gid=None if info is None else dict(implementation_identifier=info['publisher_gid']['implementation_identifier'],data=list(info['publisher_gid']['data'])))
  with (out/'raw_callback_audit.jsonl').open('a') as callback_log:callback_log.write(json.dumps(entry)+'\n')
  callback_audit_count+=1
 if kind=='status':status[key]=int(m.vo_state)
 else:odom[key]=m
 if key in visual_timeouts:
  record=visual_timeouts[key];record.setdefault(kind+'_arrival_after_timeout_s',time.monotonic()-record['timeout_monotonic_s']);record.setdefault(kind+'_arrival_current_stamp_ns',globals().get('ns'))
  record[kind+'_timing']=timing
node.create_subscription(VisualSlamStatus,'/visual_slam/status',lambda m,info:note_visual_arrival(m,'status',info),100)
node.create_subscription(Odometry,'/visual_slam/tracking/odometry',lambda m,info:note_visual_arrival(m,'odom',info),100)
fusion=DualImuFusion([[.057,0,.16597],[-.057,0,.16597]],[[0,0,0,1],[0,0,1,0]])
alignment=np.array([.734,.950,np.pi]);localizer=None;scan_fit=None
cruise_speed=float(os.getenv('R12_CRUISE_SPEED','.899795667' if os.getenv('R12_USE_LATEST_PPO')=='1' else '.12'))
if not .01<=cruise_speed<=(.899795667 if os.getenv('R12_USE_LATEST_PPO')=='1' else .16):raise ValueError('Cruise speed exceeds configured motor no-load ceiling')
envelope=MotionEnvelope(speed=cruise_speed);last_command_ns=None
continuous_path=ContinuousPath(speed=cruise_speed,arrival_radius=goal_stop.radius_m) if continuous_mode else None
transport_footprint=json.loads((root/'configs/sensors/transport_footprint.json').read_text())['vertices']
if oriented_mode:
 from live_oriented_path import LiveOrientedPath
 oriented_session=LiveOrientedPath(transport_footprint,out/'oriented_live_candidates')
 oriented_session.set_goal(mission['index'])
if os.getenv('R12_ASYNC_PATH_SMOOTHING','0')=='1' and not oriented_mode:
 path_worker=AsyncPathSmoother(root/'models/navigation/maze_grid.npz',root/'configs/sensors/transport_footprint.json',root/'scripts/sensor_runtime/path_smoothing.py')
latest_policy=LatestPolicy(root/'models/swerve/deployed') if os.getenv('R12_USE_LATEST_PPO')=='1' else None
residual_policy=ResidualController(root/'models/swerve/rl/policy.pt') if os.getenv('R12_USE_PPO','0')=='1' else None
shared_frame_enabled=os.getenv('R12_SHARED_FRAME_DIAGNOSTIC','0')=='1'
shared_frame_session=None;shared_frame_sequence=0;shared_capture_run=None
sequence_recording=os.getenv('R12_RECORD_SENSOR_SEQUENCE','0')=='1'
sequence_frames=[];sequence_bytes=0
if sequence_recording and not (stationary_capture and shared_frame_enabled):raise ValueError('Sequence recording requires stationary owned shared snapshots')

if shared_frame_enabled:
 if not stationary_capture or os.getenv('R12_MISSION_PERCEPTION','1')=='1':raise ValueError('Shared frames require stationary capture without perception')
 from arm_practice.shared_frame_prototype import snapshot
 from arm_practice.capture_bundle_identity import inspect_capture_bundle
server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind(('127.0.0.1',int(os.getenv('R12_SENSOR_PORT','19612'))));server.listen(1);print('SENSOR_ROS_READY',flush=True);conn,_=server.accept();reader=conn.makefile('r');rows=[]
for line in reader:
 packet=json.loads(line);packet_received_monotonic_ns=time.monotonic_ns();ns=packet['stamp_ns'];ts=Time(sec=ns//10**9,nanosec=ns%10**9);clock.publish(Clock(clock=ts));extr=json.loads((out/'extrinsics.json').read_text());trans=[]
 signal_consumed_at_receive=False
 if os.getenv('R12_SIGNAL_AT_RECEIVE','0')=='1' and perception_required and signal_armed and not recorder.complete and signal_error is None:
  # Record historical signal evidence while fresh, before expensive publications.
  # This does not authorize motion: later localization/transport gates still apply.
  arrival_color=checked_signal_color(packet,now_ns=time.monotonic_ns(),max_age_ns=perception_max_age_ns)
  try:recorder.observe(ns/1e9,arrival_color)
  except ValueError as e:signal_error=str(e)
  signal_consumed_at_receive=True
 if callback_audit_enabled:
  capture_links[ns]=dict(run_id=packet.get('capture_run_id'),capture_monotonic_ns=packet.get('frame_capture_monotonic_ns'),packet_received_monotonic_ns=packet_received_monotonic_ns)
  while len(capture_links)>64:capture_links.pop(next(iter(capture_links)))
 for side,T in extr.items():
  T=np.array(T);m=TransformStamped();m.header.frame_id='base_link';m.child_frame_id=f'r07_camera1_{side}_optical';m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=T[:3,3].tolist();q=Rotation.from_matrix(T[:3,:3]).as_quat();m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=q.tolist();trans.append(m)
 # Freeze the map alignment used by TF, control and clouds for this packet.
 # A new lidar fit below becomes effective on the next packet.
 packet_alignment=alignment.copy()
 mt=TransformStamped();mt.header.frame_id='map';mt.child_frame_id='odom';mt.header.stamp=ts;mt.transform.translation.x=float(alignment[0]);mt.transform.translation.y=float(alignment[1]);mt.transform.rotation.z=float(np.sin(alignment[2]/2));mt.transform.rotation.w=float(np.cos(alignment[2]/2));map_tf.publish(TFMessage(transforms=[mt]))
 for name,T in json.loads((out/'lidar_extrinsics.json').read_text()).items():
  T=np.array(T);lt=TransformStamped();lt.header.frame_id='base_link';lt.child_frame_id='lidar_'+name;lt.transform.translation.x,lt.transform.translation.y,lt.transform.translation.z=T[:3,3].tolist();quat=Rotation.from_matrix(T[:3,:3]).as_quat();lt.transform.rotation.x,lt.transform.rotation.y,lt.transform.rotation.z,lt.transform.rotation.w=quat.tolist();trans.append(lt)
 vm=TransformStamped();vm.header.frame_id='map';vm.child_frame_id='vslam_map';vm.transform.translation.x=.734;vm.transform.translation.y=.950;vm.transform.rotation.z=1.;vm.transform.rotation.w=0.;trans.append(vm)
 for i,x in enumerate([.057,-.057]):
  it=TransformStamped();it.header.frame_id='base_link';it.child_frame_id=['opencr_front_imu','opencr_rear_imu'][i];it.transform.translation.x=x;it.transform.translation.z=.16597;it.transform.rotation.w=1. if i==0 else 0.;it.transform.rotation.z=0. if i==0 else 1.;trans.append(it)
 dynamic=[]
 for key,T in packet.get('camera_transforms',{}).items():
  if key.startswith('r07_camera1'):continue
  T=np.array(T);m=TransformStamped();m.header.frame_id='base_link';m.child_frame_id=key;m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=T[:3,3].tolist();quat=Rotation.from_matrix(T[:3,:3]).as_quat();m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=quat.tolist()
  if key.startswith('arm_camera'):m.header.stamp=ts;dynamic.append(m)
  else:trans.append(m)
 tf.publish(TFMessage(transforms=trans))
 if dynamic:map_tf.publish(TFMessage(transforms=dynamic))
 for name,ranges in packet['scans'].items():
  scan=LaserScan();scan.header.stamp=ts;scan.header.frame_id='lidar_'+name;scan.angle_min=-np.pi/2;scan.angle_increment=np.pi/89;scan.angle_max=np.pi/2;scan.range_min=.03;scan.range_max=6.;scan.ranges=ranges;scan_pubs[name].publish(scan)
 samples=[]
 for i,v in enumerate(packet['imu']):
  m=Imu();m.header.stamp=ts;m.header.frame_id=['opencr_front_imu','opencr_rear_imu'][i];m.orientation_covariance[0]=-1;m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=v['gyro'];m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=v['accel'];m.angular_velocity_covariance=np.eye(3).flatten().tolist();m.linear_acceleration_covariance=np.eye(3).flatten().tolist();imu_pub[i].publish(m);samples.append(Sample(ns/1e9,np.array(v['gyro']),np.array(v['accel']),np.eye(3)*.0004,np.eye(3)*.04))
 error=None;fused=None
 try:
  if not all(v['valid'] for v in packet['imu']):raise ValueError('Invalid physics IMU')
  fused=fusion.update(samples)
 except ValueError as e:error=str(e)
 if fused:
  m=Imu();m.header.stamp=ts;m.header.frame_id='base_link';m.orientation_covariance[0]=-1;m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=fused['gyro'].tolist();m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=fused['accel'].tolist();m.angular_velocity_covariance=fused['gyro_cov'].flatten().tolist();m.linear_acceleration_covariance=fused['accel_cov'].flatten().tolist();fused_pub.publish(m)
 m=JointState();m.header.stamp=ts;m.name=packet['names'];m.position=packet['position'];m.velocity=packet['velocity'];joint_pub.publish(m)
 lookup={n:i for i,n in enumerate(packet['names'])};steer=[lookup[p+'_steer_joint'] for p in ['lh1','rh1','lh2','rh2']];wheel=[lookup[p+'_wheel_joint'] for p in ['lh1','rh1','lh2','rh2']]
 encoder_valid=bool(packet.get('velocity_valid',False))
 try:twist,residual=wheel_odometry(np.array(packet['position'])[steer],np.array(packet['velocity'])[wheel])
 except ValueError:twist=np.zeros(3);residual=1.;encoder_valid=False
 od=Odometry();od.header.stamp=ts;od.header.frame_id='odom';od.child_frame_id='base_link';od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.angular.z=twist.tolist()
 for k in [0,7,35]:od.twist.covariance[k]=.0004+residual**2
 odom_pub.publish(od)
 publication_stages={}
 publication_operations=[]
 publication_auxiliary=[]
 def measured_aux(name,fn,*args):
  if os.getenv('R12_PUBLICATION_OPERATION_TIMING','0')!='1':return fn(*args)
  begin=time.monotonic_ns();value=fn(*args);end=time.monotonic_ns()
  publication_auxiliary.append(dict(operation=name,start_ns=begin,end_ns=end,duration_ns=end-begin));return value
 def depth_to_u16(array):
  return np.where(np.isfinite(array)&(array>0)&(array<10),np.clip(array*1000,0,65535),0).astype(np.uint16)
 def timed_image_publish(message,array,publisher):
  if os.getenv('R12_IMAGE_ARRAY_BUFFER','0')=='1':
   from array import array as byte_array
   t0=time.monotonic_ns();payload=byte_array('B');payload.frombytes(array.tobytes());t1=time.monotonic_ns();message.data=payload;t2=time.monotonic_ns();publisher.publish(message);t3=time.monotonic_ns()
   if os.getenv('R12_PUBLICATION_OPERATION_TIMING','0')=='1':publication_operations.append(dict(frame_id=message.header.frame_id,encoding=message.encoding,bytes=len(payload),start_ns=t0,to_bytes_ns=t1-t0,message_assignment_ns=t2-t1,publish_ns=t3-t2,end_ns=t3))
   return
  if os.getenv('R12_PUBLICATION_OPERATION_TIMING','0')!='1':
   message.data=array.tobytes();publisher.publish(message);return
  t0=time.monotonic_ns();payload=array.tobytes();t1=time.monotonic_ns()
  message.data=payload;t2=time.monotonic_ns();publisher.publish(message);t3=time.monotonic_ns()
  publication_operations.append(dict(frame_id=message.header.frame_id,encoding=message.encoding,bytes=len(payload),start_ns=t0,to_bytes_ns=t1-t0,message_assignment_ns=t2-t1,publish_ns=t3-t2,end_ns=t3))
 acquisition_timing_enabled=os.getenv('R12_ACQUISITION_TIMING','0')=='1'
 shared_frame_ack=None
 if packet.get('fault')!='camera':
  if acquisition_timing_enabled:publication_stages['image_block_start']=time.monotonic_ns()
  if shared_frame_enabled:
   descriptor=packet['shared_frame_descriptor']
   if shared_frame_session is None:shared_frame_session=descriptor['session'];shared_capture_run=packet['capture_run_id']
   if descriptor['session']!=shared_frame_session or descriptor['sequence']!=shared_frame_sequence+1 or packet['capture_run_id']!=shared_capture_run:raise ValueError('shared_frame_session_or_sequence_changed')
   data,shared_frame_ack=snapshot(descriptor)
   identity=inspect_capture_bundle(data,packet)
   if not identity['matched'] or identity['frame_index']!=descriptor['sequence']-1:raise ValueError('shared_frame_identity_mismatch')
   shared_frame_sequence=descriptor['sequence']
   if sequence_recording:
    size=sum(a.nbytes for a in data.values())
    if len(sequence_frames)>=20 or sequence_bytes+size>320*1024**2:raise ValueError('Sequence recording bound exceeded')
    sequence_frames.append((json.loads(json.dumps(packet)),{k:a.copy() for k,a in data.items()}));sequence_bytes+=size

  elif os.getenv('R12_FRAME_MEMBER_CACHE','0')=='1':
   # Materialize once so provenance checks and publishers share identical arrays.
   with np.load(Path(os.getenv('R12_FRAME_FILE',str(out/'frame.npz')))) as archive:data={key:archive[key] for key in archive.files}
  else:data=np.load(Path(os.getenv('R12_FRAME_FILE',str(out/'frame.npz'))))
  require_archive_geometry(data)
  if publication_diagnostics is not None:publication_diagnostics.begin(data,packet)
  if acquisition_timing_enabled:publication_stages['bundle_check_done']=time.monotonic_ns()
  for i,side in enumerate(images):
   info=CameraInfo();info.header.stamp=ts;info.header.frame_id=f'r07_camera1_{side}_optical';info.width=640;info.height=480;info.distortion_model='plumb_bob';info.d=[0.]*5;info.k=camera_matrix().ravel().tolist();info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.p=projection_matrix(right=i!=0).ravel().tolist();infos[side].publish(info)
   im=Image();im.header=info.header;im.height=480;im.width=640;im.encoding='rgb8';im.step=1920;timed_image_publish(im,data['rgb'][i],images[side])
   if acquisition_timing_enabled:publication_stages[info.header.frame_id]=time.monotonic_ns()
   if publication_diagnostics is not None:measured_aux('diagnostics_published',publication_diagnostics.published,im,info)
  depth=data['depth'];depth=measured_aux('depth_conversion',depth_to_u16,depth);im=Image();im.header.stamp=ts;im.header.frame_id='r07_camera1_left_optical';im.height=480;im.width=640;im.encoding='16UC1';im.step=1280;timed_image_publish(im,depth,depth_pub)
  for key,(pub,infopub,dpub) in extra_publishers.items():
   ci=CameraInfo();ci.header.stamp=ts;ci.header.frame_id=key;ci.width=640;ci.height=480;ci.distortion_model='plumb_bob';ci.d=[0.]*5;ci.k=camera_matrix().ravel().tolist();ci.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];ci.p=projection_matrix(right='right' in key).ravel().tolist();infopub.publish(ci)
   im=Image();im.header=ci.header;im.height=480;im.width=640;im.encoding='rgb8';im.step=1920;timed_image_publish(im,data[key],pub)
   if acquisition_timing_enabled:publication_stages[ci.header.frame_id]=time.monotonic_ns()
   if publication_diagnostics is not None:measured_aux('diagnostics_published',publication_diagnostics.published,im,ci)
   if dpub:
    dep=data[key[:-len('_left_optical')]+'_depth'];dep=measured_aux('depth_conversion',depth_to_u16,dep);im=Image();im.header=ci.header;im.height=480;im.width=640;im.encoding='16UC1';im.step=1280;timed_image_publish(im,dep,dpub)
  if publication_diagnostics is not None:measured_aux('diagnostics_finish',publication_diagnostics.finish)
  if acquisition_timing_enabled:publication_stages['image_block_done']=time.monotonic_ns()
 # DDS acknowledgement proves middleware receipt, not cuVSLAM callback completion.
 # Charge diagnostic waits to the existing transport deadline.
 visual_wait_budget=float(os.getenv('R12_SIM_SENSOR_WAIT_S','1.0'));ack_start=time.monotonic();image_delivery_ack={}
 if os.getenv('R12_BOUND_VISUAL_WAIT_TO_FRESHNESS','0')=='1':
  capture_ns=packet.get('frame_capture_monotonic_ns')
  remaining_ns=perception_max_age_ns-(time.monotonic_ns()-capture_ns)-50_000_000 if type(capture_ns) is int else 0
  visual_wait_budget=min(visual_wait_budget,max(0.,remaining_ns/1e9))
 if os.getenv('R12_IMAGE_ACK_AUDIT','0')=='1':
  audit_publishers={**{side:pub for side,pub in images.items()},**{key:pubs[0] for key,pubs in extra_publishers.items()}}
  for name,pub in audit_publishers.items():
   remaining=max(0.,visual_wait_budget-(time.monotonic()-ack_start))
   image_delivery_ack[name]={'subscriptions':pub.get_subscription_count(),'acked':pub.wait_for_all_acked(Duration(seconds=min(.05,remaining)))}
 ack_elapsed=time.monotonic()-ack_start
 visual_pair_wait=wait_visual_pair(ns,status,odom,lambda seconds:rclpy.spin_once(node,timeout_sec=seconds),timeout=max(0.,visual_wait_budget-ack_elapsed))
 visual_pair_wait['budget_seconds']=visual_wait_budget;visual_pair_wait['dds_ack_seconds']=ack_elapsed;visual_pair_wait['image_delivery_ack']=image_delivery_ack
 if visual_pair_wait['timed_out']:visual_timeouts[ns]={'stamp_ns':ns,'timeout_monotonic_s':time.monotonic(),'status_received_at_timeout':ns in status,'odom_received_at_timeout':ns in odom}
 if len(rows)%100==0:atomic_json(out/'visual_arrival_audit.json',list(visual_timeouts.values()))
 for _ in range(3):rclpy.spin_once(node,timeout_sec=.002)
 if ns in odom:
  # Keep received evidence intact for attitude checks and archived visual_pose.
  vo=deepcopy(odom[ns]);vo.header.frame_id='odom';vo.child_frame_id='base_link'
  for k in [0,7,14]:vo.pose.covariance[k]=max(vo.pose.covariance[k],.0025)
  for k in [21,28,35]:vo.pose.covariance[k]=max(vo.pose.covariance[k],.01)
  visual_pub.publish(vo)
 for _ in range(5):rclpy.spin_once(node,timeout_sec=.002)
 est=filtered.get('message');pose=None if est is None else [est.pose.pose.position.x,est.pose.pose.position.y,est.pose.pose.orientation.z,est.pose.pose.orientation.w]
 if localizer is None:localizer=ScanLocalizer(map_data,json.loads((out/'lidar_extrinsics.json').read_text()))
 map_pose=None
 if pose is not None:
  c,s=np.cos(packet_alignment[2]),np.sin(packet_alignment[2]);R=np.array([[c,-s],[s,c]]);xy=packet_alignment[:2]+R@np.array(pose[:2]);yaw=packet_alignment[2]+2*np.arctan2(pose[2],pose[3])
  if len(rows)%5==0:
   scan_fit=localizer.fit(packet['scans'],[xy[0],xy[1],yaw])
   if scan_fit:
    fit=scan_fit['pose'];new_angle=fit[2]-2*np.arctan2(pose[2],pose[3]);c,s=np.cos(new_angle),np.sin(new_angle);new_xy=fit[:2]-np.array([[c,-s],[s,c]])@np.array(pose[:2]);alignment+=.5*(np.r_[new_xy,new_angle]-alignment)
  map_pose=[float(xy[0]),float(xy[1]),float(yaw)]
 if map_pose is not None and packet.get('fault')!='camera' and len(rows)%10==0:
  Tmap=np.eye(4);Tmap[:2,:2]=[[np.cos(map_pose[2]),-np.sin(map_pose[2])],[np.sin(map_pose[2]),np.cos(map_pose[2])]];Tmap[:2,3]=map_pose[:2];cloud_parts=[];cloud_colors=[]
  for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
   pixels=data['rgb'][0] if camera=='r07_camera1' else data[camera+'_left_optical'];dd=data['depth'] if camera=='r07_camera1' else data[camera+'_depth']
   xyz,colors=project_cloud(pixels,dd,Tmap@np.array(packet['camera_transforms'][camera+'_left_optical']));cloud_parts.append(xyz);cloud_colors.append(colors)
  xyz=np.concatenate(cloud_parts);colors=np.concatenate(cloud_colors)
  for colored,pub in [(False,cloud_pub),(True,rgb_cloud_pub)]:
   cloud=PointCloud2();cloud.header.stamp=ts;cloud.header.frame_id='map';cloud.height=1;cloud.width=len(xyz);cloud.is_dense=True;cloud.is_bigendian=False;cloud.point_step=16 if colored else 12;cloud.row_step=cloud.point_step*len(xyz)
   cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(['x','y','z'])]
   if colored:cloud.fields.append(PointField(name='rgb',offset=12,datatype=PointField.UINT32,count=1))
   cloud.data=pack_rgb(xyz,colors) if colored else xyz.tobytes();pub.publish(cloud)
   if cloud_audit_enabled and len(cloud_publication_audit)<2000:cloud_publication_audit.append(dict(topic='/r12/points_rgb' if colored else '/r12/points',stamp_ns=ns,points=cloud.width,bytes=len(cloud.data),subscriptions=pub.get_subscription_count(),publish_returned_ns=time.monotonic_ns(),scope='publish_return_not_delivery'))
 if path_worker is not None:
  path_result=path_worker.poll(mission['index'])
  if path_result is not None:
   mission['busy']=False
   if path_result['ready_for_goal']:mission['path']=path_result['path']
   else:mission['plan_errors'].append('path_worker:'+path_result['reason']+':'+path_result.get('error',''))
   path_result_audit.append({k:v for k,v in path_result.items() if k!='path'})
 arm_ix=[lookup[n] for n in arm_cfg['joint_names']];arm_position=np.array(packet['position'])[arm_ix];arm_velocity=np.array(packet['velocity'])[arm_ix].tolist()
 base_stopped=bool(encoder_valid and np.max(np.abs(np.array(packet['velocity'])[wheel]))<.2)
 if not stationary_capture and not auto_fold_started and (not perception_required or (recorder.complete and barcode is not None and signal_error is None)) and arm_gate.mode=='hold' and base_stopped and max(abs(v) for v in arm_velocity)<.02:
  arm_gate.select('transport',arm_velocity);transport=TransportPosture(transport_cfg,require_clearance=True);auto_fold_started=True
 if arm_gate.mode=='transport':
  fold_target=transport.step(ns/1e9,arm_position,arm_velocity,base_stopped,valid=encoder_valid)
  if fold_target is not None:arm_gate.submit('transport',arm_cfg['joint_names'],fold_target.tolist(),ns/1e9)
  elif transport.state=='await_swept_volume_clearance':
   # Revoke any cached trajectory as well as refusing the next fold target.
   arm_gate.target=None;arm_gate.reference=None;arm_gate.hold_target=arm_position.tolist()
 transport_ok=bool(arm_gate.mode=='transport' and transport.ready)
 if oriented_session is not None:
  oriented_session.set_goal(mission['index'])
  oriented_feedback_valid=bool(encoder_valid and fused is not None and status.get(ns)==1 and est is not None and 0<=ns-stamp(est)<300_000_000 and packet.get('fault') is None)
  oriented_twist=[0.,0.,0.] if est is None or fused is None else [est.twist.twist.linear.x,est.twist.twist.linear.y,float(fused['gyro'][2])]
  oriented_session.note_sensor(map_pose,oriented_twist,sim_stamp_ns=ns,received_monotonic_s=packet_received_monotonic_ns/1e9,valid=oriented_feedback_valid,transport_settled=transport_ok)
  oriented_pending=oriented_session.consume_pending_path()
  if oriented_pending is not None:
   mission['busy']=False;path_result_audit.append(dict(oriented_pending_path=oriented_pending))
  if not oriented_session.path_request_in_flight():mission['busy']=False
  oriented_result=oriented_session.poll()
  if oriented_result is not None:
   if oriented_result['accepted']:mission['path']=oriented_result['path']
   else:mission['path']=[];mission['plan_errors'].append('oriented:'+oriented_result['reason'])
   path_result_audit.append({k:v for k,v in oriented_result.items() if k!='path'})
 path_state=None;guard=None;goal_observation_state=None;candidate_detections=[]
 motion=encoder_valid and os.getenv('R12_SENSOR_MOTION','0')=='1' and status.get(ns)==1 and fused is not None
 command=[.08 if ns<5_000_000_000 else -.08,0,0] if motion and 2_000_000_000<ns<9_000_000_000 else [0,0,0]
 if os.getenv('R12_MISSION','0')=='1':
  command=[0.,0.,0.]
  fresh=encoder_valid and est is not None and 0<=ns-stamp(est)<300_000_000 and status.get(ns)==1 and fused is not None and slice_state.get('known_cells',0)>0 and 0<=ns-slice_state.get('stamp_ns',0)<1_000_000_000
  perception_check=check_packet_perception(packet,now_ns=time.monotonic_ns(),max_age_ns=perception_max_age_ns)
  perception=packet.get('perception',{}) if perception_check['matched'] else {}
  if perception_required:
   # 신호등 색: 추정 맵 자세 + 카메라 FK 로 신호등 위치를 투영해 칸별 밝기로 판정 (시뮬 정답 자세 사용 안 함)
   signal_color='UNKNOWN';tower_signal=None
   if map_pose is not None and signal_frame_fresh(packet,now_ns=time.monotonic_ns(),max_age_ns=perception_max_age_ns):
    Tmap=np.eye(4);Tmap[:2,:2]=[[np.cos(map_pose[2]),-np.sin(map_pose[2])],[np.sin(map_pose[2]),np.cos(map_pose[2])]];Tmap[:2,3]=map_pose[:2]
    T_world_cam=Tmap@np.array(packet['camera_transforms']['r07_camera1_left_optical'])
    tower_signal=tower_signal_color(data['rgb'][0],T_world_cam,camera_matrix(),depth=data['depth'])
    signal_color=tower_signal['color']
   codes=perception.get('barcodes',[])
   if len(set(codes))==1:barcode=codes[0]
   if fresh and barcode and not signal_armed and signal_color=='OFF':signal_armed=True
   if signal_armed and not recorder.complete and signal_error is None and not signal_consumed_at_receive:
    try:recorder.observe(ns/1e9,signal_color)
    except ValueError as e:signal_error=str(e)
   for d in perception.get('detections',[]):
    ids=d.get('tag_ids',[]);cls=d.get('class_id',99);pose_e=d.get('pose',{})
    candidate_detections.append({'camera':perception.get('camera'),'class_id':cls,'tag_ids':ids,'role':'search_candidate_not_confirmed_inventory'})
    if len(ids)==1 and cls<4 and pose_e.get('translation_valid') and pose_e.get('confidence',0)>=.25 and d.get('depth_consistent') and map_pose is not None and perception.get('stamp_ns')==ns:
     camera_center=np.asarray(pose_e.get('center_camera_m'),float)
     if camera_center.shape!=(3,) or not np.isfinite(camera_center).all():continue
     Tmap=np.eye(4);Tmap[:2,:2]=[[np.cos(map_pose[2]),-np.sin(map_pose[2])],[np.sin(map_pose[2]),np.cos(map_pose[2])]];Tmap[:2,3]=map_pose[:2]
     object_map=(Tmap@np.array(packet['camera_transforms'][perception['camera_frame']])@np.r_[pose_e['center_camera_m'],1])[:3]
     candidate={'id':ids[0],'color':'R' if cls<2 else 'G','shape':'cylinder' if cls%2==0 else 'cuboid','pose_camera':pose_e,'center_map_m':object_map.tolist(),'map_height_assumption':'base z=0; no measured vertical localization','depth_surface_m':d['depth_surface_m'],'depth_consistent':bool(d['depth_consistent']),'stamp_ns':perception['stamp_ns'],'arm_encoder_position_rad':arm_position.tolist(),'camera_transform_base':packet['camera_transforms'][perception['camera_frame']]}
     wrist_ready=bool(observation is not None and observation.state=='observing')
     wrist_stationary=bool(base_stopped and fused is not None and abs(float(fused['gyro'][2]))<.02 and est is not None and np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y)<.015)
     unique=sum(x.get('tag_ids')==ids for x in perception.get('detections',[]))==1
     confirmed=wrist_identity.observe(candidate,camera=perception.get('camera'),camera_frame=perception.get('camera_frame'),run_id=perception.get('run_id'),stamp_ns=ns,fresh=bool(fresh and perception_check['matched'] and perception_check['live_freshness_verified']),view_ready=wrist_ready,stationary=wrist_stationary,arm_velocity=arm_velocity,unique_in_frame=unique)
     for invalid_id in wrist_identity.invalidated_ids:inventory.pop(invalid_id,None)
     if confirmed is not None:inventory[ids[0]]=confirmed
   if len(inventory)==12 and recorder.complete and barcode:
    try:atomic_json(out/'observed_mission_plan.json',plan(recorder.sequence,barcode,list(inventory.values())))
    except ValueError as e:signal_error=str(e)
  departure_allowed=not perception_required or (recorder.complete and barcode is not None and signal_error is None)
  if goal_dwell is not None and not (fresh and departure_allowed):goal_dwell.reset()
  if goal_stop is not None and (not mission['finished'] or continuous_mode):
   stop_goal_index=min(mission['index'],len(goals)-1)
   goal_distance=None if map_pose is None else float(np.linalg.norm(np.array(map_pose[:2])-np.array(goals[stop_goal_index][1])))
   goal_speed=None if est is None else float(np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y))
   goal_stationary=bool(base_stopped and fused is not None and abs(float(fused['gyro'][2]))<.02)
   goal_stop.update(ns,stop_goal_index,goal_distance,goal_speed,goal_stationary,bool(fresh and departure_allowed and transport_ok))
  if fresh and departure_allowed and not mission['finished']:
   # Fixed map->odom alignment is the known start pose, not live simulator truth.
   xy=np.array(map_pose[:2]);yaw=map_pose[2];name,goal=goals[mission['index']];goal=np.array(goal)
   arrival_check=region_arrival_ready if continuous_mode else goal_arrival_ready
   if arrival_check(float(np.linalg.norm(xy-goal)),float(np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y)),goal_stop):
    if mission['arrival'] is None:mission['arrival']=ns
    arrival_complete=ns-mission['arrival']>=(3_000_000_000 if name=='START' else 500_000_000)
    if goal_dwell is not None and name!='START':
     stationary=bool(base_stopped and transport_ok and fused is not None and np.isfinite(fused['gyro'][2]) and abs(fused['gyro'][2])<.02)
     inspected_camera=perception.get('camera') if perception_check['matched'] and perception.get('stamp_ns')==ns and perception.get('detection_performed',False) else None
     goal_observation_state=goal_dwell.update(ns,stationary,inspected_camera)
     arrival_complete=goal_observation_state['complete']
    if arrival_complete:
     mission['completed'].append({**goal_metadata[mission['index']],'name':name,'stamp_ns':ns,'estimated_xy':xy.tolist(),'camera_inspection':goal_observation_state});mission['index']+=1;mission['path']=[];mission['arrival']=None
     if path_worker is not None:path_worker.invalidate()
     if oriented_session is not None:oriented_session.set_goal(mission['index'])
     if goal_dwell is not None:goal_dwell.reset()
     if goal_stop is not None and not (continuous_mode and mission['index']==len(goals)):goal_stop.reset()
     if mission['index']==len(goals):mission['finished']=True
   else:
    mission['arrival']=None
    if goal_dwell is not None:goal_dwell.reset()
    if not (goal_stop is not None and goal_stop.active) and not mission['busy'] and ns-mission['last_plan']>2_000_000_000 and client.server_is_ready() and (oriented_session is None or oriented_session.should_replan()):
     nav_request=None if oriented_session is None else oriented_session.begin_path_request(mission['index'])
     if oriented_session is None or nav_request is not None:
      g=ComputePathToPose.Goal();g.goal.header.frame_id='map';g.goal.header.stamp=ts;g.goal.pose.position.x=float(goal[0]);g.goal.pose.position.y=float(goal[1]);g.goal.pose.orientation.z=1.;g.goal.pose.orientation.w=0.;g.planner_id='GridBased';g.use_start=False;mission['busy']=True;mission['last_plan']=ns;client.send_goal_async(g).add_done_callback(lambda result,goal_index=mission['index'],nav_request=nav_request:accepted(result,goal_index,nav_request))
    if mission['path'] and not (goal_stop is not None and goal_stop.active):
     if oriented_session is not None:
      command=np.zeros(3);path_state={'reason':'oriented_reference_pending'}
     elif continuous_mode:
      command,path_state=continuous_path.propose(mission['path'],xy,yaw,float(np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y)))
      guard=None
     else:
      command,guard,path_state=follow(mission['path'],xy,yaw,goal,map_data['clearance'],float(map_data['resolution']),cruise_speed,packet['scans'],json.loads((out/'lidar_extrinsics.json').read_text()),footprint=transport_footprint,occupied=map_data['occupied'],braking_stop_distance_m=.01 if goal_stop is None else goal_stop.braking_stop_distance_m)
     command=command.tolist()
 dt=.05 if last_command_ns is None else (ns-last_command_ns)/1e9;last_command_ns=ns
 healthy=encoder_valid and fused is not None and status.get(ns)==1 and (transport_ok or (observation is not None and mission['finished']))
 if os.getenv('R12_MISSION','0')=='1':healthy=healthy and fresh
 visual_tilt=None
 visual_attitude_check={'reason':'missing_prerequisite','encoder_valid':bool(encoder_valid),'imu_fused':fused is not None,'tracking':status.get(ns),'odom_received':ns in odom}
 if encoder_valid and fused is not None and status.get(ns)==1 and ns in odom:
  visual=odom[ns];vq=visual.pose.pose.orientation;quat=np.array([vq.x,vq.y,vq.z,vq.w]);cov=np.array(visual.pose.covariance).reshape(6,6)
  qnorm=float(np.linalg.norm(quat));tilt_cov=np.diag(cov)[3:5]
  checks={'frame':visual.child_frame_id=='base_link','quaternion_finite':bool(np.isfinite(quat).all()),'quaternion_unit':bool(abs(qnorm-1)<.01),'covariance_finite':bool(np.isfinite(cov).all()),'tilt_covariance_range':bool(np.all((tilt_cov>=0)&(tilt_cov<=.05)))}
  visual_attitude_check.update(reason='accepted' if all(checks.values()) else 'rejected',checks=checks,child_frame_id=visual.child_frame_id,quaternion_norm=qnorm if np.isfinite(qnorm) else None,tilt_covariance=[float(v) if np.isfinite(v) else None for v in tilt_cov])
  if all(checks.values()):visual_tilt=Rotation.from_quat(quat).as_euler('xyz')[:2]
 if os.getenv('R12_MISSION','0')=='1':healthy=healthy and visual_tilt is not None
 oriented_force_stop=False
 if oriented_session is not None:
  oriented_gate=oriented_session.gate()
  oriented_force_stop=oriented_gate['force_stop'] or not healthy or not departure_allowed or not mission['path'] or mission['finished'] or goal_stop.active
  path_state={'reason':'oriented_reference','oriented_phase':oriented_gate['phase'],'oriented_reason':oriented_gate['reason'],'oriented_hold_eligible':bool(oriented_force_stop and oriented_session.hold_safe())}
  command=np.zeros(3)
  if not oriented_force_stop:
   if oriented_gate['allow_translation']:
    live=oriented_session.reference_inputs()
    command,preview=safe_reference(continuous_path,live['path'],live['xy'],live['yaw'],float(np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y)),envelope,fused['gyro'],fused['accel'],visual_tilt,packet['scans'],json.loads((out/'lidar_extrinsics.json').read_text()),live['footprint'],live['occupied'],live['resolution'],heading_reference=live['heading_reference'])
    path_state.update(preview,oriented_phase=oriented_gate['phase'],oriented_reason=oriented_gate['reason'])
    guard='invalid_sensor' if path_state.get('preview_reason')=='invalid_sensor' else None
   else:command=np.array([0.,0.,oriented_gate['desired_yaw_rate']])
 elif continuous_mode and healthy and path_state is not None and mission['path'] and not goal_stop.active:
  command,path_state=safe_reference(continuous_path,mission['path'],map_pose[:2],map_pose[2],float(np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y)),envelope,fused['gyro'],fused['accel'],visual_tilt,packet['scans'],json.loads((out/'lidar_extrinsics.json').read_text()),transport_footprint,map_data['occupied'],float(map_data['resolution']))
  guard='invalid_sensor' if path_state.get('preview_reason')=='invalid_sensor' else None
 command_stages={'before_envelope':np.asarray(command).tolist()}
 command,motion_state=envelope.step(command, np.zeros(3) if fused is None else fused['gyro'],np.zeros(3) if fused is None else fused['accel'],dt,valid=healthy,visual_tilt=visual_tilt)
 command_stages['after_envelope']=np.asarray(command).tolist()
 command,post_guard=constrain_velocity(command,packet['scans'],json.loads((out/'lidar_extrinsics.json').read_text()),footprint=transport_footprint)
 command_stages['after_lidar']=np.asarray(command).tolist()
 if post_guard is not None:envelope.velocity=command.copy();envelope.accel[:]=0
 guard=post_guard or guard
 if os.getenv('R12_MISSION','0')=='1' and map_pose is not None and not motion_clear(map_pose[:2],map_pose[2],command,transport_footprint,map_data['occupied'],float(map_data['resolution'])):
  command=np.zeros(3);envelope.velocity[:]=0;envelope.accel[:]=0;guard='rotation_sweep_blocked'
 if guard in ('no_visible_forward_target','lidar_blocked_target','invalid_sensor'):
  # A blocked path cannot retain a previous translation via the acceleration filter.
  command=np.zeros(3);envelope.velocity[:]=0;envelope.accel[:]=0
 if goal_stop is not None and goal_stop.active:
  # A captured low-speed stop cannot retain the acceleration filter's old motion.
  # Physical braking and position-hold torques remain actuator limited.
  command=np.zeros(3);envelope.velocity[:]=0;envelope.accel[:]=0
 if oriented_session is not None:
  oriented_live_blocked=not oriented_force_stop and not oriented_session.final_clear(command)
  if oriented_force_stop or oriented_live_blocked:
   command=np.zeros(3);envelope.velocity[:]=0;envelope.accel[:]=0
   if oriented_live_blocked:guard='oriented_live_sweep_blocked'
 command=command.tolist()
 wheel_action=np.zeros(4)
 if latest_policy is not None and fused is not None:
  wheel_action=latest_policy.action(command,np.array(packet['position'])[steer],np.array(packet['velocity'])[wheel],fused['gyro'],fused['accel'])
  if latest_policy.model is None:command=[0.,0.,0.]
 wheel_scale=1.
 if residual_policy is not None and fused is not None:wheel_scale=residual_policy.scale(command,np.array(packet['position'])[steer],np.array(packet['velocity'])[wheel],fused['gyro'],fused['accel'])
 if observation is not None and mission['finished']:
  # Navigation stays finished and base output stays zero throughout observation.
  command=[0.,0.,0.];wheel_action=np.zeros(4)
  observation_valid=bool(fresh and visual_tilt is not None and np.max(np.abs(visual_tilt))<np.deg2rad(4) and not motion_state.get('latched',False) and packet.get('fault') is None)
  observation_stopped=bool(base_stopped and est is not None and np.hypot(est.twist.twist.linear.x,est.twist.twist.linear.y)<.01 and fused is not None and abs(float(fused['gyro'][2]))<.03)
  if wrist_surface_guard is not None:
   wrist_surface_guard.update(data,packet)
   target=observation.step(ns/1e9,arm_position,arm_velocity,map_pose,observation_stopped,observation_valid,run_id=packet.get('capture_run_id'),confirmations=[v for v in inventory.values() if v.get('stamp_ns')==ns],invalidated_ids=wrist_identity.invalidated_ids)
  else:
   target=observation.step(ns/1e9,arm_position,arm_velocity,map_pose,observation_stopped,observation_valid)
  if target is not None:
   if arm_gate.mode!='planner':arm_gate.select('planner',arm_velocity)
   arm_gate.submit('planner',arm_cfg['joint_names'],target.tolist(),ns/1e9)
  elif observation.state in ('failed','paused'):
   # Hold current feedback immediately, without finishing the old trajectory.
   arm_gate.target=None;arm_gate.reference=None;arm_gate.hold_target=arm_position.tolist()
 arm_ix=[lookup[n] for n in arm_cfg['joint_names']];arm_velocity=np.array(packet['velocity'])[arm_ix].tolist();arm_target=arm_gate.step(ns/1e9,np.array(packet['position'])[arm_ix].tolist(),ns/1e9)
 grip_fresh=gripper_request['stamp'] is not None and 0<=ns-gripper_request['stamp']<250_000_000
 result={'command_stages':{**command_stages,'final':list(command)},'visual_attitude_check':visual_attitude_check,'visual_pair_wait':visual_pair_wait,'transport_state':transport.state,'base_stopped':base_stopped,'transport_posture_ok':transport_ok,'path_follower':path_state,'visual_tilt_deg':None if visual_tilt is None else np.degrees(visual_tilt).tolist(),'configured_speed_ceiling_m_s':cruise_speed,'gripper_width_m':gripper_request['width'] if grip_fresh else None,'gripper_aperture_m':packet.get('gripper_aperture_m'),'arm_target':arm_target,'arm_mode':arm_gate.mode,'arm_command_status':arm_gate.reason,'signal_ready':signal_armed,'signal_sequence':recorder.sequence.copy(),'signal_complete':recorder.complete,'signal_error':signal_error,'barcode':barcode,'recognized_puck_ids':sorted(inventory),'execution_scope':'perception_inventory_survey_not_pick_place' if perception_required else 'navigation_only','wheel_action':wheel_action.tolist(),'policy_version':None if latest_policy is None else latest_policy.version,'policy_load_error':None if latest_policy is None else latest_policy.error,'lidar_guard':guard,'motion_envelope':motion_state,'fault':packet.get('fault'),'total_goals':len(goals),'route_file':route_file,'command':command,'wheel_scale':wheel_scale,'actuator_policy':'latest_ppo_v2' if latest_policy is not None else ('ppo_adaptive_residual' if residual_policy else 'bc_baseline'),'estimated_map_pose':map_pose,'scan_fit_rmse':None if scan_fit is None else scan_fit['rmse'],'mission_index':mission['index'],'completed':mission['completed'].copy(),'finished':mission['finished'],'plan_errors':mission['plan_errors'][-3:],'visual_xyz':None if ns not in odom else [odom[ns].pose.pose.position.x,odom[ns].pose.pose.position.y,odom[ns].pose.pose.position.z],'estimated_odom_pose':pose,'nvblox':slice_state.copy(),'tracking':status.get(ns),'odom_received':ns in odom,'imu_fused':fused is not None,'imu_error':error,'encoder_valid':encoder_valid,'encoder_residual':residual,'lidar_min':{k:min(v) for k,v in packet['scans'].items()}}
 visual_msg=odom.get(ns)
 result['visual_response_timing']=deepcopy(visual_response_timings.get(ns,{}))
 result['stamp_ns']=ns
 result['signal_observation_audit']={'raw':packet.get('signal_observation'),'checked_color':checked_signal_color(packet,now_ns=time.monotonic_ns(),max_age_ns=perception_max_age_ns)}
 result['publication_diagnostics']=None if publication_diagnostics is None else dict(records=publication_diagnostics.count,error=publication_diagnostics.error,path=str(publication_diagnostics.path))
 result['visual_pose']=None if visual_msg is None else {'stamp_ns':stamp(visual_msg),'frame_id':visual_msg.header.frame_id,'child_frame_id':visual_msg.child_frame_id,'position':[visual_msg.pose.pose.position.x,visual_msg.pose.pose.position.y,visual_msg.pose.pose.position.z],'quaternion_xyzw':[visual_msg.pose.pose.orientation.x,visual_msg.pose.pose.orientation.y,visual_msg.pose.pose.orientation.z,visual_msg.pose.pose.orientation.w],'covariance':list(visual_msg.pose.covariance)}
 result['visual_pose_provenance']=None if visual_msg is None else {'schema':'r12_raw_visual_pose_v1','source_topic':'/visual_slam/tracking/odometry','field_representation':'received_unadapted','source_session_verified':False,'capture_association_verified':False,'covariance_calibrated':False}
 result['goal_observation']=goal_observation_state
 result['perception_candidates']=candidate_detections
 result['identity_confirmation']={'required_camera':'arm_camera','reason':wrist_identity.reason,'per_station_arm_view_integrated':wrist_surface_guard is not None,'grasp_pose_accepted':False}
 if wrist_surface_guard is not None:
  result['wrist_cycle']=observation.status()
  result['wrist_surface_guard']=dict(wrist_surface_guard.audit)
  wrist_packet=packet.get('perception',{})
  if wrist_packet.get('camera')=='arm_camera' and wrist_packet.get('stamp_ns')==ns and wrist_packet.get('detection_performed'):
   result['wrist_perception']={k:wrist_packet.get(k) for k in ('run_id','stamp_ns','camera','camera_frame','tags','detections','backend')}
   result['wrist_perception']['fresh']=bool(perception_check['matched'] and perception_check['live_freshness_verified'])
  if observation.done:atomic_json(out/'wrist_inspection_report.json',dict(observation.status(),surface_guard=wrist_surface_guard.audit,inventory=list(inventory.values()),stamp_ns=ns))
 result.update(arm_feedback_record(packet,arm_cfg['joint_names']))
 if os.getenv('R12_CAPTURE_JOINT_EPOCH','0')=='1':
  from arm_practice.capture_joint_epoch import joined_arm_feedback
  result.update(joined_arm_feedback(packet,result,received_monotonic_ns=packet_received_monotonic_ns,now_monotonic_ns=time.monotonic_ns()))
 result['base_sensor_feedback']=base_feedback_record(packet,twist,encoder_valid,fused,residual)
 result['perception_check']=check_packet_perception(packet,now_ns=time.monotonic_ns(),max_age_ns=perception_max_age_ns)
 if acquisition_timing_enabled:result.update(packet_parts_ns=packet.get('packet_parts_ns',{}),readback_parts_ns=packet.get('readback_parts_ns',{}),acquisition_stages_monotonic_ns=packet.get('acquisition_stages_monotonic_ns',{}),publication_stages_monotonic_ns=publication_stages,publication_operations=publication_operations,publication_auxiliary=publication_auxiliary)
 if shared_frame_enabled:result['shared_frame_ack']=shared_frame_ack
 result.update(capture_run_id=packet.get('capture_run_id'),frame_capture_monotonic_ns=packet.get('frame_capture_monotonic_ns'),packet_received_monotonic_ns=packet_received_monotonic_ns,packet_processed_monotonic_ns=time.monotonic_ns())
 if observation is not None:
  result['observation_waypoint_index']=observation.waypoint_index
  result['observation_state']=observation.state;result['observation_error']=observation.reason
  result['observation_sensor_checks']={'fresh':bool(fresh),'tracking':status.get(ns),'visual_tilt_available':visual_tilt is not None,'base_stopped':base_stopped}
  result['navigation_finished']=mission['finished']
  result['finished']=bool(mission['finished'] and observation.state=='complete')
  result['diagnostic_stop']=observation.state=='failed'
  result['execution_scope']='stationary_wrist_observation_only_not_grasp'
 goal_hold_allowed=bool(goal_stop is not None and goal_stop.status()['position_hold_allowed'])
 planner_hold_allowed=bool(os.getenv('R12_PLANNER_STOP_HOLD','0')=='1' and continuous_mode and not mission['finished'] and planner_stop_hold_allowed(command,path_state,healthy=healthy,transport_ok=transport_ok,base_stopped=base_stopped,guard=guard,visual_tilt=visual_tilt,motion_state=motion_state))
 hold_requested=bool((observation is not None and mission['finished']) or goal_hold_allowed or planner_hold_allowed)
 hold_enabled=bool(hold_requested and healthy and visual_tilt is not None and np.max(np.abs(visual_tilt))<np.deg2rad(4) and not motion_state.get('latched',False) and np.linalg.norm(command)<1e-6)
 retain_hold=bool((observation is not None and observation.state=='paused') or (goal_stop is not None and goal_stop.active))
 result['stationary_wheel_hold_torque']=wheel_hold.step(np.array(packet['position'])[wheel],hold_enabled,retain_anchor=retain_hold).tolist()
 result['stationary_wheel_hold_enabled']=hold_enabled
 result['planner_stop_hold_allowed']=planner_hold_allowed
 result['goal_stop_hold']=None if goal_stop is None else goal_stop.status()
 if goal_stop is not None and goal_stop.failed:result['diagnostic_stop']=True
 # Braking authorization uses only fresh fused sensors and existing motion interlocks.
 result['stop_brake_valid']=bool(healthy and fused is not None and transport_ok and guard is None and motion_state.get('reason') in (None, 'limited') and not motion_state.get('latched', False) and np.linalg.norm(command)<1e-6)
 result['stop_brake_gyro_z']=float(fused['gyro'][2]) if result['stop_brake_valid'] else None
 if stationary_capture:
  result.update(command=[0.,0.,0.],wheel_action=[0.]*4,wheel_scale=1.,arm_target=None,gripper_width_m=None,stationary_wheel_hold_torque=[0.]*4,stationary_wheel_hold_enabled=False,stop_brake_valid=False,execution_scope='stationary_sensor_capture_only')
  result['command_stages']['final']=[0.,0.,0.]
 if oriented_session is not None:
  wire,send_issue=encode_base_response(result,oriented_session._context_reason)
  if send_issue is not None:envelope.velocity[:]=0;envelope.accel[:]=0
 else:wire=(json.dumps(result)+'\n').encode()
 conn.sendall(wire)
 rows.append({'stamp_ns':ns,**result})
 if len(rows)%100==0:
  if result_journal is not None:result_journal.checkpoint(rows)
  else:atomic_json(out/'ros_results.json',rows)
  if callback_duration_rows:atomic_json(out/'callback_durations.json',callback_duration_rows)
  if path_result_audit:atomic_json(out/'path_result_audit.json',path_result_audit)
if path_worker is not None:path_worker.close()
if oriented_session is not None:oriented_session.close()
if callback_duration_rows:atomic_json(out/'callback_durations.json',callback_duration_rows)
if path_result_audit:atomic_json(out/'path_result_audit.json',path_result_audit)
if cloud_audit_enabled:atomic_json(out/'cloud_publication_audit.json',cloud_publication_audit)
if os.getenv('R12_SLICE_CALLBACK_TIMING','0')=='1':atomic_json(out/'slice_callback_timings.json',slice_callback_timings)
if sequence_recording:
 import hashlib
 sequence_dir=out/'sensor_sequence';sequence_dir.mkdir(exist_ok=False)
 manifest=[]
 for index,(record_packet,arrays) in enumerate(sequence_frames):
  path=sequence_dir/f'{index:03d}.npz';np.savez_compressed(path,**arrays)
  manifest.append(dict(index=index,file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),packet=record_packet))
 atomic_json(sequence_dir/'manifest.json',dict(scope='stationary temporal sequence; not moving robot or real hardware',frames=manifest,uncompressed_bytes=sequence_bytes,original_timestamps_preserved=True))
 sequence_frames.clear()
if publication_diagnostics is not None:publication_diagnostics.flush()
if result_journal is not None:result_journal.checkpoint(rows)
atomic_json(out/'ros_results.json',rows)
if result_journal is not None:result_journal.close()
atomic_json(out/'visual_arrival_audit.json',list(visual_timeouts.values()));node.destroy_node();rclpy.shutdown()
