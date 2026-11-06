"""Domain33 localhost ROS publication and subscriber audit; never mission commands."""
import argparse,hashlib,json,os,socket,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parent;sys.path.insert(0,str(root/'src'))
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet,receive_packet,peer_credentials,require_peer
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
from arm_practice.native_rgbd.ros_current_contract import CurrentROSGuard
from arm_practice.native_rgbd.renderer_epoch_issuer import canonical
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import Image,Imu,JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from builtin_interfaces.msg import Time
p=argparse.ArgumentParser();p.add_argument('--socket');p.add_argument('--producer-pid',type=int);p.add_argument('--run-id');p.add_argument('--output');a=p.parse_args()
assert os.environ['ROS_DOMAIN_ID']=='33' and os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST'
schema=MixedCameraSchema.from_file(root/'configs/mixed_camera_schema.json');out=Path(a.output)
sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);sock.settimeout(25);sock.connect(a.socket);require_peer(peer_credentials(sock),a.producer_pid,os.getuid())
rclpy.init();pubnode=Node('r12_native_diagnostic_publisher');subnode=Node('r12_native_diagnostic_subscriber');executor=SingleThreadedExecutor();executor.add_node(pubnode);executor.add_node(subnode)
qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST)
prefix='/r12/native_diagnostic/'+a.run_id;publishers={};subscriptions=[];expected={};received={};errors=[];records=[]
def ns(t):return t.sec*10**9+t.nanosec
def tm(n):return Time(sec=n//10**9,nanosec=n%10**9)
def observe(topic,msg):
    callback_start=time.monotonic_ns()
    if isinstance(msg,Image):
        copy_start=time.monotonic_ns();raw_copy=memoryview(msg.data).toreadonly();copy_end=time.monotonic_ns();digest_start=time.monotonic_ns();raw_digest=hashlib.sha256(raw_copy).hexdigest();digest_end=time.monotonic_ns();image_callback_details.append(dict(topic=topic,bytes=len(raw_copy),copy_start_ns=copy_start,copy_end_ns=copy_end,hash_start_ns=digest_start,hash_end_ns=digest_end))
        key=(topic,ns(msg.header.stamp));actual=dict(stamp_ns=key[1],frame=msg.header.frame_id,sha256=raw_digest,shape=[msg.height,msg.width],encoding=msg.encoding)
    elif isinstance(msg,Imu):key=(topic,ns(msg.header.stamp));actual=dict(stamp_ns=key[1],frame=msg.header.frame_id,gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],accel=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
    elif isinstance(msg,JointState):key=(topic,ns(msg.header.stamp));actual=dict(stamp_ns=key[1],names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
    elif isinstance(msg,Clock):key=(topic,ns(msg.clock));actual=dict(stamp_ns=key[1])
    elif isinstance(msg,String):actual=json.loads(msg.data);key=(topic,actual['stamp_ns'])
    else:
        key=(topic,ns(msg.transforms[0].header.stamp));actual=dict(stamp_ns=key[1],transforms=[dict(frame=x.header.frame_id,child=x.child_frame_id,stamp_ns=ns(x.header.stamp),translation=[x.transform.translation.x,x.transform.translation.y,x.transform.translation.z],quaternion=[x.transform.rotation.x,x.transform.rotation.y,x.transform.rotation.z,x.transform.rotation.w])for x in msg.transforms])
    if key not in expected or actual!=expected[key] or key in received:errors.append(dict(topic=topic,stamp_ns=key[1],reason='unexpected_modified_or_duplicate_ROS_sample'))
    else:received[key]=actual
    callbacks.append(dict(topic=topic,stamp_ns=key[1],start_ns=callback_start,end_ns=time.monotonic_ns()))

def add(topic,kind):
    name=prefix+'/'+topic;publishers[topic]=pubnode.create_publisher(kind,name,qos);subscriptions.append(subnode.create_subscription(kind,name,lambda msg,t=topic:observe(t,msg),qos))
for name in schema.config['cameras']:add(name,Image)
for name in schema.config['depth']:add(name,Image)
for name in ['front_imu','rear_imu']:add(name,Imu)
for name,kind in [('clock',Clock),('current_arm_encoder',JointState),('capture_tf',TFMessage),('capture_perception_request',String)]:add(name,kind)
deadline=time.monotonic()+10
while not all(x.get_subscription_count()>=1 for x in publishers.values()):
    if time.monotonic()>deadline:raise RuntimeError('local_ROS_discovery_timeout')
    executor.spin_once(timeout_sec=.05)
guard=CurrentROSGuard(a.run_id,hashlib.sha256((root/'native_entrypoint_manifest.json').read_bytes()).hexdigest());early_records=[];last_early=None
send_packet(sock,dict(kind='ready',run_id=a.run_id,domain=33))
try:
 while True:
    receive_start=time.monotonic_ns()
    message,blobs=receive_packet(sock,wire_version=schema.wire_schema,camera_schema=schema,allow_control_finish=True)
    receive_end=time.monotonic_ns()
    if message['kind']=='finish':break
    if message['kind']=='current_feedback':
        if blobs:raise ValueError('unexpected_feedback_blobs')
        require_peer(peer_credentials(sock),a.producer_pid,os.getuid())
        validation_start=time.monotonic_ns();step=guard.validate(message,validation_start);validation_end=time.monotonic_ns()
        feedback=message['feedback'];position=feedback['position'];velocity=feedback['velocity'];original_hash=hashlib.sha256(canonical(feedback)).hexdigest();stamp=step*10**9//960
        expected={};received={};errors=[];callbacks=[]
        j=JointState();j.header.stamp=tm(stamp);j.name=['arm_joint'+str(i)for i in range(1,6)];j.position=position['q'];j.velocity=velocity['velocity']
        expected[('current_arm_encoder',stamp)]=dict(stamp_ns=stamp,names=list(j.name),position=list(j.position),velocity=list(j.velocity))
        publish_start=time.monotonic_ns();publishers['current_arm_encoder'].publish(j);publish_end=time.monotonic_ns();deadline=time.monotonic()+1
        while len(received)<1 and not errors and time.monotonic()<deadline:executor.spin_once(timeout_sec=.002)
        callback_end=time.monotonic_ns()
        if errors or len(received)!=1:raise ValueError('early_ROS_feedback_callback_failed')
        guard.validate(message,callback_end,commit=False)
        if hashlib.sha256(canonical(feedback)).hexdigest()!=original_hash:raise ValueError('early_ROS_feedback_changed')
        last_early=dict(kind='current_feedback_published',sequence=message['sequence'],step=step,source_read_start_ns=velocity['read_start_ns'],source_read_end_ns=velocity['read_end_ns'],feedback_sha256=original_hash,receive_ns=receive_end,validation_start_ns=validation_start,validation_end_ns=validation_end,publish_start_ns=publish_start,publish_end_ns=publish_end,callback_start_ns=callbacks[0]['start_ns'],callback_end_ns=callbacks[0]['end_ns'],consume_validation_end_ns=callback_end,receive_age_ms=(receive_end-velocity['read_start_ns'])/1e6,callback_age_ms=(callbacks[0]['start_ns']-velocity['read_start_ns'])/1e6,consume_age_ms=(callback_end-velocity['read_start_ns'])/1e6,current_feedback_fresh=True,motion_authorized=False)
        early_records.append(last_early);(out/'ROS_current_feedback_results.json').write_text(json.dumps(early_records,indent=2));send_packet(sock,last_early);continue
    body=message['certificate']['body'];plan=message['plan'];verification=message['live_issuer_verification'];expected={};received={};errors=[];pending=[];callbacks=[];hash_timings=[];publish_timings=[];image_callback_details=[];image_construction=[];spin_timings=[];construction_start=time.monotonic_ns()
    if message['run_id']!=a.run_id or body['run_id']!=a.run_id or not verification['rgbd_capture_association_verified'] or verification['capture_physics_step']!=body['capture_physics_step']:raise ValueError('invalid_live_producer_verification_binding')
    if time.monotonic_ns()-body['capture_monotonic_lower_bound_ns']>500_000_000:raise ValueError('capture_stale_before_ROS_publish')
    hist=plan['historical'];current=plan['current_encoder'];capture_stamp=hist['stamp_ns'];current_stamp=plan['clock']['stamp_ns'];guard.match_image_plan(current)
    if hist['camera_tf']!=body['encoder_history']['camera_fk'] or hist['image_hashes']!=body['pixel_sha256'] or hist['depth_hashes']!=body['depth_sha256']:raise ValueError('historical_plan_changed')
    for descriptor,blob in zip(message['arrays'],blobs):
        name=descriptor['name'];kind=descriptor['kind'];target=body['pixel_sha256'if kind=='rgb'else'depth_sha256'][name]
        hash_start=time.monotonic_ns()
        if hashlib.sha256(blob).hexdigest()!=target:raise ValueError('raw_bytes_changed')
        hash_timings.append(dict(name=name,start_ns=hash_start,end_ns=time.monotonic_ns(),bytes=len(blob)))
        image_start=time.monotonic_ns();height,width=descriptor['shape'][:2];im=Image();im.header.stamp=tm(capture_stamp);im.header.frame_id=name if kind=='rgb'else body['depth_camera_map'][name];im.height=height;im.width=width;im.encoding='rgb8'if kind=='rgb'else'32FC1';im.step=width*(3 if kind=='rgb'else 4);assign_start=time.monotonic_ns();im.data=blob;image_end=time.monotonic_ns();image_construction.append(dict(name=name,start_ns=image_start,assign_start_ns=assign_start,end_ns=image_end))
        expected[(name,capture_stamp)]=dict(stamp_ns=capture_stamp,frame=im.header.frame_id,sha256=target,shape=[height,width],encoding=im.encoding);pending.append((name,im))
    transforms=[];tf_expected=[]
    for name,matrix in hist['camera_tf'].items():
        matrix=np.asarray(matrix);tr=TransformStamped();tr.header.stamp=tm(capture_stamp);tr.header.frame_id='base_link';tr.child_frame_id=name;xyz=matrix[:3,3].tolist();quat=Rotation.from_matrix(matrix[:3,:3]).as_quat().tolist();tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=xyz;tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=quat;transforms.append(tr);tf_expected.append(dict(frame='base_link',child=name,stamp_ns=capture_stamp,translation=xyz,quaternion=quat))
    expected[('capture_tf',capture_stamp)]=dict(stamp_ns=capture_stamp,transforms=tf_expected);pending.append(('capture_tf',TFMessage(transforms=transforms)))
    request=hist['perception_request'];expected[('capture_perception_request',capture_stamp)]=request;pending.append(('capture_perception_request',String(data=json.dumps(request))))
    expected[('clock',current_stamp)]=dict(stamp_ns=current_stamp);pending.append(('clock',Clock(clock=tm(current_stamp))))
    for sample in plan['imu_native_samples']:
        topic=['front_imu','rear_imu'][sample['sensor_index']];im=Imu();im.header.stamp=tm(sample['stamp_ns']);im.header.frame_id=sample['frame_id'];im.orientation_covariance[0]=-1.;im.angular_velocity.x,im.angular_velocity.y,im.angular_velocity.z=sample['gyro'];im.linear_acceleration.x,im.linear_acceleration.y,im.linear_acceleration.z=sample['accel'];expected[(topic,sample['stamp_ns'])]=dict(stamp_ns=sample['stamp_ns'],frame=sample['frame_id'],gyro=sample['gyro'],accel=sample['accel']);pending.append((topic,im))
    construction_end=time.monotonic_ns()
    started=time.monotonic_ns()
    for topic,msg in pending:
        publish_start=time.monotonic_ns();publishers[topic].publish(msg);publish_timings.append(dict(topic=topic,start_ns=publish_start,end_ns=time.monotonic_ns()))
    deadline=time.monotonic()+8
    while len(received)<len(expected) and not errors and time.monotonic()<deadline:
        spin_start=time.monotonic_ns();cpu_start=time.thread_time_ns();count_before=len(callbacks);executor.spin_once(timeout_sec=.02);spin_timings.append(dict(start_ns=spin_start,end_ns=time.monotonic_ns(),thread_cpu_ns=time.thread_time_ns()-cpu_start,callbacks_before=count_before,callbacks_after=len(callbacks)))
    result=dict(kind='publication_result',all_received=len(received)==len(expected)and not errors,expected=len(expected),received=len(received),errors=errors,capture_stamp_ns=capture_stamp,current_stamp_ns=current_stamp,publication_started_ns=started,publication_finished_ns=time.monotonic_ns(),capture_wall_age_ms=(started-body['capture_monotonic_lower_bound_ns'])/1e6,current_encoder_wall_age_ms=(last_early['publish_start_ns']-current['velocity']['read_start_ns'])/1e6,motion_authorized=False,mission_admitted=False,domain=33,raw_bytes_hash_verified=len(blobs),received_samples=[dict(topic=k[0],**v)for k,v in received.items()])
    result['stages']=dict(unix_receive_start_ns=receive_start,unix_receive_end_ns=receive_end,message_construction_start_ns=construction_start,message_construction_end_ns=construction_end,raw_hash=hash_timings,publish=publish_timings,callbacks=callbacks,image_construction=image_construction,image_callback_copy_hash=image_callback_details,spin=spin_timings)
    encoder_callbacks=[x for x in callbacks if x['topic']=='current_arm_encoder']
    result['current_encoder_receipt_age_ms']=last_early['callback_age_ms'];result['early_current_feedback']=last_early;result['current_encoder_separate_from_image_batch']=True
    result['current_encoder_receipt_fresh']=last_early['current_feedback_fresh']
    result['camera_last_callback_age_ms']=max((x['end_ns']-body['capture_monotonic_lower_bound_ns'])/1e6 for x in callbacks if x['topic'] in schema.config['cameras'] or x['topic'] in schema.config['depth'])
    records.append(result);disk_start=time.monotonic_ns();(out/'ROS_publication_results.json').write_text(json.dumps(records,indent=2));disk_end=time.monotonic_ns();ack_timing={};send_packet(sock,result,timing=ack_timing)
    with (out/'ROS_ack_timeline.jsonl').open('a') as ack_log:ack_log.write(json.dumps(dict(capture_stamp_ns=capture_stamp,result_disk_start_ns=disk_start,result_disk_end_ns=disk_end,ack=ack_timing))+'\n')
finally:
 executor.shutdown();subnode.destroy_node();pubnode.destroy_node();rclpy.shutdown();sock.close()
