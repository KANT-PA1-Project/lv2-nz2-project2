"""Synthetic stationary RViz-only display shutdown comparison, no robot controller."""
import os,time,json,subprocess,signal,struct
from pathlib import Path
import yaml,rclpy
from sensor_msgs.msg import Image,PointCloud2,PointField
from r12_close_owned_window import close_owned_window
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12/v385_display_probe';out.mkdir(exist_ok=False)
reports=[]
for kind in ('image','cloud'):
 cfg=yaml.safe_load((repo/'configs/sensors/r12_mission.rviz').read_text())
 for display in cfg['Visualization Manager']['Displays']:
  display['Enabled']=display['Class']==('rviz_default_plugins/Image' if kind=='image' else 'rviz_default_plugins/PointCloud2') or display['Class']=='rviz_default_plugins/Grid'
 path=out/(kind+'.rviz');path.write_text(yaml.safe_dump(cfg));env=os.environ.copy();env['ROS_DOMAIN_ID']=str(226 if kind=='image' else 227);os.environ['ROS_DOMAIN_ID']=env['ROS_DOMAIN_ID'];rclpy.init();node=rclpy.create_node('r12_synthetic_display_probe');pubs=[]
 if kind=='image':
  for topic in ['/r12/stereo/left/image','/r12/arm_camera/left/image','/r12/r07_camera2/left/image','/r12/r07_camera3/left/image']:pubs.append(node.create_publisher(Image,topic,10))
  msg=Image();msg.header.frame_id='map';msg.height=480;msg.width=640;msg.encoding='rgb8';msg.step=1920;msg.data=bytes([80,120,160])*(640*480)
 else:
  pubs=[node.create_publisher(PointCloud2,t,10) for t in ['/r12/points','/r12/points_rgb']];msg=PointCloud2();msg.header.frame_id='map';msg.height=1;msg.width=1;msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z','rgb'])];msg.point_step=16;msg.row_step=16;msg.is_dense=True;msg.data=struct.pack('ffff',0,0,1,0)
 report={'case':kind}
 with (out/(kind+'.log')).open('w') as log:
  child=subprocess.Popen(['rviz2','-d',str(path)],env=env,stdout=log,stderr=log,start_new_session=True)
  try:
   time.sleep(4)
   for _ in range(20):
    msg.header.stamp=node.get_clock().now().to_msg()
    for pub in pubs:pub.publish(msg)
    time.sleep(.05)
   report['subscriptions']=[p.get_subscription_count() for p in pubs];time.sleep(1);report['close']=close_owned_window(child.pid)
   try:child.wait(timeout=5);report['normal_close']=True
   except subprocess.TimeoutExpired:report['normal_close']=False
  finally:
   if child.poll() is None:
    os.killpg(child.pid,signal.SIGINT)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
   report['exit_code']=child.returncode
   node.destroy_node();rclpy.shutdown()
 reports.append(report);(out/'results.json').write_text(json.dumps(reports,indent=2));print(json.dumps(report),flush=True)
