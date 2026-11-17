"""Owned RViz signal-shutdown backtraces; synthetic data, no robot commands."""
import os,time,json,subprocess,signal
from pathlib import Path
import yaml,rclpy
from sensor_msgs.msg import Image
repo=Path(__file__).resolve().parents[1]
out=repo.parents[1]/'outputs/r12/v387_gdb_probe';out.mkdir(exist_ok=False)
reports=[]
for count in (2,0):
 cfg=yaml.safe_load((repo/'configs/sensors/r12_mission.rviz').read_text());n=0
 for d in cfg['Visualization Manager']['Displays']:
  image=d['Class']=='rviz_default_plugins/Image'
  if image and count==0:d['Enabled']=False
  if image:n+=1
 path=out/f'image{count}.rviz';path.write_text(yaml.safe_dump(cfg))
 os.environ['ROS_DOMAIN_ID']=str(230 if count==2 else 231)
 rclpy.init();node=rclpy.create_node('r12_gdb_probe')
 pubs=[node.create_publisher(Image,t,10) for t in ['/r12/stereo/left/image','/r12/arm_camera/left/image','/r12/r07_camera2/left/image','/r12/r07_camera3/left/image']]
 msg=Image();msg.header.frame_id='map';msg.height=480;msg.width=640;msg.encoding='rgb8';msg.step=1920;msg.data=bytes([80,120,160])*(640*480)
 report={'image_display_count':count,'other_displays':'original mission enabled flags','stop_method':'SIGTERM to owned inferior','not_equivalent_to_WM_DELETE':True}
 with (out/f'image{count}.log').open('w') as log:
  p=subprocess.Popen(['gdb','-q','-batch','-ex','set pagination off','-ex','handle SIGTERM nostop noprint pass','-ex','run','-ex','thread apply all bt 12','--args','/opt/ros/lyrical/lib/rviz2/rviz2','-d',str(path)],stdout=log,stderr=log,start_new_session=True)
  inferior=None
  try:
   time.sleep(6)
   for _ in range(20):
    msg.header.stamp=node.get_clock().now().to_msg()
    for pub in pubs:pub.publish(msg)
    time.sleep(.05)
   report['subscriptions']=[pub.get_subscription_count() for pub in pubs]
   children=Path(f'/proc/{p.pid}/task/{p.pid}/children').read_text().split()
   for pid in children:
    if Path(f'/proc/{pid}/comm').read_text().strip()=='rviz2':inferior=int(pid)
   report['inferior_pid']=inferior
   if inferior:os.kill(inferior,signal.SIGTERM)
   try:p.wait(timeout=15)
   except subprocess.TimeoutExpired:report['timeout']=True
  finally:
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGKILL);p.wait()
   report['gdb_exit_code']=p.returncode
   report['inferior_alive']=bool(inferior and Path(f'/proc/{inferior}').exists())
   node.destroy_node();rclpy.shutdown()
 reports.append(report);(out/'results.json').write_text(json.dumps(reports,indent=2));print(json.dumps(report),flush=True)
