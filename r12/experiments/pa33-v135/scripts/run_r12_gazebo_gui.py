"""Owned Gazebo GUI, ROS sensor bridge and RViz2 launch for scan-drive porting."""
import argparse
import fcntl
import json
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as E
from pathlib import Path
import yaml
from r12_gazebo_env import environment

repo=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
args=ap.parse_args();scene=args.scene.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
lock=(out.parent/'gazebo_gui.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
env=environment();env.update(ROS_DOMAIN_ID='88',GZ_PARTITION='r12_gazebo_88',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',QT_QPA_PLATFORM='xcb')
env.update(CUDA_VISIBLE_DEVICES='0',__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia',__VK_LAYER_NV_optimus='NVIDIA_only')
profile=out.parent/'v309_shm64m.xml'
env['FASTDDS_DEFAULT_PROFILES_FILE']=str(profile)
env.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
env.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
env['QML_IMPORT_PATH']=str(out.parent/'gazebo_runtime/root/usr/lib/x86_64-linux-gnu/qt6/qml')
env['QML2_IMPORT_PATH']=env['QML_IMPORT_PATH']
bridges=[]
def bridge(gz,ros,typ,gztyp):
    bridges.append(dict(gz_topic_name=gz,ros_topic_name=ros,ros_type_name=typ,gz_type_name=gztyp,direction='GZ_TO_ROS',qos_profile='SENSOR_DATA'))
bridge('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock')
bridge('/world/r12_scan_drive/model/r12/joint_state','/r12/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model')
for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
    for side in ['left','right']:
        gz='/r12/gz/'+camera+'_'+side+'_optical';ros='/r12/'+('stereo' if camera=='r07_camera1' else camera)+'/'+side
        bridge(gz+'/image',ros+'/image','sensor_msgs/msg/Image','gz.msgs.Image')
        bridge(gz+'/camera_info',ros+'/camera_info','sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo')
        if side=='left':
            bridge(gz+'/depth_image',ros+'/depth','sensor_msgs/msg/Image','gz.msgs.Image')
            bridge(gz+'/points','/r12/'+camera+'/points','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked')
            if camera=='r07_camera1':
                for roscloud in ['/r12/points','/r12/points_rgb']:
                    bridge(gz+'/points',roscloud,'sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked')
for i in [1,2]:bridge(f'/r12/gz/imu{i}',f'/opencr_{i}/imu','sensor_msgs/msg/Imu','gz.msgs.IMU')
for name in ['front_right','rear_left','low_front_left','low_rear_right']:
    bridge('/r12/gz/lidar/'+name,'/r12/lidar/'+name,'sensor_msgs/msg/LaserScan','gz.msgs.LaserScan')
(out/'bridge.yaml').write_text(yaml.safe_dump(bridges))
robot=E.parse(scene/'robot.urdf').getroot()
for mesh in robot.findall('.//mesh'):
    mesh.set('filename',Path(mesh.get('filename')).as_uri())
# Gazebo and RViz share the calibrated wrist extrinsic, not the older URDF wrist approximation.
cfg=json.loads((repo/'configs/sensors/camera_rig.json').read_text())['arm_camera']
for side,sign in [('left',1),('right',-1)]:
    name='arm_camera_'+side+'_optical'
    E.SubElement(robot,'link',name=name)
    joint=E.SubElement(robot,'joint',name=name+'_fixed',type='fixed')
    E.SubElement(joint,'parent',link='arm_link5');E.SubElement(joint,'child',link=name)
    xyz=cfg['optical_center_xyz_m'].copy();xyz[1]+=sign*cfg['stereo_baseline_m']/2
    E.SubElement(joint,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,cfg['optical_rpy_rad'])))
description=E.tostring(robot,encoding='unicode')
(out/'robot_description.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':description,'use_sim_time':True}}}))
rviz=yaml.safe_load((repo/'configs/sensors/r12_mission.rviz').read_text());rviz['Visualization Manager']['Global Options']['Fixed Frame']='base_link'
rviz['Visualization Manager']['Global Options']['Frame Rate']=10
rviz['Visualization Manager']['Views']['Current'].update(Distance=3.,Focal_Point={'X':0,'Y':0,'Z':.4})
rviz['Visualization Manager']['Views']['Current']['Focal Point']={'X':0,'Y':0,'Z':.4}
for display in rviz['Visualization Manager']['Displays']:
    if display['Class'] in ['rviz_default_plugins/Path','rviz_default_plugins/Marker']:display['Enabled']=False
    if display['Class'] in ['rviz_default_plugins/Image','rviz_default_plugins/PointCloud2','rviz_default_plugins/LaserScan']:
        display['Topic'].update({'Reliability Policy':'Best Effort','History Policy':'Keep Last','Depth':1})
        display['Queue Size']=1
    if display['Class']=='rviz_default_plugins/TF':display['Enabled']=True
(out/'gazebo.rviz').write_text(yaml.safe_dump(rviz))
children=[];files=[];report={'stage':'Gazebo GUI sensor bringup; navigation/inference adapter not yet connected','domain':88,'partition':env['GZ_PARTITION'],'gui':True,'rviz':True,'drive_commands_enabled':False,'requested_gpu_index':0,'render_selection':'NVIDIA PRIME OpenGL offload; verify renderer, CUDA_VISIBLE_DEVICES alone does not select graphics','start_time':time.time(),'children':[]}
def start(cmd,name,childenv=env):
    f=(out/(name+'.log')).open('w');files.append(f)
    p=subprocess.Popen(cmd,env=childenv,cwd=repo,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p)
    report['children'].append({'name':name,'pid':p.pid,'command':cmd});(out/'running.json').write_text(json.dumps(report,indent=2));return p
stop=False
def interrupted(*_):
    global stop
    stop=True
signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
try:
    gz=start(['gz','sim','-s','-r','-v','3',str(scene/'world.sdf')],'gazebo')
    gzprefix=out.parent/'gazebo_runtime/root/opt/ros/lyrical/opt/gz_sim_vendor'
    gui=start([str(gzprefix/'libexec/gz/sim10/gz-sim-gui-client'),'-v','3',
               '--gui-config',str(gzprefix/'share/gz/gz-sim/gui/gui.config')],'gazebo_gui')
    br=start(['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(out/'bridge.yaml')],'bridge')
    hostenv=os.environ.copy();hostenv.update({k:env[k] for k in ['ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','QT_QPA_PLATFORM','CUDA_VISIBLE_DEVICES','__NV_PRIME_RENDER_OFFLOAD','__GLX_VENDOR_LIBRARY_NAME','__VK_LAYER_NV_optimus','FASTDDS_DEFAULT_PROFILES_FILE']})
    hostenv.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
    hostenv.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
    rsp=start(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(out/'robot_description.yaml'),'-r','joint_states:=/r12/joint_states'],'robot_state_publisher',hostenv)
    rv=start(['rviz2','-d',str(out/'gazebo.rviz'),'--ros-args','-p','use_sim_time:=true'],'rviz',hostenv)
    while not stop and not (out/'STOP').exists():
        report['returncodes']={r['name']:p.poll() for r,p in zip(report['children'],children)}
        report['rss_kib']={}
        for r,p in zip(report['children'],children):
            try:
                status=Path(f'/proc/{p.pid}/status').read_text()
                report['rss_kib'][r['name']]=int(next(s.split()[1] for s in status.splitlines() if s.startswith('VmRSS:')))
            except (OSError,StopIteration):pass
        report['checked_at']=time.time();(out/'running.json').write_text(json.dumps(report,indent=2))
        exited={name:code for name,code in report['returncodes'].items() if code is not None}
        if exited:
            report['stop_reason']={'required_process_exited':exited}
            break
        time.sleep(2)
finally:
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
    deadline=time.monotonic()+10
    for p in children:
        try:p.wait(timeout=max(.1,deadline-time.monotonic()))
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
    report['ended_at']=time.time();report['returncodes']={r['name']:p.returncode for r,p in zip(report['children'],children)}
    (out/'completion.json').write_text(json.dumps(report,indent=2))
    for f in files:f.close()
