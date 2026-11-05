from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 root=Path(__file__).resolve().parent
 repo=root.parents[1];urdf=repo/'models/r12/r12_mobile_gripper.urdf';tree=ET.parse(urdf);robot=tree.getroot()
 # Sensor transforms are owned by the timestamped sensor bridge; avoid two TF publishers.
 for element in list(robot):
  if element.get('name','').startswith(('r07_camera','sim_','lidar_')):robot.remove(element)
 for mesh in robot.iter('mesh'):
  mesh.set('filename','file://'+str((urdf.parent/mesh.get('filename')).resolve()))
 return LaunchDescription([
 Node(package='robot_state_publisher',executable='robot_state_publisher',name='r12_robot_state_publisher',parameters=[{'robot_description':ET.tostring(robot,encoding='unicode'),'use_sim_time':True}],remappings=[('joint_states','/r12/joint_states')],output='screen'),
 Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[str(root/'ekf.yaml')],output='screen'),
 Node(package='nvblox_ros',executable='nvblox_node',name='nvblox_node',parameters=[str(root/'nvblox.yaml')],remappings=[(f'/camera_{i}/depth/{kind}',f'/r12/{name}/left/'+('depth' if kind=='image' else 'camera_info')) for i,name in enumerate(['stereo','r07_camera2','r07_camera3','arm_camera']) for kind in ['image','camera_info']],output='screen'),
 Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[str(root/'nav2.yaml')],output='screen'),
 Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_planning',parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':30.0,'node_names':['planner_server']}],output='screen')])
