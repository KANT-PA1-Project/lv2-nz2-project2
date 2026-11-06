from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
 frames=[];remaps=[]
 for index,camera in enumerate(['r07_camera1','r07_camera2','r07_camera3']):
  for side_index,side in enumerate(['left','right']):
   i=index*2+side_index;frames.append(camera+'_'+side+'_optical');base='/r12/'+('stereo' if index==0 else camera)+'/'+side
   remaps.extend([(f'visual_slam/image_{i}',base+'/image'),(f'visual_slam/camera_info_{i}',base+'/camera_info')])
 node=ComposableNode(package='isaac_ros_cuvslam',plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',name='r12_visual_slam',parameters=[{'enable_debug_mode':True,'debug_dump_path':'/home/seongmin/Documents/Codex/2026-10-30/https-github-com-seongminjaden-manipulator-isaac61/outputs/r12/v329_debug','use_sim_time':True,'num_cameras':6,'min_num_images':6,'tracking_mode':0,'enable_ground_constraint_in_odometry':False,'enable_ground_constraint_in_slam':False,'rectified_images':True,'base_frame':'base_link','odom_frame':'odom','map_frame':'vslam_map','camera_optical_frames':frames,'publish_odom_to_base_tf':False,'publish_map_to_odom_tf':False,'enable_localization_n_mapping':True,'image_jitter_threshold_ms':200.,'image_qos':'DEFAULT','image_qos_depth':10}],remappings=remaps)
 return LaunchDescription([ComposableNodeContainer(name='r12_vslam_container',namespace='',package='rclcpp_components',executable='component_container_mt',composable_node_descriptions=[node],output='screen')])
