"""MoveIt connected to the separately running Isaac Sim R12 bridge/adapter."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (MoveItConfigsBuilder('r12', package_name='r12_arm_moveit_config')
              .robot_description(file_path='config/r12_exploration.urdf' if os.environ.get('R12_EXPLORATION') == '1' else 'config/r12.urdf')
              .robot_description_semantic(file_path='config/r12.srdf')
              .planning_pipelines(pipelines=['ompl'])
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .to_moveit_configs())
    params = config.to_dict()
    manager = params['moveit_simple_controller_manager']
    manager['r12/arm_controller'] = manager.pop('arm_controller')
    manager['controller_names'] = ['r12/arm_controller']
    params['use_sim_time'] = True
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description, {'use_sim_time':True}],
             remappings=[('joint_states','/r12/joint_states')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id','world','--child-frame-id','base_link'],
             parameters=[{'use_sim_time':True}]),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[params], remappings=[('joint_states','/r12/joint_states')],output='screen'),
    ])
