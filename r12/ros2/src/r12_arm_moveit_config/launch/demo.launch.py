"""Start R12 planning and a mock-hardware trajectory controller."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('r12_arm_moveit_config'))
    config = (MoveItConfigsBuilder('r12', package_name='r12_arm_moveit_config')
              .robot_description(file_path='config/r12_exploration.urdf' if os.environ.get('R12_EXPLORATION') == '1' else 'config/r12.urdf')
              .robot_description_semantic(file_path='config/r12.srdf')
              .planning_pipelines(pipelines=['ompl'])
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .to_moveit_configs())
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'base_link']),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(share/'config/ros2_controllers.yaml')], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['arm_controller', '--param-file', str(share/'config/ros2_controllers.yaml')]),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[config.to_dict()], output='screen'),
    ])
