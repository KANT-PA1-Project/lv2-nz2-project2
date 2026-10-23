# =============================================================
# 패키지 : nz2_description
# 파일   : launch/display.launch.py
# 역할   : URDF 확인용. robot_state_publisher + joint_state_publisher + RViz
#
# [입력]  launch 인자 use_rviz (기본 true)
# [출력]  /robot_description, /tf, /tf_static
# [해야 할 일 / TODO]
#   - RViz 설정 파일(.rviz)을 만들면 rviz2에 -d 옵션으로 넘긴다
# [ROS2 메모] robot_state_publisher = URDF와 관절 각도로 TF를 계산해 발행하는 표준 노드
# =============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('nz2_description'), 'urdf', 'nz2_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('use_rviz')),
             output='screen'),
    ])
