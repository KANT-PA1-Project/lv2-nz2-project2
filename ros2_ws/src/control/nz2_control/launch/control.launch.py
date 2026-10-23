# =============================================================
# 패키지 : nz2_control
# 파일   : launch/control.launch.py
# 레이어 : control
# 역할   : swerve_controller · arm_controller · opencr_bridge 실행 (params 로드)
# =============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('nz2_control')
    swerve = os.path.join(share, 'config', 'swerve_params.yaml')
    arm = os.path.join(share, 'config', 'arm_params.yaml')
    return LaunchDescription([
        Node(package='nz2_control', executable='swerve_controller_node', name='swerve_controller_node',
             parameters=[swerve], output='screen'),
        Node(package='nz2_control', executable='arm_controller_node', name='arm_controller_node',
             parameters=[arm], output='screen'),
        Node(package='nz2_control', executable='opencr_bridge_node', name='opencr_bridge_node',
             parameters=[swerve], output='screen'),
    ])
