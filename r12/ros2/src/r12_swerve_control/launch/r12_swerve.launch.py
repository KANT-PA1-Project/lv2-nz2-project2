"""r12 스월브 실물 스택: 결정론적 컨트롤러 + OpenCR 브리지. 공용 파라미터는 config/r12_swerve.yaml.

bridge:=false 로 브리지 없이 컨트롤러만(시뮬/상위 테스트). 학습 노드(r12_learned_swerve)와는
둘 다 /r12/swerve_joint_commands 를 발행하므로 동시에 쓰지 않는다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'r12_swerve_control'


def generate_launch_description():
    params = os.path.join(get_package_share_directory(PKG), 'config', 'r12_swerve.yaml')
    bridge = LaunchConfiguration('bridge')
    return LaunchDescription([
        DeclareLaunchArgument('bridge', default_value='true'),
        Node(package=PKG, executable='swerve_controller',
             name='r12_swerve_controller', parameters=[params], output='screen'),
        Node(package=PKG, executable='opencr_bridge',
             name='r12_opencr_bridge', parameters=[params], output='screen',
             condition=IfCondition(bridge)),
    ])
