# =============================================================
# 패키지 : nz2_planning
# 파일   : launch/planning.launch.py
# 레이어 : planning
# 역할   : mission_manager_node 실행
#
# [해야 할 일 / TODO]
#   - Nav2 bringup(nav2_bringup의 navigation_launch.py)을 include 하고 nav2_params.yaml을 넘긴다
# =============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('nz2_planning'), 'config', 'mission.yaml')
    # TODO(판단): Nav2 bringup include 자리
    return LaunchDescription([
        Node(package='nz2_planning', executable='mission_manager_node', name='mission_manager_node',
             parameters=[params], output='screen'),
    ])
