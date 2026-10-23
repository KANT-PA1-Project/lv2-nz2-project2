# =============================================================
# 패키지 : nz2_bringup
# 파일   : launch/bringup.launch.py
# 역할   : 로봇 전체 실행. 각 레이어 launch를 include 하는 최상위 launch
#
# [해야 할 일 / TODO]
#   - 스켈레톤: control·planning만 실제 include, 나머지는 자리만 주석으로 둔다
#   - config/robot.yaml의 공통 파라미터(프레임·네임스페이스)를 각 launch로 넘긴다
# [ROS2 메모] IncludeLaunchDescription = 다른 launch 파일을 불러와 함께 실행
# =============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(pkg, file):
    path = os.path.join(get_package_share_directory(pkg), 'launch', file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def generate_launch_description():
    return LaunchDescription([
        # TODO: include('nz2_description', 'display.launch.py') — 실제 로봇에서는 RViz 없이 robot_state_publisher만
        # TODO: include('nz2_perception', 'perception.launch.py') — 카메라 연결 후
        include('nz2_control', 'control.launch.py'),
        include('nz2_planning', 'planning.launch.py'),
    ])
