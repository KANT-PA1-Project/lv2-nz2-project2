"""신호 점멸 해독 노드 실행 (Jetson). 바코드·신호등 영상 처리는 nz2_perception이 맡는다. respawn: 크래시 시 자동 재시작(대회 규정상 허용)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("nz2_signal"), "config", "signal.yaml")
    return LaunchDescription([
        Node(package="nz2_signal", executable="signal_decoder_node", name="signal_decoder",
             parameters=[cfg], output="screen", respawn=True, respawn_delay=1.0),
    ])
