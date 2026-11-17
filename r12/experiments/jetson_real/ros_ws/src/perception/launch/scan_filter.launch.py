"""라이다 4개 몸체 가림 필터 실행 [2026-11-02]

실물 (Jetson): 라이다 드라이버 4개를 /scan/<이름>_raw 로 내보낸 뒤
    ros2 launch perception scan_filter.launch.py
Gazebo 시험 (개발 PC, 시뮬 라이다는 이미 /scan/<이름> 으로 나옴 → 결과를 /scan_filtered/<이름> 으로):
    ros2 launch perception scan_filter.launch.py test:=true
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMES = ["front_right", "front_left", "rear_left", "rear_right"]


def generate_launch_description():
    cfg = str(Path(get_package_share_directory("perception")) / "config" / "scan_filter.yaml")
    sim = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    test = {"inputs": [f"/scan/{n}" for n in NAMES], "outputs": [f"/scan_filtered/{n}" for n in NAMES]}
    return LaunchDescription([
        DeclareLaunchArgument("test", default_value="false", description="true: Gazebo 시험 (/scan/* → /scan_filtered/*)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="perception", executable="scan_body_filter.py", name="scan_body_filter", output="screen",
             parameters=[cfg, sim], condition=UnlessCondition(LaunchConfiguration("test"))),
        Node(package="perception", executable="scan_body_filter.py", name="scan_body_filter", output="screen",
             parameters=[cfg, sim, test], condition=IfCondition(LaunchConfiguration("test"))),
    ])
