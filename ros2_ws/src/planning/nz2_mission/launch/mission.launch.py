"""미션 FSM 실행 (Jetson). respawn=True: 크래시 시 자동 재시작 → 체크포인트로 RESUME."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("nz2_mission")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="competition"),
        DeclareLaunchArgument("run_root", default_value=os.path.expanduser("~/nz2_runs")),
        Node(package="nz2_mission", executable="mission_fsm_node", name="mission_fsm", output="screen",
             respawn=True, respawn_delay=1.0,
             parameters=[os.path.join(share, "config", "mission.yaml"),
                         {"locations_file": os.path.join(share, "config", "mission_locations.yaml"),
                          "mode": LaunchConfiguration("mode"),
                          "run_root": LaunchConfiguration("run_root")}]),
    ])
