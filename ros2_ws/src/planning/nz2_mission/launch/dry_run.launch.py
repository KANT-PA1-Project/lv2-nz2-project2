"""개발 PC 전용: 가짜 팔·Nav2·인지 + FSM(dry_run 모드). 하드웨어 없이 전체 흐름 검증."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("nz2_mission")
    return LaunchDescription([
        DeclareLaunchArgument("fail_prob", default_value="0.0"),
        DeclareLaunchArgument("sequence", default_value="RGRRGRGGYGYG"),
        DeclareLaunchArgument("barcode", default_value="P2-01"),
        Node(package="nz2_mission", executable="fake_skill_server", name="fake_skill_server", output="screen",
             parameters=[{"fail_prob": LaunchConfiguration("fail_prob")}]),
        Node(package="nz2_mission", executable="mission_fsm_node", name="mission_fsm", output="screen",
             respawn=True, respawn_delay=1.0,
             parameters=[os.path.join(share, "config", "mission.yaml"),
                         {"locations_file": os.path.join(share, "config", "mission_locations.yaml"),
                          "mode": "dry_run", "run_root": "/tmp/nz2_runs_dry",
                          "dry_signal_sequence": LaunchConfiguration("sequence"),
                          "dry_barcode": LaunchConfiguration("barcode"),
                          "timeout_arm_s": 5.0, "timeout_nav_s": 10.0, "no_progress_s": 8.0,
                          "hold_s": 0.5, "observe_settle_s": 0.1}]),
    ])
