"""Start only the serial bridge; defaults to read-only operation."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("nz2_arm_bridge"), "config", "arm_bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("port", default_value="",
                              description="Empty: identify one arm among /dev/ttyACM*"),
        DeclareLaunchArgument("allow_motion", default_value="false"),
        Node(
            package="nz2_arm_bridge", executable="arm_bridge_node.py",
            name="arm_bridge", output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
                    "allow_motion": ParameterValue(
                        LaunchConfiguration("allow_motion"), value_type=bool),
                },
            ],
        ),
    ])
