"""카메라 드라이버/기존 신호·바코드 노드는 시작하지 않는 독립 launch 초안."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            FindPackageShare("perception"), "config", "barcode_signal.yaml"])),
        DeclareLaunchArgument("perception_root", default_value=""),
        DeclareLaunchArgument("r12_src", default_value=""),
        DeclareLaunchArgument("fixed_camera_settings_confirmed", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="perception", executable="barcode_signal_node.py", name="barcode_signal", output="screen",
             parameters=[LaunchConfiguration("params_file"), {
                 "perception_root": ParameterValue(LaunchConfiguration("perception_root"), value_type=str),
                 "r12_src": ParameterValue(LaunchConfiguration("r12_src"), value_type=str),
                 "fixed_camera_settings_confirmed": ParameterValue(
                     LaunchConfiguration("fixed_camera_settings_confirmed"), value_type=bool),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
             }]),
    ])
