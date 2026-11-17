"""Single-puck preview using the existing RealSense, YOLO, tag and registry nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_launch", default_value="/home/nz2/nz2/cameras.launch.py"),
        DeclareLaunchArgument("profile", default_value="640x480x30"),
        DeclareLaunchArgument("depth_profile", default_value="640x480x30"),
        DeclareLaunchArgument("runtime_params",
                              default_value="/home/nz2/p2_runtime_ws/runtime/runtime_nodes.yaml"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LaunchConfiguration("camera_launch")),
            launch_arguments={"gripper": "true", "top": "false",
                              "profile": LaunchConfiguration("profile"),
                              "depth_profile": LaunchConfiguration("depth_profile")}.items()),
        Node(package="perception", executable="yolo_detector", name="yolo_detector",
             parameters=[LaunchConfiguration("runtime_params"), {
                 "image_reliable": False, "publish_debug_image": True,
                 "process_hz": 10.0}], output="screen"),
        Node(package="perception", executable="tag_reader", name="tag_reader",
             parameters=[{"image_reliable": False, "process_hz": 5.0,
                          "tag_size_m": .016, "output_frame": ""}], output="screen"),
        Node(package="perception", executable="puck_registry.py", name="puck_registry",
             parameters=[{"track_frame": "", "track_frame_fallback": "", "base_frame": "",
                          "publish_markers": False, "publish_debug_image": True,
                          "registry_path": "/tmp/nz2_single_puck_registry.json"}],
             output="screen"),
    ])
