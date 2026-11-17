"""NZ2 D435 2대 시리얼 고정 실행 (Jetson).

  ros2 launch ~/nz2/cameras.launch.py                      # 둘 다
  ros2 launch ~/nz2/cameras.launch.py top:=false           # 그리퍼만
  ros2 launch ~/nz2/cameras.launch.py profile:=1280x720x15 exposure:=150

  토픽: /gripper/color/image_raw, /gripper/aligned_depth_to_color/image_raw, /gripper/color/camera_info
        /top/color/image_raw,     /top/aligned_depth_to_color/image_raw,     /top/color/camera_info
  시리얼 (2026-11-15 사진으로 확인): 그리퍼 261322074035, 상단 261722075504
  지금은 둘 다 USB 2.1 연결 → 기본 640x480x15. USB 3 로 옮긴 뒤 profile 올릴 것.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERIALS = {'gripper': '261322074035', 'top': '261722075504'}


def _camera(name, context):
    g = lambda k: LaunchConfiguration(k).perform(context)
    p = {
        'camera_namespace': '',
        'camera_name': name,
        'serial_no': SERIALS[name],                  # Node dict 로 넘기면 문자열 그대로 (CLI 용 따옴표 트릭 불필요)
        'enable_color': True,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'align_depth.enable': True,
        f'{name}.color.image_raw.compressed.jpeg_quality': 45,
        'rgb_camera.color_profile': g('profile'),
        'depth_module.depth_profile': g('depth_profile'),
    }
    if g('exposure'):
        p['rgb_camera.enable_auto_exposure'] = False
        p['rgb_camera.exposure'] = int(g('exposure'))
    return Node(package='realsense2_camera', executable='realsense2_camera_node',
                namespace='', name=name, parameters=[p], output='screen',
                condition=IfCondition(LaunchConfiguration(name)))


def _setup(context):
    return [_camera('gripper', context), _camera('top', context)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='true'),
        DeclareLaunchArgument('top', default_value='true'),
        DeclareLaunchArgument('profile', default_value='640x480x15', description='컬러 WxHxFPS'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15'),
        DeclareLaunchArgument('exposure', default_value='', description='비우면 자동노출'),
        OpaqueFunction(function=_setup),
    ])
