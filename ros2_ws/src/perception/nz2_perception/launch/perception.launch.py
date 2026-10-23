# =============================================================
# 패키지 : nz2_perception
# 파일   : launch/perception.launch.py
# 레이어 : perception
# 역할   : 인지 노드 4개(+ 선택적으로 realsense2_camera)를 파라미터와 함께 실행
#
# [입력]  launch 인자 use_camera (기본 false: 카메라 없이 노드만 띄움)
# [출력]  실행된 노드들
# [해야 할 일 / TODO]
#   - 카메라 2대가 확정되면 realsense2_camera를 네임스페이스별로 2개 띄운다 [확인 필요]
# [ROS2 메모] launch 파일 = 여러 노드를 한 번에 실행하는 스크립트. `ros2 launch nz2_perception perception.launch.py`
# =============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('nz2_perception'), 'config', 'perception_params.yaml')
    use_camera = LaunchConfiguration('use_camera')

    nodes = [
        Node(package='nz2_perception', executable=name, name=name, parameters=[params], output='screen')
        for name in ('puck_detector_node', 'puck_pose_estimator_node', 'tag_reader_node', 'traffic_light_node')
    ]
    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{'align_depth.enable': True}],
        condition=IfCondition(use_camera),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='false', description='realsense2_camera 실행 여부'),
        camera,
        *nodes,
    ])
