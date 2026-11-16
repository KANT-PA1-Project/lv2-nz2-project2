"""NZ2 인지 노드 일괄 실행.

실행:
    ros2 launch nz2_perception perception.launch.py                       # 카메라 드라이버는 따로 켠 경우
    ros2 launch nz2_perception perception.launch.py use_camera:=true \\
        gripper_serial:=<시리얼> exposure:=<값>                         # 그리퍼 D435 도 같이 실행
    ros2 launch nz2_perception perception.launch.py yolo:=py             # PC: Python YOLO(.pt) 사용 (yolo_env 에서)
    ros2 launch nz2_perception perception.launch.py yolo:=py use_sim_time:=true   # [2026-11-02] Gazebo 영상으로 (시뮬 시간)
    시리얼 확인: rs-enumerate-devices -s   (또는 training/tools/capture_d435.py --list)

[수정 2026-10-24]
  - use_camera:=true 면 RealSense 를 노드들이 기다리는 토픽 이름으로 띄운다.
      camera_namespace='' + camera_name=gripper → /gripper/color/image_raw, /gripper/aligned_depth_to_color/...
  - yolo:=py 기본 모델을 models/current.pt 로

[수정 2026-10-28] 카메라 재설계 + 미션 v5
  - 전면 D435 삭제 → 인지용 카메라는 그리퍼 D435 1대뿐 (front_serial 인자 삭제)
    전방위 3캠(cuVSLAM 전용)은 이 런치가 아니라 VSLAM 런치에서 띄운다
  - obstacle_detector 제외 (전면 depth 기반이었음. 동적 장애물은 하단 센서 4개로 이관)
  - signal_recorder 추가 (신호 12회 점멸 → signal.json)
  - tag_reader 추가 (AprilTag 퍽 0~11 / 장소 12~14 → /nz2/tags)

[수정 2026-10-29]
  - puck_registry 추가 (YOLO + 태그 ID + depth/tf → /perception/puck_tracks, 등록표 /nz2/puck_registry)
  - dice_judge 추가 (장소 태그 기준 3×3 격자 판정 → /nz2/dice_result, 증거 ~/nz2_logs/dice/)

[수정 2026-10-30] 지금 안 쓰는 노드 뺌 (소스·빌드는 그대로, 필요하면 아래 Node 다시 추가)
  - puck_localizer : 태그 ID·추적 없는 옛 퍽 위치 → puck_registry(/perception/puck_tracks) 로 대체
  - shelf_detector : Isaac ROS AprilTag 전용, 미션 v5 장소는 tag_reader + dice_judge 가 처리 (use_shelf 인자 삭제)
  - obstacle_detector 는 10-28 에 이미 뺌
  남은 노드: yolo_detector(.py) · tag_reader · puck_registry · dice_judge · traffic_light_filter · signal_recorder ·
            barcode_reader · perception_master

[수정 2026-11-02] use_sim_time 인자 추가 (기본 false = 실물). Gazebo 와 같이 쓸 때 true —
  안 주면 노드 시계(실제 시각)와 Gazebo 영상 시각(시뮬 시각)이 수억 초 차이 나서 tag_reader(0.5 s)·puck_registry(1 s)가
  "오래된 영상"으로 보고 전부 버림 → puck_debug 영상·puck_tracks 가 안 나옴

전제: apriltag / amcl 등은 별도 런치에서 올린다 (dataflow 참고).
      base_link → gripper_link tf 는 로봇 URDF(robot_state_publisher)가 내야 한다.
      tf 가 없으면 puck_localizer 는 카메라 좌표(frame_id=카메라)로 발행.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def _realsense(name, serial, fps, exposure, wb):
    """RealSense D435 1대. 토픽 /<name>/color/image_raw, /<name>/aligned_depth_to_color/image_raw,
    /<name>/depth/image_rect_raw, /<name>/color/camera_info. 프레임 <name>_color_optical_frame"""
    p = {
        'camera_namespace': '',
        'camera_name': name,
        'rgb_camera.color_profile': f'848,480,{fps}',    # 합성 데이터·엔진과 같은 848x480
        'depth_module.depth_profile': f'848,480,{fps}',
        'align_depth.enable': True,
        'enable_sync': True,
        # 적외선(IR) 영상은 인지에서 안 씀 → 꺼서 USB 대역폭·CPU 절약
        'enable_infra1': False,
        'enable_infra2': False,
    }
    if serial:
        p['serial_no'] = f"'{serial}'"     # 숫자로 해석되지 않게 따옴표 포함
    if exposure:                           # 노출 고정 (촬영 capture_d435.py --exposure 와 같은 값)
        p['rgb_camera.enable_auto_exposure'] = False
        p['rgb_camera.exposure'] = int(float(exposure))
    if wb:
        p['rgb_camera.enable_auto_white_balance'] = False
        p['rgb_camera.white_balance'] = float(wb)
    return Node(package='realsense2_camera', executable='realsense2_camera_node',
                namespace='', name=name, parameters=[p], output='screen')


def _cameras(context):
    g = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    if g('use_camera').lower() != 'true':
        return []
    # 그리퍼 D435 1대 (시리얼 비우면 연결된 첫 카메라)
    return [_realsense('gripper', g('gripper_serial'), g('fps'), g('exposure'), g('white_balance'))]


def generate_launch_description():
    params = str(Path(get_package_share_directory('nz2_perception')) / 'config' / 'perception.yaml')
    model_path = LaunchConfiguration('model_path')

    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    common = dict(package='nz2_perception', output='screen', parameters=[params, sim])

    return LaunchDescription([
        DeclareLaunchArgument('yolo', default_value='cpp',
                              description='cpp: C++ (.engine=TensorRT / .onnx=OpenCV) / py: Python ultralytics(.pt, PC 테스트)'),
        DeclareLaunchArgument('model_path',
                              default_value='~/nz2_models/current.pt',
                              description='yolo:=py 일 때 쓸 .pt 경로 (cpp 는 perception.yaml 의 engine_path)'),
        DeclareLaunchArgument('use_camera', default_value='false', description='그리퍼 RealSense 같이 실행'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Gazebo 와 같이 쓸 때 true'),
        DeclareLaunchArgument('gripper_serial', default_value='',
                              description='그리퍼 D435 시리얼 (비우면 연결된 첫 카메라)'),
        DeclareLaunchArgument('fps', default_value='30', description='카메라 fps [2026-11-03 15→30] (USB 3 필요, 자동 노출이면 어두울 때 15 로 떨어짐 → exposure 지정)'),
        DeclareLaunchArgument('exposure', default_value='',
                              description='컬러 노출 고정값 (비우면 자동 노출 — 대회에선 반드시 지정)'),
        DeclareLaunchArgument('white_balance', default_value='', description='화이트밸런스 고정값 K'),
        OpaqueFunction(function=_cameras),
        # 둘 중 하나만 실행된다(같은 토픽에 두 노드가 발행하면 신호등 확정이 깨짐).
        Node(executable='yolo_detector', name='yolo_detector',
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('yolo'), 'cpp')), **common),
        Node(package='nz2_perception', executable='yolo_detector.py', name='yolo_detector',
             output='screen', parameters=[params, sim, {'model_path': model_path}],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('yolo'), 'py'))),
        Node(executable='traffic_light_filter', name='traffic_light_filter', **common),
        Node(executable='barcode_reader', name='barcode_reader', **common),
        Node(executable='tag_reader', name='tag_reader', **common),
        Node(executable='puck_registry.py', name='puck_registry', **common),   # [2026-10-29] 퍽 등록·추적
        Node(executable='perception_master', name='perception_master', **common),
    ])
