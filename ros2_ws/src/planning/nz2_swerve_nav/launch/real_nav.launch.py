# nz2_swerve_nav/launch/real_nav.launch.py
# 실행 위치: Jetson (실물 로봇). 6단계
#
#   mode:=bringup  라이다 4 + 병합 + RSP + r12(컨트롤러·OpenCR 브리지) + 중계 + odom   (바퀴 띄움 시험, 텔레옵)
#   mode:=mapping  bringup + Cartographer (지도 작성. 저장은 save_map_carto.sh)
#   mode:=nav      bringup + nav2_stack (map_server + AMCL + MPPI). Cartographer 와 동시 금지 (둘 다 map→odom)
#
#   /cmd_vel (TwistStamped, base_footprint, 팔 쪽 +x)  ← 텔레옵 / Nav2 velocity_smoother
#     → cmd_frame_adapter (정렬 게이트: 측정 조향 /r12/joint_states) → /r12/cmd_vel (base_link)
#     → r12_swerve_controller (팀 코드) → /r12/swerve_joint_commands
#     → r12_opencr_bridge (팀 코드) → OpenCR swerve_left / swerve_right
#   /r12/joint_states (조향 position, 바퀴 velocity) → swerve_controller_node (odom 전용)
#     → /odom, TF odom→base_footprint, /swerve/footprint
#   LDS-03 ×4 (coin_d4_driver) → /lidar_{front,left,rear,right}/scan → scan_merger_node → /scan
#
# ⚠ URDF 라이다 이름은 실제와 180° 반대 (정면 = 팔 쪽 = base_link −X). 포트 인자는 '실제 장착 위치' 이름으로 받음
#     실제 팔 쪽(앞)   → URDF lidar_rear     실제 뒤        → URDF lidar_front
#     실제 왼쪽        → URDF lidar_right    실제 오른쪽     → URDF lidar_left     (왼/오 = 팔 쪽을 바라볼 때)
#
#   ros2 launch nz2_swerve_nav real_nav.launch.py mode:=bringup
#   ros2 launch nz2_swerve_nav real_nav.launch.py mode:=mapping
#   ros2 launch nz2_swerve_nav real_nav.launch.py mode:=nav map:=~/slam_maps/nz2_real_v1_mission.yaml
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 실제 위치 → (URDF 이름, 포트 인자). scan_merger input_topics 순서(front, left, rear, right)와 별개로 이름으로 연결
LIDARS = [
    ('lidar_front', 'port_back'),    # URDF front = 실제 뒤
    ('lidar_left', 'port_right'),    # URDF left  = 실제 오른쪽
    ('lidar_rear', 'port_arm'),      # URDF rear  = 실제 팔 쪽(앞)
    ('lidar_right', 'port_left'),    # URDF right = 실제 왼쪽
]


def _truthy(v):
    return v.lower() in ('true', '1', 'yes')


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    mode = lc('mode')
    if mode not in ('bringup', 'mapping', 'nav'):
        raise RuntimeError(f'mode:={mode} — bringup / mapping / nav 중 하나')

    pkg = get_package_share_directory('nz2_swerve_nav')
    swerve_cfg = os.path.join(pkg, 'config', 'swerve_nav.yaml')
    real_cfg = os.path.join(pkg, 'config', 'real_robot.yaml')
    r12_cfg = os.path.join(get_package_share_directory('r12_swerve_control'), 'config', 'r12_swerve.yaml')
    r12_over = os.path.expanduser(lc('r12_overrides') or os.path.join(pkg, 'config', 'r12_real_overrides.yaml'))
    urdf_path = os.path.expanduser(lc('urdf'))
    for f in (swerve_cfg, real_cfg, r12_cfg, r12_over, urdf_path):
        if not os.path.isfile(f):
            raise RuntimeError(f'[real_nav] 파일 없음: {f}')

    with open(urdf_path, encoding='utf-8') as f:
        urdf = f.read()
    # 메시 상대경로 → 절대경로 (RViz 표시용. TF 계산에는 무관)
    urdf = urdf.replace('filename="meshes/', f'filename="file://{os.path.dirname(urdf_path)}/meshes/')
    real_time = {'use_sim_time': False}
    lin_s, yaw_s = float(lc('odom_linear_scale')), float(lc('odom_yaw_scale'))
    print(f'[real_nav] mode={mode}  urdf={urdf_path}\n[real_nav] r12 덮어쓰기={r12_over}  '
          f'odom 배율 직선 {lin_s} / 회전 {yaw_s}')

    acts = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, real_time]),
    ]

    # ---------- 라이다 4개: ROBOTIS LDS-03 = coin_d4_driver single_coin_d4_node ×4 (v2, 11/15) ----------
    #   단일 노드는 port 파라미터를 그대로 받음 → 노드 4개를 이름공간으로 나눠 띄움 (토픽 /<urdf_name>/scan, SensorDataQoS)
    if _truthy(lc('lidars')):
        try:
            get_package_share_directory('coin_d4_driver')
        except PackageNotFoundError:
            raise RuntimeError('[real_nav] coin_d4_driver 없음 → source ~/nz2_nav_ws/install/setup.bash 확인')
        ports = {}
        for urdf_name, port_arg in LIDARS:
            port = lc(port_arg)
            if not port:
                raise RuntimeError(f'[real_nav] {port_arg} 비어 있음 (실제 위치 기준 포트. lidars:=false 면 생략 가능)')
            if not os.path.exists(port):
                raise RuntimeError(f'[real_nav] {port_arg}:={port} 포트 없음 → ls -l /dev/lidar* 와 USB 허브 확인')
            dev = os.path.realpath(port)
            if dev in ports:
                raise RuntimeError(f'[real_nav] {port_arg} 와 {ports[dev]} 가 같은 장치({dev})')
            ports[dev] = port_arg
        for urdf_name, port_arg in LIDARS:
            port = lc(port_arg)
            acts.append(Node(
                package='coin_d4_driver', executable='single_coin_d4_node', name='lidar_node',
                namespace=urdf_name, output='screen', respawn=True, respawn_delay=3.0,
                # frame_id 는 scan_merger 가 input_frames(URDF 프레임)로 덮어쓰므로 참고용
                parameters=[{'port': port, 'frame_id': f'{urdf_name}_scan', 'baudrate': int(lc('lidar_baud')),
                             'version': 4, 'topic_name': 'scan', 'reverse': _truthy(lc('lidar_reverse')),
                             'warmup_time': 5}, real_time]))
            print(f'[real_nav] {port_arg:<10} → /{urdf_name}/scan  ({port} = {os.path.realpath(port)})')
    else:
        print('[real_nav] lidars:=false — /lidar_*/scan 은 따로 띄울 것')
    acts.append(Node(package='nz2_swerve_nav', executable='scan_merger_node', output='screen',
                     parameters=[swerve_cfg, real_cfg, real_time]))

    # ---------- 구동: 중계 → r12 컨트롤러 → OpenCR 브리지 ----------
    acts += [
        Node(package='nz2_swerve_nav', executable='cmd_frame_adapter.py', output='screen',
             parameters=[{'joint_state_topic': '/r12/joint_states'}, real_time]),
        # /cmd_vel(Twist) 구독은 Nav2·텔레옵 TwistStamped 와 타입 충돌 → 빈 토픽으로
        Node(package='r12_swerve_control', executable='swerve_controller', name='r12_swerve_controller',
             output='screen', parameters=[r12_cfg, r12_over, real_time],
             remappings=[('/cmd_vel', '/r12/cmd_vel_twist')]),
    ]
    if _truthy(lc('bridge')):
        acts.append(Node(package='r12_swerve_control', executable='opencr_bridge', name='r12_opencr_bridge',
                         output='screen', parameters=[r12_cfg, r12_over, real_time]))
    else:
        print('[real_nav] bridge:=false — OpenCR 에 명령 안 보냄 (토픽만 확인)')

    # ---------- odom 전용 ----------
    acts.append(Node(package='nz2_swerve_nav', executable='swerve_controller_node', output='screen',
                     parameters=[swerve_cfg, real_cfg,
                                 {'drive_enable': False, 'sim_joint_topics': False,
                                  'odom_linear_scale': lin_s, 'odom_yaw_scale': yaw_s}, real_time],
                     remappings=[('joint_states', '/r12/joint_states')]))

    # ---------- SLAM / Nav2 ----------
    if mode == 'mapping':
        try:
            get_package_share_directory('cartographer_ros')
        except PackageNotFoundError:
            raise RuntimeError('[real_nav] cartographer_ros 없음 → source ~/carto_ws/install/setup.bash 먼저')
        lua = os.path.expanduser(lc('carto_lua') or os.path.join(pkg, 'config', 'cartographer', 'nz2_real_2d.lua'))
        if not os.path.isfile(lua):
            raise RuntimeError(f'[real_nav] lua 없음: {lua}')
        print(f'[real_nav] Cartographer lua={lua}  격자 {lc("carto_resolution")} m')
        acts.append(TimerAction(period=float(lc('slam_delay')), actions=[
            Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
                 output='screen', parameters=[real_time],
                 arguments=['-configuration_directory', os.path.dirname(lua),
                            '-configuration_basename', os.path.basename(lua)],
                 remappings=[('scan', '/scan'), ('odom', '/odom')]),
            Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                 name='cartographer_occupancy_grid_node', output='screen', parameters=[real_time],
                 arguments=['-resolution', lc('carto_resolution'), '-publish_period_sec', '1.0']),
        ]))
    elif mode == 'nav':
        acts.append(TimerAction(period=float(lc('slam_delay')), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2_stack.launch.py')),
            launch_arguments={'map': lc('map'), 'params_file': lc('nav2_params'), 'use_sim_time': 'false',
                              'x': lc('x'), 'y': lc('y'), 'yaw': lc('yaw'),
                              'batch_size': lc('batch_size'), 'visualize': lc('visualize')}.items())]))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='bringup', description='bringup / mapping / nav'),
        DeclareLaunchArgument('urdf', default_value='~/nz2_urdf/real_final_plz_no_more_edit_v99_nz2fix.urdf',
                              description='최종 URDF (nz2fix). meshes/ 는 같은 폴더'),
        DeclareLaunchArgument('lidars', default_value='true', description='false 면 라이다 드라이버 안 띄움'),
        # 실제 장착 위치 기준 포트. Jetson udev 고정 이름 /dev/lidar_* (꽂힌 USB 자리 기준) 사용
        DeclareLaunchArgument('port_arm', default_value='', description='실제 팔 쪽(앞) 라이다 포트 (예: /dev/lidar_rear)'),
        DeclareLaunchArgument('port_back', default_value='', description='실제 뒤 라이다 포트'),
        DeclareLaunchArgument('port_left', default_value='', description='실제 왼쪽 라이다 포트 (팔 쪽을 볼 때 왼쪽)'),
        DeclareLaunchArgument('port_right', default_value='', description='실제 오른쪽 라이다 포트'),
        DeclareLaunchArgument('lidar_baud', default_value='230400', description='LDS-03 통신 속도'),
        DeclareLaunchArgument('lidar_reverse', default_value='false', description='RViz 에서 스캔 좌우가 뒤집혀 보이면 true'),
        DeclareLaunchArgument('bridge', default_value='true', description='false 면 OpenCR 브리지 안 띄움'),
        DeclareLaunchArgument('r12_overrides', default_value='', description='비우면 config/r12_real_overrides.yaml'),
        DeclareLaunchArgument('odom_linear_scale', default_value='1.0', description='2 m 직진 실측 후 실제/odom'),
        DeclareLaunchArgument('odom_yaw_scale', default_value='1.0', description='360° 회전 실측 후 실제/odom'),
        DeclareLaunchArgument('slam_delay', default_value='10.0', description='bringup 뒤 SLAM/Nav2 시작까지 [s] (라이다 예열 5 s 포함)'),
        DeclareLaunchArgument('carto_lua', default_value='', description='비우면 config/cartographer/nz2_real_2d.lua'),
        DeclareLaunchArgument('carto_resolution', default_value='0.02', description='지도 격자 [m] (시뮬 지도와 같음)'),
        DeclareLaunchArgument('map', default_value='~/slam_maps/nz2_real_v1_mission.yaml', description='mode:=nav 지도'),
        DeclareLaunchArgument('nav2_params', default_value='', description='비우면 config/nav2_params.yaml'),
        DeclareLaunchArgument('x', default_value='0.734', description='AMCL 초기 x (mission 좌표)'),
        DeclareLaunchArgument('y', default_value='0.950'),
        DeclareLaunchArgument('yaw', default_value='3.14159', description='실전 출발 π (팔이 −x 를 봄)'),
        DeclareLaunchArgument('batch_size', default_value='1000', description='MPPI 샘플 수 (Jetson 1000)'),
        DeclareLaunchArgument('visualize', default_value='false', description='MPPI 후보 궤적 발행 (개발 PC RViz 로 볼 때만 true)'),
        OpaqueFunction(function=_setup),
    ])
