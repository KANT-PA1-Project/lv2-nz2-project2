# nz2_swerve_nav/launch/sim_nav.launch.py
# 개발 PC: Gazebo + 팀 r12_swerve_controller 구동 (설계 v2, 4a 단계) + Nav2 (5단계, nav2:=true)
#
#   /cmd_vel (TwistStamped, base_footprint, 텔레옵/Nav2)
#     → cmd_frame_adapter → /r12/cmd_vel (base_link)
#     → r12_swerve_controller (팀 코드 그대로) → /r12/swerve_joint_commands
#     → r12_cmd_to_gz → /swerve/sim/<joint>/cmd_pos|cmd_vel → gz 브리지 → Gazebo
#   /joint_states (gz) → swerve_controller_node (odom 전용) → /odom, TF odom→base_footprint, /swerve/footprint
#   /lidar_*/scan → scan_merger_node → /scan
#
#   ros2 launch nz2_swerve_nav sim_nav.launch.py rviz:=true                      (통로 시험)
#   ros2 launch nz2_swerve_nav sim_nav.launch.py world:=maze slam:=true rviz:=true (4단계: 경기장 지도 작성)
#   ros2 launch nz2_swerve_nav sim_nav.launch.py world:=/절대경로/arena.sdf
#   ros2 launch nz2_swerve_nav sim_nav.launch.py world:=maze nav2:=true rviz:=true (5단계: 지도 + AMCL + MPPI)
#     nav2:=true 는 slam:=true 와 같이 못 씀. 지도 기본 ~/slam_maps/nz2_maze_sim_v3.yaml (map:= 로 변경)
#
# visual: lite(기본) = urdf_dir/meshes_lite (정점 2 mm 격자 병합, 430만 → 27만 면, 외곽 오차 ≤ 0.64 mm) + URDF 원래 색 188개
#         full = urdf_dir/meshes 원본, simple = 충돌 도형(색 없음, 가장 가벼움)
#
# world 별칭: corridor → urdf_dir/corridor_test.sdf, maze → urdf_dir/maze_arena.sdf
#   maze 는 mission.pdf 좌표(왼쪽 아래 기둥 중심 원점, m). x·y 를 안 주면 출발 중심 (0.734, 0.950), yaw 0
#   yaw 0(+X) 으로 지도를 시작해야 shift_map_origin.py 로 map 프레임 = mission 좌표가 됨
import os
import copy
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _simple_visuals(urdf):
    """visual:=simple — 시각 메시(STL 약 220 MB, 430만 삼각형)를 각 링크의 충돌 도형(상자·원통)으로 교체.
    gpu_lidar 4개와 GUI 가 매 프레임 이 메시를 그리느라 RTF 가 0.1 % 대로 떨어짐 → 주행 시험에선 단순 도형"""
    root = ET.fromstring(urdf)
    n_old = n_new = 0
    for link in root.findall('link'):
        vis = link.findall('visual')
        n_old += len(vis)
        for v in vis:
            link.remove(v)
        for c in link.findall('collision'):
            v = ET.SubElement(link, 'visual')
            for child in c:
                v.append(copy.deepcopy(child))
            n_new += 1
    print(f'[sim_nav] 시각 요소 {n_old}개 → 단순 도형 {n_new}개 (visual:=simple)')
    return ET.tostring(root, encoding='unicode')


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    urdf_dir = os.path.expanduser(lc('urdf_dir'))
    world = lc('world') or 'corridor'
    aliases = {'corridor': 'corridor_test.sdf', 'maze': 'maze_arena.sdf'}
    is_maze = world == 'maze' or os.path.basename(world) == 'maze_arena.sdf'
    if world in aliases:
        world = os.path.join(urdf_dir, aliases[world])
    world = os.path.expanduser(world)
    # 시작 위치: 비우면 경기장은 출발 중심, 그 외는 원점
    x = lc('x') or ('0.734' if is_maze else '0.0')
    y = lc('y') or ('0.950' if is_maze else '0.0')
    yaw = lc('yaw') or '0.0'
    use_slam = lc('slam').lower() in ('true', '1')
    use_nav2 = lc('nav2').lower() in ('true', '1')
    if use_slam and use_nav2:
        raise RuntimeError('slam:=true 와 nav2:=true 는 같이 못 씀 (둘 다 map→odom TF 를 냄)')
    # 11/14: 기본 = 최종 URDF(v99_nz2fix) 에 Gazebo 플러그인·질량 축소를 넣은 assembly_nz2_v99.urdf
    #        (tools/make_nz2_urdf.py 로 생성. 옛 assembly_nz2.urdf 와 차이 = 상단 카메라 프레임 4개뿐, 라이다·바퀴 동일)
    urdf_path = os.path.join(urdf_dir, lc('urdf_file'))
    bridge_cfg = os.path.join(urdf_dir, 'ros_gz_bridge_nz2.yaml')

    pkg = get_package_share_directory('nz2_swerve_nav')
    swerve_cfg = os.path.join(pkg, 'config', 'swerve_nav.yaml')
    r12_cfg = os.path.join(get_package_share_directory('r12_swerve_control'), 'config', 'r12_swerve.yaml')
    r12_over = lc('r12_overrides') or os.path.join(pkg, 'config', 'r12_sim_overrides.yaml')
    slam_cfg = lc('slam_params') or os.path.join(pkg, 'config', 'slam_toolbox_mapping.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'nz2_nav.rviz' if use_nav2 else
                            ('nz2_slam.rviz' if use_slam else 'nz2_sim.rviz'))
    for f in (urdf_path, world, bridge_cfg, r12_cfg, r12_over, rviz_cfg) + ((slam_cfg,) if use_slam else ()):
        if not os.path.isfile(f):
            raise RuntimeError(f'파일 없음: {f}')

    with open(urdf_path, encoding='utf-8') as f:
        urdf = f.read()
    vis = lc('visual').lower()
    if vis not in ('lite', 'full', 'simple'):
        raise RuntimeError(f'visual:={vis} — lite / full / simple 중 하나')
    mesh_dir = os.path.join(urdf_dir, 'meshes')
    if vis == 'lite':
        lite_dir = os.path.join(urdf_dir, 'meshes_lite')
        if os.path.isdir(lite_dir):
            mesh_dir = lite_dir
        else:
            print(f'[sim_nav] {lite_dir} 없음 → 원본 메시(full) 사용')
    urdf = urdf.replace('filename="meshes/', f'filename="file://{mesh_dir}/')
    # 카메라 8개(D435 ×2 × depth·color·IR 2) 30 Hz 렌더링이 물리 스텝을 붙잡아 RTF 가 떨어짐 → 주행 시험에선 뺌
    # 라이다 4개(gpu_lidar)는 유지. 카메라가 필요하면 cameras:=true
    if lc('cameras').lower() not in ('true', '1'):
        urdf, n = re.subn(r'<sensor\s+name="[^"]*"\s+type="(?:camera|depth_camera|rgbd_camera)".*?</sensor>',
                          '', urdf, flags=re.S)
        print(f'[sim_nav] 카메라 센서 {n}개 제외 (cameras:=false)')
    if vis == 'simple':
        urdf = _simple_visuals(urdf)
    else:
        print(f'[sim_nav] 시각 요소 원본 그대로 (색 포함), 메시 폴더 {mesh_dir}')
    # 정답 자세 (점검용): Gazebo OdometryPublisher → /model/robot/ground_truth (gz) → 같은 이름 ROS nav_msgs/Odometry
    #   TF 는 발행하지 않음 (gz 쪽 tf_topic 만, 브리지 안 함) → slam·odom TF 와 충돌 없음
    gt_plugin = ('<gazebo><plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
                 '<odom_frame>gt_odom</odom_frame><robot_base_frame>base_footprint</robot_base_frame>'
                 '<odom_publish_frequency>50</odom_publish_frequency><dimensions>2</dimensions>'
                 '<odom_topic>/model/robot/ground_truth</odom_topic><tf_topic>/model/robot/ground_truth_tf</tf_topic>'
                 '</plugin></gazebo>\n</robot>')
    urdf = urdf[:urdf.rfind('</robot>')] + gt_plugin
    sim = {'use_sim_time': True}
    print(f'[sim_nav] world={world}  시작 x={x} y={y} yaw={yaw}  slam={use_slam}  nav2={use_nav2}')

    slam = []
    if use_slam:
        # slam_toolbox 기본 런치(라이프사이클 configure·activate 자동) 를 그대로 포함. 로봇·/scan 이 뜬 뒤 시작
        slam_launch = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        slam = [TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'slam_params_file': slam_cfg, 'use_sim_time': 'true'}.items())])]

    nav2 = []
    if use_nav2:
        # 로봇 스폰(4 s) → odom·/scan·footprint 가 뜬 뒤 시작. AMCL 초기 자세 = 스폰 자세
        # 11/14: 10 → nav2_delay(기본 20 s). Gazebo·RViz 초기 부하가 끝난 뒤 시작 (Fast DDS 응답 유실 완화)
        nav2 = [TimerAction(period=float(lc('nav2_delay')), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2_stack.launch.py')),
            launch_arguments={'map': lc('map'), 'params_file': lc('nav2_params'), 'use_sim_time': 'true',
                              'x': x, 'y': y, 'yaw': yaw, 'batch_size': lc('batch_size')}.items())])]

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               urdf_dir + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, sim]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': bridge_cfg}, sim]),
        # 정답 자세 브리지 (odom_check.py 용)
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gt_bridge', output='screen',
             arguments=['/model/robot/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry'], parameters=[sim]),
        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'robot', '-topic', 'robot_description',
                            '-x', x, '-y', y, '-z', '0.003', '-Y', yaw]),
        ]),
        # ① /cmd_vel(base_footprint) → /r12/cmd_vel(base_link)
        Node(package='nz2_swerve_nav', executable='cmd_frame_adapter.py', output='screen', parameters=[sim]),
        # ② 팀 컨트롤러 (코드 수정 없음). /cmd_vel(Twist) 구독은 타입 충돌 → 빈 토픽으로 리매핑
        Node(package='r12_swerve_control', executable='swerve_controller', name='r12_swerve_controller',
             output='screen', parameters=[r12_cfg, r12_over, sim],
             remappings=[('/cmd_vel', '/r12/cmd_vel_twist')]),
        # ③ r12 관절 명령 → Gazebo 관절 토픽
        Node(package='nz2_swerve_nav', executable='r12_cmd_to_gz.py', output='screen', parameters=[sim]),
        # ④ odom 전용 (구동 안 함)
        Node(package='nz2_swerve_nav', executable='swerve_controller_node', output='screen',
             parameters=[swerve_cfg, {'drive_enable': False, 'sim_joint_topics': False,
                                      'odom_linear_scale': float(lc('odom_linear_scale')),
                                      'odom_yaw_scale': float(lc('odom_yaw_scale'))}, sim]),
        Node(package='nz2_swerve_nav', executable='scan_merger_node', output='screen',
             parameters=[swerve_cfg, sim]),
        Node(package='rviz2', executable='rviz2', output='screen', parameters=[sim],
             arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ] + slam + nav2


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_dir', default_value='~/Downloads/assembly_URDF'),
        DeclareLaunchArgument('urdf_file', default_value='assembly_nz2_v99.urdf',
                              description='urdf_dir 안 파일 이름. 옛 로봇 = assembly_nz2.urdf'),
        DeclareLaunchArgument('world', default_value='corridor',
                              description='corridor / maze / SDF 절대경로'),
        DeclareLaunchArgument('x', default_value='', description='비우면 maze 0.734, 그 외 0'),
        DeclareLaunchArgument('y', default_value='', description='비우면 maze 0.950, 그 외 0'),
        DeclareLaunchArgument('yaw', default_value='', description='비우면 0'),
        DeclareLaunchArgument('odom_linear_scale', default_value='1.0',
                              description='오도메트리 직선 보정 배율 (odom_check 직진 시험 후 1/비)'),
        DeclareLaunchArgument('odom_yaw_scale', default_value='1.053',
                              description='오도메트리 회전 보정 배율. 11/13 시뮬 실측 1/0.950 = 1.053'),
        DeclareLaunchArgument('slam', default_value='false', description='true 면 slam_toolbox online_async (지도 작성)'),
        DeclareLaunchArgument('slam_params', default_value='', description='비우면 config/slam_toolbox_mapping.yaml'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='true 면 map_server + AMCL(Omni) + MPPI(Omni) (5단계)'),
        DeclareLaunchArgument('map', default_value='~/slam_maps/nz2_maze_sim_v3.yaml', description='nav2:=true 일 때 지도'),
        DeclareLaunchArgument('nav2_params', default_value='', description='비우면 config/nav2_params.yaml'),
        DeclareLaunchArgument('nav2_delay', default_value='20.0', description='launch 시작 뒤 Nav2 시작까지 [s]'),
        DeclareLaunchArgument('batch_size', default_value='2000', description='MPPI 샘플 수 (개발 PC 2000)'),
        DeclareLaunchArgument('r12_overrides', default_value='', description='비우면 config/r12_sim_overrides.yaml'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('visual', default_value='lite',
                              description='lite: 감량 메시 27만 면 + 원래 색(기본) / full: 원본 430만 면(느림) / simple: 충돌 도형(색 없음)'),
        DeclareLaunchArgument('cameras', default_value='false', description='true 면 D435 카메라 8개 포함 (느려짐)'),
        OpaqueFunction(function=_setup),
    ])
