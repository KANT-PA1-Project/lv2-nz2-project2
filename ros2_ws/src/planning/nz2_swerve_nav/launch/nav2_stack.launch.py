# nz2_swerve_nav/launch/nav2_stack.launch.py
# Nav2 위치추정 + 내비게이션 (시뮬·실물 공용). 로봇·/scan·/odom·TF(odom→base_footprint)·/swerve/footprint 는 따로 띄워져 있어야 함
#
#   map_server + amcl                                   (lifecycle_manager_localization)
#   controller_server(MPPI) + planner_server(NavFn) + behavior_server(wait) + bt_navigator + velocity_smoother
#                                                       (lifecycle_manager_navigation)
#
#   토픽: controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel (TwistStamped, base_footprint)
#         /local_costmap/footprint ← /swerve/footprint (조향각 따라 바뀌는 외곽, PolygonStamped)
#
#   시뮬:  sim_nav.launch.py world:=maze nav2:=true  가 이 파일을 포함함
#   단독:  ros2 launch nz2_swerve_nav nav2_stack.launch.py map:=~/slam_maps/nz2_maze_sim_v3.yaml use_sim_time:=false yaw:=3.14159
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    pkg = get_package_share_directory('nz2_swerve_nav')
    bt_pkg = get_package_share_directory('nav2_bt_navigator')

    map_yaml = os.path.expanduser(lc('map'))
    params = os.path.expanduser(lc('params_file') or os.path.join(pkg, 'config', 'nav2_params.yaml'))
    bt_xml = os.path.expanduser(lc('bt_xml') or os.path.join(pkg, 'behavior_trees', 'nz2_nav_to_pose.xml'))
    for f in (map_yaml, params, bt_xml):
        if not os.path.isfile(f):
            raise RuntimeError(f'[nav2_stack] 파일 없음: {f}')
    sim = lc('use_sim_time').lower() in ('true', '1')
    x, y, yaw = float(lc('x')), float(lc('y')), float(lc('yaw'))
    batch = int(lc('batch_size'))
    print(f'[nav2_stack] map={map_yaml}\n[nav2_stack] params={params}\n[nav2_stack] bt={bt_xml}\n'
          f'[nav2_stack] 초기 자세 x={x} y={y} yaw={yaw}  use_sim_time={sim}  MPPI batch={batch}')

    # BT 검색 폴더: 없는 폴더를 넣으면 bt_navigator 가 configure 실패 (11/14 실측: Lyrical apt 에는 behavior_trees/subtrees 없음)
    bt_dirs = [d for d in (os.path.dirname(bt_xml),
                           os.path.join(bt_pkg, 'behavior_trees'),
                           os.path.join(bt_pkg, 'behavior_trees', 'subtrees')) if os.path.isdir(d)]
    print(f'[nav2_stack] BT 검색 폴더 {bt_dirs}')
    common = {'use_sim_time': sim}
    log = ['--ros-args', '--log-level', lc('log_level')]

    def nav_node(pkg_name, exe, extra=None, remap=()):
        p = [params, common] + ([extra] if extra else [])
        return Node(package=pkg_name, executable=exe, name=exe, output='screen',
                    parameters=p, arguments=log, remappings=list(remap))

    loc = [
        nav_node('nav2_map_server', 'map_server', {'yaml_filename': map_yaml}),
        nav_node('nav2_amcl', 'amcl', {'set_initial_pose': True,
                                       'initial_pose.x': x, 'initial_pose.y': y,
                                       'initial_pose.z': 0.0, 'initial_pose.yaw': yaw}),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[params, common]),
    ]
    nav = [
        nav_node('nav2_controller', 'controller_server',
                 {'FollowPath.batch_size': batch,
                  'FollowPath.visualize': lc('visualize').lower() in ('true', '1')},
                 remap=[('cmd_vel', 'cmd_vel_nav'),
                        ('/local_costmap/footprint', '/swerve/footprint')]),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_bt_navigator', 'bt_navigator',
                 {'default_nav_to_pose_bt_xml': bt_xml,
                  'bt_search_directories': bt_dirs}),
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 remap=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[params, common]),
    ]
    # (11/14) 기동 순서 분리: 위치추정 먼저, 내비게이션은 nav_delay 뒤
    #   Gazebo·RViz·노드 10여 개가 한꺼번에 뜰 때 Fast DDS 가 lifecycle change_state 응답을 못 보내는 경우가 있음
    #   ('failed to send response to /…/change_state (timeout)') → lifecycle_manager 가 그대로 멈춤 (5회 중 2회)
    #   → 한꺼번에 뜨는 노드 수를 줄여 발견(discovery) 부하를 낮춤
    delay = float(lc('nav_delay'))
    return loc + ([TimerAction(period=delay, actions=nav)] if delay > 0 else nav)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='~/slam_maps/nz2_maze_sim_v3.yaml'),
        DeclareLaunchArgument('params_file', default_value='', description='비우면 config/nav2_params.yaml'),
        DeclareLaunchArgument('bt_xml', default_value='', description='비우면 behavior_trees/nz2_nav_to_pose.xml'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.734', description='AMCL 초기 x [m] (mission 좌표)'),
        DeclareLaunchArgument('y', default_value='0.950'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='시뮬 0 / 실전 출발 3.14159'),
        DeclareLaunchArgument('batch_size', default_value='1000', description='MPPI 샘플 수 (개발 PC 2000, Jetson 1000)'),
        DeclareLaunchArgument('visualize', default_value='true',
                              description='MPPI 후보 궤적 RViz 발행 (Jetson 은 false: CPU 절약)'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('nav_delay', default_value='8.0', description='위치추정 기동 뒤 내비게이션 노드 시작까지 [s]'),
        OpaqueFunction(function=_setup),
    ])
