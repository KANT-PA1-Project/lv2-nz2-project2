# nz2_swerve_nav/launch/sim_corridor.launch.py
# 개발 PC: Gazebo 통로 시험 (assembly_nz2.urdf + corridor_test.sdf + 브리지 + RSP + 스워브 노드)
#   ros2 launch nz2_swerve_nav sim_corridor.launch.py
#   ros2 launch nz2_swerve_nav sim_corridor.launch.py urdf_dir:=$HOME/Downloads/assembly_URDF rviz:=true
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    urdf_dir = os.path.expanduser(LaunchConfiguration('urdf_dir').perform(context))
    world = LaunchConfiguration('world').perform(context) or os.path.join(urdf_dir, 'corridor_test.sdf')
    urdf_path = os.path.join(urdf_dir, 'assembly_nz2.urdf')
    bridge_cfg = os.path.join(urdf_dir, 'ros_gz_bridge_nz2.yaml')
    for f in (urdf_path, world, bridge_cfg):
        if not os.path.isfile(f):
            raise RuntimeError(f'파일 없음: {f}')

    with open(urdf_path, encoding='utf-8') as f:
        urdf = f.read()
    # 상대 메시 경로 → 절대 경로 (RViz·Gazebo 모두 읽도록)
    urdf = urdf.replace('filename="meshes/', f'filename="file://{urdf_dir}/meshes/')

    pkg = get_package_share_directory('nz2_swerve_nav')
    swerve_cfg = os.path.join(pkg, 'config', 'swerve_nav.yaml')
    sim = {'use_sim_time': True}

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               urdf_dir + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf}, sim]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': bridge_cfg}, sim]),
        # Gazebo 가 뜬 뒤 로봇 생성 (base_footprint = 바닥)
        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.003']),
        ]),
        Node(package='nz2_swerve_nav', executable='swerve_controller_node', output='screen',
             parameters=[swerve_cfg, {'sim_joint_topics': True}, sim]),
        Node(package='nz2_swerve_nav', executable='scan_merger_node', output='screen',
             parameters=[swerve_cfg, sim]),
        Node(package='rviz2', executable='rviz2', output='screen', parameters=[sim],
             arguments=['-d', os.path.join(pkg, 'config', 'nz2_sim.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_dir', default_value='~/Downloads/assembly_URDF'),
        DeclareLaunchArgument('world', default_value='', description='비우면 urdf_dir/corridor_test.sdf'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
