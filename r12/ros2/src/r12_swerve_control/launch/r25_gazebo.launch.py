"""R25 스월브 가제보(gz sim) 구동 — r25_rev1.urdf 기준.

흐름: teleop(/cmd_vel) → r12_swerve_controller(IK) → /r12/swerve_joint_commands(JointState)
      → gz_command_relay → 관절별 Float64 → ros_gz_bridge → gz → r25_rev1 내장 플러그인(물리).

r25_rev1.urdf 가 gz JointPositionController(조향·팔)·JointController(휠)·JointStatePublisher 를
내장하므로 ros2_control 주입은 하지 않는다(r21 방식 폐기). 메시 package:// 는 절대 file:// 로 바꿔
벤더 빌드 여부와 무관하게 로드되게 한다. 벤더 URDF 는 수정하지 않는다(임시 결합본을 /tmp 에 쓴다).

headless:=true (기본) 서버만. 화면으로 보려면 headless:=false.
"""
import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = 'r12_swerve_control'


def _vendor_dir():
    """R25 지오메트리(urdf·meshes) 위치. env → r25_description 설치 share → 소스 트리 폴백."""
    env = os.environ.get('R25_VENDOR_DIR')
    if env:
        return env
    try:
        return get_package_share_directory('r25_description')
    except PackageNotFoundError:
        here = os.path.dirname(os.path.realpath(__file__))   # 소스: r12/ros2/r12_swerve_control/launch
        return os.path.normpath(os.path.join(here, '..', '..', '..', 'assets', 'vendor', 'R25_URDF_USD'))


def _robot_description():
    vendor = _vendor_dir()
    with open(os.path.join(vendor, 'urdf', 'r25_rev1.urdf')) as f:
        urdf = f.read()
    # rev1 은 메시를 상대경로 '../meshes/' 로 참조 → robot_description 문자열론 CWD 기준이라 깨진다.
    # 절대 file:// 로 치환(벤더 빌드 여부·CWD 무관하게 로드). package:// 형식도 혹시 몰라 함께 치환.
    mesh_abs = 'file://' + os.path.join(vendor, 'meshes') + '/'
    urdf = urdf.replace('../meshes/', mesh_abs)
    urdf = urdf.replace('package://r25_description/meshes/', mesh_abs)
    out = os.path.join(tempfile.gettempdir(), 'r25_rev1_combined.urdf')
    with open(out, 'w') as f:
        f.write(urdf)
    return urdf


def generate_launch_description():
    urdf_xml = _robot_description()
    share = get_package_share_directory(PKG)
    params = os.path.join(share, 'config', 'r12_swerve.yaml')
    bridge_cfg = os.path.join(share, 'config', 'r25_gz_bridge.yaml')
    headless = LaunchConfiguration('headless')
    gz_args = PythonExpression(["'-r -s empty.sdf' if '", headless, "' == 'true' else '-r empty.sdf'"])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': urdf_xml}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'r25', '-z', '0.2'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': bridge_cfg}], output='screen')
    controller = Node(package=PKG, executable='swerve_controller',
                      name='r12_swerve_controller', parameters=[params], output='screen')
    relay = Node(package=PKG, executable='gz_command_relay', name='gz_command_relay', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        gz, rsp, spawn, bridge, controller, relay,
    ])
