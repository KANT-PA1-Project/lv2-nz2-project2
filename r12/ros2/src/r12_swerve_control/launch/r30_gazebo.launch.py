"""R30 스월브 가제보(gz sim) 구동 — r30_ros.urdf 기준.

R30 은 아직 r30_rev1.urdf(gz 제어 플러그인 내장)이 없어서, r25_rev1 과 동일한 gz 네이티브
플러그인(JointPositionController/JointController/JointStatePublisher)을 런치 시점에 주입한다.
벤더 URDF 는 수정하지 않는다(임시 결합본을 /tmp 에 쓴다). 모델에 r30_rev1 이 생기면 그걸로
교체하고 이 주입은 제거한다.

흐름: real_teleop(별도 터미널, 자체 IK) → /r12/swerve_joint_commands(JointState)
      → gz_command_relay → 관절별 Float64 → ros_gz_bridge → gz → 주입된 플러그인(물리).
real_teleop 은 키보드(termios)라 런치에 넣지 않고 사용자가 별도 터미널에서 실행한다.

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
STEER = ['front_left_steer_joint', 'front_right_steer_joint',
         'rear_left_steer_joint', 'rear_right_steer_joint']
WHEEL = ['front_left_wheel_joint', 'front_right_wheel_joint',
         'rear_left_wheel_joint', 'rear_right_wheel_joint']
ARM = ['arm_yaw_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
       'wrist_roll_joint', 'jaw_left_joint', 'jaw_right_joint']
WHEEL_LINK = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']


def _vendor_dir():
    """R30 지오메트리 위치. env R30_VENDOR_DIR → r30_description 설치 share → 소스 트리 폴백."""
    env = os.environ.get('R30_VENDOR_DIR')
    if env:
        return env
    try:
        return get_package_share_directory('r30_description')
    except PackageNotFoundError:
        # 소스: r12/ros2/src/r12_swerve_control/launch → r12 까지 4단계 상위
        here = os.path.dirname(os.path.realpath(__file__))
        return os.path.normpath(os.path.join(here, '..', '..', '..', '..',
                                             'assets', 'vendor', 'R30_URDF_USD'))


def _gz_control_block():
    """r25_rev1 과 동일한 gz 네이티브 제어 플러그인. 조향·팔=position, 휠=velocity."""
    out = ['<gazebo><plugin filename="gz-sim-joint-state-publisher-system" '
           'name="gz::sim::systems::JointStatePublisher"><topic>/joint_states</topic></plugin></gazebo>']
    for j in STEER:
        out.append(
            f'<gazebo><plugin filename="gz-sim-joint-position-controller-system" '
            f'name="gz::sim::systems::JointPositionController"><joint_name>{j}</joint_name>'
            f'<topic>/swerve/sim/{j}/cmd_pos</topic><p_gain>10.0</p_gain><i_gain>0.0</i_gain>'
            # cmd_max ≥ 실물 조향속 7.9(STEER_PROFILE_VEL 330). sim 이 real 보다 느리면 안 됨.
            f'<d_gain>0.0</d_gain><cmd_max>8.0</cmd_max><cmd_min>-8.0</cmd_min>'
            f'<use_velocity_commands>true</use_velocity_commands></plugin></gazebo>')
    for j in WHEEL:
        out.append(
            f'<gazebo><plugin filename="gz-sim-joint-controller-system" '
            f'name="gz::sim::systems::JointController"><joint_name>{j}</joint_name>'
            f'<topic>/swerve/sim/{j}/cmd_vel</topic><initial_velocity>0.0</initial_velocity>'
            f'</plugin></gazebo>')
    for j in ARM:
        out.append(
            f'<gazebo><plugin filename="gz-sim-joint-position-controller-system" '
            f'name="gz::sim::systems::JointPositionController"><joint_name>{j}</joint_name>'
            f'<topic>/arm/sim/{j}/cmd_pos</topic><p_gain>10.0</p_gain><i_gain>0.0</i_gain>'
            f'<d_gain>0.0</d_gain><cmd_max>2.0</cmd_max><cmd_min>-2.0</cmd_min>'
            f'<use_velocity_commands>true</use_velocity_commands></plugin></gazebo>')
    for link in WHEEL_LINK:
        out.append(f'<gazebo reference="{link}"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>')
    return '\n'.join(out)


def _robot_description():
    vendor = _vendor_dir()
    with open(os.path.join(vendor, 'urdf', 'r30_ros.urdf')) as f:
        urdf = f.read()
    mesh_abs = 'file://' + os.path.join(vendor, 'meshes') + '/'
    urdf = urdf.replace('package://r30_description/meshes/', mesh_abs)
    urdf = urdf.replace('../meshes/', mesh_abs)
    urdf = urdf.replace('</robot>', _gz_control_block() + '</robot>')
    with open(os.path.join(tempfile.gettempdir(), 'r30_gz_combined.urdf'), 'w') as f:
        f.write(urdf)
    return urdf


def generate_launch_description():
    urdf_xml = _robot_description()
    share = get_package_share_directory(PKG)
    bridge_cfg = os.path.join(share, 'config', 'r25_gz_bridge.yaml')   # 토픽 동일 → 재사용
    params = os.path.join(share, 'config', 'r12_swerve.yaml')
    headless = LaunchConfiguration('headless')
    gz_args = PythonExpression(["'-r -s empty.sdf' if '", headless, "' == 'true' else '-r empty.sdf'"])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': urdf_xml}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'r30', '-z', '0.2'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': bridge_cfg}], output='screen')
    # 실물과 동일: teleop(/cmd_vel) → swerve_controller(IK) → /r12/swerve_joint_commands → relay
    controller = Node(package=PKG, executable='swerve_controller',
                      name='r12_swerve_controller', parameters=[params], output='screen')
    relay = Node(package=PKG, executable='gz_command_relay', name='gz_command_relay', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        gz, rsp, spawn, bridge, controller, relay,
    ])
