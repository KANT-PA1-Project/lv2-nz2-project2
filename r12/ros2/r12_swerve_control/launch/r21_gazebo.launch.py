"""R21 스월브 가제보(gz sim + gz_ros2_control) 구동.

흐름: cmd_vel → r12_swerve_controller(IK) → /r12/swerve_joint_commands(JointState)
      → gz_command_relay → 조향 position·휠 velocity 컨트롤러 → 가제보 물리.

벤더 URDF(r21_ros.urdf)는 센서 gazebo 태그만 있고 구동 ros2_control 이 없어서, 런치 시점에
ros2_control + gz_ros2_control 플러그인 블록을 주입하고 메시 package:// 를 절대 file:// 로 바꾼다.
벤더 파일은 수정하지 않는다(임시 결합 URDF 를 /tmp 에 쓴다).

headless:=true (기본) 서버만. 화면으로 보려면 headless:=false.
"""
import os
import re
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = 'r12_swerve_control'


def _vendor_dir():
    """R21 지오메트리(urdf·meshes) 위치. 절대경로 하드코딩 금지(다른 PC 에서 깨짐).

    순서: ① env R21_VENDOR_DIR ② 벤더 패키지 r21_description 의 설치 share(CMakeLists 가
    urdf·meshes·usd 를 설치) ③ r21_description 을 따로 안 빌드했을 때: symlink-install 의 launch 를
    realpath 로 소스로 되돌려 소스 트리의 r12/assets/vendor/R21_URDF_USD 로 폴백.
    """
    env = os.environ.get('R21_VENDOR_DIR')
    if env:
        return env
    try:
        return get_package_share_directory('r21_description')
    except PackageNotFoundError:
        here = os.path.dirname(os.path.realpath(__file__))   # 소스: r12/ros2/r12_swerve_control/launch
        return os.path.normpath(os.path.join(here, '..', '..', '..', 'assets', 'vendor', 'R21_URDF_USD'))

STEER = ['front_left_steer_joint', 'front_right_steer_joint',
         'rear_left_steer_joint', 'rear_right_steer_joint']
WHEEL = ['front_left_wheel_joint', 'front_right_wheel_joint',
         'rear_left_wheel_joint', 'rear_right_wheel_joint']
# 팔·그리퍼는 보류(스월브 데모엔 컨트롤러 없음). sim 에서 free 면 중력에 늘어져 끌린다 →
# 결합 URDF 에서만 fixed 로 고정해 홈 자세로 붙여 둔다(소스 URDF 는 revolute/prismatic 유지).
ARM_HOLD = ['arm_yaw', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'jaw_left', 'jaw_right']


def _controllers_yaml():
    return os.path.join(get_package_share_directory(PKG), 'config', 'r21_gz_controllers.yaml')


def _ros2_control_block(controllers_yaml):
    j = []
    for name in STEER:
        j.append(f'''  <joint name="{name}">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>''')
    for name in WHEEL:
        j.append(f'''  <joint name="{name}">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>''')
    joints = '\n'.join(j)
    return f'''<ros2_control name="r21_swerve" type="system">
  <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
{joints}
</ros2_control>
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>{controllers_yaml}</parameters>
  </plugin>
</gazebo>
'''


def _build_combined_urdf():
    vendor = _vendor_dir()
    src = os.path.join(vendor, 'urdf', 'r21_ros.urdf')
    with open(src) as f:
        urdf = f.read()
    # 메시 경로를 절대 file:// 로 (gz 리소스 경로 문제 회피)
    urdf = urdf.replace('package://r21_description/meshes/',
                        'file://' + os.path.join(vendor, 'meshes') + '/')
    # 팔·그리퍼 관절을 fixed 로 (보류 — 컨트롤러 없이 free 면 늘어짐)
    names = '|'.join(j + '_joint' for j in ARM_HOLD)
    urdf = re.sub(r'(<joint name="(?:' + names + r')" type=")[^"]+(">)', r'\1fixed\2', urdf)
    controllers = _controllers_yaml()
    block = _ros2_control_block(controllers)
    urdf = urdf.replace('</robot>', block + '</robot>')
    out = os.path.join(tempfile.gettempdir(), 'r21_gz_combined.urdf')
    with open(out, 'w') as f:
        f.write(urdf)
    return out, urdf


def generate_launch_description():
    combined_path, urdf_xml = _build_combined_urdf()
    params = os.path.join(get_package_share_directory(PKG), 'config', 'r12_swerve.yaml')
    headless = LaunchConfiguration('headless')
    gz_args = PythonExpression(["'-r -s empty.sdf' if '", headless, "' == 'true' else '-r empty.sdf'"])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': urdf_xml}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'r21', '-z', '0.2'])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--param-file', _controllers_yaml()], output='screen')

    jsb = spawner('joint_state_broadcaster')
    steer_c = spawner('steer_position_controller')
    wheel_c = spawner('wheel_velocity_controller')

    controller = Node(package=PKG, executable='swerve_controller',
                      name='r12_swerve_controller', parameters=[params], output='screen')
    relay = Node(package=PKG, executable='gz_command_relay', name='gz_command_relay', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        gz, rsp, spawn,
        # 스폰 완료 후 컨트롤러를 차례로 띄운다(플러그인 controller_manager 가 뜬 뒤여야 함)
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[steer_c, wheel_c, controller, relay])),
    ])
