# 내 URDF로 MoveIt 2와 cuMotion 연결

## 1. Isaac ROS 5.0 컨테이너에서 설치

[호스트 준비](PREREQUISITES_UBUNTU24.04.md)를 끝내고 다음을 실행합니다.

```bash
isaac-ros activate
```

컨테이너 안:

```bash
source /opt/ros/lyrical/setup.bash
test "$ROS_DISTRO" = lyrical
sudo apt update
sudo apt install -y ros-lyrical-moveit ros-lyrical-moveit-setup-assistant \
  ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
  ros-lyrical-isaac-ros-cumotion-examples
ros2 pkg prefix moveit_setup_assistant
ros2 pkg prefix isaac_ros_cumotion
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

패키지 후보가 없으면 다른 배포판 이름으로 바꾸지 않습니다. [설치 가이드](INSTALL.md)의 Buildfarm 저장소·키·apt-cache 확인을 진행합니다.

## 2. Setup Assistant에서 작성

| 단계 | 입력 | 결과 |
|---|---|---|
| URDF 읽기 | 내 기구 파일·메시 | 실제 크기의 링크 |
| Self Collision | 실제 관절 범위 | 검토한 충돌 제외 행렬 |
| Virtual Joint | 고정 베이스 기준 | world와 base 연결 |
| Planning Group | arm 관절 체인 | 팔 계획 그룹 |
| End Effector | 도구·집게 | 손목과 TCP 구분 |
| Named Pose | 충돌 없는 home | 시작 자세 |
| Controllers | joint_names와 인터페이스 | FollowJointTrajectory 연결 |
| Generate | my_arm_moveit_config | SRDF·YAML·launch 패키지 |

생성한 패키지를 `$ISAAC_ROS_WS/src` 아래에 놓고 빌드합니다. 실제 패키지 이름으로 바꿉니다.

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select my_arm_moveit_config
source install/setup.bash
ros2 launch my_arm_moveit_config demo.launch.py
```

먼저 OMPL과 가상 하드웨어에서 Plan·Execute를 확인합니다. 실제 모터가 움직였다는 의미는 아닙니다.

## 3. XRDF와 cuMotion 추가

Sim의 Robot Description Editor에서 내 URDF에 맞는 XRDF를 작성합니다. collision spheres, cspace 관절, 기본 관절값, tool frame을 확인합니다. 임의의 다른 로봇 XRDF를 파일명만 바꿔 쓰지 않습니다.

```bash
export ROBOT_URDF=/absolute/path/my_arm.urdf
export ROBOT_XRDF=/absolute/path/my_arm.xrdf
test -f "$ROBOT_URDF" && test -f "$ROBOT_XRDF"
ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
  cumotion_action_server.xrdf_file_path:="$ROBOT_XRDF" \
  cumotion_action_server.urdf_file_path:="$ROBOT_URDF" \
  cumotion_action_server.tool_frame:=tool0 \
  cumotion_action_server.read_esdf_world:=false
```

다른 컨테이너 터미널에서 설치된 설정 파일 위치를 확인합니다.

```bash
find "$(ros2 pkg prefix isaac_ros_cumotion_moveit)/share" \
  -name isaac_ros_cumotion_planning.yaml -print
```

이 YAML을 내 MoveIt config에 복사하고 launch의 MoveItConfigsBuilder에 `planning_pipelines(pipelines=["ompl", "isaac_ros_cumotion"])`를 연결합니다. SRDF·kinematics.yaml·joint_limits.yaml은 내 로봇 설정을 유지합니다. RViz에서 cuMotion을 선택해 동일 목표의 경로와 충돌을 비교합니다.

## 4. 실제 제어 확인 순서

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 action list -t
```

목표 controller action 경로가 MoveIt YAML과 같고, 관절 이름·단위·측정값이 맞아야 합니다. MoveIt Servo와 정책 실행은 별도 모드로 전환합니다. 계획·Servo·정책이 동시에 같은 관절을 제어하지 않습니다.

공식 참고: [cuMotion](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_cumotion/index.html), [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html), [Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html).
