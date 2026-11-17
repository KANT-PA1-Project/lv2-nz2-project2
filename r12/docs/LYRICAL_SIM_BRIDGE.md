# Isaac Sim 6.1과 ROS 2 Lyrical 연결

제어·인식 노드는 Lyrical을 사용합니다. Sim 6.1 브리지의 공식 권장 배포판과 이 요구사항은 별도입니다. 다른 네이티브 ROS 배포판을 Sim에 직접 로드하는 경로는 실험적입니다.

기존 YOLO 저장소는 **Sim의 bundled Jazzy 브리지와 Lyrical 컨테이너를 분리**해 카메라 토픽을 검증했습니다. 그 결과가 로봇팔 action·관절 명령까지 검증했다는 뜻은 아닙니다. [기존 카메라 연결 기록](https://github.com/SeongminJaden/yolov8-ubuntu24-practice/blob/main/docs/ISAAC_SIM.md)을 참고하되 팔 제어는 아래 항목을 새로 확인합니다.

## 기본 실습: 프로세스 분리

| 프로세스 | ROS 환경 | 확인할 내용 |
|---|---|---|
| Sim 6.1 호스트 | 공식 bundled Jazzy 브리지 | scene·clock·joint state·command graph |
| 제어 컨테이너 | Isaac ROS 5.0 / Lyrical | MoveIt·controller·TF |
| DDS 통신 | 동일 domain·호환 QoS | 실제 메시지와 시간·프레임 |

Sim 터미널에 Lyrical setup을 source하지 않습니다. Lyrical 라이브러리와 bundled Jazzy 라이브러리를 한 프로세스에 섞지 않습니다. 같은 PC에서 host networking을 사용하고 domain을 맞춥니다.

Lyrical 컨테이너 터미널:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic list -t
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo base_link tool0
```

Sim의 ROS graph에서 같은 domain을 사용하고 `/joint_states` publisher와 관절 command subscriber를 설정합니다. URDF 관절 순서를 고정하고 graph의 robot target이 실제 articulation을 가리키도록 합니다. ROS bridge extension을 켠 것만으로 이 graph가 자동 완성되지는 않습니다.

## 통과 기준

1. Sim이 Play 중일 때 clock이 증가하며, 정지하면 멈춥니다.
2. JointState의 name·position·velocity 길이가 일치합니다.
3. 가상 제어 명령을 관절 하나에 보내면 그 관절만 올바른 방향으로 움직입니다.
4. TF에서 base→TCP를 찾을 수 있고 단위·타임스탬프가 일치합니다.
5. 컨트롤러의 FollowJointTrajectory 결과와 실제 시뮬레이션 도달 상태가 일치합니다.

## 선택 실험: Sim 프로세스에 네이티브 Lyrical 로드

호스트에 Ubuntu 24.04용 Isaac ROS Buildfarm Lyrical 패키지를 설치한 별도 환경에서만 시험합니다. 새 터미널에서 `/opt/ros/lyrical/setup.bash`를 source한 뒤 Sim을 실행합니다. 실패 시 bundled Jazzy 경로로 돌아가되, 같은 터미널에 Jazzy 라이브러리를 추가해 덮어쓰지 않습니다. 이 저장소에서는 네이티브 Lyrical 로드와 로봇팔 DDS 통합을 실행 검증하지 않았습니다.

공식 근거: [Sim 6.1 ROS 설치와 배포판 지원](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html).
