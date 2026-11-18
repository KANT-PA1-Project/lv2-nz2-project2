# lv2-nz2-project2

## tutor: R12 최신 개발 인계 (2026-11-05)

**로컬만 실행하며 pa24·pa33·pa28은 중단 상태입니다. 전체 미션은 아직 미완료입니다.**

- [작업 지침](AGENTS.md) · [현황과 검증 한계](docs/R12_STATUS.md)
- [아키텍처](docs/R12_ARCHITECTURE.md) · [PC별 할 일](docs/R12_TASKS.md)
- [모델·해시·실행 준비](docs/R12_MODELS.md) · [최신 개발 소스](r12/)

아래 기존 팀 소개/하드웨어 문서는 이력으로 보존했습니다. R12 최신 실험 상태와 다른 가정은 위 인계 문서에서 구분합니다.

KANT PA1 LV2 프로젝트2 팀 NZ2의 저장소다.

## 프로젝트 소개

발제 v5 「주사위 기억 로봇」을 수행하는 모바일 매니퓰레이터를 만든다. 신호등 점멸 횟수와 바코드를 읽어 목적지를 정하고, 빨강·초록 퍽을 주사위 눈 모양으로 옮겨 놓는다. 스월브 드라이브로 전방향 주행하고, 개조한 OpenMANIPULATOR-X로 퍽을 집어 옮긴다.

| 구성 | 내용 |
|---|---|
| 구동 | 스월브 4모듈 (XM430-W210 ×8 = 조향 4 + 구동 4, 터틀봇3 와플 바퀴) |
| 매니퓰레이터 | OpenMANIPULATOR-X 개조 (XM430-W350) — 링크 약 +100mm 연장, 손목 롤 관절 추가 |
| 센서 | Intel RealSense D435 ×2 (손목 `wrist_d435` + 마스트 `mast_d435`), 2D 라이다 LDS-03 ×4 (앞·뒤·좌·우). QR·바코드·AprilTag·신호등 색상 인식 |
| 연산·제어 | Jetson Orin Nano Developer Kit (비전·상위 제어), OpenCR 1.0 3대 (스월브 좌 FL·RL / 스월브 우 FR·RR / 로봇팔) |
| SLAM | 실물: 라이다 4대 병합 스캔 → Cartographer + AMCL, Nav2(MPPI). 시뮬(R12): cuVSLAM·nvBlox |

## 대회 미션과 규정

- **대회**: 로봇 기능대회 모바일 로보틱스 부문 (국제올림피아드 모바일로보틱스 규칙 준용). 대회일 **2026-11-16**. 세부 규칙은 미공개라 아래 일부는 `[가정]`이다.
- **미션 (발제 v5 「주사위 기억 로봇」)**:
  1. 출발·관측 위치 (734, 950) mm에서 −X 방향의 신호등(바닥, (80, 950))과 바코드((80, 650), 높이 450 mm)를 본다.
  2. 신호등은 경기 전부터 **무한루프**로 R·Y·G 12회 점멸을 반복한다. 12번째 뒤 3초 소등 다음부터가 한 프레임의 시작이다.
  3. R·Y·G 횟수(각 1~6, 합 12)를 바코드 규칙(P2-01/03 오름차순, P2-02/04 내림차순, 동률 R→Y→G)으로 정렬해 L1·L2·L3에 배정한다.
  4. 시작 박스 6곳의 퍽 12개(red·green, 원통 Ø30×60 mm / 직육면체 30×30×60 mm)를 탐색해 이동 전에 전체 할당을 확정한다. R 그룹 = 빨강, G 그룹 = 초록, Y 그룹 = 같은 모양(색 무관).
  5. 각 장소에 주사위 눈(점 간격 60 mm)으로 놓고, 세 장소를 다시 촬영해 재검증한 뒤 출발 구역으로 복귀해 3초 정지한다.
  - 상세 설계: [docs/planning/mission_fsm_design.md](docs/planning/mission_fsm_design.md)
- **퍽**: 빨강·초록 2색, 원통 Ø30×60 mm 또는 직육면체 30×30×60 mm. 세워져 있거나 누워 있을 수 있다.
- **선반(박스)**: 205×205×70 mm (윗면 높이 70 mm).
- **폐기 (발제 v5)**: 운영팀 배포 매핑 DB, `mission_reader`, `MAPPING` 상태는 쓰지 않는다.
- **규정 (실격 조건)**:
  - 시작·종료 시 로봇이 **400×400mm 이내**에 들어와야 한다(높이 제한 없음).
  - 벽·선반에 **접촉하면 실격**이다. 주행·파지 경로에 여유 간격을 두고 설계한다.

## 하드웨어 구성

| 항목 | 내용 |
|---|---|
| 구동부 | 스월브 드라이브 4모듈. 모듈당 조향 1 + 구동 1, 총 **XM430-W210 ×8**. 바퀴는 터틀봇3 와플 바퀴(반지름 33 mm) |
| 모터 ID | 구동 FL 1 · FR 2 · RR 3 · RL 4, 조향 FL 7 · FR 6 · RR 8 · RL 5 (2026-11-12 실물 확인) |
| 구동 기어 | 구동 모터→바퀴 **외부 1:4 증속** (바퀴 = 모터 × 4). 이론 무부하 약 0.9 m/s(11.1 V), 운용 상한 0.6 m/s. 조향은 1:1, 범위 ±90° |
| 매니퓰레이터 | OpenMANIPULATOR-X 개조. 구동 모터 **XM430-W350**, 링크 약 +100mm 연장, 손목 롤 모터 추가(관절 5: arm_yaw·shoulder·elbow·wrist_pitch·wrist_roll). 그리퍼는 표준 OMX 그리퍼 그대로(XM430-W350-T 1개, 대칭 평행 핑거). 팔 베이스는 차체 중심에서 **+X 약 154 mm 전방**에 장착하며, 팔 쪽이 로봇 정면이다 |
| 제어기 | OpenCR 1.0 3대. `swerve_left`(FL·RL 구동·조향 + IMU), `swerve_right`(FR·RR 구동·조향 + IMU), `arm`(팔 관절 ID 11~14·손목 롤 16·그리퍼 15). 브리지는 `I` 핸드셰이크의 역할명으로 포트를 찾는다. 상위 컴퓨터는 Jetson Orin Nano |
| 카메라 | Intel RealSense D435 ×2 (R30 모델 기준). `wrist_d435`: 손목 롤 링크, 하향 30°. `mast_d435`: 마스트, 바닥 기준 약 681 mm, 하향 35°. 시뮬 카메라 설정(`r12/configs/sensors/camera_rig.json`)은 아직 이전 구성이라 실물과 다르다 |
| 라이다 | **LDS-03 ×4** (front·rear·left·right, 차체 중심 ±189 mm). 드라이버 `coin_d4_driver`, 포트 `/dev/lidar_*`(udev: `r12/system/udev/99-usb-serial.rules`). LDS-02·`nz2_ld08_driver`는 폐기. rear·left 실제 위치는 `[확인 필요]` |
| 퍽 적재 | 차체 상판 퍽 슬롯 **8개** (좌 4 + 우 4, 차체 중심 기준 x ±35 mm, y ±95·±145 mm) |
| 전원 | 12V SMPS 약 30A, OpenCR 12V 5A |
| 기구 | 2020 프로파일 프레임 외곽 360×360 mm, 스월브 모듈 간격 248 mm(중심 ±124), base_link 높이 181 mm. 운반 자세(팔 접음·그리퍼 열림) footprint 약 515×500 mm(+패딩 40 mm). 팔 제외 높이 `[확인 필요]`, 목표 총중량 5kg 이내. 2020 프로파일 450mm 설치. Fusion 360 설계 + Bambu Lab 3D 프린팅 |

**아직 정해지지 않은 항목**

- 로봇 전체 높이 (팔 포함)
- 시작·종료 시 400×400 규정을 만족하는 팔 접기 자세 (운반 자세 footprint는 400 mm 초과)

## 소프트웨어 아키텍처

상위·하위 2단으로 나눈다. 상위(Jetson)는 인지·판단·경로 계획을, 하위(OpenCR 3대)는 실시간 모터 제어를 맡는다.

```
[Jetson Orin Nano] ROS 2
  ├─ 인지     RealSense D435 → 깊이·영상, AprilTag·바코드·신호등 색상 인식
  ├─ 주행     라이다 4대 → Cartographer + AMCL, Nav2(MPPI), 스월브 IK (r12_swerve_control)
  ├─ 팔 계획  MoveIt 2 (r12_arm_moveit_config: KDL + OMPL), cuMotion용 XRDF 준비
  └─ 미션 FSM 신호 기억 → 바코드 정렬 → 탐색 → 할당·운반 계획 → 주사위 배치 → 재검증 → 복귀 (docs/planning/mission_fsm_design.md)

        │  USB 시리얼, 보드 식별 `I` 핸드셰이크 (proto 1)
        ├─ r12_opencr_bridge ─┬──────────────────┐        └─ nz2_arm_bridge ─┐
  [OpenCR swerve_left]   [OpenCR swerve_right]                  [OpenCR arm]
  FL·RL 조향+구동, IMU    FR·RR 조향+구동, IMU                    관절 11~14, 롤 16, 그리퍼 15
  (JSON 한 줄)            (JSON 한 줄)                           (텍스트 명령 A/G/K/T/S/R/Q/D/E)
```

- **주행 제어**: 스월브 역기구학은 **Jetson**에서 계산한다(`r12/ros2/src/r12_swerve_control`의 `swerve_controller_node`, 해석 IK). OpenCR은 모듈별 조향각·구동 각속도 명령만 받는다.
- **Jetson↔OpenCR 통신**: micro-ROS가 아닌 **커스텀 USB 시리얼**이다. 한 줄 = JSON 1개, 부팅 시와 `I` 명령에 보드가 역할(`board`)을 답해 ttyACM 번호와 무관하게 포트를 배정한다. 상세는 [r12/docs/OPENCR_HANDSHAKE.md](r12/docs/OPENCR_HANDSHAKE.md).
- **자기위치·주행(실물)**: LDS-03 4대 스캔을 병합(`scan_merger`)해 Cartographer로 지도를 만들고 AMCL로 위치를 추정하며, Nav2 컨트롤러는 MPPI를 쓴다(`ros2_ws/src/planning/nz2_swerve_nav`). cuVSLAM·nvBlox는 R12 시뮬 아키텍처에서 쓴다.
- **ROS 설정(실물)**: 브리지 관절 상태 토픽은 `/r12/joint_states`, `ROS_DOMAIN_ID=78`. `nz2_bringup/config/robot.yaml`의 `/nz2` 네임스페이스와 다르다.
- **팔 제어**: 팔 OpenCR이 다이나믹셀 위치·전류 제어를 담당하고, 상위에서 팔 경로를 계획한다. 최신 팔 펌웨어는 `r12/experiments/jetson_real/firmware/nz2_arm_v2_j2diag/`(FW 2.0, limits_rev 8, diag_rev 2, 커밋 `d63ebcd`), Jetson 쪽 브리지는 `r12/experiments/jetson_real/ros_ws/src/nz2_arm_bridge/`(관절 공간 시리얼 브리지, 기본 읽기 전용 `allow_motion: false`)다. 둘 다 PA33 실물 작업의 보관 사본이며, 새 경로에서의 빌드·실물 실행은 검증되지 않았다. 경로 계획은 MoveIt 2 설정(`r12/ros2/src/r12_arm_moveit_config`, KDL + OMPL)이 있고 cuMotion용 `r12.xrdf`를 준비해 두었다.
- **인지**: RealSense D435로 퍽 위치·자세(세움/누움)를 추정하고, AprilTag·바코드·신호등 색상으로 과제 순서와 위치를 판별한다.
- **시뮬레이션**: Isaac Sim으로 경기장·퍽을 재현하고, 도메인 랜덤화로 sim-to-real 간극을 줄인다. 실제 적용 여부는 `[확인 필요]`.
- **ROS 2 패키지 구성**: Jetson 쪽 코드는 `ros2_ws/src/` 아래 역할별 레이어(interfaces·perception·planning·control)로 나눈다. 자세한 구분은 아래 "디렉터리 구조"를 본다.
- **로봇 모델**: `ros2_ws/src/nz2_description`의 xacro는 아직 placeholder 값(400×400, 모듈 ±150 mm, 팔 x=0)이다. 실제 모델은 R30 URDF(`r12/assets/vendor/R30_URDF_USD/urdf/`)이며, Jetson은 그 원본 `real_final_plz_no_more_edit_v99_nz2fix.urdf`를 저장소 밖 `~/nz2_urdf/`에 복사해 쓴다.
- 각 패키지 버전은 아래 "환경과 버전" 표를 기준으로 한다.

## 팀 NZ2

| 역할 | 이름 | GitHub |
|---|---|---|
| 팀장 | 천경호 | [@pizzaafterhangover](https://github.com/pizzaafterhangover) |
| 팀원 | 최형준 | [@chj1319](https://github.com/chj1319) |
| 팀원 | 정수용 | [@affluentmind12](https://github.com/affluentmind12) |
| 팀원 | 이성민(튜터) | [@SeongminJaden](https://github.com/SeongminJaden) |

## 환경과 버전

다른 기기에서 clone한 뒤 같은 결과를 얻기 위한 기준 환경이다. [미정] 항목은 팀에서 정한 뒤 채운다.

| 항목 | 개발 PC | 로봇 (Jetson Orin Nano Developer Kit) |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS | Ubuntu 24.04.5 LTS (L4T R39.2.1) |
| ROS 2 배포판 | Lyrical (ros-base 0.13.0) | Lyrical (ros-base 0.13.0) |

사용하는 소프트웨어·라이브러리

| 이름 | 버전 | 설치 방법 | 사용처 |
|---|---|---|---|
| Isaac Sim | 6.1.0 | 소스 빌드 ([isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim) 태그 `v6.1.0`) | 시뮬레이션 |
| Isaac Lab | v3.0.0-EA | 소스 (태그 `v3.0.0-EA`, Sim 6.1 대응) | 개발 PC · 정책 학습 (r12) |
| nvblox | `[확인 필요]` | Isaac ROS 5.0 | 시뮬 · 3D 장애물 맵 (`r12/configs/sensors/nvblox.yaml`) |
| Gazebo (gz) | `[확인 필요]` | `[확인 필요]` | 개발 PC · R25/R30 주행 시뮬 (`r30_gazebo.launch.py`) |
| Isaac ROS | 5.0 | Docker ([isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) 태그 `v5.0-0`, isaac-ros-cli 2.6.0) | cuVSLAM |
| realsense2_camera | 4.57.7 | apt (NVIDIA Isaac ROS 저장소) | Jetson · RealSense 드라이버 |
| robot_state_publisher | 3.5.5 | apt (NVIDIA Isaac ROS 저장소) | Jetson · URDF → TF |
| xacro | 2.1.1 | apt (NVIDIA Isaac ROS 저장소) | Jetson · URDF |
| ros2_control | 6.9.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 하드웨어 제어 프레임워크 |
| ros2_controllers | 6.9.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 팔·그리퍼 컨트롤러 |
| dynamixel_hardware_interface | 1.5.1 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 다이나믹셀 하드웨어 인터페이스 |
| dynamixel_sdk | 4.0.3 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 다이나믹셀 통신 |
| navigation2 | 1.5.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 주행(Nav2), nav2_bringup·nav2_amcl·nav2_mppi_controller 포함 |
| robot_localization | 3.10.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 설치됨, 현재 사용 계획 없음 |
| pointcloud_to_laserscan | 2.1.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 점구름 → 스캔 |
| cartographer_ros | `[확인 필요]` | 소스 빌드 (Jetson `~/ros2_lyrical`) | Jetson · 라이다 SLAM (nz2_swerve_nav) |
| slam_toolbox | `[확인 필요]` | `[확인 필요]` | 지도 작성 대안 (설정만 존재: `slam_toolbox_mapping.yaml`) |
| coin_d4_driver | `[확인 필요]` | `[확인 필요]` | Jetson · LDS-03 라이다 드라이버 |
| twist_mux | 4.5.1 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 수동 조작 테스트 |
| vision_msgs | 4.2.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 인지 메시지 |
| control_msgs | 6.10.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 팔·그리퍼 액션 |
| cv_bridge | 4.1.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 영상 변환 |
| MoveIt 2 | 2.15.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 팔 경로 계획 후보 (q-motion과 택일, `[미정]`) |
| joint_state_publisher | 2.4.3 | apt (NVIDIA Isaac ROS 저장소) | Jetson · URDF 확인용 (nz2_description) |
| rviz2 | 15.2.5 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 시각화·URDF 확인용 |
| colcon-common-extensions / rosdep / vcstool | 0.3.0 / 0.27.0 / 0.3.0 | apt | Jetson · 빌드 도구 |
| arduino-cli | 1.5.1 | GitHub 릴리스 (ARM64) | Jetson · OpenCR 펌웨어 빌드 |
| OpenCR 코어 | 1.5.3 | [ROBOTIS-GIT/OpenCR](https://github.com/ROBOTIS-GIT/OpenCR) 릴리스 | Jetson · OpenCR 펌웨어 빌드 (fqbn `OpenCR:OpenCR:OpenCR`) |
| Dynamixel2Arduino | 0.8.1 | 소스 (커밋 `cfbbaf7`) | Jetson · OpenCR 펌웨어 |
| arm-none-eabi-g++ | 13.2.1 | apt (`gcc-arm-none-eabi`) | Jetson · OpenCR 펌웨어 컴파일러 |
| opencr_ld | 커밋 `68ec75d` | 소스 빌드 (OpenCR 저장소) | Jetson · OpenCR 펌웨어 업로드 |
| pyzbar | 0.1.9 (apt 후보값, Jetson 설치 후 `apt policy`로 확인) | apt `python3-pyzbar` (`libzbar0t64` 함께 설치) | Jetson · 바코드 판독 (nz2_perception) |
| pytest | 7.4.4 | apt `python3-pytest` | 개발 PC · 코어 단위 테스트 (nz2_signal, nz2_mission) |
| dynamixel-sdk (pip) | `[확인 필요]` | pip (`tools/arm_motor_range_check/requirements.txt`) | 개발 PC · 팔 가동범위 측정 도구 |
| pyserial | `[확인 필요]` | pip (`tools/arm_motor_range_check/requirements.txt`) | 개발 PC · 팔 가동범위 측정 도구 |

## 다른 기기에서 실행하기

이 저장소는 누구나 다른 기기에서 `git clone`한 뒤 같은 환경으로 빌드하고 테스트할 수 있게 유지한다.

1. 위 "환경과 버전"의 기준 환경과 버전을 맞춘다. Jetson(Ubuntu 24.04 arm64)용 Lyrical은 공개 ROS 저장소에 없어서 [NVIDIA Isaac ROS 빌드팜 저장소](https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html)로 설치한다.
2. 저장소를 받아 빌드한다. `<배포판>`은 위 표의 ROS 2 배포판 이름이다.

```bash
git clone https://github.com/KANT-PA1-Project/lv2-nz2-project2.git
cd lv2-nz2-project2/ros2_ws
source /opt/ros/<배포판>/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## 버전 기록 규칙

- 새 소프트웨어나 라이브러리를 쓰거나 버전을 바꾸면, 같은 PR에서 이 README의 "환경과 버전" 표를 고친다.
- ROS 패키지 의존성은 각 패키지의 `package.xml`에 선언해 `rosdep`으로 설치되게 한다.
- zip, pdf 파일은 저장소에 올리지 않는다.

## 프로젝트 2 제출 안내

> 스켈레톤(2026-11-17). `TODO`는 채워야 할 곳이다. 하드웨어·**환경과 버전 표(단일 출처)**는 위 절을 따르며 여기서 복제하지 않는다.

### 1. 프로젝트 목표
출발 구역에서 바코드(Code 128)와 신호등 12회 점멸을 관측해 R·Y·G 횟수를 기억하고, 바코드 정렬 규칙으로 L1~L3에 그룹을 배정한 뒤 퍽 12개로 주사위 눈을 만들고, 새 관측으로 재검증하고 출발 구역으로 복귀한다. 규칙 원문: `mission.pdf`, 문제 조건: [`problem/problem.md`](problem/problem.md).

### 2. 사용 장비·소프트웨어 버전
- 하드웨어·버전: 위 "하드웨어 구성", "환경과 버전" 표 참조.
- 펌웨어(현재 저장소 기준): 스월브 `r12/firmware/opencr_swerve_left`·`opencr_swerve_right` v0.2.2, 팔 `r12/experiments/jetson_real/firmware/nz2_arm_v2_j2diag` FW 2.0 / limits_rev 8 (`d63ebcd`). 보드의 `I`·`?` 회신으로 실제 탑재 버전을 확인해 기록한다.
- 공식 경기에 실제 사용한 구성(카메라 대수·라이다 유무·펌웨어 커밋): TODO

### 3. 설치
위 "다른 기기에서 실행하기" 절차를 따른다. 로봇 구동 소스는 `r12/`에 있으므로 아래를 추가로 한다.

```bash
git lfs install && git lfs pull                 # 모델·CAD (pointer만 받은 상태로 실행 금지)
python3 r12/tools/verify_model_manifest.py
# r12 ROS 패키지 (스월브 제어·팔 MoveIt 설정)
cd r12 && source /opt/ros/lyrical/setup.bash
colcon --log-base reports/local/colcon_logs build \
  --base-paths assets/vendor/R12_NO_USB_URDF_USD ros2 \
  --build-base reports/local/ros_build --install-base reports/local/ros_install --symlink-install
# 팔 브리지 (COLCON_IGNORE 폴더라 경로를 직접 지정)
cd experiments/jetson_real/ros_ws && colcon build --base-paths src --executor sequential --parallel-workers 1
# 펌웨어 빌드 (업로드는 사람이 벤치에서)
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR r12/firmware/opencr_swerve_left
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR r12/firmware/opencr_swerve_right
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR r12/experiments/jetson_real/firmware/nz2_arm_v2_j2diag
```

`r12/assets/vendor/`의 원본 CAD ZIP은 Git에 없다(배포 위치 TODO). Jetson 카메라·인식 런타임은 `r12/experiments/jetson_real/jetson/runtime_env.sh`가 `/home/nz2/...` 고정 경로를 쓰므로 배포 경로를 맞춘다. 추가 단계: TODO (Jetson 컨테이너·TensorRT 엔진 재생성 `jetson/engine_build/`)

### 4. 실행 순서
| 단계 | 장치 | 명령 | 완료 확인 |
|---|---|---|---|
| 1a. 스월브 하위 제어 | Jetson | `ros2 launch r12_swerve_control r12_swerve.launch.py` | 브리지가 `swerve_left`·`swerve_right`를 모두 찾고 상태 `en=1` |
| 1b. 팔 브리지 | Jetson | `ros2 launch nz2_arm_bridge arm_bridge.launch.py` (기본 읽기 전용, 동작은 `allow_motion` 명시 시만) | `/arm/status`에 `fw=2.0 limits_rev=8 ok=6/6` |
| 2. 인식 기동 | Jetson | TODO (`nz2_perception`, `nz2_signal`) | TODO |
| 3. 주행 기동 | Jetson | TODO (`nz2_swerve_nav`) | TODO |
| 4. 미션 FSM | Jetson | TODO (`nz2_mission` `mission.launch.py`) | TODO |
| 5. 시작 명령 | 관제 PC | TODO | TODO |

> 2~4단계에 `ros2_ws`의 `nz2_*` 패키지를 쓸지, `r12/` 쪽 코어(`r12/src/arm_practice/`)를 쓸지는 TODO(결정 필요). 토픽 이름공간이 `/r12/*`·`/arm/*`와 `/nz2`로 갈려 있다.

### 5. 경기 초기화
경기장·퍽 초기화 절차는 [`problem/problem.md` "초기화와 경기 운영"](problem/problem.md)을 따른다. 로봇 측 초기화: 상판 퍽 슬롯 8개와 그리퍼를 비우고, 팔을 접은 자세·토크 OFF로 출발 구역에 둔다. 팔 펌웨어는 부팅 시 잠금 상태라 사람이 `R` → `T 1`을 보내야 토크가 켜진다. 시작·종료 시 400×400 mm를 만족하는 접기 자세, 로그 디렉터리·체크포인트 정리: TODO

### 6. 정지 방법
- 스월브 비상정지: RC-100 아무 버튼 → `estop` 래치(구동 0, 조향 토크 유지). 물리 NC 스위치는 배선 방식만 확정, 현재 비활성(`ESTOP_PIN = -1`). 복구는 원인 확인 후 텔레옵 `R`(`/r12/swerve_clear_faults` → 양 보드 `E`). 상세: [r12/docs/OPENCR_FAULT_RECOVERY.md](r12/docs/OPENCR_FAULT_RECOVERY.md)
- 팔 비상정지: 시리얼 `S`(줄 중간에 와도 즉시) → 전 모터 목표 = 현재 위치, 토크·그리퍼 유지, 래치. 해제는 `R`, 토크는 `T 1`. 하드웨어 E-stop 핀은 미배선(`ESTOP_PIN = -1`)
- 명령 두절: 스월브 워치독 500 ms(구동 0), 팔 워치독 150 ms(자세 유지, A/G 수신 시 복귀). 한 스월브 보드 고장 시 브리지가 두 보드를 함께 정지(`stop_on_fault`)
- 펌웨어 비상정지는 전원 차단이 아닌 능동 제동이다. 소프트웨어 정지 토픽(`/state/estop`)·전원 차단 순서: TODO

### 7. 주요 파일 경로
| 용도 | 경로 |
|---|---|
| 문제 패키지 | `problem/` |
| 설계 | `docs/architecture.md` |
| 시험 결과 | `results/test-results.csv` |
| 관측 신호 | `results/signal.json` |
| 할당·검증 | `results/allocation.json` |
| 공식 경기 회차별 판정 | `results/official/round-1/`, `round-2/` |
| 재현 시험 | `results/reproduction.md` |
| 개인 기여 | `docs/contributions.md` |
| 미션 파라미터 | `ros2_ws/src/planning/nz2_mission/config/` |
| 스월브 제어·브리지 | `r12/ros2/src/r12_swerve_control/` |
| 팔 MoveIt 설정 | `r12/ros2/src/r12_arm_moveit_config/` |
| 스월브 펌웨어 | `r12/firmware/opencr_swerve_left/`, `opencr_swerve_right/` |
| 팔 펌웨어·브리지 | `r12/experiments/jetson_real/firmware/nz2_arm_v2_j2diag/`, `.../ros_ws/src/nz2_arm_bridge/` |
| 미션·인식 코어 (r12) | `r12/src/arm_practice/` |

### 8. 재현 시 주의사항
- 실물 시험은 바퀴 띄운 벤치 → 지면 순. 비상정지 담당을 둔다.
- TB3 배터리(3S, 연속 약 9A)·OpenCR 10A 퓨즈 공유 → 팔·구동 동시 고부하 금지.
- XM430 온도 70°C 소프트 중단.
- 기록 시 펌웨어 버전·배터리 전압을 함께 남긴다.
- `r12/experiments/`는 실행 사본이 아닌 보관본이다. 팔 펌웨어·브리지도 여기 있어 저장소 밖 고정 경로(`/home/nz2/...`, `/home/pa33/...`)가 남아 있다.
- 팔 펌웨어 한계각을 바꾸면 `FW_LIMITS_REV`를 올리고 브리지(`arm_v2_protocol.py`의 기대값)도 같이 고친다. 불일치하면 브리지가 연결을 거부한다.
- TODO: 현장 조명, 신호등 노출 설정 등 팀이 겪은 함정.

### 9. 공식 경기 결과 요약
| 회차 | 원점수 | 완주 | 판정표 | 영상 |
|---|---|---|---|---|
| 1 | TODO | TODO | [round-1](results/official/round-1/scorecard.md) | TODO |
| 2 | TODO | TODO | [round-2](results/official/round-2/scorecard.md) | TODO |

최종 반영 점수(높은 회차 전체 점수): TODO
