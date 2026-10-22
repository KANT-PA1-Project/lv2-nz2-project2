# lv2-nz2-project2

KANT PA1 LV2 프로젝트2 팀 NZ2의 저장소다.

## 프로젝트 소개

국제올림피아드 모바일로보틱스 규칙을 바탕으로, 빨강·초록·파랑 퍽을 지정된 순서대로 선반에서 다른 선반으로 옮기는 모바일 매니퓰레이터를 만든다. 스월브 드라이브로 전방향 주행하고, 개조한 OpenMANIPULATOR-X로 퍽을 집어 옮긴다.

| 구성 | 내용 |
|---|---|
| 구동 | 스월브 4모듈 (XM430-W210 ×8 = 조향 4 + 구동 4, 터틀봇3 와플 바퀴) |
| 매니퓰레이터 | OpenMANIPULATOR-X 개조 (XM430-W350) — 링크 약 +100mm 연장, 손목 롤 관절 추가 |
| 센서 | Intel RealSense D435 깊이 카메라 (집게 상부, 대수 `[확인 필요]`), QR·바코드·AprilTag·신호등 색상 인식 |
| 연산·제어 | Jetson Orin Nano Developer Kit (비전·상위 제어), OpenCR 1.0 2대 (#1 스월브 8모터 / #2 로봇팔) |
| SLAM | cuVSLAM + RealSense, 위치 추정은 우선 AMCL |

## 대회 미션과 규정

- **대회**: 로봇 기능대회 모바일 로보틱스 부문 (국제올림피아드 모바일로보틱스 규칙 준용). 대회일 **2026-11-11**. 세부 규칙은 미공개라 아래 일부는 `[가정]`이다.
- **미션**: RGB 퍽을 지정된 순서에 맞춰 선반에서 다른 선반으로 옮긴다.
- **퍽**: 반지름 3cm·높이 6cm 원통 또는 3×3×6cm 직육면체. 세워져 있거나 누워 있을 수 있어 인식·파지 전략이 자세에 따라 달라진다.
- **선반 높이**: 80mm.
- **규정 (실격 조건)**:
  - 시작·종료 시 로봇이 **400×400mm 이내**에 들어와야 한다(높이 제한 없음).
  - 벽·선반에 **접촉하면 실격**이다. 주행·파지 경로에 여유 간격을 두고 설계한다.

## 하드웨어 구성

| 항목 | 내용 |
|---|---|
| 구동부 | 스월브 드라이브 4모듈. 모듈당 조향 1 + 구동 1, 총 **XM430-W210 ×8**. 바퀴는 터틀봇3 와플 바퀴 |
| 매니퓰레이터 | OpenMANIPULATOR-X 개조. 구동 모터 **XM430-W350**, 링크 약 +100mm 연장, 손목 롤 모터 추가. 차체 중앙 상판에 장착하고 카메라는 집게 상부에 둔다 |
| 제어기 | OpenCR 1.0 2대. **#1** = 스월브 8모터, **#2** = 로봇팔. 상위 컴퓨터는 Jetson Orin Nano |
| 센서 | Intel RealSense D435 깊이 카메라 (대수 `[확인 필요]`), QR·바코드·AprilTag·신호등 색상 인식 |
| 전원 | 12V SMPS 약 30A, OpenCR 12V 5A |
| 기구 | 차체 약 400×400mm, 팔 제외 높이 267mm, 목표 총중량 5kg 이내. 2020 프로파일 450mm 설치. Fusion 360 설계 + Bambu Lab 3D 프린팅 |

**아직 정해지지 않은 항목**

- 스월브 역기구학 계산 위치 (OpenCR / Jetson)
- 적재통(퍽 임시 보관) 위치·형식
- 로봇 전체 높이 (팔 포함)

## 소프트웨어 아키텍처

상위·하위 2단으로 나눈다. 상위(Jetson)는 인지·판단·경로 계획을, 하위(OpenCR 2대)는 실시간 모터 제어를 맡는다.

```
[Jetson Orin Nano] ROS 2
  ├─ 인지     RealSense D435 → 깊이·영상, AprilTag·바코드·신호등 색상 인식
  ├─ 주행     Nav2, cuVSLAM, 위치 추정은 우선 AMCL
  ├─ 팔 계획  MoveIt 2 → 팔 경로 계획, 퍽 파지
  └─ 미션 FSM 퍽 순서 시퀀스, 실격 회피 로직
        │  (Jetson ↔ OpenCR 브리지)   [통신 방식 확인 필요]
        ├──────────────┬──────────────┐
  [OpenCR #1]                    [OpenCR #2]
  스월브 8모터 제어                로봇팔·그리퍼 제어
  (조향 4 + 구동 4)                (XM430-W350 + 손목 롤)
```

- **주행 제어**: 스월브 역기구학으로 각 모듈의 조향각·구동속도를 계산한다. 계산을 OpenCR에서 할지 Jetson에서 할지는 미정이다.
- **팔 제어**: OpenCR #2가 다이나믹셀 위치·전류 제어를 담당하고, 상위에서 MoveIt 2로 경로를 계획한다.
- **인지**: RealSense D435로 퍽 위치·자세(세움/누움)를 추정하고, AprilTag·바코드·신호등 색상으로 과제 순서와 위치를 판별한다.
- **시뮬레이션**: Isaac Sim으로 경기장·퍽을 재현하고, 도메인 랜덤화로 sim-to-real 간극을 줄인다. 실제 적용 여부는 `[확인 필요]`.
- **ROS 2 패키지 구성**: Jetson 쪽 코드는 `ros2_ws/src/` 아래 역할별 레이어(interface·perception·planning·control)로 나눈다. 자세한 구분은 아래 "디렉터리 구조"를 본다.
- 각 패키지 버전은 아래 "환경과 버전" 표를 기준으로 한다.

## 팀 NZ2

| 역할 | 이름 | GitHub |
|---|---|---|
| 팀장 | 천경호 | [@pizzaafterhangover](https://github.com/pizzaafterhangover) |
| 팀원 | 최형준 | [@chj1319](https://github.com/chj1319) |
| 팀원 | 정수용 | [@affluentmind12](https://github.com/affluentmind12) |
| 팀원 | 이성민(튜터) | [@SeongminJaden](https://github.com/SeongminJaden) |

## 디렉터리 구조

```
lv2-nz2-project2/
├── README.md
├── ros2_ws/                ROS 2 colcon 워크스페이스
│   └── src/
│       ├── interface/      메시지·서비스·액션 정의, Jetson↔OpenCR 브리지
│       ├── perception/     카메라·인식 (RealSense, AprilTag·바코드·신호등)
│       ├── planning/       미션 FSM, 주행·팔 경로 계획
│       └── control/        스월브·팔 제어 명령 생성
└── firmware/               OpenCR 1.0 펌웨어 소스코드
```

| 폴더 | 내용 |
|---|---|
| [ros2_ws/](ros2_ws/) | ROS 2 colcon 워크스페이스. `src/` 아래 4개 레이어 폴더는 그룹 디렉터리이며, 실제 패키지는 각 폴더 안에 `nz2_*` 접두사로 추가한다 (예: `src/perception/nz2_puck_detector`) |
| [firmware/](firmware/) | OpenCR 1.0 펌웨어 소스코드 |

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
| Isaac ROS | 5.0 | Docker ([isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) 태그 `v5.0-0`, isaac-ros-cli 2.6.0) | cuVSLAM |
| realsense2_camera | 4.57.7 | apt (NVIDIA Isaac ROS 저장소) | Jetson · RealSense 드라이버 |
| robot_state_publisher | 3.5.5 | apt (NVIDIA Isaac ROS 저장소) | Jetson · URDF → TF |
| xacro | 2.1.1 | apt (NVIDIA Isaac ROS 저장소) | Jetson · URDF |
| ros2_control | 6.9.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 하드웨어 제어 프레임워크 |
| ros2_controllers | 6.9.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 팔·그리퍼 컨트롤러 |
| dynamixel_hardware_interface | 1.5.1 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 다이나믹셀 하드웨어 인터페이스 |
| dynamixel_sdk | 4.0.3 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 다이나믹셀 통신 |
| navigation2 | 1.5.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 주행(Nav2), nav2_bringup 포함 |
| robot_localization | 3.10.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 설치됨, 현재 사용 계획 없음 |
| pointcloud_to_laserscan | 2.1.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 점구름 → 스캔 |
| twist_mux | 4.5.1 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 수동 조작 테스트 |
| vision_msgs | 4.2.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 인지 메시지 |
| control_msgs | 6.10.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 팔·그리퍼 액션 |
| cv_bridge | 4.1.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 영상 변환 |
| MoveIt 2 | 2.15.0 | apt (NVIDIA Isaac ROS 저장소) | Jetson · 팔 경로 계획 |
| colcon-common-extensions / rosdep / vcstool | 0.3.0 / 0.27.0 / 0.3.0 | apt | Jetson · 빌드 도구 |
| arduino-cli | 1.5.1 | GitHub 릴리스 (ARM64) | Jetson · OpenCR 펌웨어 빌드 |
| OpenCR 코어 | 1.5.3 | [ROBOTIS-GIT/OpenCR](https://github.com/ROBOTIS-GIT/OpenCR) 릴리스 | Jetson · OpenCR 펌웨어 빌드 (fqbn `ROBOTIS:OpenCR:OpenCR`) |
| Dynamixel2Arduino | 0.8.1 | 소스 (커밋 `cfbbaf7`) | Jetson · OpenCR 펌웨어 |
| arm-none-eabi-g++ | 13.2.1 | apt (`gcc-arm-none-eabi`) | Jetson · OpenCR 펌웨어 컴파일러 |
| opencr_ld | 커밋 `68ec75d` | 소스 빌드 (OpenCR 저장소) | Jetson · OpenCR 펌웨어 업로드 |

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
