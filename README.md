# lv2-nz2-project2

KANT PA1 LV2 프로젝트2 팀 NZ2의 저장소다.

## 프로젝트 소개

국제올림피아드 모바일로보틱스 규칙을 바탕으로, 빨강·초록·파랑 퍽을 지정된 순서대로 선반에서 다른 선반으로 옮기는 모바일 매니퓰레이터를 만든다.

| 구성 | 내용 |
|---|---|
| 구동 | 스월브 4모듈 (구동 XM430-W210, 조향 XM430-W350) |
| 매니퓰레이터 | OpenMANIPULATOR-X 개조 — 링크 연장, 손목 롤 관절 추가 |
| 센서 | Intel RealSense 깊이 카메라, 그리퍼 카메라, 거리 센서 |
| 연산·제어 | Jetson Orin Nano Developer Kit (비전·상위 제어), OpenCR 1.0 (모터 제어) |
| SLAM | cuVSLAM + RealSense |

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
├── ros2_ws/        ROS 2 colcon 워크스페이스
│   └── src/        ROS 2 패키지
└── firmware/       OpenCR 1.0 펌웨어 소스코드
```

| 폴더 | 내용 |
|---|---|
| [ros2_ws/](ros2_ws/) | ROS 2 colcon 워크스페이스. 패키지는 `ros2_ws/src/` 아래에 둔다 |
| [firmware/](firmware/) | OpenCR 1.0 펌웨어 소스코드 |

## 환경과 버전

다른 기기에서 clone한 뒤 같은 결과를 얻기 위한 기준 환경이다. [미정] 항목은 팀에서 정한 뒤 채운다.

| 항목 | 개발 PC | 로봇 (Jetson Orin Nano Developer Kit) |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS | [미정] (JetPack 버전) |
| ROS 2 배포판 | Lyrical (ros-base 0.13.0) | [미정] |

사용하는 소프트웨어·라이브러리

| 이름 | 버전 | 설치 방법 | 사용처 |
|---|---|---|---|
| Isaac Sim | 6.1.0 | 소스 빌드 ([isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim) 태그 `v6.1.0`) | 시뮬레이션 |
| Isaac ROS | 5.0 | Docker ([isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) 태그 `v5.0-0`, isaac-ros-cli 2.6.0) | cuVSLAM |

## 다른 기기에서 실행하기

이 저장소는 누구나 다른 기기에서 `git clone`한 뒤 같은 환경으로 빌드하고 테스트할 수 있게 유지한다.

1. 위 "환경과 버전"의 기준 환경과 버전을 맞춘다.
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
