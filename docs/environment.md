# 재현 환경과 버전

다른 기기에서 clone한 뒤 같은 결과를 얻기 위한 기준 환경이다.
새 소프트웨어를 쓰거나 버전을 바꾸면 같은 PR에서 이 문서를 고친다.
[미정] 항목은 팀에서 정한 뒤 채운다.

## 기준 환경

| 항목 | 개발 PC | 로봇 (Jetson Orin Nano) |
|---|---|---|
| OS | [미정] | [미정] (JetPack 버전) |
| ROS 2 배포판 | [미정] | [미정] |

- 팀장 개발 PC는 Ubuntu 24.04.5 LTS + ROS 2 Lyrical이다. 매니퓰레이터(손목 롤) 패키지는 이 환경에서 작성했다.

## 소프트웨어·라이브러리

| 이름 | 버전 | 설치 방법 | 사용처 |
|---|---|---|---|
| librealsense | [미정] | [미정] | RealSense 드라이버 |
| realsense-ros | [미정] | [미정] | RealSense ROS 2 노드 |
| Isaac ROS Visual SLAM (cuVSLAM) | [미정] | [미정] | SLAM |
| open_manipulator | [미정] | [미정] | 매니퓰레이터 description·bringup |
| dynamixel_hardware_interface | [미정] | [미정] | ros2_control 하드웨어 인터페이스 |
| MoveIt 2 | [미정] | [미정] | 매니퓰레이터 경로 계획 |

## 펌웨어

| 장치 | 버전 |
|---|---|
| OpenCR 1.0 | [미정] |
| DYNAMIXEL XM430-W210 / W350 | [미정] |
| Intel RealSense | [미정] |
