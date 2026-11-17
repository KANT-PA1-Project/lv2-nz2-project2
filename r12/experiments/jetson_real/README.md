# PA33 실물 통합 작업 정리

2026-11-16 기준, feat/barcode-signal의 3bfc804 이후 추가할 1~5번 작업이다.
원본은 /home/pa33/git/p2_perception 및 Jetson에 있다.
UPLOAD_MANIFEST.json에 원본 경로와 복사본 SHA256을 기록했다.

## 포함 범위

1. 신호등·바코드 후속 수정은 ../barcode_signal/에 반영했다. 수정 29개와 B28~B34 새 설정 7개다.
2. firmware/에는 실제 적용했던 nz2_arm_v2_j2diag 소스와 j2 진단·시험 도구가 있다. FW 2.0, limits_rev 8, diag_rev 2 기준이다.
3. ros_ws/src/nz2_arm_bridge/에는 팔 시리얼 프로토콜·ROS 브리지·설정·launch가 있다. jetson/에는 같은 면 태그 보정 및 팔 시험 도구가 있다.
4. ros_ws/src/perception/에는 YOLO·태그·퍽 등록·신호등·바코드 인지 소스가 있다. nz2_interfaces는 이 실물 워크스페이스에서 사용한 인터페이스 사본이다.
5. jetson/과 optimize/에는 카메라·인지 실행 설정, PC 영상 조회, ONNX 변환 코드가 있다. jetson/engine_build/의 4개 파일은 Jetson에서 가져온 엔진 재생성·검증 코드와 명세다.

## 빌드 및 배포 시 확인

저장소 전체에서 colcon이 패키지 사본을 중복 발견하지 않도록 이 폴더에 COLCON_IGNORE를 뒀다.
이 사본을 별도 워크스페이스로 빌드할 때는 패키지 경로를 직접 지정한다.

```bash
cd /path/to/lv2-nz2-project2/r12/experiments/jetson_real/ros_ws
colcon build --base-paths src --executor sequential --parallel-workers 1
```

Jetson 배포 경로는 기존 설정에 남아 있다. runtime_env.sh는 /home/nz2/nz2_camera_ws와 /home/nz2/p2_runtime_ws를 사용한다.
카메라 launch와 팔 보정 도구는 /home/nz2/nz2를 사용하며, 팔 진단 도구·프로토콜은 /home/nz2/nz2/arm_j2_diag에 배포했던 구성이다.
YOLO 엔진 경로는 /home/nz2/models/perception_6class_20261115/yolo_fp16.engine이다.
barcode_signal.yaml 또는 runtime_nodes.yaml의 perception_root는 ../barcode_signal 소스를 배포한 실제 경로로 지정해야 한다.
이 정리는 소스 보관용이다. 전체 복사본의 colcon 빌드와 새 경로에서의 실물 실행은 이번 정리 과정에서 검증하지 않았다.

## 실험 상태

그리퍼 카메라·YOLO·태그·퍽 등록의 실제 메시지 수신과 퍽 인식을 확인했다.
single_puck_preview.launch.py는 카메라·YOLO·태그·등록을 실행한다. 신호등·바코드는 별도 barcode_signal.launch.py에 있다.
팔은 신경망으로 계산한 5mm 접근을 시험했다. 실제 파지·리프트·운반·배치는 미실행이다.
홈 이동은 추종오차로 중단됐고 자세 유지 보상은 시험 도구에 있다. 일반 팔 ROS 브리지에는 아직 통합하지 않았다.
카메라 보정값은 작은 시험 범위의 후보이며 런타임에는 적용하지 않았다.
마지막 사용자 중단 지시에 따라 팔 토크 OFF 응답을 확인했다. 당시 모터 통신은 0/6이었다.

## 커밋 범위

현재 정리한 소스 105개와 이 폴더의 README·해시 명세·ignore 파일을 커밋한다.
추가 모델 묶음과 최종 URDF는 후속 업로드 범위다.
