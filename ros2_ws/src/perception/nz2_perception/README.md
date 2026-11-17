# nz2_perception

그리퍼 RealSense D435 1대로 퍽·신호등·바코드·AprilTag 를 측정한다. 판단(신호 해독·바코드 확정·퍽 상태표·놓기 검증)은
`nz2_signal`·`nz2_mission`(#23/#24)이 하고, 이 패키지는 측정값만 낸다. 노드 간 계약은 `docs/planning/mission_fsm_design.md` §4.0.

## 노드

| 노드 | 입력 | 출력 |
|---|---|---|
| yolo_detector (C++) | /gripper/color/image_raw | /perception/detections (vision_msgs/Detection2DArray), /perception/gripper/debug_image |
| yolo_detector.py (개발 PC 시험용) | 〃 | 〃 (ultralytics .pt) |
| tag_reader (C++) | 영상, camera_info | /nz2/tags (vision_msgs/Detection3DArray, AprilTag 36h11) |
| puck_registry.py | detections, tags, depth, tf | /perception/puck_tracks (PuckArray, frame map, tag_id = 퍽 ID 0~11), /nz2/puck_registry, /perception/puck_debug_image |
| traffic_light_filter (C++) | detections, 영상 | /perception/traffic_light (TrafficLight: 프레임별 raw_state, visible) |
| barcode_reader (C++) | 영상 | /perception/barcodes (BarcodeArray, 3회 연속 같은 문자열 = confidence 1.0) |
| perception_master (C++) | 위 결과 | /perception/state (PerceptionState), 서비스 /perception/set_mode (SetPerceptionMode) |

- YOLO 백엔드는 `engine_path` 확장자로 정해진다: `.engine` = TensorRT FP16(GPU), `.onnx` = OpenCV DNN(CPU).
  Ubuntu 24.04 기본 OpenCV 4.6 은 YOLOv8 ONNX 를 실행하지 못한다 → 개발 PC 는 `yolo:=py` 또는 TensorRT.
- 클래스 9개: tower_light + {cyl,box}_{red,green}_{up,down} (config/perception.yaml `class_names`, 학습 순서와 같아야 함)
- 모델 파일은 커밋하지 않는다. 기본 위치 `~/nz2_models/current.engine`

## 빌드 (Jetson / 개발 PC)
```bash
sudo apt install libzbar-dev                       # 바코드(Code 128)
cd ros2_ws && source /opt/ros/<배포판>/setup.bash
colcon build --packages-select nz2_interfaces nz2_perception 2>&1 | grep -i -E "tensorrt|zbar|error"
source install/setup.bash
```
빌드 출력에 `TensorRT 백엔드 사용`이 나와야 GPU 추론이 된다 (CUDA·TensorRT 를 찾으면 자동).

## TensorRT 엔진 만들기 (엔진은 만든 기기 전용 — Jetson 엔진은 Jetson 에서)
```bash
# 개발 PC: ONNX (입력 크기 고정 384x640)
yolo export model=<best.pt> format=onnx imgsz=384,640 opset=12 simplify=True dynamic=False
# Jetson: 엔진
/usr/src/tensorrt/bin/trtexec --onnx=puck.onnx --fp16 --saveEngine=$HOME/nz2_models/puck_jetson_fp16.engine
ln -sf puck_jetson_fp16.engine $HOME/nz2_models/current.engine
```
`scripts/build_engine.sh`(ultralytics 로 .pt → .engine), `scripts/bench.py`(속도·결과 확인)도 쓸 수 있다.

## 실행
```bash
ros2 launch nz2_perception perception.launch.py use_camera:=true fps:=30 exposure:=<값>
ros2 launch nz2_perception perception.launch.py use_camera:=true yolo:=py model_path:=<best.pt>   # 개발 PC
```
- `fps` 는 카메라, YOLO 추론 상한은 perception.yaml `/yolo_detector process_hz`
- 대회에서는 `exposure` 를 반드시 지정 (자동 노출이면 켜진 램프가 포화)

## 노드 하나씩
```bash
P=$(ros2 pkg prefix nz2_perception)/share/nz2_perception/config/perception.yaml
ros2 run nz2_perception yolo_detector --ros-args -r __node:=yolo_detector --params-file $P
ros2 run nz2_perception tag_reader --ros-args -r __node:=tag_reader --params-file $P
ros2 run nz2_perception puck_registry.py --ros-args -r __node:=puck_registry --params-file $P
ros2 run nz2_perception traffic_light_filter --ros-args -r __node:=traffic_light_filter --params-file $P
ros2 run nz2_perception barcode_reader --ros-args -r __node:=barcode_reader --params-file $P
ros2 run nz2_perception perception_master --ros-args -r __node:=perception_master --params-file $P
```
`-r __node:=<이름>` 은 yaml 의 `/yolo_detector:` 같은 이름과 같아야 파라미터가 들어간다.

## 확인
```bash
ros2 topic hz /perception/detections
ros2 topic echo /perception/puck_tracks --field header.frame_id    # map 이어야 nz2_mission 이 받음 (SLAM 필요)
ros2 run rqt_image_view rqt_image_view /perception/puck_debug_image
```

## 주의
- `/perception/puck_tracks` 는 tf map 이 없으면 odom(→ 카메라) 기준으로 나간다. nz2_mission 은 map 이 아니면 버린다.
- 퍽 좌표가 map 으로 나오려면 tf 가 모두 이어져야 한다: map→odom(SLAM), odom→base_link(오도메트리), base_link→팔→카메라(URDF + /joint_states).
