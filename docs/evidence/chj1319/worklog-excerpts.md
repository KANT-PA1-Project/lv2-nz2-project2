# 개인 기여 근거용 기존 작업 기록 발췌

원본은 pa33의 p2_perception/수정.md다. 기존 항목을 날짜·본문 그대로 발췌했다.
로컬 소스 경로와 과거 장비 상태는 당시 기록이며 현재 장비 상태는 별도 조회가 필요하다.

## 2026-10-28 · AprilTag 판독 노드(tag_reader) 추가 → /nz2/tags

**이유**: 퍽 ID(0~11) 등록·재검증(8점), 장소 태그(12~14) 기준 주사위 판정에 태그 판독이 필요한데 perception 패키지에 없었음. 클라우드에만 있던 팀 정렬판(`nz2_perception`)의 tag_reader 를 가져옴. 출력 토픽은 팀 레포 규약 `/nz2/tags` 로 결정 (choi)

| 파일 | 수정 내용 |
|---|---|
| `ros_ws/src/perception/src/tag_reader_node.cpp` | **신규**. OpenCV ArUco(tag36h11) 검출 → solvePnP(IPPE_SQUARE) 자세 → `/nz2/tags` (vision_msgs/Detection3DArray: `id`, `results[0].pose`, `bbox.size.x/y`=태그 크기 m, `bbox.size.z`=영상 한 변 px). 그리퍼 토픽 기본, RELIABLE, QR 제거, ID 0~`max_id`(14) 밖은 버림, `output_frame` 지정 시 tf 변환, `max_image_age_s`=0 이면 지연 검사 끔(rosbag 용), ArUco 없는 OpenCV 면 시작 실패 |
| `ros_ws/src/perception/src/tag_detector.cpp`, `include/nz2_perception/tag_detector.hpp` | **신규**(팀 정렬판에서 복사). OpenCV 4.7+ / contrib 둘 다 지원, 코너 서브픽셀 보정 |
| `ros_ws/src/perception/CMakeLists.txt` | `tag_reader` 실행 파일 추가 + 설치 목록 |
| `ros_ws/src/perception/config/perception.yaml` | `/tag_reader` 섹션 (16 mm / 장소 12~14 48 mm, max_id 14, output_frame "") |
| `ros_ws/src/perception/launch/perception.launch.py` | `tag_reader` 노드 추가 |
| `ros_ws/README.md` | 노드↔토픽 표에 tag_reader |

**확인 방법 (클라우드 ROS 2)**: 합성 영상(848x480, f=617)에 태그 3개 — ID 5(16 mm, 0.30 m), ID 11(16 mm, 0.45 m, 한 변 22 px), ID 13(48 mm, 0.60 m) → 3개 모두 ID 정답, 거리 오차 3~9 mm

**실행** (`~/git/p2_perception/ros_ws`)
```bash
colcon build --packages-select perception && source install/setup.bash
ros2 launch perception perception.launch.py use_camera:=true yolo:=py model_path:=...   # tag_reader 포함
ros2 topic echo /nz2/tags
```

**남은 일**: 퍽 박스(YOLO)↔태그 ID 묶기(퍽 등록), 장소 태그 기준 격자(B-2), `output_frame: base_link`(손-눈 캘리브레이션 후)

---


## 2026-10-29 · 합성 v4: 대회 맵 장면(`scene: field`) + 직육면체 태그 규칙(위아래 확정·옆면 랜덤) + 누운 원통 제외

**이유**
- 실물 퍽 규칙 확인: 직육면체는 **윗·아랫면 태그 확정, 옆면 4장은 랜덤**. 기존 `mix` 는 옆면 4장을 한꺼번에 켜고 꺼서 실물과 다름
- 선반 1개 장면 대신 **대회 맵(미션 v5 주사위 맵)** 에서 그리퍼 시점으로 찍은 합성이 필요 (전면 카메라 삭제, 실제 관측 거리·배경·박스 배치 반영)
- 미션상 원통은 항상 서 있음 → 누운 원통 합성 불필요

**파일별 수정** (`training/`)

| 파일 | 수정 내용 |
|---|---|
| `synth/field_layout.py` | **신규** (순수 파이썬, Isaac 없이 실행·테스트). 맵 고정물: 외곽 4.404×6.300 m + 내부 벽 16개(p2 그림에서 읽음 [가정 ±30 mm]), 벽 높이 0.30 m [가정], 시작 박스 6·L1·L3(205), 중앙 410×4 (받침대 중심 (2.202,3.495), L2 격자 (2.202,3.700)), 장소 태그 12~14 (48 mm, 높이 35 mm, L1 서쪽면·L2 받침대 북면·L3 남쪽면). `sample_state()` = 시작 박스 0~2개(X±40 mm) + 장소 주사위 눈 0~6 (p4 배치, 격자 60 mm, ±8 mm, 일부만 놓인 상태), 퍽끼리 겹침 검사(캡슐). `sample_camera()` = approach/overhead/search, 차체 400×400 이 벽·박스와 안 겹치는 위치만, 벽 가림 검사 |
| `synth/gen_synth_pucks.py` L29~37 | v4 설명 |
| 〃 L71~107 | 인자: `--box_faces tb_rand`, `--box_side_prob`, `--scene shelf/field`, `--field_view`, `--cyl_down_ratio`, `--box_down_prob`, `--per_combo`, `--wall_h` |
| 〃 L41~42, L111 | `field_layout` import (Isaac 시작 전) |
| 〃 L268~319 | `tag_id_of()`, `tag_png_by_id()` / field 장면 생성(바닥·벽·3030 기둥·박스 12·장소 태그 3) / shelf 장면은 기존 그대로 |
| 〃 L358, L367~368 | spec 모드 ID 파싱 공용화 / random 모드 퍽 마커 풀에서 ID 12~14 제외 |
| 〃 L390 | `faces_for()` 에 `tb_rand` (6면 판 생성) |
| 〃 L404~451 | 퍽 생성: 조합마다 `--per_combo` 개, `cyl_down_ratio<=0` 이면 누운 원통 안 만듦, `POOL`·`CYL_DOWN` 기록, 로그 문구 |
| 〃 L486~489 | shelf 모드에서도 누운 원통 비율 적용 (`cyl_down_ratio`, 기본 1.0 = 예전 동작) |
| 〃 L518~523 | field 박스 색(회색/나무색), 기둥 색 랜덤 |
| 〃 L560~561 | 태그 가시성: `tb_rand` 면 옆면을 면마다 `box_side_prob` 로 |
| 〃 L723~795 | `place_field()`, `place_field_confusers()`, 메인 루프 field 분기(카메라·조명·`field_meta.jsonl`) |
| `pipeline.py` L226~239 | `cmd_synth` 가 `--box_side_prob`, `--scene`, `--cyl_down_ratio`, `--per_combo`, (field) `--field_view`, `--box_down_prob`, `--wall_h` 전달 |
| `config.yaml` L43~47 | `markers.box_faces: mix → tb_rand`, `box_side_prob: 0.5` 추가 |
| `config.yaml` L70~83 | `synth.cyl_down_ratio: 0`, `per_combo: 2`, `box_down_prob: 0.5`, `wall_h`(주석) / **jobs 교체**: field_approach 800, field_overhead 600, field_search 300, hand(선반) 400, val_field 150, val_hand 100. 이전 job 목록은 주석으로 보존 (s3·s4 는 이미 생성돼 영향 없음) |
| `docs/PIPELINE.md` | 합성 생성기 절 v4 설명, config 표에 새 항목 |

**확인 방법 (클라우드, Isaac 대신 usd-core + 가짜 Isaac 모듈)**
- `field_layout`: 박스 12개 모두 벽과 안 겹침, 3000 장면 퍽 겹침·박스 밖 0건, 카메라 뽑기 성공 99.9% (approach/overhead/search 고르게), 위에서 본 배치 그림이 미션 p2 맵과 일치
- 생성기 field 40프레임: 모든 프레임에 퍽이 화면 안에 1개 이상 (approach 평균 6.0, overhead 3.5, search 4.8), 퍽 높이 서있음 0.100 / 누움 0.085 m, 누운 원통 0, 직육면체 위아래 태그 6/6 보임·옆면 7/12, 장소 태그 위치 L1 (3.567,0.950,0.035)·L2 (2.202,3.905,0.035)·L3 (3.670,5.251,0.035)
- 기존 shelf 모드(d435·hand, mix·all·spec) 회귀 실행 정상
- **실제 Isaac Sim 렌더링은 아직 안 돌림** → 아래 시험 생성으로 확인

**실행해야 할 것** (`~/git/p2_perception/training`, yolo_env)
```bash
python synth/field_layout.py                                  # 배치 계산만 확인 (Isaac 불필요)
python pipeline.py synth --version s5_test --scale 0.02       # 시험 생성 (job 당 2~16장)
#  → data/synth/raw/s5_test/field_*/rgb_*.png 눈으로 확인 (벽·박스·퍽·태그), _check 모아보기 확인
python pipeline.py synth --version s5                         # 본 생성
#  → config.yaml train.synth_versions 에 s5 추가 후 학습
```

**남은 일**
- 벽 높이·색, 박스 색, 바닥 색 실물 확인 → `synth.wall_h` 등 반영
- 벽 좌표는 그림에서 읽은 값 (±30 mm). 현장 실측값 받으면 `field_layout.WALLS` 갱신
- 로봇 차체·팔이 카메라에 비치는 부분은 미반영
- 신호등 근처 장면(시작 구역)은 실사로 보강

---


## 2026-10-30 · train:val:test = 8:1:1 로 나누기 (실사·합성) + resplit 명령

**이유**: 지금까지 train/val 2개뿐(실사 80:20, 합성 89:11)이라 val 이 best.pt 고르는 데 쓰여 점수가 낙관적 → 학습·선택 모두에 안 쓰는 test 세트로 버전 비교 (choi)

| 파일 | 수정 내용 |
|---|---|
| `training/pipeline.py` | `_real_block_split()` 추가: 촬영 폴더별 10장 묶음 중 5번째마다 떼어 두고(예전 val 과 같은 자리), 떼어 둔 묶음을 번갈아 val/test → 80/10/10. `cmd_ingest` 가 이 함수 사용 |
| | `cmd_resplit()` / `resplit` 명령 추가: 이미 만든 데이터를 다시 나눔. **실사 train 은 그대로**(v7 이 학습한 사진이 val/test 로 새지 않음), 지금 val 덩어리만 번갈아 val/test. 합성은 train+val 을 모아 파일 이름 crc32 해시로 0.8/0.1/0.1 (같은 이름은 항상 같은 쪽). `--dry`, `--no-real`, `--synth s7 ...` |
| | `cmd_synth`: 변환 끝나면 `synth.split` 이 있으면 자동 resplit |
| | `_sets()`: 평가 세트를 synth_val / real_val / **synth_test / real_test** 로 (예전엔 test 에 합성·실사 섞임). 비교표 열 `synth_val, real_val, synth_test, real_test mAP50, real_test_R`. `status` 에 합성 test 장수 |
| `training/config.yaml` | `real.test_split: true`, `synth.split: [0.8, 0.1, 0.1]` |

best.pt 고르는 val 은 그대로 (실사 val 전부 + 합성 val 150장). 원본·검수 폴더는 안 건드림 (data/*/yolo 안에서 images/labels 만 옮김).

**확인 방법**: 클라우드 가짜 데이터 — 실사 폴더 300장(예전 방식 268/63/0 중) → train 240 그대로, val 30 / test 30, ingest 함수 결과와 일치 / 23장 폴더 → 20/3/0 / 8장 → 8/0/0. 합성 2350장 → 1887/224/239. 두 번 실행해도 0장 이동(같은 결과)

**실행해야 할 것** (노트북, yolo_env):
```bash
cd ~/git/p2_perception/training && source ~/yolo_env/bin/activate
python pipeline.py resplit --dry      # 몇 장 옮기는지
python pipeline.py resplit            # 실사 1025_tower·1026_tower + 합성 s7 을 8:1:1 로
python pipeline.py status             # train/val/test 장수 확인
python pipeline.py eval --name v7     # v7 을 새 val/test 로 다시 평가 (비교 기준)
python pipeline.py train --dry && python pipeline.py train --name v8
```
→ 비교표에서 v8 의 `real_test_mAP50` 이 v7 보다 높으면 `python pipeline.py release --name v8`

**남은 일**: 대회장에서 새로 찍는 세션은 통째로 `real.test_sessions` 에 넣어 "처음 보는 장면" 시험용으로 쓰는 것 권장

---


## 2026-11-04 · 담당 변경: 팀원 모델 2개(YOLO, CenterPose) 최적화 시작 — YOLO ONNX 내보내기 (optimize/yolo_export.py, 새 파일)

**이유**: 담당이 바뀜 — 팀원이 만든 `drive-download-20261104T022039Z-1-001/` 의 YOLO(`yolo_best.pt`)와 CenterPose(`centerpose_best.pt`, `centerpose_last.pt`)를 Jetson 에서 빠르게, **메모리 1 GB 이하**로 돌게 최적화. 기존 p2 인지 노드는 안 쓸 예정. 첫 단계 = ONNX
- 모델 확인(torch 없이 체크포인트 구조 읽음): YOLO = **YOLOv8n-P2**(stride 4 출력 추가), 클래스 6(red/green cylinder·cuboid, box, apriltag), 학습 imgsz 800, 40 epoch, fitness 0.856 (팀원 레포 turtlebot-swerve-cuvslam, train-v11-domain). CenterPose = **DLA-34** + dla_up/ida_up + 헤드 7개(hm 1, wh 2, hps 16, reg 2, hm_hp 8, hp_offset 2, scale 3), 약 1,890만 파라미터, 20 epoch, 메모 "Custom crop harness ... normalized metric depth ... symmetry-aware corners", 퍽 30×30×60 mm·상자 205/410 mm. last.pt 는 옵티마이저 포함(225 MB) → 추론은 best 만
- CenterPose ONNX 는 팀원의 모델 정의·입력 크기·crop 코드가 있어야 함 (체크포인트엔 가중치만)

| 파일 | 내용 |
|---|---|
| `optimize/yolo_export.py` (새) | yolo_best.pt → ONNX 4가지 크기(800x800 학습 크기, 448x800 카메라 비율, 640x640, 384x640), opset 17, simplify, 고정 크기, FP32(FP16 은 TensorRT). 크기마다 ① 무작위 입력으로 PyTorch(Conv+BN 합친 모델) vs ONNX Runtime 원시 출력 최대 오차 ② 실제 사진 8장을 같은 전처리(LetterBox 고정 크기, 회색 114)로 넣어 원시 오차 + NMS 후 클래스·점수 비교. 결과 optimize/onnx/yolo_HxW.onnx + yolo_export_report.json |

**확인 방법**: 문법만 (클라우드에 torch 설치 불가). 실행은 노트북

**실행해야 할 것** (개발 PC, yolo_env): `pip install onnx onnxslim onnxruntime` → `python optimize/yolo_export.py`
**남은 일**: TensorRT FP16 엔진(메모리 줄이는 빌드 옵션) 크기별 메모리·속도, mAP(팀원 val 데이터 필요). CenterPose 코드 받기

---


## 2026-11-15 (42) · 6클래스 YOLO·Git 원본 CenterPose 재변환 및 Jetson 엔진 검증

**이유**

사용자가 Downloads/drive-download-20261104T022039Z-1-001 모델로 인지 엔진 생성·Jetson 배포를 요청했다. YOLO 클래스는 red_cylinder, red_cuboid, green_cylinder, green_cuboid, box, apriltag 순서다.

**파일별 생성·수정**

- `/tmp/nz2-centerpose-verified/rebuild.py` 19행에 지정된 가중치 폴더, 68행에 Git 원본 load_model, 73행에 CenterPose ONNX export, 158행에 YOLO 800x800 export를 기록했다. 원본 모델 정의는 turtlebot-swerve-cuvslam의 feature/nvblox-swerve-navigation 커밋 97e163f7ea9fd05f6a8fdd028b431157751daba8에서 추출했다.
- `/tmp/nz2-centerpose-verified/validate_engine.py` 17행에 기존 jetson_pipeline.TrtModel 재사용, 91행에 복사 포함 지연 측정, 98행에 CP 위치 차이 1mm, 106행에 YOLO 박스 1px·점수 0.02·검출 개수 동일이라는 검토 기준을 추가했다. 이 수치는 이번 변환 비교를 위해 선택한 기준이다.
- `/tmp/nz2-centerpose-verified/build_engines.py` 19행에 FP32/FP16 순차 빌드, 31행에 trtexec 실행, 38행에 빌드 후 출력 검증을 추가했다.
- Jetson `nz2@10.2.11.63`의 `/home/nz2/models/perception_6class_20261115/`에 ONNX 2개, FP32/FP16 엔진 4개, 검증 입력·원본 출력, 검증 스크립트, 로그를 생성했다. 모델 파일은 바이너리이므로 줄 번호가 없다.
- 같은 Jetson 폴더의 `selected_models.json`에 YOLO FP16·CenterPose FP32 조합과 해시·입력 크기를 기록했다. 로컬 결과 사본은 `/tmp/nz2-centerpose-verified/jetson-results/`에 있다.
- `수정.md` 3행부터 이번 배포·검증 내역을 추가했다.

**확인 방법과 결과**

- Downloads의 YOLO SHA256은 e844ec62ac01bb42272ce4a05f2f39128e2d57306b9857d96ef855c10757c473, CenterPose는 3ed5fe5711a5d96298a829a491ab0b1cc33f642434a3e3dc0a166ba60b2d1ae4다. Jetson 전송 후 ONNX 해시를 확인했다.
- ONNX Runtime과 Git PyTorch를 비교했다. CP는 이미지 크롭 16개·48개 시점, YOLO는 이미지 3장에서 정합 검사를 통과했다. 기존 128x128/batch3/4출력 CP도 같은 검사에 통과했고 재구성 PyTorch와 Git 모델의 출력 차이는 0이었다.
- Jetson TensorRT 10.16.2.10에서 엔진 4개 빌드·추론이 성공했다. YOLO 입력은 [1,3,800,800], CP는 [3,3,128,128]이며 hm/hps/reg/scale을 출력한다.
- YOLO FP32/FP16 평균 추론은 23.18/10.40ms, CP FP32/FP16은 9.95/4.49ms다. 각 50회, CPU↔GPU 복사를 포함한 단독 엔진 측정이다.
- YOLO FP16은 3장 검출 19개를 유지했고 박스 최대 차이 0.173px, 점수 최대 차이 0.00964였다. CP FP16은 ONNX 대비 중심 최대 차이 1.14269mm라 이번 1mm 검토 기준을 초과했다. 따라서 기본 연결 후보로 CP FP32를 선택했다. CP FP32의 중심 최대 차이는 0.000519mm다.
- 선택한 두 엔진을 함께 로드해 기존 사진 1장을 30회 처리했다. 전체 처리 p50 37.04ms, 반복 루프 24.66FPS, 시스템 추가 사용량 약 410MiB였다. 샘플은 프레임당 검출 2개·퍽 후보 1개·유효 자세 0개였다. 이는 변환·실행 검증이며 실물 위치 정확도 검증은 남아 있다. 해당 사진의 K는 기존 검증 자료에서 가져왔다.
- 사용자 요청에 따라 I 식별 응답으로 현재 OpenCR 3대를 확인했다. ttyACM0=swerve_right 0.2.2, ttyACM1=swerve_left 0.2.2, ttyACM2=nz2_arm v2 handshake1/FW2.0/limits_rev8이다.

**실행해야 할 것**

Jetson에서 `/home/nz2/models/perception_6class_20261115/selected_models.json`의 엔진 경로를 6클래스 인지 런타임에 적용한다. 검증 재실행은 해당 디렉터리에서 `python3 -B validate_engine.py yolo fp16` 및 `python3 -B validate_engine.py centerpose fp32`로 한다.

**남은 일**

D435 USB3 연결·실제 CameraInfo와 왜곡 보정·노출 설정 확인, 실제 퍽의 검출·태그 ID·중심 위치·파지용 자세 검증, 최종 v99 TF·6클래스 ROS 계약 연결이 필요하다. Cartographer는 패키지 설치와 라이다 udev 4개까지 확인했으며 실물용 드라이버·Lua·launch 연결이 필요하다. 허브 포트 변경 후에는 현재 물리 경로 기반 라이다 udev를 다시 맞춰야 한다.


## 2026-11-15 (44) · Jetson 카메라 Lyrical 호환 수정 및 jetson_mon 배포

**이유**

팔 D435 영상 실행을 막은 bool 파라미터 범위 오류 해결과 jetson_mon 복사를 요청받았다.

**파일별 수정**

- Jetson `/home/nz2/nz2_camera_ws/src/realsense-ros`에 공식 4.57.7 태그, 커밋 5c2244ca5cd9867c9ee63769668891430f460dfd를 받아 별도 카메라 워크스페이스를 생성했다.
- 해당 경로의 `realsense2_camera/src/sensor_params.cpp` 144~147행에서 bool 파라미터에는 integer_range를 붙이지 않고 정수 파라미터에만 적용하도록 수정했다.
- Jetson 카메라 소스 `realsense2_camera/CMakeLists.txt` 208행의 rolling 분기에 lyrical을 추가해 최신 ROS API 분기로 빌드하도록 수정했다.
- Jetson `/home/nz2/nz2_bringup/start_gripper_camera.sh` 1~5행에 기본 ROS 및 수정된 카메라 overlay를 로드하는 실행 진입점을 생성했다.
- `/home/pa33/Desktop/jetson_mon/`의 3개 파일을 Jetson `/home/nz2/jetson_mon/`에 복사했다. 원본 모니터 코드는 그대로 사용했다.
- 제공된 install.sh를 실행해 Jetson `/home/nz2/.bashrc`에 jmon 함수를 등록했다.
- Jetson `/home/nz2/nz2_bringup/perception_preview.py` 1~199행에 ROS 컬러·CameraInfo 최신 프레임 구독, 기존 jetson_pipeline의 YOLO·CP 디코딩 재사용, 브라우저 MJPEG와 JSON 상태 화면을 추가했다. 640×480/30FPS 카메라와 YOLO FP16·CP FP32를 사용하며 HTTP는 LAN 8765 포트다.
- Jetson `/home/nz2/nz2_bringup/start_live_perception.sh` 1~5행에 ROS·카메라 overlay를 로드하고 실시간 인지 미리보기를 실행하는 진입점을 생성했다.
- Jetson `/home/nz2/.bashrc` 132행에 jmon 함수가 등록됐다.
- 이 파일 3행부터 변경 기록을 추가했다.

**확인 방법**

jetson_mon.py --local --once 실행으로 CPU·GPU·메모리·온도·전력·프로세스 출력을 확인했다. 카메라 드라이버는 MAKEFLAGS=-j1 -l1, CMAKE_BUILD_PARALLEL_LEVEL=1, colcon --executor sequential로 빌드한다. realsense2_camera_msgs 3분 2초, 카메라 드라이버 5분 33초 빌드가 성공했다. ROS 시험에서 color 5프레임(rgb8), 정렬 depth 3프레임(16UC1), CameraInfo 7개를 수신해 통과했다. 프레임은 gripper_color_optical_frame이다. 결과는 Jetson /tmp/nz2_gripper_camera_verification.json에 있다.

실시간 인지 화면을 백그라운드로 실행했다. http://10.2.11.63:8765 에서 노트북 접속 및 multipart MJPEG 바이트 수신을 확인했다. 4422프레임 시점 평균 26FPS, 해당 프레임 처리 23.1ms였다. 현재 가까운 바닥을 바라봐 검출 0개이며 퍽 실물 검출·CenterPose 정확도 검증은 남아 있다. 실행 PID 파일과 로그는 Jetson /home/nz2/nz2_bringup/perception_preview.pid 및 perception_preview.log, 카메라 로그는 gripper_camera.log다.

**실행해야 할 것**

젯슨에서 `source ~/.bashrc` 후 `jmon` 실행, 한 번만 확인하려면 `jmon --once`, 종료는 Ctrl+C다. 카메라만 실행은 `/home/nz2/nz2_bringup/start_gripper_camera.sh`, 카메라+인지+웹 미리보기는 `/home/nz2/nz2_bringup/start_live_perception.sh`다. 웹 미리보기 실행·노트북 접속 검증 후 사용자 요청으로 종료했다. 현재 카메라·인지·웹 서버는 꺼져 있다. 종료는 `kill -TERM "$(cat /home/nz2/nz2_bringup/perception_preview.pid)"`다.

**남은 일**

SDK 직접 시험에서 컬러 640×480 BGR 및 정렬 깊이 640×480 수신을 확인했다. 현재 시야는 가까운 바닥이며 깊이 유효율은 0.015319였다. 실제 컬러 내참수는 fx=607.9054, fy=607.6830, cx=323.3750, cy=249.5932였다. ROS 빌드·토픽 수신 검증은 통과했다. 퍽을 카메라에 보여 실제 검출과 자세를 검증해야 한다.

추가 USB 확인에서 오른쪽 라이다가 1-2.4.4 경로의 ttyUSB3으로 인식됐던 로그를 발견했다. 기존 right 규칙은 1-2.3.3이라 새 경로에서는 별칭이 붙지 않는다. 같은 허브 1-2.4의 -71 오류 및 이후 허브·팔 보드·라이다 동시 분리 로그를 확인했다. 이후 사용자가 다시 연결한 상태에서 ttyUSB3, 1-2.4.4 재인식을 확인했고 오른쪽 매핑 변경을 승인받아 후속 (45) 항목대로 적용했다.


## 2026-11-15 (86) · 팔 시험 종료 토크 유지 수정·7.2kg 질량 보정 재계산

**이유**

사용자가 받침 접촉을 확인하고 자동 토크 OFF 수정을 요청했다. 이어 전체 질량7.2kg 조건을 반영한 무게 재계산을 요청했다. 이전85번의5.81Nm 계산에는 기존 시뮬 mass_scale 보정이 빠져 있었으며 이 항목에서 보정값으로 정정한다.

**파일별 수정**

- firmware/tools/arm_j2_tune_test.py 18행 check에 토크 유지 정지 상태 허용을 추가했다.
- 같은 파일62행 supported-off 단계,63행 포트 지정,64행 시작 자세 복귀 옵션,66행 물리적 받침 확인 옵션을 추가했다. supported-off는 support-confirmed가 있어야 포트를 열며 해당 단계133행에서 정지·토크OFF·명시 gain 복원·실제 토크 설정 조회를 수행한다.
- 같은 파일75행 SIGTERM을 정리 경로로 처리했다. 180행 부근 토크ON ACK 유실 시에도 K를 전송해 기존 펌웨어 watchdog을 활성화한다.
- 같은 파일190행 선택적 시작 자세 복귀와196행 종료 처리를 변경했다. 정상/오류/Ctrl+C/SIGTERM 종료 시 S 정지·토크 상태 조회를 수행한다. 토크ON이면 유지 상태를 기록하고 P복원은 pending으로 남긴다. USB/정지 실패 시 자동 T0로 진행하지 않으며 상태 미확인을 기록한다. 전원 상실·모터 보호로 인한 토크 차단은 소프트웨어가 방지할 수 없다.
- firmware/tools/test_arm_j2_tune_test.py 1행부터 회귀시험을 추가했다. 성공·오류5종·시작 자세 복귀·gain복원 보류·명시 받침OFF·받침 확인 누락을 가짜 시리얼로 검사한다.
- Jetson /home/nz2/nz2/arm_j2_diag에 위 두 파일을 배포했다. 기존 시험 파일은 arm_j2_tune_test.py.before_torque_hold_1794747779065513595로 백업했다. 파일 SHA256 비교와 compile 후 원자적 교체했다.
- /tmp/nz2_j2_static_torque_scaled_7p2kg_20261115.json에 재계산 결과를 저장했다. 원본 URDF 및 시뮬 설정은 유지했다.
- 이 파일3행부터 기록을 추가했다.

**질량·토크 정정**

B33/B34 config 및 common.py에서 전체 링크 질량·관성을 균일하게 스케일하는 것을 확인했다. URDF 원본 총43.15431794kg 대비7.2kg 배율은0.1668430958이다. 이 시뮬 가정으로 손목·그리퍼1.567347kg은0.261501kg, shoulder 이하3.080292kg은0.513925kg, 홈0자세 중력토크5.809865Nm은0.969336Nm로 보정된다. 손목·그리퍼 기여는0.716624Nm이다. 최근 자세의 이전 부호 가정 토크는1.014269Nm이며, 영점·축 부호·자세 검증이 남아 있다. 총질량만으로 개별 실물 링크 질량 분포를 확정할 수 없고 카메라 질량 포함 여부도 미확인이다. 이전의 무보정 모터 스톨 초과 비교를 실물 용량 부족 근거로 사용하지 않는다.

**확인 방법**

로컬 yolo_env에서 구문검사·help 및 unittest6개 모두 통과했다. Jetson 시스템Python에서도 동일6개 테스트가 통과했다. 가짜 연결만 사용했으며 이번 수정 검증으로 실물 토크/이동 명령을 실행하지 않았다. 기존 firmware watchdog과 S는 목표를 현재 위치로 설정하며 토크를 유지하는 코드 경로를 확인했다.

**실행해야 할 것·남은 일**

다음 실물시험은 수정된 Jetson 스크립트를 사용한다. 기본 종료는 토크 유지이며 --return-to-start로 기록된 시작 자세 복귀 후에도 유지한다. 받침이 실제 무게를 받치는 것을 확인한 뒤 --gain800 --stage supported-off --support-confirmed --log 경로로 토크OFF 및P800 복원이 가능하다. 받침을 떠난 자력 유지 성공과 실물 무게 분포는 아직 검증되지 않았다.


## 2026-11-15 (89) · 받침 없는 P800 홈 이동 재현·토크 유지 정지

사용자가 홈 시험 진행을 승인했다. 현재 토크ON에서 R 뒤 현재위치 A로 재개하고 기존 P800 상태·최대5도/s S곡선·6도 추종 중단 및 기존 전류 기준으로 실행했다.

- j2 시작39.90도, j3 26.19도에서 홈0도 목표로 약11.97초 궤적을 계획했다. 그리퍼 각도는-101.87도로 유지했다. 약4.783초에 j2 목표25.8776도/실제31.99도/오차6.1124도로 중단했다. j3 실제19.60도/오차2.6142도였다. 홈 도달 및10초 홈유지 단계는 미실행이다.
- S 응답fail0/ms6, 토크ON·HW/fault/offline오류0을 확인했다. T0·게인/펌웨어변경은 수행하지 않았다.
- 이동 구간 전압10.6~11.2V, 레지스터 j2전류 최대절대944.19mA, 마지막 Q전류985mA, PWM최대절대388/한도885, 온도35~37도. 호스트 추종 기준으로 정지했으며 하드웨어 오류 정지는 관측하지 않았다.
- 정지후 추가조회 약3초에서 j2 33.75~33.84도, j3 약20.30~20.39도, j2전류약0.374A·10.9V·37도·토크ON이었다. 정지 명령시점31.99도 대비약1.85도 침하했다.
- Jetson /tmp/arm_home_unsupported_p800_20261115.jsonl 및 /tmp/arm_home_unsupported_stopped_20261115.json에 기록했다. 이 파일3행부터 추가했다.
- 받침이 있는 이전P800시험과 유사하게 j2 실제약32도에서6도 추종오차가 재현됐다. 원인 분리를 위해 전원 상태·부하 하 정적오차·제어설정 검증이 남는다. 현재 자세 자력 유지 확인 범위를 홈 도달/전체 자세 성능으로 확대하지 않는다.


## 2026-11-15 (90) · 기존 ROS 인지 노드 실물 연결·6클래스 호환·시험 중단

**이유**

사용자가 단일 퍽 인지·파지 연결을 승인했고 이후 기존 노드 사용 여부를 확인했다. 초기 단독 RGBD 점검 후 기존 ROS 카메라·YOLO·태그·등록 노드 연결로 전환했다. 사용자가 잠시 후 재개하겠다며 토크OFF를 요청해 종료했다.

**파일별 수정**

- ros_ws/src/perception/src/puck_registry.py 71행부터 red_cylinder/red_cuboid/green_cylinder/green_cuboid를 기존 shape/color/posture 내부 표현으로 해석하도록 추가했다. 미션의 전부 서있는 퍽 조건으로 초기 posture up을 사용한다. 이전8개 up/down 라벨과 비퍽5개 처리도 보존했다.
- jetson/single_puck_preview.launch.py 1행부터 기존 RealSense·yolo_detector·tag_reader·puck_registry를 실행하는 launch를 만들었다. 실제6클래스 runtime_nodes.yaml, 그리퍼1대·30Hz·domain78, YOLO10Hz·태그5Hz, 퍽 추적은 카메라 좌표로 설정했다. 지도/팔TF 및 팔 모션 실행은 포함하지 않는다.
- jetson/single_puck_probe.py 1행부터 한정 RGBD 버스트·실제6클래스 TensorRT·색상depth 검토·정적CAD 좌표 조회 도구를 작성했다.20행 camera_transform은 최종URDF 축에 맞춘 모터각 부호근사와 CAD광학프레임을 사용하며 실물 외부보정은 미완료로 표시한다.50행 픽셀별depth 역투영,87행 실행부. 센서 촬영후 종료하며 모션명령은 없다. 기존 ROS 노드 연결 검증 후 이 도구의 추가 실행은 중단했다.
- 위 launch·측정도구를 Jetson /home/nz2/nz2에 배포했다. 기존 puck_registry.py 수정은 /home/nz2/p2_runtime_ws/src/perception/src 및 install/perception/lib/perception에 백업 후 반영했다.
- 이 파일3행부터 기록을 추가했다. 팀 저장소·펌웨어·모델 가중치는 수정하지 않았다.

**확인 결과**

라벨4개·이전라벨8개·비퍽5개 검사, Python구문검사, 측정도구의 유효depth·색상거부·무효depth·고정상단TF·제로자세손목TF·회전직교성 검사를 통과했다. PC Isaac Sim Python 환경에서 동봉 NODEIK/refiner를 로드해 zero-FK 목표의16후보 추론을 확인했다(최소위치오차약8.54e-8m). 이는 오프라인 자기일관성 시험이다.

기존 ROS실행에서12초 동안 컬러93·정렬depth316·검출113·태그60·퍽추적95 메시지를 구독했다. depth시작 경고가 있었지만 메시지수신을 확인했고 당시 유효depth 비율약2.95%로 낮았다. 퍽을 손으로 렌즈정면에 보여줬을 때 red_cuboid 신뢰도약0.96, tag0, /perception/puck_tracks의 color1/shape2/posture1/track_id3/tag_id0 중심위치 약카메라전방0.27m를 확인했다. yaw는NaN이다. 사용자가 받침에 고정한 후에는 퍽이 시야밖이거나 화면우상단에서 잘려 추적0건이었고 실제파지는 실행하지 않았다.

관측자세 확보용 j2/j3 각각-5도 제한 시험에서 실제 j2 35.33도/j3 35.68도, 약10.8V·0.557A, 오류0으로 S토크유지 정지했다. 이후 사용자의 중단지시로 T0 성공·torque=false를 확인했다. 이번 실행한 launch PID24494를 SIGINT로 종료해 카메라·YOLO·태그·등록4프로세스가 정상종료한 로그를 확인했다. 같은domain의 기존 스워브/라이다 노드는 조작하지 않았다.

**재개 경로·남은 일**

Jetson /home/nz2/nz2/single_puck_preview.launch.py, 로그 /tmp/nz2_single_puck_ros.log, PID /tmp/nz2_single_puck_ros.pid. /tmp/single_puck_visible_ros_check.json에 실제검출·태그·퍽등록 근거, puck_latest_ros.json 및 이미지에 고정배치 시야제약, puck_observation_raise_20261115.jsonl에 팔 계측이 있다. 실제 집기 위해 렌즈와 고정퍽 관측 배치, 카메라→팔 외부보정·영점/축부호 검증, TCP실측, 파지yaw·접근궤적·그리퍼동작 연결이 남는다.


## 2026-11-16 (93) · 같은 ID의 다른 태그 면 분리·신경망 5mm 접근·업로드 범위 확인

**이유와 파일별 수정**

- jetson/arm_puck_calibrate.py 26행 corrected_goal에서 느린 위치 보상과 j3±6도/기타팔±4도 한계를 적용했다. 38행 main에 실제 정렬depth·CameraInfo를 태그 시각0.1초 이내로 연결하는 수집, 같은ID의 다른면을 법선으로 거르는 처리, 선택 관절 소각도 스윕, 최대±5도 손목관측 조절, 관절별5도 이내의 명시적 목표, 최대60초 추가 유지를 추가했다. 명시적 목표는 그리퍼 개방각을 보존하며 기존 전류·5도 이탈·0.8도 도착오차·종료 S토크유지 처리를 사용한다. 일반 ROS 브리지에 보상을 통합한 상태는 아니다.
- jetson/fit_arm_camera_calibration.py 1행부터 추가했다. 23행 phase_observations는 같은ID의 다른면을 base법선으로 분리하고, 78행 fit은 고정태그 외부변환을 학습/미사용 자세로 나눠 적합한다. 123행 main은 부호 가설 비교와 provisional/runtime_applied=false 결과를 기록한다.
- jetson/single_puck_preview.launch.py 9행 generate_launch_description에 컬러/depth profile 인자를 추가했다. 기본640x480x30을 보존하고 실제 실행에서1280x720x30 컬러·640x480x30 깊이를 선택했다. 장치 지원모드 조회 후 직접 소유한 카메라/인지 그룹만 재시작했다. 새 PID31162, 로그/tmp/nz2_single_puck_ros.log다.
- 위 소스3개를 Jetson /home/nz2/nz2에 배포했다. 수정.md 3행부터 이 기록을 추가했다.

**검증·결과·제약**

구문, 보상함수의 정적4.5도 부하 오차 수렴/변화율/범위/그리퍼 유지, 합성 정답 외부변환 복원과 미사용자세 검증을 통과했다. 위치 보상 중 시험자세의 j3 오차0~0.09도 관측을 얻었다. 일부 회차는 도착오차 또는 태그관측 중단으로 정지했다. S가 현재각으로 목표를 다시 쓰므로 종료 후 처짐까지 해결됐다고 해석하지 않는다.

태그0이 옆면과 윗면 사이에서 전환된 한 자세는 고정된 점처럼 비교할 때 약32mm 차이를 만들었다. 기존17mm 차이를 독립적인 hand-eye 오차로 단정했던 설명을 정정했다. RGBD4154개 제어샘플에서 같은 면을 고른11자세의 기존변환 미사용자세 RMS약0.830mm, 후보보정 약0.598mm, 최대약0.739mm다. 모두 작은 시험범위 결과이며 후보변환은 런타임에 적용하지 않았다. 같은면 선택에 기존CAD와 음의 모터부호 가정을 사용하므로 전역/독립적인 물리 보정 인증으로 해석하지 않는다.

PC의 neural-refiner-v1로 계산한5mm 접근을 Jetson에서 실제 실행했다. 목표모터각 [28.375967,-59.478283,67.423386,43.481636,-7.571964,-102.66]도, 종료전 실측 [28.39,-59.50,67.68,43.42,-7.56,-102.66]도, 최대목표오차약0.257도였다. 엔코더+FK 기반TCP이동약5.522mm, 샘플최대j3전류404mA였다. 실제TCP의 외부계측은 하지 않았다. /tmp/arm_puck_neural_5mm_test.json과 PC /tmp/arm_puck_same_face_fit.json에 근거가 있다.

최신퍽 좌표로31개/5mm 간격의 약152mm 접근을 오프라인 계산했지만 사용자가 진행 중단을 지시해 실행하지 않았다. 실제 퍽 파지/리프트/운반/배치는 미실행이다. 이후 사용자가 토크OFF를 요청했고 OK T0·torque=false를 확인했다. 당시 보드는모터0/6, fault3, offline63으로 개별모터 레지스터 조회는 불가능했다.

무기한 토크유지와 도착오차 조건을 제거한 후속스윕은 자동승인검토에서 거부됐다. 무기한유지는0~60초 유한유지로 변경하고, 스윕의0.8도 도착오차 조건을 복원한 후에만 후속 실행했다.

**업로드 범위 조회**

원격main=8fe9f5736cd568732dfb3e849c4ad69d52fd8693, feat/barcode-signal=3bfc80426b76bcfa0e3e63ae46e82d9d2bf56939을 ls-remote로 확인했다. 브랜치의 YOLO/CenterPose LFS SHA가 Downloads 원본과 같다. 로컬 tower_signal의 scripts/src/sim_assembly/configs 비교에서 수정29개·새설정7개(B28~B34)가 확인됐다. 추가분은 실물ROS/C++인지·팔브리지·diag_rev2/limits_rev8 펌웨어와 진단도구·Jetson 실행설정/모델변환도구·같은면 보정/접근 시험코드다. 팀 저장소는 조회만 했으며 파일복사·stage·commit·push는 수행하지 않았다.

