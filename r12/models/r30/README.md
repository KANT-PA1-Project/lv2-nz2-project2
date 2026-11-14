# 스워브 이동 로봇과 손목 롤 로봇팔

> r12 통합 참고: 로봇 지오메트리(urdf/usd/meshes)는 `r12/assets/vendor/R30_URDF_USD/`(colcon 패키지 r30_description)에만 둡니다. 이 폴더(models/r30)는 scene.xml·demo.usda·puck_slots·센서 도구만 두고 벤더 메시를 참조합니다. 제외: *.bak·mp4·sensor_sample·assembly_preview.jpg·pdf·카메라마운트/조립가이드. URDF는 nz2fix 원본 그대로이며 앞뒤(54476f0) 수정은 미포함.

(v26 폴더) assembly_full_mm.stl은 구 마운트(v25) 형상이라 이 폴더에서 뺐습니다. 필요하면 ../assembly_puck_slots_v25/에서 확인하세요. meshes 폴더의 STL은 링크 좌표계의 m 단위 시각 메시입니다. URDF는 meshes 폴더와 함께 사용합니다. assembly.usd는 메시를 포함한 로봇 단독 파일입니다. demo.usda를 assembly.usd와 같은 폴더에서 열면 지면과 12초 분량의 USD Physics 구동 목표가 추가됩니다. USD 재생에는 물리 시뮬레이션을 지원하는 호스트가 필요합니다.

## MuJoCo 실행

압축을 푼 폴더에서 실행합니다. 검증 환경은 Ubuntu, Python 3.12, MuJoCo 3.3.7입니다.

~~~bash
python -m pip install mujoco==3.3.7 numpy Pillow imageio-ffmpeg
python -m mujoco.viewer --mjcf=scene.xml
~~~

뷰어의 Control 패널에서 각 축을 움직일 수 있습니다. 검증 궤적 재실행과 영상 생성 명령은 다음과 같습니다.

~~~bash
python replay.py
MUJOCO_GL=egl python replay.py --video
~~~

## 모델과 시험 조건

- 원본은 Fusion 조립본 v25입니다. 1,219개 유효 바디를 운동 링크 16개로 묶고 CAD 질량, 질량중심, 주관성 모멘트를 합산했습니다. 모델의 링크 질량 합계는 43.0208 kg입니다. 시각 메시는 Fusion 표시 메시을 원본 그대로 사용합니다. 접촉에는 박스와 타이어 원통을 사용했습니다.
- CAD 원점을 기준으로 +X를 front, +Y를 left로 이름 붙였습니다. 차륜 중심은 X, Y 각각 ±124 mm이고 Z는 −148 mm입니다. 조향은 +Z, 차륜 회전은 중립에서 +Y입니다. 타이어 접촉 반경은 33 mm입니다.
- 팔은 베이스 요, 어깨, 팔꿈치, 손목 피치, 손목 롤 순입니다. 원본 서보의 원통축과 현재 배치를 사용했습니다. 조인트가 없는 CAD에서 각 브라켓과 체결품의 소속 링크를 추정했습니다. 내부 기어와 그리퍼 링크기구는 등가 관절로 단순화했습니다.
- 제어 입력 14개에 가동 관절 15개입니다. URDF는 양쪽 손가락을 mimic으로 연결하고 MuJoCo는 등식 제약을 사용합니다. USD 단독 모델에서는 양쪽 슬라이더에 같은 위치 목표(m)를 지정합니다. demo.usda에 동기 목표가 들어 있습니다.
- 양쪽 패드의 CAD 초기 간격은 19.63 mm입니다. ROBOTIS OpenMANIPULATOR-X의 공칭 최대 폭 75 mm를 목표로 손가락당 바깥쪽 27.686 mm를 허용합니다. 위치 명령 0 m는 CAD 원자세, −0.027686 m는 최대 개방입니다. 실제 개조품의 링크와 체결부 간섭, 기계적 끝점은 미검증입니다. 조향·팔 회전각 범위와 구동 게인은 시뮬레이션 가정입니다. 바퀴 속도 명령 단위는 MuJoCo에서 rad/s, USD 각속도 드라이브에서 deg/s입니다. 지면 주행과 무부하 팔 운동을 시험했고, 링크 간 자체 충돌은 끈 상태입니다.
- verification.json은 URDF를 MuJoCo에 직접 읽고 자유 차체·바닥·제어기를 추가한 시험 결과입니다. verification_usd.json은 OpenUSD로 읽은 강체·관성·관절·충돌 형상을 MuJoCo로 옮겨 같은 제어기와 조건을 적용한 결과입니다. USD 메시도 배포된 STL의 삼각형과 대조했습니다. Isaac Sim 또는 PhysX 네이티브 재생과 실물 시험은 수행하지 않았습니다.


## 시뮬레이션 센서

손목 D435의 센서 프레임은 wrist_roll에, 상단 D435와 네 COIN-D4 스캔 프레임은 base_link에 붙어 있습니다. D435의 좌우 IR 광학 중심 간격은 50 mm이고 깊이 좌표계는 왼쪽 IR과 일치합니다. ROS 광학 프레임은 +Z 전방, +X 오른쪽, +Y 아래입니다. 방향과 위치는 Fusion 렌즈 형상에서 산출한 공칭값입니다.

| 센서 | 스트림 | 설정 |
|---|---|---|
| wrist_d435, mast_d435 | color, depth, infra1, infra2 | 각각 640×360, 30 Hz. 수평 화각 RGB 69°, 깊이·IR 87°. 깊이 유효 범위 0.105~10 m |
| lidar_front, lidar_left, lidar_rear, lidar_right | scan | 10 Hz, 수평 360°, 400빔, 0.9° 간격, 0.05~12 m |

MuJoCo에서 현재 장면의 센서 배열을 저장하려면 패키지 폴더에서 다음 명령을 실행합니다.

~~~bash
MUJOCO_GL=egl python sensor_reader.py --output sensor_sample
~~~

sensors.npz에 RGB(uint8), 깊이(float32, m), 좌우 모노 영상(uint8), 거리 스캔(float32, m)이 들어 있습니다. 깊이의 무효값은 NaN, 라이다 미검출은 +inf입니다. 반복 수집은 SensorReader.read(data)를 매 물리 스텝 뒤에 호출합니다. 기본 하드웨어 파라미터 외에 출력 해상도, 깊이 클리핑, 핀홀 투영, 순간 평면 스캔은 시뮬레이션 선택값입니다. IR 영상은 좌우 카메라 위치에서 렌더한 밝기이며 능동 IR 패턴과 실제 스테레오 매칭 오류는 모델링하지 않았습니다.

Gazebo Sim용 센서와 고정 TF는 real_final_plz_no_more_edit_v99.urdf에 있습니다. ROS 2와 ros_gz 설치 환경에서 패키지 폴더를 작업 디렉터리로 두고 아래 명령을 차례로 실행합니다. 첫 번째와 마지막 명령은 별도 터미널에서 실행합니다.

~~~bash
export GZ_SIM_RESOURCE_PATH="$PWD:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -r sensor_world.sdf
~~~

~~~bash
ros2 run ros_gz_sim create -world sensor_lab -name robot -file "$PWD/real_final_plz_no_more_edit_v99.urdf" -z 0.182
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$PWD/ros_gz_bridge.yaml"
~~~

영상은 /wrist_d435/color/image_raw 등 각 센서명/스트림 아래에 있으며 같은 경로의 camera_info에 투영 정보가 전달됩니다. 스캔은 /lidar_front/scan 등 네 방향별 토픽입니다. ROS에서 TF를 사용하려면 real_final_plz_no_more_edit_v99.urdf를 robot_state_publisher에 전달하고 가동 관절 상태를 공급합니다.

USD의 카메라 및 렌더 제품은 /Robot/<부모 링크>/<센서명>과 /Robot/Render 아래에 있습니다. 깊이는 DistanceToImagePlane, RGB·IR 원본은 LdrColor 렌더 변수입니다. USD IR 출력을 모노 영상으로 쓸 때 밝기로 변환합니다. 라이다 네 개는 /Robot/base_link 아래 NVIDIA OmniLidar입니다. Isaac Sim에서 RTX 센서 확장이 필요합니다. URDF와 USD의 구조·참조를 검사했고, 센서 출력 시험은 MuJoCo에서 진행했습니다. Gazebo/ROS 2 토픽 발행과 Isaac Sim RTX 실행은 미검증입니다.

사양 근거는 [RealSense D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/)와 [ROBOTIS LDS-03](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/)입니다.


## 퍽 스토리지 위치 추적

흰색 트레이와 ID 0~7의 바닥 태그를 반영했습니다. [인쇄용 A4 라벨](puck_slots/slot_labels_A4.pdf)과 [실물 부착·보정 방법](puck_slots/사용방법.md)을 사용하세요.

스토리지 8칸의 바닥은 base_link 기준 Z=50 mm, 입구는 Z=75 mm입니다. 태그 검출면은 Z=50.12 mm입니다. 관측 자세는 puck_slots/views.json에 있습니다.

Fusion v25에서 상단 카메라와 마스트가 35 mm 상승했습니다. 상단 카메라 4개 스트림 위치, 라이다 마운트 4개와 카메라 마운트, 스토리지 메시 및 차체 관성을 갱신했습니다.

## v26 상단 카메라 마운트 변경

상단 D435 마운트를 각도 조절식(25°·30°·35°·40°·45° 하향, 기본 35°)으로 교체했습니다. 피벗축이 mast_d435 프레임 원점을 지나므로 각도를 바꿔도 센서 위치(xyz)는 그대로이고 피치만 바뀝니다. 이 폴더는 v26 전용입니다. 수정 전 v25 파일은 ../assembly_puck_slots_v25/ 폴더에 *_v25_backup 이름으로 남아 있습니다.

- real_final_plz_no_more_edit_v99.urdf: mast_d435_* 프레임 4개 pitch 1.0472 → 0.6109 rad, base_link 시각 메시를 meshes/base_link_0_v26.stl(구 마운트·60° 카메라 제거)로 교체하고 camera_mount_base_v26, camera_cradle_v26, mast_d435_v26 메시를 추가했습니다.
- scene.xml: mast_d435 카메라 quat 4개, base_link 관성, 시각 메시를 같은 기준으로 갱신했습니다.
- assembly.usd: visual_0에서 구 마운트·카메라를 제거하고 세 메시를 추가했습니다. 카메라 orient와 base_link MassAPI를 갱신했습니다. demo.usda는 그대로 참조합니다.
- base_link 질량은 21.2711 → 21.4046 kg입니다. 새 출력물은 솔리드 PLA(1240 kg/m³), D435는 72 g으로 가정했습니다.
- verification.json과 sensor_sample/은 Windows, MuJoCo 3.3.7에서 다시 생성했습니다. 솔버 경고는 0건입니다. 상단 깊이 영상의 최소 거리는 0.243 m(로봇 팔)이고, 마운트에 의한 가림은 없습니다.
- verification_usd.json은 패키지에 생성 스크립트가 없어 다시 만들지 못했습니다. Gazebo/ROS 2와 Isaac Sim 실행은 여전히 미검증입니다.
- 출력·조립 방법은 camera_mount_tilt/ 폴더와 이 폴더의 D435_상단마운트_v26_조립가이드.md를 참고하세요. 각도별 URDF·MuJoCo·USD 값도 그 문서에 있습니다.