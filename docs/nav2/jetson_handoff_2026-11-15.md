# NZ2 실물 Nav2 핸드오프 — 2026-11-15

## 1. 현재 결론과 완료된 작업

대회: 2026-11-16. 이 문서는 11/15 이 대화의 마지막 사용자 제공 로그까지 반영했다. 이후 현장 변경은 확인하지 않았다.

**개발 PC → Jetson 이식(소스·URDF 복사 및 빌드)은 완료됐다. Nav2 전체 기동·실물 주행은 아직 검증하지 않았다. 현재 라이다 USB 통신 장애를 해결해야 한다.**

- 사용자 터미널에서 `rsync`로 `nz2_swerve_nav`와 최종 URDF·meshes 복사 성공.
- Jetson `~/nz2_nav_ws`에서 `colcon build --packages-select nz2_swerve_nav --symlink-install --parallel-workers 1` 성공.
- 기존 `test_swerve_kinematics`: 1 test, 0 errors, 0 failures, 0 skipped.
- launch v2: 12691 B, real_robot.yaml: 1015 B, r12_real_overrides.yaml: 1556 B, Cartographer lua: 3344 B (개발 PC 확인). Jetson launch 12691 B 확인.
- Jetson에서 r12_swerve_control, coin_d4_driver, cartographer_ros, nav2_bringup, nav2_amcl, nav2_controller, nav2_mppi_controller, nav2_planner, nav2_bt_navigator, nav2_velocity_smoother 설치 경로 확인.
- `single_coin_d4_node`, `multi_coin_d4_node` 실행 파일 확인. 보조 스크립트 설치 링크 확인.
- `/dev/lidar_right`: 약 10.016 Hz로 170초 이상 수신. `/scan` 메시지 확인. front도 사용자 위치 대조를 진행했으나 장시간 주파수 재시험은 미완료.
- 마지막 USB 목록: 라이다 CP210x 4개, OpenCR USB 장치 3개. OpenCR 역할·핸드셰이크·안정성은 [확인]. 팀원이 OpenCR 작업 중이다.

표기: `[확인]`은 아직 검증하지 않은 항목, `[가정]`은 추정이다. 로그에 확인된 사실과 구분한다.

금지·유지사항:

- `~/git/lv2-nz2-project2`와 `~/ros2_lyrical` 안에 파일을 추가하지 않는다. 기존 설치 환경을 source하는 것은 가능하다.
- 실물 라이다는 **LDS-03 ×4**, 드라이버는 **coin_d4_driver**, 포트는 **/dev/lidar_***.
- LDS-02, nz2_ld08_driver, by-path 방식, carto_ws 절차는 폐기. nz2_ld08_driver는 이 ZIP에 넣지 않았다.
- 실물 시험은 **바퀴 띄움 → 지면**. RC-100 비상정지 담당을 둔다. 팀원의 시험과 겹치지 않는다.
- 기존 `nz2_swerve`의 `swerve_driver.launch.py`는 현재 실물에 사용하지 않는다. real_nav와 동시 실행 금지.
- 원본 문서 1이 우선이고, 밤 보충 문서가 명시한 v2 반영 완료·고정 실행 파일 변경을 적용한다. 오래된 문서의 실행 파일 자동 탐색/lidar_exec 안내는 현재 코드와 맞지 않는다.

## 2. 접속과 터미널 구분

SSH: `nz2@nz2jetson.local`. 처음 IP 10.2.11.63은 폐기. 이후 10.2.12.149였고, 사용자 최신 SSH 기록에는 **10.2.12.174**가 나왔다. IP는 바뀔 수 있으므로 mDNS 이름 우선. 이름 해석 실패 시 Jetson 로컬 `ip -br address`로 현재 주소 확인.

**터미널 창이 개발 PC에 있어도 프롬프트가 `nz2@nz2jetson`이면 Jetson에서 실행 중이다.**

| 터미널 | 위치/역할 | 예상 프롬프트 |
|---|---|---|
| 3 | 개발 PC, 파일 전송 | `pa24@pa24-Legion-Pro-5-16IAX10` (다른 팀원 PC면 사용자명 다름) |
| 4 | Jetson SSH, 드라이버·launch | `nz2@nz2jetson` |
| 5 | Jetson 별도 SSH, 관측 | `nz2@nz2jetson` |

[터미널 4 또는 5 · 개발 PC — 아직 SSH 접속 전]
```bash
ssh nz2@nz2jetson.local
```

각 SSH 터미널에서 필요한 환경:

[터미널 4·5 각각 · Jetson — SSH 접속 후]
```bash
source ~/ros2_lyrical/install/setup.bash
source ~/git/lv2-nz2-project2/r12/ros2/install/setup.bash
source ~/nz2_nav_ws/install/setup.bash
export ROS_DOMAIN_ID=78
```

`~/ros2_lyrical`은 ROS 소스 빌드와 Cartographer 환경, `/opt/ros/lyrical`은 Nav2 바이너리 위치다. 패키지가 검색된 것만으로 두 환경의 런타임 호환성이 검증된 것은 아니다. `symbol lookup error` 발생 시 오류 전문을 남기고 별도 깨끗한 셸의 환경 분리를 검토한다. 무작정 source 순서를 바꾸거나 전체 재설치하지 않는다.

이전 `/scan` 검색 장애: 기본 `ros2 topic list`에는 안 보였으나 `--no-daemon`에서 보였고 명시적 타입 echo는 수신 성공. 같은 증상이면:

[터미널 5 · Jetson]
```bash
ros2 topic list -t --no-daemon
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --no-arr --qos-reliability best_effort
```

## 3. 현재 USB 장애와 첫 조치

**다음 담당자는 여기부터 시작한다. 마지막 제안인 허브 A 재연결은 아직 실행 결과를 받지 못했다.**

마지막 확인된 포트 구조:

| 링크 | 장치 | USB 경로 |
|---|---|---|
| /dev/lidar_front | ttyUSB0 | 1-2.3.1 |
| /dev/lidar_rear | ttyUSB1 | 1-2.3.2 |
| /dev/lidar_right | ttyUSB2 | 1-2.3.3 |
| /dev/lidar_left | ttyUSB3 | 1-2.3.4 |

USB 허브 A `1-2.3` (214b:7260)에 라이다 4개가 연결돼 있다. 허브 B `1-2.4`는 현재 **1a40:0101**이고 OpenCR 3개가 USB 목록에 보인다. 과거 문서의 `lsusb | grep -c 214b`로 허브 2개를 기대하는 검사는 현재 구성에 맞지 않는다.

마지막 오류:

```text
22:37:07 cp210x ttyUSB0: failed set request 0x12 status: -110
22:37:12 cp210x ttyUSB0: failed set request 0x0 status: -110
22:37:28 cp210x ttyUSB1: failed set request 0x0 status: -110
22:37:28 cp210x ttyUSB1: cp210x_open - Unable to enable UART
22:39:13 cp210x ttyUSB3: failed set request 0x0 status: -110
22:39:13 cp210x ttyUSB3: cp210x_open - Unable to enable UART
```

`single_coin_d4_node`로 rear/left를 실행하면 `Failed to open lidar port`, `Lidar port is wrong` 발생. 장치 파일·링크는 존재한다. 마지막 `sudo fuser -v /dev/ttyUSB*`에는 점유 프로세스 출력이 없었으나, 순간적으로 포트를 여닫는 프로그램까지 배제되지는 않는다.

**주의:** 마지막 프로세스 목록의 `r12_opencr_bridge`는 팀 기본 `r12_swerve.yaml`만 로드했다. 우리 `r12_real_overrides.yaml`을 적용한 프로세스가 아니다. 실행 중인 port_globs 값은 [확인]. 기본값에 by-id가 있으면 라이다 포트도 탐색할 수 있다. 이것이 이번 장애의 원인인지는 [가정]이다.

권장 확인 순서:

1. 팀원과 합의하여 구동 시험 중단, 바퀴 띄움 유지. 터미널 4에서 실패 중인 라이다 드라이버를 Ctrl+C로 종료. 다른 팀원의 프로세스를 무차별 종료하지 않는다.
2. 실행 중인 브리지의 실제 설정 확인.

[터미널 5 · Jetson]
```bash
ros2 param get /r12_opencr_bridge port_globs
pgrep -af '[c]oin_d4|[r]eal_nav|[o]pencr_bridge'
sudo fuser -v /dev/ttyUSB*
```

브리지 설정에 `/dev/serial/by-id/*` 또는 `/dev/ttyUSB*`가 있으면 담당 팀원이 그 브리지 실행 터미널에서 종료한다. 팔 브리지의 탐색 범위도 [확인].

3. 새 로그를 켠다.

[터미널 5 · Jetson]
```bash
sudo dmesg --follow-new --human
```

4. 팀원이 구동 프로그램을 멈춘 뒤 **허브 A 상위 USB 케이블을 같은 자리에 재연결**. 라이다 4개의 개별 포트 위치를 바꾸지 않는다. 외부 전원 허브라면 전원·표시등도 확인한다. OpenCR 허브 B·Jetson 전원은 건드리지 않는다. [가정] 공통 허브 전원·접촉·USB 통신 불안이 의심되지만 고장 부품은 미확정이다.
5. 복귀 확인.

[터미널 4 · Jetson]
```bash
lsusb
ls -l /dev/lidar_*
ls -l /dev/ttyUSB*
for d in /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right; do
  echo "=== $d ==="
  if [ -e "$d" ]; then
    readlink -f "$d"
    udevadm info -q path -n "$d"
  else
    echo "링크가 없거나 연결된 장치가 없습니다"
  fi
done
```

**장치 파일 존재만으로 복구 완료가 아니다. 다음 단독 실행에서 UART 열기와 스캔 수신까지 확인한다.** 실패하면 새 로그를 보존하고, 케이블/허브를 한 번에 하나씩 교체해 원인을 구분한다. udev 수정·chmod 777·펌웨어 업로드부터 하지 않는다.

## 4. 라이다 위치 대조

사용자가 확정한 방향 기준은 **로봇 뒤에서 팔 쪽 정면을 바라보는 방향**이다. 과거 문서의 '팔 쪽을 볼 때'라는 모호한 표현으로 좌우를 다시 뒤집지 않는다.

| 포트 | 실제 위치 | 상태 |
|---|---|---|
| /dev/lidar_front | 팔 밑, 실제 앞 | 사용자 확인 |
| /dev/lidar_right | 실제 오른쪽 | 사용자 확인 |
| /dev/lidar_rear | 미확정 | [확인] USB 열기 실패로 중단. 이름만 보고 뒤로 확정하지 말 것 |
| /dev/lidar_left | 미확정 | [확인] USB 열기 실패로 중단 |

복구 후 **left부터** 이어서 시험한다. 한 번에 드라이버 하나만 실행한다.

[터미널 4 · Jetson]
```bash
ros2 run coin_d4_driver single_coin_d4_node --ros-args \
  -p port:=/dev/lidar_left \
  -p baudrate:=230400 \
  -p version:=4 \
  -p topic_name:=scan \
  -p frame_id:=test_scan \
  -p warmup_time:=5
```

예열 5초 뒤:

[터미널 5 · Jetson]
```bash
ros2 topic hz /scan --qos-reliability best_effort
```

2~3분 약 10 Hz 유지 확인 후 **터미널 5에서만 Ctrl+C**. 드라이버 터미널 4는 유지한다.

[터미널 5 · Jetson]
```bash
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --no-arr --qos-reliability best_effort
ros2 topic echo /scan sensor_msgs/msg/LaserScan --field ranges --once --qos-reliability best_effort
```

판을 센서에서 약 20~30 cm 떨어뜨려 한 개씩 놓고 치우며 거리값 변화가 따라오는 실제 위치를 기록한다. 센서를 만지거나 USB를 움직이지 않는다. 전체 ranges가 작은 값일 필요 없다. 0/nan, range_min 미만 값은 위치 확인의 유효 거리로 취급하지 않는다. 앞서 /scan은 range_min 약 0.1, range_max 100을 보고했으나 100 m 성능이 입증된 것은 아니다. 기존 real_robot의 range_max 12.0을 유지한다.

left 완료 후 터미널 4 Ctrl+C → 위 드라이버 명령의 port만 `/dev/lidar_rear`로 변경하여 동일 시험. rear 완료 후 다른 라이다도 복구 후 정상인지 재확인. 네 개 동시 구동 안정성은 별도 확인한다.

## 5. 복사·빌드 재현

**현재 Jetson에는 이미 복사·빌드됐다. 손실·변경이 없다면 재복사할 필요 없다.** 이 절은 다른 팀원이 이 ZIP으로 복원할 때 사용한다.

ZIP을 개발 PC 홈에서 풀면 `~/nz2_nav2_handoff_2026-11-15/`가 생긴다. 압축 파일 위치가 다르면 unzip 인자만 바꾼다.

[터미널 3 · 개발 PC — SSH 접속하지 않은 터미널]
```bash
unzip -n ~/Music/nz2_nav2_handoff_2026-11-15.zip -d ~/
cd ~/nz2_nav2_handoff_2026-11-15
sha256sum -c SHA256SUMS.txt
ssh nz2@nz2jetson.local 'mkdir -p ~/nz2_nav_ws/src ~/nz2_urdf'
rsync -av ./ros2_ws/src/nz2_swerve_nav nz2@nz2jetson.local:~/nz2_nav_ws/src/
rsync -av ./nz2_urdf/real_final_plz_no_more_edit_v99_nz2fix.urdf ./nz2_urdf/meshes nz2@nz2jetson.local:~/nz2_urdf/
```

`rsync`는 기존 파일을 덮어쓸 수 있으므로, 현장에서 팀원이 수정했다면 차이와 백업을 먼저 확인한다. `--delete`는 사용하지 않는다.

[터미널 4 · Jetson]
```bash
source ~/ros2_lyrical/install/setup.bash
source ~/git/lv2-nz2-project2/r12/ros2/install/setup.bash
source ~/nz2_nav_ws/install/setup.bash
export ROS_DOMAIN_ID=78
cd ~/nz2_nav_ws
colcon build --packages-select nz2_swerve_nav --symlink-install --parallel-workers 1
```

위 빌드가 성공한 뒤:

[터미널 4 · Jetson]
```bash
source ~/nz2_nav_ws/install/setup.bash
colcon test --packages-select nz2_swerve_nav --event-handlers console_direct+
colcon test-result --verbose
grep -n port_globs ~/nz2_nav_ws/install/nz2_swerve_nav/share/nz2_swerve_nav/config/r12_real_overrides.yaml
```

port_globs는 `['/dev/ttyACM*']`. 이 파일이 올바르더라도 이미 실행 중인 다른 브리지에 자동 적용되지는 않는다.

## 6. 복구 후 bringup과 주행 순서

1. §3 USB 복구, §4 실제 위치 표 4개 완성.
2. 팀 제어 노드·기존 launch 담당자와 실행을 조율하고 중복 실행 방지. **bridge:=false는 이번 launch의 브리지만 생략하며, 이미 켜진 팀 브리지를 끄지 않는다.**
3. 아래 실제 위치 포트를 채운 뒤 바퀴를 띄운 채 bringup. 아직 모르는 값은 비워 두었다. 빈 인자는 현재 launch가 차단하므로 임의로 추정해서 채우지 않는다.

[터미널 4 · Jetson — 네 위치 대조 완료 후에만]
```bash
PORT_ARM=/dev/lidar_front
PORT_RIGHT=/dev/lidar_right
PORT_BACK=''
PORT_LEFT=''
```

PORT_BACK·PORT_LEFT를 실측 표로 채우고 나서:

[터미널 4 · Jetson]
```bash
ros2 launch nz2_swerve_nav real_nav.launch.py mode:=bringup bridge:=false \
  port_arm:="$PORT_ARM" port_back:="$PORT_BACK" \
  port_left:="$PORT_LEFT" port_right:="$PORT_RIGHT"
```

[터미널 5 · Jetson]
```bash
ros2 topic list -t --no-daemon
ros2 topic hz /scan --qos-reliability best_effort
```

필요한 개별 토픽은 `/lidar_front/scan`, `/lidar_left/scan`, `/lidar_rear/scan`, `/lidar_right/scan`이다. 여기서 front/left/rear/right는 **URDF 이름**이며 실제 위치 이름이 아니다. launch는 실제 위치 인자를 다음처럼 변환한다:

| 실제 위치 인자 | 출력 이름공간/URDF 이름 |
|---|---|
| port_arm | lidar_rear |
| port_back | lidar_front |
| port_left | lidar_right |
| port_right | lidar_left |

4. 개발 PC RViz에서 ROS_DOMAIN_ID=78로 4개 스캔이 벽에 맞게 겹치는지 확인. 장착 yaw·좌우·팔 쪽 기준을 직접 확인한다. 화면이 틀어졌다고 무조건 reverse를 바꾸지 않는다. 설치 방향과 TF를 먼저 확인하고, 스캔 반전이 확인될 때만 `lidar_reverse:=true` 재시험.
5. 라이다 4개 동시 구동 5분 이상 끊김 확인. OpenCR 3개 역할·핸드셰이크 확인 후 팀원과 모터 시험 조율.
6. 바퀴 띄움: 브리지의 ACM 전용 설정 적용 확인 → 조향·바퀴 부호·wv 단위·명령 끊김 정지 확인 → 저속 텔레옵. real_wheel_check.py는 실제 모터 명령을 보내는 도구이므로 준비 없이 실행하지 않는다.
7. 지면: 저속 수동 운전 → 2 m 직진·180도 회전 두 번으로 odom 배율 확인. 펌웨어 wv가 모터축 단위라 약 4배 오차면 단순 배율로 덮지 말고 팀과 단위 수정.
8. mapping: Cartographer로 실제 지도 작성 → save_map_carto.sh 및 transform_map.py로 mission 좌표 지도 저장. 시작 자세·좌표를 확인하고 스크립트 인자를 검토한 뒤 실행.
9. nav: 실제 저장한 mission 지도와 초기 자세를 확인하고 짧은 목표부터. mode:=mapping과 mode:=nav는 동시에 실행하지 않는다.

후반 모터 시험과 지도 저장 인자는 이번 작업에서 검증하지 않았다. 앞 단계 통과를 확인한 뒤 동봉 소스와 원본을 읽고 현장 담당자와 실행한다. 검증하지 않은 일괄 자동 실행 스크립트는 넣지 않았다.

기존 설정: 조향 4.0 rad/s, 램프 0, 조향 ±90도, 선속도 0.2 m/s, 각속도 0.4 rad/s, timeout 0.25 s. 램프 0을 쓰기 전 펌웨어 구동 Profile Acceleration 확인 필요. 0(무제한)이면 원본 문서의 램프 0.3·align_gate false 대안 검토. 조향 한계·속도 상향은 기본 실물 시험 전에는 하지 않는다.

## 7. ZIP 구성과 제한

- `README_먼저읽기.md`: 이 문서. 최신 진행 상황과 첫 조치.
- `ros2_ws/src/nz2_swerve_nav/`: 개발 PC 소스 전체. launch, 설정, C++ 노드, 보조 스크립트, 테스트, behavior tree 포함.
- `nz2_urdf/real_final_plz_no_more_edit_v99_nz2fix.urdf`, `nz2_urdf/meshes/`: 최종 모델과 메시. .bak 모델은 사용 금지라 제외.
- `docs/nz2_handoff_2026-11-15_lidar_usb_원본.md`: 사용자 지정 문서 1 원본. **그 뒤 완료된 복사·빌드와 현재 허브 구조는 이 README로 갱신해서 읽을 것.**
- `docs/문서2_3_대화제공내용_요약.md`: 사용자가 붙여 넣은 보충 문서와 실물 6단계 문서의 요약. 원본 파일을 확보한 것으로 오인하지 말 것.
- `logs/`: 사용자가 제공한 USB·라이다 출력. 비밀번호·SSH 개인키 미포함.
- `SHA256SUMS.txt`: 동봉 파일 무결성 목록. ZIP 자체 무결성도 생성 시 검사했다.

이 ZIP은 **기존 Jetson 설치를 이어받는 소스 묶음**이다. ROS/Nav2/Cartographer 바이너리·coin_d4_driver 소스·팀 r12 패키지·실제 지도는 포함하지 않았다. coin_d4_driver는 Jetson 설치 확인은 했지만 개발 PC 원본을 확보하지 못했다. 새 Jetson의 완전 오프라인 설치용 묶음은 아니다. 팀 r12 설치는 기존 Jetson 경로를 사용한다. 새 장비 이식 시 의존성 확보가 별도로 필요하다.

기구학 테스트 통과는 센서 품질·실제 통로 여유·모터 방향·비상정지·Nav2 런타임 호환성을 보증하지 않는다. 사용자 로그로 확인된 결과와, 작성자가 원격 실행해서 확인한 결과를 혼동하지 말 것: Jetson SSH 자동 인증 수단이 없어 빌드·센서 결과는 사용자가 제공한 출력에 근거한다.

## 8. 인수인계 기록표

| 항목 | 인수 시 상태 | 다음 담당 기록 |
|---|---|---|
| OpenCR 3개 USB 인식 | 마지막 목록 3개, 역할·안정성 [확인] | |
| 실행 중 브리지 port_globs | 팀 기본 yaml만 로드, 실제 값 [확인] | |
| 허브 A 같은 자리 재연결 | 제안만 함, 결과 [확인] | |
| left UART 열기/10 Hz | 실패 중 | |
| rear UART 열기/10 Hz | 실패 중 | |
| front·right 복구 후 재시험 | [확인] | |
| 실제 위치 표 | front=팔 밑 앞, right=실제 오른쪽, 나머지 미확정 | |
| 4개 스캔 벽 정합·5분 안정성 | 미실행 | |
| bridge false bringup | 미실행 | |
| 바퀴 띄움/정지 시험 | 미실행 | |
| odom 보정·실제 지도·Nav2 목표 | 미실행 | |
