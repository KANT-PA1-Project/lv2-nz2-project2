# OpenCR 보드 식별 핸드셰이크

스월브 좌/우 + 팔, OpenCR 세 대를 USB 허브로 Jetson Orin Nano 에 연결한다.
리눅스는 꽂힌 순서대로 `/dev/ttyACM0..2` 를 주므로, 어느 포트가 어느 보드인지
호스트가 미리 알 수 없다. 그래서 **포트 번호가 아니라 보드가 스스로 말하는 역할(role)**
로 포트를 배정한다.

| role | 보드 | 스케치 |
| --- | --- | --- |
| `swerve_left` | 스월브 FL·RL | `firmware/opencr_swerve_left` |
| `swerve_right` | 스월브 FR·RR | `firmware/opencr_swerve_right` |
| `arm` | 팔 | 아직 저장소에 없음 (아래 계약만 정의) |

## 프로토콜

한 줄 = JSON 1개, 개행(`\n`)으로 끝난다. USB CDC 이므로 `HOST_BAUDRATE`(115200)는
형식일 뿐 실제 전송 속도를 바꾸지 않는다. 부분 줄·JSON 이 아닌 줄은 호스트가 버린다.

### 호스트 → 보드

```
I
```

### 보드 → 호스트

```json
{"hello":1,"proto":1,"board":"swerve_left","side":"left","kind":"swerve","fw":"opencr_swerve_left 0.2.2","order":["FL","RL"]}
```

- `board`: 이 보드의 role. **가장 먼저 보는 값**이다.
- `proto`: 핸드셰이크 포맷 버전. 지금은 `1`.
- `side`: `left` / `right` (스월브만). `board` 가 없을 때의 차선책.
- `kind`: `swerve` / `arm`.
- `fw`: 스케치 이름 + 버전.
- `order`: 이 보드가 맡은 모듈 코드의 순서. `C` 프레임의 값 순서와 같다.
  스월브 좌 `["FL","RL"]`, 우 `["FR","RR"]`.

보드는 부팅 직후에도 같은 줄을 한 번 보낸다. 호스트가 늦게 포트를 열면 `I` 를 다시
보내므로, 부팅 줄을 놓쳐도 된다. `I` 응답은 모터 레지스터를 읽지 않으므로 즉답한다.

포트를 열 때 보드가 리셋되는 경우가 있어 호스트는 `I` 를 0.2 s 간격으로 되풀이해
보낸다. 그래도 응답이 없으면 마지막 40% 구간에서 `?` 를 섞어 구 펌웨어까지 찾는다.

### 구 펌웨어 폴백

`I` 를 모르는 펌웨어는 `unknown_cmd` 로 거절한다. 그러면 호스트가 `?` 를 보내
기존 `cfg` 응답(`{"cfg":1,...,"side":"left","fw":...}`)으로 대신 식별한다.
`cfg` 응답에는 모터 레지스터 읽기가 포함되어 느릴 수 있다.

### role 판정 순서

1. `board` 값
2. `side` → `left`=`swerve_left`, `right`=`swerve_right`
3. `fw` 접두사 `opencr_<role>`

셋 다 없으면 그 보드는 무시한다. 같은 role 을 두 보드가 보고하면 브리지가 에러를
남기고, 먼저 잡은 보드만 쓴다.

## 펌웨어에 넣는 것

`config.h`:

```c
#define BOARD_ROLE "swerve_left"   // 보드마다 반드시 다르게
#define BOARD_KIND "swerve"        // 팔은 "arm"
#define FW_VERSION "0.2.2"
```

`.ino`:

- `sendIdentity()` — 위 `hello` 한 줄 출력
- `handleLine()` 의 `case 'I': sendIdentity(); break;`
- `setup()` 끝에서 `sendIdentity();` 한 번 (부팅 알림)

스월브 좌/우 스케치는 위가 이미 들어가 있다. 새 보드(팔)를 만들 때도 같은 계약을
지키면 호스트 코드는 그대로 쓴다.

## 호스트 쪽

- `r12_swerve_control/opencr_discovery.py`
  - `identify(port)` — `I` → 없으면 `?` 로 한 포트의 role 정보를 얻는다.
  - `candidate_ports(patterns)` — `glob` + `realpath` 로 by-id 중복을 제거한다.
  - `assign_roles(infos, roles)` — role 별 포트 배정, 중복 보고.
  - ROS 를 import 하지 않아 pty 로 단위 테스트할 수 있다.
- `opencr_bridge_node.py` — 시작할 때와 `discovery_interval_s` 마다 후보 포트를 훑어
  `board_roles` 에 있는 보드만 연다. 스월브 브리지는 `arm` 을 무시한다.
- `config/r12_swerve.yaml`

```yaml
/r12_opencr_bridge:
  ros__parameters:
    board_roles: ['swerve_left', 'swerve_right']
    port_globs: ['/dev/ttyACM*', '/dev/serial/by-id/*']
    # ports: ['/dev/serial/by-id/usb-ROBOTIS_OpenCR_...']  # 탐색을 끄려면
    discovery_interval_s: 2.0
    discovery_timeout_s: 1.2
    require_all_boards: true
    stop_on_fault: true
```

`ports` 를 지정하면 탐색 대신 그 포트만 쓴다(테스트·수동 배선용).

## 안전 규칙

명령 프레임은 `C <en> <w...> <s...>` 이고, 아래 중 하나라도 참이면 `en=0` 이다.

- `require_all_boards: true` 인데 설정한 role 이 하나라도 없다
- 두 보드가 같은 모듈 코드를 맡고 있다
- 보드가 `flt`(고장) 또는 `wd`(워치독)를 보고했다 (`stop_on_fault`)
- `/r12/swerve_joint_commands` 가 `cmd_timeout_s` 넘게 끊겼다

`en=0` 이어도 조향각은 마지막 값을 유지한다(차체가 급격히 틀지 않게). 휠은 0 이다.
브리지는 `E`(토크 ON)를 자동으로 보내지 않는다. 사람이 명시적으로 켠다.

## 팔 보드 계약 (스케치 작성 시)

저장소에 팔 펌웨어가 아직 없다. 팔 스케치를 만들 때 아래만 지키면 된다.

- `BOARD_ROLE "arm"`, `BOARD_KIND "arm"`
- `I` 에 `{"hello":1,"proto":1,"board":"arm","kind":"arm","fw":"...","order":[...]}`
  로 즉답. 모터 레지스터를 읽지 않는다.
- `order` 는 그 보드가 받는 관절/모듈 코드 순서
- `?` 의 `cfg` 에도 `"board":"arm"` 을 넣어 구 호스트와도 맞춘다

팔 쪽 브리지는 `board_roles: ['arm']` 로 띄우면 스월브 보드는 무시한다.

## 확인

실물 보드 없이( pty ):

```bash
source /opt/ros/lyrical/setup.bash
cd ros2/r12_swerve_control
python3 -m pytest -q test
```

실물 보드가 꽂힌 Jetson 에서, 브리지를 끈 상태로:

```bash
# /dev/ttyACM* 읽기 권한(dialout 그룹). 이미 되어 있으면 건너뛴다.
sudo usermod -aG dialout "$USER"   # 적용은 다시 로그인한 뒤
```

```bash
source /opt/ros/lyrical/setup.bash
source reports/local/ros_install/setup.bash
python3 - <<'PY'
from r12_swerve_control.opencr_discovery import identify_ports, role_from_info
for port, info in identify_ports(['/dev/ttyACM*', '/dev/serial/by-id/*']).items():
    print(port, role_from_info(info), info)
PY
```

기대: `swerve_left` / `swerve_right` / `arm` 이 각각 한 번씩, 포트는 꽂힌 순서와
무관하게 나온다. role 이 `None` 이거나 중복이면 `BOARD_ROLE` 과 업로드한 스케치를
확인한다.

## 관련 문서

- [`OPENCR_FAULT_RECOVERY.md`](OPENCR_FAULT_RECOVERY.md) — 고장 비트, 자동복구·자동 해제 조건,
  비상정지, LED·부저, 시리얼 이벤트
