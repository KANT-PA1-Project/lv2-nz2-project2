# R20D 그리퍼 카메라 인지 연결 시험

## 앞쪽85mm 가정의 실제 관절 구동 시험 (2026-11-12)

사용자가 시험용으로 앞쪽85mm 장착을 허용했다. `config_assumed_front85*.json`은
원본 필수입력 템플릿과 구분된 **임시 시험 가정**이다.

- 팔 베이스: R20D 중심에서 `[0.085,0,0.187]`m, rpy=`[0,0,0]`.
  x85mm는 사용자 지정, y0/상부 알루미늄 프레임 높이187mm/같은 방향은 시험 가정.
- 차체는 고정, 실제 PhysX 관절 위치 제어로 팔을 이동한다. 카메라는 arm_link5의
  기존 CAD 장착변환을 상속하며 카메라 시점을 순간 이동시키지 않는다.
- 기존 HSV/ZXing/SignalRecorder 사용. 카메라640×480·30Hz, 바코드 확대 옵션2.
  강화학습·주행 체크포인트·실물 카메라/ROS는 실행하지 않는다.
- `v4` 자세는 명목 좌표 기반 FK 계획 후보다. 바코드 목표 거리12cm·손목롤0,
  신호등 목표 거리30cm. 도달/가림/검출은 실제 렌더로 검증해야 한다.
- 물리960Hz/PGS 후보, 관절 토크2Nm/속도0.4rad/s 및 기존 관절범위 유지.
  native/동일 physics-step FD 모두0.02rad/s 미만0.5초 정착 기준은 완화하지 않는다.
  접촉력 및 관절 피드백을 별도로 기록한다. 다른 물리 설정/실물 검증으로 일반화하지 않는다.
- 새 importer의 `Physics=physx` payload를 명시 선택한다. R20D CAD 내부의 접촉하는
  간략 충돌상자 때문에 섀시9링크 사이만 supplied CAD의 내부충돌 필터 지시를 적용한다.
  팔 자기충돌과 환경3cm 검사는 유지한다. 여러 벽이 합쳐진 static triangle mesh는
  전체 미로 상자 대신 각 면을 감싼 상자를 검사하며, convex hull은 나누지 않는다.

```bash
cd "$HOME/git/lv2-nz2-project2"
PYTHONDONTWRITEBYTECODE=1 python3 -B r12/experiments/r20d_perception/prepare.py \
  --config r12/experiments/r20d_perception/config_assumed_front85_v4.json \
  --out /tmp/r20d-front85-prepared-new
python3 -B r12/experiments/r20d_perception/launch.py \
  --config r12/experiments/r20d_perception/config_assumed_front85_v4.json \
  --prepared /tmp/r20d-front85-prepared-new \
  --out r12/reports/local/r20d-front85-new
```

`launch.py`는 실제 자식 프로세스 종료코드를 보존한다. 종료코드0만으로 성공시키지 않으며,
최종 `result.json`이 없거나 passed가 아니면 실행기 자체도 실패한다. 중단/실패 자료도 보존한다.
결과는 `r12/reports/local/r20d-front85-*/`와 옆 `.log`에 있다.
정착 실패·바코드 실패·팔 이동·신호 기록을 단계별로 판정해야 한다.
이 절의 설정은 기존 R20D 실제 장착 설계를 확정한 것이 아니다.

### 실행 결과: 전체 미통과

`r20d-front85-09`(v4/960Hz/PGS/기본 격자 바닥)의640×480·30fps 영상389프레임:

- 바코드3프레임 해독 후1.035초에P2-02저장, 실제 팔 관절 구동 시작.
- 3.602초에 팔 정착을 확인하고 신호등 준비 단계 진입. 바코드는 이동 후에도 유지.
- 관절1은약+0.151→−0.541rad, 그리퍼 카메라 영상도 실제 팔에 따라 변했다.
- 화면에 신호등과 열린 그리퍼가 보이지만 몸통 검색 성공0프레임. READY/12회기록은 미도달.
- 도착영상의 초록 HSV 연결요소에 파란 격자바닥/그리퍼가 합쳐지고, 위에서 보는 구도의
  빨강/노랑 연결요소 외접상자도 겹쳐 기존 `_stacked` 조건을 만족하지 못했다.
  이는 몸통 검색의 알려진 실패이며 색 판독 정확도를 측정한 결과가 아니다.
- 검색 실패가 반복돼 해당 시험만SIGINT중단. Kit가프로세스0으로끝냈어도
  최종결과부재를launch가실패로처리했다. `audit.json`에단계별판정/관절/접촉자료를추가했다.

`r20d-front85-10`의중립색바닥대조는초기정착중관절속도검사에서중단됐다.
중립색바닥으로검출문제가해결됐다고주장하지않는다. 재현용별도설정은
`config_assumed_front85_neutral_floor.json`이며, 이설정도prepare를새폴더에실행해야한다.
기본설정v4는09와같은기본격자바닥을사용한다. 바닥선택분기의최종코드는AST확인만했다.

01~08의importer/API/정착/바코드구도실패도각결과폴더에보존했다.
다음은관측자세/실제바닥·그리퍼외관과검색영역을검토할단계다.
**강화학습이필수인문제가아니다.** 기존HSV/색판독임계값을이번시뮬결과에맞춰바꾸지않았다.

## 구현 범위

R20D 섀시 + 기존 R12 팔/그리퍼를 별도 URDF로 조합한다. **차체와 바퀴는 고정**하고
팔 관절의 물리 구동으로 바코드 관측 자세에서 신호등 관측 자세로 이동한다.
실물 ROS/펌웨어·기존 `r12_sensor_sim.py`·기존 주행 정책을 바꾸거나 실행하지 않는다.
초기 템플릿은 코드 작성·정적 문법 확인만 수행했다. 이후85mm 가정 시험의 결과/실패는
위 별도 설정과 `r12/reports/local/r20d-front85-*/`의 결과 파일로 구분한다.

이 시험은 아래 순서를 위한 후보 코드다.

1. 바코드 관측 자세에서 실제 렌더 RGB를 ZXing으로 읽는다. 최근 5초 동일 코드 3회 확정.
2. 확정 바코드를 유지하고 설정한 팔 경유점을 순서대로 구동한다. 이동 중 신호등 기준은 잡지 않는다.
3. 마지막 관절 목표 오차 <0.01rad, native 속도와 동일 120Hz physics-step 차분 속도 모두 <0.02rad/s가 0.5초 연속이면 정지로 인정한다.
4. 그리퍼 카메라 RGB에서 HSV 몸통 검색·1초 소등 기준 확보 후 READY를 낸다.
5. 시험용 신호 발생기는 READY 후 1초 기다린 뒤 12회(ON 1초/OFF 1초)를 켠다.
   최종 3초 소등까지 `MissionSession`/tutor `SignalRecorder`로 기록한다.
6. 완료 기록·바코드를 보존하고 ROI를 해제한다. 추가 R 점등을 2초 영상에 남겨 기록이 보존되는지 검사한다.

fixture의 정답 색/코드/좌표는 **장면 생성과 결과 채점에만** 쓴다. 색 판독에는 RGB만,
정착 판단에는 실제 관절 피드백만 들어간다. 신호등의 실제 영상 위치를 정답으로 주지 않는다.
바코드 자세에서 신호등이 안 보였는지는 자동 확정하지 않으며 `barcode_saved.png`/영상으로 별도 확인한다.

## 필수 설정의 뜻

`config.json`의 `null`은 **정해지지 않은 실제 설계값/시험 자세**다. 값을 추측해 실행하지 않는다.

| 항목 | 입력할 값 |
| --- | --- |
| `arm_mount.xyz_m` | R20D 중심 아래 지면 원점에서 **arm_joint1 원점까지** x,y,z(m). +X 앞, +Y 왼쪽, +Z 위. 기존 275mm를 자동으로 더하지 않는다. |
| `arm_mount.rpy_rad` | R20D 기준 팔 베이스 roll,pitch,yaw(rad). 방향이 같다고 확정됐을 때만 `[0,0,0]`. |
| `spawn_xyz_yaw_rad` | 경기장 좌표에서 차체 x,y,z(m), yaw(rad). 원점이 지면이므로 불필요한 차체 높이를 더하지 않는다. |
| `barcode_arm_q_rad` | 시작 바코드 관측 자세의 arm_joint1..5 관절각(rad). |
| `tower_arm_waypoints_rad` | 신호등 자세까지의 5관절 경유점 배열. 마지막 항목이 정지 관측 자세. |

“중앙보다 앞쪽 장착”은 x>0이라는 방향 정보다. 앞쪽 몇 mm·높이 몇 mm·장착 각도가 정해져야
카메라 위치가 정해진다. 이 값들을 입력하기 전에는 준비 스크립트도 이유를 출력하고 종료한다.
현재 설정의 바코드 위치 `[0.08,0.65,0.45]`도 **시나리오 가정**이며 실제 부착 위치 확정이 아니다.
팔 자세는 `configs/r12_robot.json` 범위 안이어야 한다. R20D 장착 후 도달/충돌 여부는 추가 확인 대상이다.

## 실행 명령 (설정 입력 후)

Isaac 실행은 이 작업에서 수행하지 않았다. 팀의 로컬 실행 지침에 따라 한 실험씩 실행한다.
아래 준비/결과 디렉터리는 새 디렉터리여야 하며 기존 결과를 덮어쓰지 않는다.

```bash
cd "$HOME/git/lv2-nz2-project2"
python3 r12/experiments/r20d_perception/prepare.py \
  --config r12/experiments/r20d_perception/config.json \
  --out /tmp/r20d-perception-prepared-01

PYTHONDONTWRITEBYTECODE=1 "$HOME/pa-course/.venv-isaacsim/bin/python" -B \
  r12/experiments/r20d_perception/run.py \
  --config r12/experiments/r20d_perception/config.json \
  --prepared /tmp/r20d-perception-prepared-01 \
  --out "$HOME/git/lv2-nz2-project2/r12/reports/local/r20d-perception-01"
```

`prepare.py`는 시스템 libarchive로 Desktop 원본을 읽어 필요한 URDF/mesh/검사 자료만 새 폴더에 풀고,
기존 R12에서 팔·손가락·TCP만 복사한다. 구형 섀시·구형 상단 카메라를 중복으로 붙이지 않는다.
아카이브 내부 스크립트는 실행하지 않는다. 원본 아카이브, URDF, mesh, 설정과 실행 소스 해시를 저장한다.
설정/원본이 바뀌면 새 디렉터리로 다시 prepare해야 한다.

## 결과와 판정

- `camera.mp4`: **실제 그리퍼 장착 카메라** 640×480/20Hz 영상 및 단계 표시. 실사 카메라가 아니다.
- `barcode_saved.png`, `ready.png`, `signal_saved.png`: 각 단계 원본 영상.
- `frames.jsonl`: 시뮬 시각, 단계, 해독 결과, 관절 피드백, 판독 색/이유, 누적 순서.
- `mission_summary.json`: 인지 요구사항(L1/L2/L3 그룹·개수). 퍽 목록/파지/운반 계획이 아니다.
- `inputs.json`: 설정·소스 해시·준비 자료 이력.
- `result.json`: 12개 순서/바코드 정답 비교, 이동 중 바코드 보존, 완료 후 점등 무시 결과.

`passed`여도 **고정 차체 + 입력한 팔 자세 + 해당 렌더 장면의 인지 연결** 결과다.
신호등 실사 검증, 위치 오차 전 범위, R20D 주행 정책 호환성, 전체 미션 완료를 뜻하지 않는다.

## 실패하면 확인할 곳

| 실패 | 확인/수정 |
| --- | --- |
| `arm_mount... required` | 위 표의 실측/CAD 장착값 입력. 값을 임의로 채워 통과시키지 않는다. |
| `outside ... joint limits` | arm_joint1..5 순서·rad 단위·실제 가능한 자세 확인. 한계를 늘려 해결하지 않는다. |
| `Barcode acquisition timeout` | `barcode_saved` 전 영상에서 크기·정면 구도·실제 줄무늬 렌더 확인. barcode 위치/시작 관절각 조정. |
| `motion/settling timeout` | 관절 목표 도달·native/차분 속도·토크 한계 확인. 다른 장착에 R12 가정 동역학이 충분한지 검토. |
| `CAD clearance <3cm` | 팔/그리퍼와 장면 충돌체의 보수적 AABB 근접 정지. 경유점/장착/시작 위치 수정. 검사 제거 금지. |
| `Automatic preparation timeout` | 신호등 전체 3단이 보이는지, HSV가 몸통을 찾는지, 렌더 소등 밝기 확인. |
| `Signal recording failed` | 프레임 `reason`과 영상을 대조. UNKNOWN을 임의로 OFF로 치환하거나 정답 시간표를 사후 이동하지 않는다. |
| `Source ... changed` | 새 디렉터리로 prepare. 오래된 조합을 새 설정으로 실행하지 않는다. |

근접 정지는 CAD AABB에 기반한 보수적 외부 환경 검사이며 완전한 swept-volume 충돌 계획이 아니다.
PhysX 자기충돌을 활성화하지만 자기충돌 없는 경유점을 증명하지 않는다. 구동/관성도 후보값이다.
미로 벽은 유지하고 이전 회색 임시 패널·기존 바코드·가로 램프만 시험 장면에서 제외한다.

## 스워브 재사용과 남은 통합

스워브 역기구학 구조는 재사용할 수 있으나 R20D 반지름 0.033m, 모듈 중심 x/y=±0.124m,
조향 0도=네 바퀴 모두 +X 기준으로 파라미터/부호/조인트 순서를 맞춰야 한다.
R12 반지름 약0.0306872m, 모듈 ±0.1m와 CAD 조향 영점 설정을 그대로 쓰지 않는다.

**이 실험에서는 바퀴를 고정해 주행을 연결하지 않았다.** 기존 R12 학습 체크포인트의
관측/행동 계약 확인·R20D 주행 오차/안전 veto 검증, 이동 경로 계획은 별도 남은 작업이다.
추후 받은 제어 모델은 이 확인 후 기존 통합 런타임에 연결해야 한다.
현재 코드가 로봇을 자율 주행시켜 두 관측 장소를 찾아가는 것은 아니다.

## 2026-11-12 프로파일 앞끝160mm 대조

사용자가 말한 프로파일 끝은 **차체 팔 베이스 자리**다. 임시 x160/y0/z187mm,
rpy0이다. y0는 양쪽 레일 사이 브리지 판을 가정하며 실제 장착 확정값이 아니다.
차체 고정 시작 X를 +75mm 이동하여 v4와 동일한 world 팔 관측 위치를 유지한다.
그리퍼 손목/양 손가락은 명목 검은색으로 덮고 충돌형상은 유지한다.
`lv_module6_usd/scene.usda`의 바닥 visual quad와 OmniPBR만 참조한다.
원본 장면의 조명/퍽/선반/벽은 추가하지 않는다. 바닥 물리/조명/인지 규칙은 이전과 같다.
바닥·텍스처 원본 해시는 prepare와 run에서 대조한다.

```bash
cd "$HOME/git/lv2-nz2-project2"
python3 -B r12/experiments/r20d_perception/prepare.py \
  --config r12/experiments/r20d_perception/config_profile_end160_black_carpet.json \
  --out /tmp/r20d-perception-profile160-01
python3 -B r12/experiments/r20d_perception/launch.py \
  --config r12/experiments/r20d_perception/config_profile_end160_black_carpet.json \
  --prepared /tmp/r20d-perception-profile160-01 \
  --out r12/reports/local/r20d-profile160-01
```

동일 폴더가 이미 있으면 새 번호로 바꾼다. 성공 여부는 result.json과 실제 영상을 함께 확인한다.

### 프로파일160 실제 시험 결과 (2026-11-12)

| 결과 폴더 (`r12/reports/local/`) | 설정/확인 | 결과 |
| --- | --- | --- |
| `r20d-profile160-01` | black_carpet, 명목 바코드12cm | 영상258프레임, 읽힘0, 바코드8초 timeout, 종료1 |
| `r20d-profile160-02` | black_carpet_v2, 명목10cm | 영상259프레임, 읽힘0, 바코드8초 timeout, 종료1 |
| `r20d-profile160-tower01` | tower_only, 이전 신호등 관측자세 | 영상194프레임, 소등검색180프레임 모두 미검출, 준비6초 timeout, 종료1 |

세 시험에서 카펫/검은 그리퍼의 RGB 적용과 실제 팔 정착을 확인했다.
처음 두 시험은 바코드가 확정되지 않아 팔 이동 단계까지 가지 않았다.
세 번째는 `trial_mode=tower_only`로 **처음부터 신호등 관측 자세에서 정착**한다.
바코드는 주입하지 않으며 바코드 연결·팔 이동 시험 성공으로 집계하지 않는다.
`config_profile_end160_tower_only.json`을 위 명령의 config로 사용하고 새 준비/결과 폴더를 지정한다.

마지막 영상의 RGB HSV 성분은 R(322,247,62,47), Y(322,277,55,39),
G(322,301,48,33)로 각각 분리된다. 이전 파란 그리퍼/바닥과 G가 합쳐지던 문제는 사라졌다.
하지만 약56.6도 내려다보는 자세에서 성분 사각형이 R/Y=-17px, Y/G=-15px 겹쳐
기존 `_stacked`의 허용(-15.6px/-13.2px)을 벗어나 자동 위치 고정이 실패한다.
**색별 성분 분리 확인이지, 켜짐 판독·12회 기록 통과가 아니다.**
각 결과의 `camera.mp4`, `first_frame.png`, `failure.png`, `audit.json`,
`result.json`, `process_exit.json`에 근거를 보관했다. tower01에는 `body_diagnostic.json`도 있다
(해당 파일은 첫 프레임 성분, `audit.json`은 최종 실패 프레임 성분).
현재 다음 수정 대상은 관측 구도 및 바코드 렌더/해독 실패 원인이다.
실사 판정 임계값을 이 시뮬 실패에 맞춰 변경하지 않았다.

또한 prepare 단계의 외부 시간 제한을 run.py에 추가했다. 기존에는 Preparation 내부가
시간초과 이유만 반환해 전체 시뮬 예산까지 검색할 수 있었다. 지금은 설정 초과 시 실패 종료한다.
모든 시험 후 프로세스 종료를 확인했다. 커밋·푸시는 하지 않았다.
