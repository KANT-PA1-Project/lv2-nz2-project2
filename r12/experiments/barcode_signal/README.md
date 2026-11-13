# 신호등 판독 시험

## 저장소 반영 안내 (2026-11-13, feat/barcode-signal)

원본: `p2_perception/Claude outputs/tower_signal/` + `p2_perception/firmware/sign_tower_v3/`. 같은 PR 에 `r12/experiments/r20d_perception/` 포함.
아래 기록에 나오는 `reports/...` 결과·영상은 용량(약 3.9 GB)과 AGENTS.md(raw 로그·영상 커밋 금지) 때문에 **올리지 않았다**(원래 PC 에만 있음).

| 제외 | 이유 | 대체 |
|---|---|---|
| `reports/` 의 영상(mp4)·프레임 로그(jsonl)·화면 PNG·콘솔 로그·`imported/`·`sources/` | 실행 영상·raw 로그(약 780 MB) | 원래 PC 보관. **결과·감사·분석 스크립트는 `results/<실행 이름>/` 에 포함** (`result.json`, `audit.json`, `mission_summary.json`, `inputs.json`, `codex_audit_*.py` 등, B11~B16 은 `results/LINKS_B11-B16.txt` 의 원본 이름 폴더) |
| `vendor/` (zxing-cpp 바이너리) | 설치 패키지 | `pip install -r requirements-barcode.txt` |
| `validation_set_20261110/images/` (17 MB) | 사진 원본(LFS 미설정) | `labels.csv`·`labels.json` 의 `source`/`sha256` |
| `.serena/`, `__pycache__/`, `.pytest_cache/` | 캐시 | - |

**외부 자산(저장소 밖) — 실행 시 필요**
- 최종 조립 URDF: 설정 `assembly_dir`(원래 `~/Desktop/assembly_puck_slots.`). 다른 경로면 `NZ2_ASSEMBLY_DIR=<폴더>`.
  같은 모델이 `feat/32-r21-model` 의 `r12/assets/vendor/R21_URDF_USD/` 에 있음(관절 이름 동일, 경로 구성은 다를 수 있어 미검증).
- 바닥 USD: 설정 `floor_visual_usd`(원래 `~/git/p2_perception/lv_module6_usd/scene.usda`, 167 MB). 없으면 `NZ2_FLOOR_USD=` (빈 값 = 기본 바닥).
- YOLO 모델(`tower_yolo_signal.py`, `check_mission_perception.py`): 원래 `~/git/p2_perception/models/current.pt`. `--model` 로 지정.
- tutor `r12/src`(SignalRecorder 등): 기본 `~/git/lv2-nz2-project2/r12/src`, 다르면 `R12_SRC`.
- 공용 `r12/scripts` 수정 제안은 `scripts/r12_sensor_*.py`·`mission_scene_objects.py` 사본과 `diffs/` 로만 둠(공용 파일 미수정).


## 최종 조립 URDF 손목 카메라 시험 환경 (2026-11-12, Claude Code)

- 새 폴더 `sim_assembly/` 추가. `~/Desktop/assembly_puck_slots./assembly.urdf`(최종 조립본)로 로봇 모델을 바꾼
  바코드 → 팔 이동 → 신호등 Isaac 시험. tutor `r20d_perception` 흐름 사본이며 tutor 쪽·기존 scripts 는 수정 안 함.
- 핵심: 손목 카메라 640×480(실측 fx 607.9, 수평 55.5°), 차체 yaw 0(팔·센서가 base −X), 관측 위치 (734, 950)에서
  바코드 0.215 m(라벨 약 202 px)·신호등 0.353 m(내려다봄 35°, 손목 한계상 최소, 3단 약 184 px) 자세 후보.
- **B03 통과 (2026-11-12)**: 바코드 위치 (638.7, 710.6)에서 0.94 s P2-02 확정 → 차체 0.1 m/s 연속 이동+팔 동시 이동 → (734, 950)에서 12회 RGYRGYGRGRGG 정답, 810프레임 전부 정답. 실측 관절 한계 적용.
  영상 `reports/assembly-sim-B03/camera.mp4`. 상세·가정·한계: `sim_assembly/README.md`.

## 640×480 시뮬 인지 연결 시험 (2026-11-12)

실사 실행기와 같은 HSV/ZXing/미션 기록 함수를 사용하는 별도 시험:
**카메라 640×480·시뮬 시각 30Hz, 신호등 원본640, 바코드 실패 시 2배 확대 재시도**.
고해상도 카메라 스트림을 추가로 켜지 않는다. 2배 확대는 원본에 없는 세부 정보를 복구하지 않는다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 "$HOME/pa-course/.venv-isaacsim/bin/python" -B \
  scripts/test_mission_camera_isaac.py --auto-start \
  --width 640 --height 480 --fps 30 \
  --exposure 150 --wb 4600 --barcode-upscale 2 \
  --out reports/field640-new
```

출력 폴더는 새 경로로 지정한다. 기본 바코드는 P2-02, 정면15cm이며
`--barcode P2-04 --barcode-distance 0.15`처럼 시험 조건을 바꿀 수 있다.
바코드 저장 → 카메라 시점 전환 → 신호등 준비 → 12회/최종3초소등 → 완료 후 추가점등 무시를 확인한다.
`camera.mp4`, `frames.jsonl`, `mission_summary.json`, `result.json`, 입력/소스해시를 저장한다.

**미로 장면의 고정 카메라 시험이며 R20D·팔·실물 카메라는 포함하지 않는다.**
시점 전환은 물리 구동이 아니다. R20D 시험은 장착값/관측 자세가 입력돼야 별도로 가능하다.
`--exposure 150 --wb 4600`은 실사 비교용 메타데이터로 기록하며 RTX에 동등한 값으로 적용하지 않는다.
화각은 기존 fx400/640 기준 시뮬 가정이며 실제 D435 캘리브레이션이 아니다.
30Hz는 시뮬 관측 시각이다. 실시간 처리속도는 영상의 WALL FPS 및 결과의 wall 지표로 구분한다.
종료 중 충돌/비정상 종료는 인지 결과와 별도로 실패로 기록한다.

### 2026-11-12 실행 결과

`reports/20261112-field640-full-01/`에서 640×480/30Hz, 확대 옵션2, 정면 바코드15cm,
신호등30cm 조건을 한 번 완료했다. 바코드 P2-02는 시뮬0.4초에 확정,
신호 RGYRGYGRGRGG 12개 일치, 판독810/810 정답·UNKNOWN0이었다.
최종3초소등 후 추가 R점등54프레임 동안 기록을 보존했다.
전체929프레임 영상을 저장했고 외부 실행 감시가 프로세스 종료코드0을 확인했다.
첫 원본 프레임 재해독은 `candidate_upscale / decode_scale=2`로 성공해,
이번 조건에서 실제 후보영역2배확대 경로가 사용되는 것도 확인했다.
실행 전체 약31.6초(시작/종료 포함)였으며 Jetson/실물 처리 부하 측정은 아니다.

앞선1080p 짧은 시험은 영상 인코더 열기 실패1회 및 종료 중 malloc 오류1회로 실패 기록을 남겼다.
최종640 시험은 시스템 FFmpeg 저장과 Kit 기본 정리/종료 경로를 사용했고
외부에서 종료코드를 확인했다. 종료 오류를 인지 통과로 덮지 않았다.

## YOLO 없는 최적화 실행기 (2026-11-11)

신규 실행 파일은 `scripts/live_mission_optimized.py`다. PT/ONNX/engine·Ultralytics·Torch를
사용하지 않고 기존 HSV 색 기준과 Code128·미션 기록/배정 규칙을 재사용한다.
폴더 안의 기존 모듈이 필요하므로 실행 파일 하나만 다른 곳으로 복사하지 않는다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 "$HOME/yolo_env/bin/python" -B scripts/live_mission_optimized.py --auto-start --exposure 150 --wb 4600 --barcode-upscale 2 --out reports/live-optimized
```

150/4600은 예시다. 실제 확인된 노출/WB를 사용한다.
카메라·팔 고정 및 실제 소등 유지 → READY 출력 → 운영자가 신호 시작 순서다.
12회+마지막3초소등 확인후 신호기록보존·상자해제. 바코드미확정이면 바코드만해독하며
카메라를바코드로움직일수있다. 바코드까지완료하면L1/L2/L3요구량을저장하고관측종료.
`r`은새회차, `q`는종료, `v`는원본영상녹화. 수동준비는 --auto-start 대신 --no-auto 사용후 o/m.

### 실패 회차 자동 재시도 (2026-11-11)

`--auto-start`에서 신호 UNKNOWN·프레임 누락·12회 미달·횟수 오류가 나면 실패한
부분 순서를 버리고 자동으로 새 회차를 준비한다. 이미 확정된 바코드는 유지한다.
일반 점멸 사이 OFF는 1초이므로, 재준비는 몸통이 안정적으로 보이고 판독이 OFF인 상태가
**2초 연속** 유지되어야 한다(기존 2.5초에서 단축). 그동안 위치와 OFF 기준을 다시 저장한 후 READY를 출력한다.
카메라 누락·위치 흔들림·켜짐·UNKNOWN이 있으면 소등 대기시간을 처음부터 센다.
이는 종료 소등 3초 안에서 다음 회차 준비를 마치기 위한 후보값이며 실물 검증 전이다.
시계만으로 꺼짐을 보장하지 않으며, OFF 판독 자체가 틀리면 경계 확인도 틀릴 수 있다.

첫 색을 본 후 27초(12×2+3)가 지나도 유효한 기록이 완료되지 않으면 재시도로 보낸다.
이는 타임아웃 상한이며 매 27초를 새 회차 시작으로 가정하지 않는다.
첫 색이 전혀 보이지 않으면 READY 후 대기를 유지하므로, 영상만으로 시작하지 않은
시퀀스와 모든 점멸을 놓친 시퀀스를 구분하지는 못한다.
재준비 중 30초 탐색 제한에 걸리면 재준비만 다시 시도한다. 바코드 오류는 자동으로 지우지 않는다.
`--no-signal-retry`를 추가하면 기존 오류 정지 동작이다. 수동 --no-auto에는 자동 재시도를 적용하지 않는다.

실패 순서·원인·마지막 상세 판정은 `mission_*.jsonl`의 `kind: signal_retry`에 저장한다.
완료된 12회+최종 3초 OFF 기록은 기존처럼 보존하고, 성공 후에는 자동 재시작하지 않는다.
소프트웨어는 신호등 제어기에 재생 명령을 보내지 않는다. 장치가 자동 반복하지 않는다면
다시 READY가 나온 뒤 운영자가 **12회 시퀀스 전체를 재생**해야 한다.
실패한 회차에서 놓친 색을 복원하거나 중간부터 이어 세지 않는다.
12회 미달을 최종 소등 3초 시점에야 감지하면 이미 다음 회차가 시작될 수 있으므로,
재준비 조건이 맞지 않는 다음 회차는 건너뛰고 이후 긴 소등을 기다릴 수 있다.

### 줄인 작업

- 검색 기본10Hz, 고정후 검색종료. 소등/색판독은 매수신프레임에서 수행하고 0.25초 누락거부 기준은 유지한다.
- 신호등검색영상 가로최대640, 렌즈주변만RGB변환·기준5장보관. 세칸색띠확인은HSV1회변환공유.
- 바코드는원본BGR에서별도작업기본5Hz, 처리중작업이있으면추가큐를만들지않는다.
- GUI는기본10Hz만복사/그리기. --no-show면표시복사제거. 신호완료후에는표시에필요할때만작은영상생성.
- 선택검색영역 --barcode-roi X Y W H는원본픽셀단위이며흰여백을포함해야한다. 기본2초마다전체화면으로도검색한다.
- 준비조건·기록·결과JSONL형식은기존규칙재사용. 색판정임계값은이번에바꾸지않았다.

### 바코드 후보 잘라내기·회전 보정 (2026-11-11)

최적화 실행기의 바코드 해독 순서:
1. 원본 전체 화면을 Code128로 해독.
2. --barcode-upscale 2이면 전체 화면을 2배 확대해 재시도.
3. 둘 다 실패하면 수직 줄무늬의 수평 밝기 변화로 후보를 찾아 최대 6개 선택.
4. 후보 주변의 실제 영상 여백(가로 방향 양쪽 각각 후보 폭의 18% 또는 10px 중 큰 값,
   세로 양쪽 각각 높이의 15% 또는 4px 중 큰 값)을 포함해 자르고 회전을 수평으로 보정.
5. 후보 원본 크기와, 옵션이 2인 경우 후보 2배 확대에서 해독.
6. 후보도 실패하면 그 안의 밝은 종이 외곽을 Otsu 이진화·윤곽으로 찾고, 중앙을 포함하는
   라벨 경계 1개를 정제해 배경을 줄인 뒤 원본/2배 해독한다. 고정 비율로 막대 영역을 자르지 않는다.

실물 라벨 변경이나 YOLO/학습은 필요 없다. 가짜 흰 여백을 생성하지 않고 촬영된 주변을 포함한다.
1차 회전 보정은 줄무늬 묶음의 최소 회전 사각형 기준이다. 추가 라벨 경계가 사각형으로
잡히면 그 외곽으로 원근 보정하고, 그 외에는 회전 사각형으로 근사한다. 가림·휨·흐림은 복원하지 못한다.
후보 긴 축의 수평 성분이 0.65 이상인 구도만 처리하고, 확장 영역이 화면 밖이면 해당 후보를 건너뛴다.
후보 검사로 추가 처리 시간이 생기지만 기존 단일 비동기 작업 방식을 유지해 해독 대기열을 쌓지 않는다.

기존 실행 명령을 그대로 사용하며 --barcode-upscale 2를 유지하면 두 단계 확대를 모두 시도한다.
`mission_*.jsonl`의 barcode 이벤트에 `decode_stage`(full_native/full_upscale/
candidate_native/candidate_upscale/label_native/label_upscale/none), `candidate_count`,
`label_refinement_count`, `selected_candidate_quad`를 추가 기록한다.
reads.corners와 선택 후보 좌표는 원본 카메라 좌표로 복원하며, --barcode-roi 위치 오프셋도 반영한다.
기존 5초 내 3회 확정 규칙은 그대로 적용한다.
영상 비교(확대 옵션2): 모니터 배경01 139/139 유지,02 104/132→123/132,
03 147/147 유지,04 0/141→141/141. 실제 벽을 배경으로 손에 든 라벨 영상은
5프레임마다 뽑은179장에서55/179→114/179. 별도의 새 현장 영상으로 검증한 것은 아니다.
앞서 해독되던 프레임의 손실 여부와 모서리 원본 좌표 범위도 검사한다.
결과·프레임별 기록은 reports/20261111-barcode-label-refinement에 저장한다.
이는 정답 코드를 디코더에 주지 않은 자동 경로의 결과이나, 개발에 사용한 녹화 자료에 대한 검사다.
새 실시간 카메라·Jetson 처리량·다른 벽색/조명·훼손 라벨에서의 성공은 보장하지 않는다.

### 바코드 확정 규칙 (2026-11-11)

최근 **5초 안의 서로 다른 해독 프레임에서 같은 등록 코드가 3회** 나오면 확정한다.
연속일 필요는 없다. 시간창/횟수는 `scripts/mission_summary.py` 상단의
`BARCODE_WINDOW_S=5.0`, `BARCODE_MIN_MATCHES=3`으로 정한다.
미해독과 P2-01~04 이외 문자열은 투표하지 않으며, 기존 유효 표를 즉시 지우거나 오류로 정지하지 않는다.
다만 5초보다 오래된 표는 만료된다. 같은 프레임의 중복 박스는 한 표로 센다.

최근 5초에 서로 다른 등록 코드가 있으면 다수결로 고르지 않고 확정을 보류한다.
요구량 확정 전 다른 코드가 새로 읽히면 기존 바코드 확정도 해제하며, 충돌 표가 시간창에서
빠진 후 한 코드의 유효 표 3개가 남아야 다시 확정한다. 미해독만으로 확정 결과를 지우지는 않는다.
미등록 문자열도 `mission_*.jsonl`의 원래 reads에 그대로 남긴다.
전체 결과가 ready가 된 뒤에는 기존처럼 관측 결과 반영을 종료한다.
최적화 실행기는 촬영 시각 capture_s를, 기존 실행기 연결부는 전달 시각 t_s를 사용한다.
로그 재생은 capture_s가 있으면 그 값을 우선하므로 재생 속도가 시간창을 바꾸지 않는다.

### 바코드 결과 형식

`configs/barcodes.json`의 barcodes 배열을 정렬 정의의 원본으로 사용한다.
각 항목은 barcode/sortOrder/sortLabel이며 P2-01·03은 ASC/오름차순, P2-02·04는 DESC/내림차순이다.
실제 확정된 코드만 결과의 barcodes 배열에 넣는다. 미확정이면 빈 배열, sortOrder/sortLabel은 null이다.
예를 들어 P2-01 확정시 결과에 다음 필드를 포함한다.

```json
{
  "barcodes": [
    {"barcode": "P2-01", "sortOrder": "ASC", "sortLabel": "오름차순"}
  ]
}
```

같은 메타데이터를 최상위 sortOrder/sortLabel에도 제공하며 기존 barcode/direction은 호환용으로 유지한다.
미션 정렬도 이 설정의 sortOrder를 사용한다. 동률 R→Y→G는 유지한다.
GUI에는 ASC/DESC를 표시하고 한국어 sortLabel은 JSON/터미널 결과에 남긴다.
설정은 시작시 로드하므로 변경후 프로세스를 재시작한다. ROS 메시지 발행 구현을 추가한 것은 아니다.

### 실시간 FPS 표시

최적화 실행기 화면 첫줄에 최근2초실측 `FPS RX / SIG / GUI`를 표시한다.
RX는프로그램이수신한컬러프레임, SIG는고정후렌즈색판독호출, GUI는실제화면갱신의초당횟수다.
카메라설정 --fps값을그대로표시하는것이아니며센서가내부생성한전체프레임률도아니다.
기본GUI제한은10Hz이므로RX/SIG가30이어도GUI는약10일수있다.
고정전과신호완료후SIG0은정상이다. 시작0.5초는집계대기라0으로표시한다.
--no-show에서는같은수치를터미널에1초마다출력한다.
종료시 runtime_metrics.json에 last_window_fps/fps_window_s/fps_event_counts도저장한다.
FPS추가후 --help로딩만확인했고실제카메라표시는아직미검증이다.

### 해상도와 출력

기본은기존과같은640×480/30fps다. 바코드화질비교에는위명령에 `--width 1280 --height 720`을추가할수있다.
바코드는원본해상도, 신호등은가로640으로축소하되화면비율은유지한다.
로컬D435지원프로필조회에서640×480/30과1280×720/30 BGR8지원확인. 고해상도실제수신·해독성공은별도확인이필요하다.
해상도를바꾸면촬영/전송부하는증가할수있으므로최적화효과와별도로평가한다.
표시·탐색·해독주기는파일상단기본값또는 --display-hz/--search-hz/--barcode-hz로조정한다.

`reports/live-optimized/`에 다음을 저장한다.

- `mission_summary.json`: 최신상태·순서·횟수·바코드·장소별그룹/개수.
- `mission_*.jsonl`: 회차별관측·준비·오류/초기화 기록.
- `signal_record_*.json`: 완료신호의독립보관본. r로초기화해도남음.
- `runtime_config*.json`: 실행옵션·소스해시.
- `runtime_metrics.json`: 종료시처리시간/바코드시간의중앙값·p95. CPU/RAM사용률측정치는아니며GUI/프레임수신시간은처리시간에서제외된다.

이파일은ROS노드가아니다. Python에서검출/기록/판단을먼저확인하고,ROS이식후에는Jetson의메시지전송·동시노드까지포함해자원사용량을재측정한다.
기존 `check_mission_perception.py`는여전히YOLO실행기를호출하는별도점검도구다. 위최적화실행기는해당자동채점명령에아직연결하지않았다.
검사범위: --help 로딩과지원프로필조회. 신규실행기의카메라스트리밍·실물12회·회귀시험·속도비교는실행하지않았다. 실제성능향상수치는아직없다.

## 탐색 전 미션 요구량 요약 (2026-11-10)

신호등 12회와 바코드만으로 정렬 방향, L1·L2·L3별 그룹·퍽 개수를 출력한다.
개별 퍽 ID 할당과 Y 그룹의 원통/직육면체 선택은 실제 퍽 목록 확보 후 수행한다.
미션 PDF v5 3쪽 기준: P2-01/03 오름차순, P2-02/04 내림차순, 동률 R→Y→G.
예: R4/Y2/G6 + P2-01이면 L1=Y 2개, L2=R 4개, L3=G 6개다.

### 신호 기록 완료 후 자동 해제 (2026-11-11)

12회 기록만으로 즉시 종료하지 않고 마지막3초연속소등까지 확인한다.
12번째 점등을 잡아도 아직 저장 완료가 아니다. 이때 화면에 `12/12; WAIT final OFF; KEEP STILL`,
소등 중에는 `12/12; final OFF 1.2/3.0s; KEEP STILL`처럼 대기 상태를 표시한다.
`SIGNAL SAVED; MOVE TO BARCODE`와 터미널 `SIGNAL COMPLETE: saved; camera may move to barcode`가
나온 뒤 카메라를 바코드로 이동한다. 마지막 점등/최종 소등 확인 중 이동하여 고정 칸을 잃으면
실패 회차로 처리한다. 완료된 recorder는 후속 점등/UNKNOWN/프레임 간격 입력을 무시하며,
완료된 기록의 초기화는 새 회차를 명시적으로 시작하는 r 동작으로 구분한다.
완료되면 신호 순서/횟수를 `signal_record_<실행ID>_<회차>.json`으로 따로 저장하고
상자·OFF기준을 해제한다. 그 뒤에는 신호등을 다시 찾거나 색을 추가로 읽지 않는다.

바코드가 없으면 화면 상태는 `waiting_barcode`다. 이때는 카메라를 바코드 쪽으로 옮겨도 된다.
바코드 해독이 확정되면 기존 신호 기록으로 장소별 요구량을 계산하고 `ready`가 된다.
새로 신호등을 잡으려면 `r`을 누른다. 자동 모드는 새 회차의 소등 유지 상태에서 자동 준비,
수동 모드는 o/m으로 준비한다. 이전 회차의 별도 signal_record 파일은 지우지 않는다.
프로그램을 다시 켰을 때 이전 신호를 자동 복원하는 기능은 아직 없다.
이번 변경은 --help 로딩만 확인했으며 실제 카메라/CPU회귀 시험은 실행하지 않았다.

### 키 없이 자동 준비 (2026-11-11)

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 "$HOME/yolo_env/bin/python" -B scripts/live_tower_mission.py --auto-start --exposure 150 --wb 4600 --barcode-upscale 2 --out reports/live-mission
```

`--auto-start`는 READY까지 운영자가 신호등 소등과 카메라 고정을 유지한다는 운용 조건을 선언한다.
영상이 어둡다는 이유만으로 소등이 보장되는 것은 아니다. 이 조건을 유지할 수 없으면 수동 모드를 사용한다.
`--no-auto`와 함께 지정할 수 없다. 노출150/WB4600은 예시이며 실물에서 확인한 값을 사용한다.

- 위치를 찾고 OFF 후보 판정·색띠 잔존을 연속1초 확인한다. 첫 상자 대비 흔들림3px 이내, 프레임 간격0.25초 이내가 필요하다.
- 마지막5장의 기준 프레임으로 고정하고 현재OFF 확인 후 기록기를 자동 준비한다. o/m은 필요 없다.
- 터미널에 `READY: automatic preparation complete; operator may start signal`이 뜬 뒤 운영자가 신호를 시작한다.
- `--prepare-timeout 30`이 기본이다. 시간 내 준비되지 않으면 오류를 유지하고 READY를 보내지 않는다.
- 시작시점부터 이미 점멸 중인 상황을 지원하는 기능이 아니다. READY 이전 신호는 기록하지 않는다.
- 12회 신호와 마지막3초소등이 확인되면 신호등 상자/기준을 즉시 해제하고 판독을 멈춘다. 바코드가 없어도 신호 기록은 보관한다. 바코드까지 확인되면 요구량을 확정하고 바코드 결과 반영도 멈춘다.
- `r`은 새 회차를 시작한다. 자동 모드에서는 다시 소등을 유지하고 READY를 기다려야 한다. 공식 경기의 오류 회차를 이어가는 용도가 아니다.

JSONL arm에는 `confirmed_off=prestart_off_contract`와 안정시간·기준장수·운용조건을 남긴다.
재생기는 기존 수동 operator_o와 새 자동 운용조건 근거를 구분한다.
이번 추가분은 --help 로딩만 확인했다. 자동화 후 실물·시뮬 시험 및 CPU회귀검사는 미실행이다.
아래87개 결과는 이전 버전의 검사 기록이며 이번 자동화 검증 결과가 아니다.

### 수동으로 실제 카메라에서 실행

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 "$HOME/yolo_env/bin/python" -B scripts/live_tower_mission.py --mission --no-auto --exposure 150 --wb 4600 --out reports/live-mission
```

실행기는 두 종류다. `live_tower_mission.py`는 카메라 좌표 없이 화면 전체 HSV로 위치를 찾는 미션 확인기다. `live_tower_check.py`는 카메라 자세를 입력받는 투영·주변 검색 확인기이며 `--cam-z`가 필수다. 두 검출 경로의 결과를 동일한 검증으로 취급하지 않는다.

2026-11-10 실행 점검: 실제 D435의 640×480 RGB 30프레임 수신 및 정상 종료 확인. GUI 표시·소등 기준·색 판독·바코드·12회 미션 성공을 확인한 시험은 아니다. 시작 점검 후 `Stopped before mission requirements completed`가 기록되는 것은 준비키를 누르지 않고 종료했기 때문이다.

150/4600은 실행 예시이며 실물 보정 완료값이 아니다. 실제 정한 노출/WB로 바꾼다.
카메라와 팔을 고정하고 실제 소등을 확인한 뒤 `o` → 수동 고정 완료 → 현재 OFF에서 `m` →
준비 완료 출력 후 운영자가 첫 신호를 시작한다. 바코드는 별도 작업에서 약 5 Hz로 해독하며
최근 5초 안에 같은 등록 코드가 3회 나오고 다른 등록 코드와 충돌하지 않을 때 채택한다. 신호 기록이 완료되면 고정 상자를 해제하고, 바코드가 없으면 바코드 해독만 계속한다.
영상의 자동 소등 후보 판정만으로는 미션을 시작하지 않는다. 수동 소등 확인 또는 위의 시작 전 소등 운용조건이 필요하다.

- GUI: 방향, 기록 수, 완료 후 장소별 그룹·개수.
- `mission_summary.json`: `status=ready`일 때만 유효한 `locations`를 제공한다. 그 외에는 빈 목록이다.
- `mission_<시각>.jsonl`: 바코드 관측, 준비, 매 판독 프레임, 초기화·오류 기록.
- `r`: 새 회차 초기화 및 소등 기준 해제. 공식 경기 오류 후 같은 회차를 수동 복구하는 용도로 쓰지 않는다.
- 신호 UNKNOWN·0.25초 초과 프레임 간격은 신호 오류다. 바코드 미등록 문자열은 무시하고, 등록 코드 충돌은 확정을 보류한다. 요구량 확정 후에는 관측을 종료한다.
- 고정 칸에 두 색띠가 남아 있는지 확인해 부재를 거부한다. 물체 신원 확인이나 이동 추적은 아니다.

### 실시간 검색 색 범위 보정 (2026-11-10)

카메라 확인기는 scripts/tower_live_hsv.py를 사용한다. 소등 렌즈 검색용 후보 HSV범위는
R=0~6/165~179, Y=7~38, G=39~110, S>60/V>25다. 색띠 잔존검사도 같은 범위를 쓴다.
기존 검색이 실패한643×448 화면에서 보정후(144,107)-(190,206) 검색과 OFF판독을 확인했다.
해당1장을 보정에 사용했으므로 독립 실사 정확도/실시간 성공을 뜻하지 않는다.
켜짐색판정·tutor투영검색·오토라벨검색은 그대로다.
변경후 실행중창을q로 닫고 같은명령으로 재실행, 소등에서 상자확인→o→OFF확인→m 순서다.
전체회귀검사와 실제12회 점멸은 이번보정후 미실행이다.

### 카메라 없이 확인

```bash
PYTHONDONTWRITEBYTECODE=1 "$HOME/yolo_env/bin/python" -B -m pytest tests -q -p no:cacheprovider
PYTHONDONTWRITEBYTECODE=1 "$HOME/yolo_env/bin/python" -B scripts/replay_mission_summary.py reports/20261110-mission-summary/P2-01-synthetic.jsonl --out /tmp/p2-mission-replay.json
```

검사 87개 통과. 가상 카메라/GUI로 실시간 루프의 준비→12회→3초 소등→요약과
신호등 부재 오류를 검사했고 저장 로그 재생 결과도 일치한다.
4개 실제 바코드 이미지 해독과 합성 시간표 재생은 통과했다.
`reports/20261110-mission-summary/validation.json`에 소스 해시·한계를 기록한다.
노트북 CPU 요구량 계산 중앙값 약 0.002 ms, 작은 고정 ROI 잔존 검사 약 0.037 ms,
바코드 원본 이미지 해독 중앙값 2.4~4.3 ms다. 전체 실시간 처리량이나 Jetson 수치가 아니다.

실물 카메라/Jetson/Isaac 실행은 이번 검증에 포함되지 않는다.
기존 신호등 색 판정 기준과 알려진 실사 실패는 그대로이며 이번 요약 검증으로 해결된 것이 아니다.
이 경로는 HSV·Code128 해독·횟수 계산이므로 양자화할 신경망은 없다.
추후 신경망을 도입하고 처리량 측정상 필요할 때 양자화를 검토한다.
tutor 원본은 읽기만 하며 `R12_SRC`로 다른 tutor 소스 경로를 지정할 수 있다.

## 범위

- 작업 폴더: `$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal`.
- YOLO 없이 고정 신호등의 렌즈 영역을 카메라 자세로 투영하고, 밝기·흰 포화 면적으로 R/Y/G/OFF/UNKNOWN을 판정한다.
- tutor는 미로 자산·카메라 리그·SignalRecorder를 읽는 용도다. 여기의 연결 수정본은 tutor에 적용되지 않았다.
- `--field`는 실제 tutor 미로 벽·박스를 불러온다. 임시 회색 판과 바코드, 예전 가로 램프는 비활성화한다.
- 로봇 시작 기준은 `(0.734, 0.950, 0.008 m, yaw 180도)`로 고정한다. 로봇 제어·주행·팔 도달성 시험은 아니다.
- 실사 고정 노출 정답 영상 검증 전이다. 시뮬 결과를 실물 정확도나 실물 완료로 표시하지 않는다.

## 신호등 개선판 (2026-11-10)

**후보 규칙, 실사 정답 영상 검증 전.** 바코드를 제외하고 다음을 수정했다.

1. 실제 렌즈 몸통을 따로 표시한 실사 ROI를 사용한다. 단순 전체 라벨 3등분을 비교 기준으로만 유지한다. 시뮬/실제 자세 입력 경로는 원통 앞쪽 내부 패치를 사각형 폴리곤으로 투영해 롤이 있어도 덮개/배경 혼입을 줄인다.
2. 한 칸의 흰 포화 면적 ≥0.30, 다른 두 칸은 각각 ≤0.10인 경우 우선 선택한다. 여러 칸이 포화되면 강제로 선택하지 않는다. 후보값이며 독립 실사 영상 검증 전이다.
3. 확실한 꺼짐 3프레임의 RGB 중앙값을 기준으로 비교하는 API를 추가했다. 밝기가 40 이상 상승했거나 기존에는 없던 흰 포화가 생긴 픽셀만 켜짐 증거에 쓴다. 자동 예측 OFF나 다른 자세/노출의 사진으로 기준을 생성하지 않는다.
4. 꺼짐 기준은 ROI/영상 크기/카메라 자세/내부 파라미터/노출 식별값에 묶는다. 기준이 맞지 않으면 UNKNOWN이다. 호출자는 노출을 실제로 고정하고 정확한 식별값을 전달해야 한다. 이 API가 하드웨어 노출을 설정하거나 자동 측정하는 것은 아니다.

### 실사 비교 결과

같은 seed33 표본 48장 중 3단이 보이는 46장을 사용했다. 색 정답과 몸통 영역은 어시스턴트가 시각 확인한 임시 주석이다. 새로운 독립 검증 세트가 아니다.

| 방법 | 일치 | UNKNOWN | 다른 상태 |
|---|---:|---:|---:|
| 기존 라벨 3등분 + 이전 규칙 | 35 | 8 | 3 |
| 렌즈 몸통 ROI 보정 + 이전 규칙 | 41 | 2 | 3 |
| 렌즈 몸통 ROI 보정 + 흰 포화 우선 | 43 | 0 | 3 |

R13/Y4/G2는 모두 일치했고, OFF27 중 3장이 Y로 판정됐다. 같은 자세/고정 노출의 확실한 꺼짐 기준 영상이 없어 **실사에서 기준 비교를 평가한 사진은 0장**이다. 남은 OFF→Y 3장이 기준 비교로 해결됐다고 표시하지 않는다. 원본 라벨과 이미지 파일은 수정하지 않았다. 몸통 주석은 테스트용이며 자동 위치 검출기나 실제 카메라 보정을 대체하지 않는다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_photos.py --out reports/20261110-signal-improved-real-seed33
```

새 seed 표본에는 새 렌즈 몸통 주석이 필요하다. `--roi-annotations FILE.json`으로 지정하며 이미지 SHA256이 다르면 거부한다. seed33의 좌표를 다른 사진에 재사용하지 않는다. 초기 진단 기록 `20261110-real-photos-seed33`은 보존하고 개선판 결과는 위 별도 폴더에 쓴다.

### 추가 실사: 이전 표본과 중복 없는 seed34

이전 seed33와 이미지 SHA256 중복이 없는 **48장**을 추가 진단했다. 기존 촬영 그룹에서 캡처 번호 간격 최소10인 40장, 이전에 쓰지 않은 어두운 `tower_light_on_3` 그룹에서 간격 최소20인 8장이다. 번호 간격은 시간 간격이 아니며, 별도 환경의 독립 검증을 뜻하지 않는다. 판정 코드 SHA256을 표본 선택 시 고정했으며 임계값은 바꾸지 않았다.

| 방법 | 일치/채점 | UNKNOWN | 다른 상태 |
|---|---:|---:|---:|
| 기존 라벨 3등분 + 이전 규칙 | 31/42 | 8 | 3 |
| 수동 렌즈 ROI + 이전 규칙 | 36/42 | 6 | 0 |
| 수동 렌즈 ROI + 현재 규칙 | **42/42** | 0 | 0 |

색별 R11/Y6/G9/OFF16. 흐림·잘림·위치 불명확 6장은 삭제하지 않고 미채점으로 기록했다. 42장 중 번짐 때문에 렌즈 경계가 불확실한 수동 ROI 4장이 포함되며, 이를 제외해도 38/38 일치다. 색 정답도 예측 전 육안 확인한 임시 정답이다. 밝기4배 이미지는 어두운 경계 검토용으로만 쓰고 판정에는 원본 RGB를 넣었다. **자동 위치 검출 정확도·실물 12회 기록 성공률은 아니다. 기존 seed33의 OFF→Y 3건은 미해결**이며 실제 동일 자세/노출 OFF 기준 비교는 여전히0장이다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_photos.py --out reports/20261110-signal-real-unseen-seed34 --roi-annotations references/tower-photo-body-axis-seed34.json
```

기존 `samples.json`으로 재평가한다. 결과는 해당 폴더의 `summary.json`, `audit.json`, `predictions.jsonl`, `photo_*_prediction.jpg`. 표본 준비 옵션 `--exclude-samples PREVIOUS/samples.json --min-frame-gap 10`은 이전 경로/해시·인접 캡처 번호를 제외한다. 새 표본에는 별도 육안 정답과 해시가 연결된 ROI 주석이 필요하다. CPU56/56 통과.

### GUI: 바코드 없이 꺼짐 기준까지 포함한 전체 점멸

```bash
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera start --renderer PathTracing --emissive 0.5 1 2 --off-reference --gui --out reports/run-signal-improved-start-01
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera random --random-views 5 --seed 33 --renderer PathTracing --emissive 1 --off-reference --gui --out reports/run-signal-improved-random-01
```

두 명령은 하나가 종료된 다음 다른 하나를 실행한다. 출력 폴더는 새 이름을 쓴다. 조건마다 점멸 정답 시간표 시작 전에 램프를 OFF로 명령하고 3프레임 기준을 따로 저장한다. 초기화 캡처/기준 캡처는 채점하지 않으며 정답 프레임을 사후 이동하지 않는다. 모든 채점 RGB가 단색/검정이면 오류로 중단한다.

`off_reference_*.png/npz`에 기준 RGB·카메라·식별값을 저장한다. `frames_*.jsonl`에 원본 밝은 면적, 기준보다 변화한 면적, 기준 없는 새 규칙/이전 후보의 같은 ROI 판정을 함께 기록한다. summary의 `off_reference_used`와 `off_reference_frames == 3`을 확인한다. 이것은 시뮬 OFF 명령에 따른 기준 검증이며 실제 고정 노출 기준 영상 검증은 아니다.

### 개선판 실행 결과

- `reports/20261110-signal-improved-gui-start`: GUI PathTracing, 시작 구도 발광 0.5/1/2의 3조건 모두 570프레임·12회 순서 일치, 원본/채점 오판0.
- `reports/20261110-signal-improved-gui-random`: 같은 미로·로봇 고정, seed33 카메라 5시점 모두 570프레임·12회 순서 일치, 원본/채점 오판0.
- 합계 **8조건·4,560프레임 오판0, 순서8/8 일치**. 각 조건에 OFF 기준3장 사용. 바코드 제외.
- CPU 최신 **56/56 통과**. 기준 확인/문맥 불일치·새 흰 포화/밝은 OFF·색광 상승·다중 포화·ROI 마스크·물리 렌즈 투영 중심을 검사했다.
- 같은 보정 ROI의 이전 후보 규칙도 이 시뮬 조건들은 통과했다. 시뮬 결과는 새 규칙의 실물 우수성을 증명하지 않는다.
- 코드/주석 SHA와 종합 결과는 `reports/20261110-signal-improvements-validation.json`, 세부 비교는 실사 폴더 `predictions.jsonl`과 각 GUI 폴더 `frames_*.jsonl`에 있다. GUI 실행은 종료했다.

실사 몸통 주석을 보정한 비교는 35→41→43/46이며, 실제 촬영에서 OFF 기준 비교 평가0장·OFF→Y3장 미해결 상태는 유지한다. 향후 새로운 사진에서는 새 이미지용 ROI 주석이 필요하다. 실제 자동 검출기 정확도나 실제 12회 점멸 성공률로 해석하지 않는다.

### 추가 GUI 시험: 새 구도·연속 같은 색·위치 오차

`reports/20261110-signal-gui-seed34-batch/validation.json`에 결과를 모았다. PathTracing GUI, 미로 벽, 바코드/임시 패널 제외, 발광1, seed34, 순서 `RRYYGGRGYRGY`(각 색4회), 조건마다 확인된 시뮬 OFF기준3장. 판정 코드/임계값 변경 없이 실행했다. 0cm의 첫2구도를 1/2/3cm에서도 똑같이 사용했고 0cm에만 세 번째 구도를 추가했다.

| 수평 위치 오차 | 12회 순서 통과 | 원본 프레임 오판/전체 | 실패 구도 증상 |
|---|---:|---:|---|
| 0cm | 3/3 | 0/1710 | 없음 |
| 1cm | 1/2 | 160/1140 | 빨강·노랑에서 두 칸이 밝아 UNKNOWN, 기록 중단 |
| 2cm | 1/2 | 160/1140 | R→Y 80, Y→G 80 |
| 3cm | 1/2 | 240/1140 | R→Y 80, Y→G 80, G→OFF 80 |

총9조건·5,130프레임. 이는 시뮬 정답 시간표와 원본 전 프레임 비교다. `summary.json`의 `frame_accuracy`는 전환 주변 프레임을 제외하므로 이 표와 분모가 다르다. 실패를 정상 통과로 바꾸거나 정답을 사후 이동하지 않았다. 각 프로세스 exit0은 실행 완료만 뜻하며 성공 판정은 `sequence_match`/`passed`로 한다. 4개 GUI 실행 후 정상 종료했다.

**해석/수정 방향**: 정확한 자세 투영에서는 순서 기록이 되지만, 한 높은 시점은 위치 오차1cm부터 실패했다. 판정 칸이 이웃 렌즈로 밀리는 문제이므로 색 임계값 완화로 해결하면 잘못된 색을 더 쉽게 채택할 수 있다. 투영 위치 주변에서 실제 렌즈 몸통/경계를 영상으로 맞추는 보정과, 보정이 불확실하면 판독을 거부하는 검사가 다음 작업이다. 카메라 내부값·장착 변환·실제 렌즈 높이도 보정해야 한다. 이 표는 고정 seed의 수평 오차 방향만 시험한 것이며 일반적인 허용오차 규격이나 실제 팔 도달성 결과가 아니다. 실사 정답 영상 검증 전 상태를 유지한다.

재실행(한 번에 하나씩 실행하며 새 출력 폴더를 사용):

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
for cm in 0 1 2 3; do
  views=2
  if [ "$cm" = 0 ]; then views=3; fi
  PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_isaac.py --field --camera random --random-views "$views" --seed 34 --renderer PathTracing --emissive 1 --off-reference --gui --sequence RRYYGGRGYRGY --pose-error-m "0.0${cm}" --out "reports/recheck-seed34-error${cm}cm-01"
done
```

`--pose-error-m`은 카메라 영상은 유지하고 판정에 전달한 수평 위치만 틀리게 만든다. 실패 시 `debug_*.png/npz`, `frames_*.jsonl`의 truth/predicted/reason과 `camera_world`/`camera_projection_used`를 확인한다. ROI가 벗어났으면 위치/투영을 고치고, ROI가 정확한데 색만 실패하면 실제 고정 노출 정답 영상으로 포화·반사·밝기 기준을 검토한다. 근거 없이 1.8배/0.30 임계값을 낮춰 통과시키지 않는다.

### 별도 실사61장 검증 (validation_set_20261110)

원본 이미지61장과 labels.json/csv는 수정하지 않았다. R16/Y15/G14/OFF16, 파일 해시61개 모두 고유하며 이전 seed33/34 96장과 SHA256 중복0. 색 라벨은 시각적으로 일관되지만 제어기 로그 정답은 아니다. 렌즈 몸통을 예측 전에 수동 표시한 뒤 기존 후보 규칙을 고정해 평가했다.

| 구분 | 결과 |
|---|---|
| 수동 렌즈 ROI 평가 가능54장 | 정답51 / UNKNOWN2 / 오판1 |
| 61장 전체 출력 | 정답51 / UNKNOWN9 / 오판1 |
| ROI 확보 불가7장 | 어두운06/07/13/14/20/52, 원본 오른쪽 일부 잘린50 |
| val22 OFF→Y | 노랑 렌즈 밝은 비율0.15175로0.15 기준 초과 |
| val48 G→UNKNOWN | 녹색 번짐으로 노랑 칸 흰 포화0.333, 초록1.0; 단일 흰 칸 우선조건 불충족 |
| val54 OFF→UNKNOWN | 꺼진 R/Y 밝은 비율0.160/0.182로 두 칸 켜짐 후보 |

어두운6장의 색 라벨은 유지했고 칸 경계가 불확실하므로 ROI를 임의로 만들지 않았다. 50은 G가 보이나 원본1280px 오른쪽에서 신호등 일부가 잘려 전체3단 위치 평가에서 제외했다. 33/36/56의 약한 흐림은 유지했다. 41/43/44는 시각상Y가 맞고 퍽 폴더 이름을 정답으로 쓰지 않았다. 실제 labels.json에서 1026_tower 장면은41/43/44/56의4장이다. 34와53은 서로 다른 폴더이므로 전환3프레임 짝이 아니며, 29/53이 동일폴더0090/0087이다. 49/55도3번호 차이지만 카메라가 이동했으므로 OFF 기준으로 묶지 않았다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_photos.py --out reports/20261110-real-validation61 --roi-annotations references/tower-photo-body-axis-validation61.json
```

결과는 `reports/20261110-real-validation61/{summary.json,audit.json,predictions.jsonl,failures-contact.jpg}`. 51/54는 **수동으로 정확한 칸 위치를 준 조건의 색 판독**이며 자동 검출 정확도가 아니다. 동일 자세/노출의 실사OFF 기준 비교는0장이다. 다음 작업은 예상 투영 주변에서 실제 몸통/렌즈 경계를 찾아 정렬하고 불확실한 정렬을 거부하는 단계, 실제 고정노출 OFF기준 확보, 번짐/반사에 대한 별도 후보 비교다. HSV 분할은 학습 없는 후보일 뿐, 포화·어둠·가림에서 몸통 찾기 성공이나 검색창만큼의 허용 위치오차를 보장하지 않는다. 튜닝한다면 이 고정 결과와 새 평가를 분리한다. CPU56/56 통과. 후보 규칙, 실사 정답 영상 검증 전.

### 현재 보완판(v2): 크기 고정·색띠 경계·ROI 계산

**카메라/팔을 움직이거나 카메라의 월드자세를 추정하는 기능이 아니다.** 제어쪽 카메라자세와 알려진 신호등치수로 예상 영역을 만들고 그 주변±60px에서 영상 속 렌즈 ROI만 이동한다. 전체화면 임의위치 검색, 일부렌즈만보임, 최초점등상태에서의획득은지원하지않는다. 확실한OFF에서3단배열을찾고같은카메라/노출을유지해야한다.

변경:

1. **크기검색 제거:** 0.85~1.15배 선택을 없애고 투영크기1.0만 사용한다. 작은패치가색일치점수로선택되는경로를제거했다. 크기입력이크게틀릴경우자동으로맞추는기능은없다.
2. **색띠경계 검사:** R→Y, Y→G 색증거의 방향성Sobel경계를 점수/최소조건에추가했다. 내부색만맞고색띠경계가지지되지않으면거부한다. 몸통외곽의완전한분할기라는뜻은아니다.
3. **ROI계산:** 검색HSV는검색창, 판독HSV는렌즈3칸을감싼영역, 밝기/포화/OFF차분은각렌즈ROI에서계산한다. 원본영상크기·문맥·ROI동일성검사는유지한다. 색임계값과판정수식은변경하지않았다.

결과: 기존과같은GUI PathTracing12조건(0/1/2/3cm seed34,3cm seed35) 모두12회순서일치,6,840원본프레임오판0. 모든검색크기는1.0. 신호등부재30/30 UNKNOWN. CPU69개통과,최적화전후무작위80조합출력완전일치. 기존61장색판독도54장중51정답/UNKNOWN2/오판1로이전결과와완전히같다. 전체심프로세스정상종료.

실사검색은61장전체를처리하되, 소등라벨16장만수동형상을주고5종이동오차로획득을평가했다. 기존65/80회복→보완75/80회복. 80은16장×5이동의상관된조건수이며독립사진80장이아니다. 실패5건은모두사진53에서같은잔여11.18px오차(수동기준허용8.52px). 켜진45장은소등확인없는초기획득거부규약검사이며검출성공률이아니다. 실제동일자세/노출OFF3프레임이나정답점멸영상은새로만들지않았다.

동일640×480시뮬영상·동일ROI를40회씩교차처리한노트북CPU중앙값: 이전36.66ms→보완0.39ms. GUI실행중보완판판독함수의조건별중앙값0.46~0.58ms,최초검색1.91~3.17ms. 렌더/비교작업이동시에돌아가는노트북측정이며Jetson성능이나전체미션FPS를뜻하지않는다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_isaac.py --field --camera random --random-views 2 --seed 34 --renderer PathTracing --emissive 1 --off-reference --locate-body --gui --sequence RRYYGGRGYRGY --pose-error-m 0.03 --out reports/body-search-v2-recheck-3cm-01
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_search_photos.py --out reports/body-search-v2-real-recheck-01
```

종합 `reports/20261110-body-search-v2/validation.json`,실사검색 `real61-02/summary.json`,색규칙동등재확인 `colour61/summary.json`,동일입력시간비교 `timing.json`. 이전색평가의source해시고정manifest는원본을보존한다. 보완판색재평가는별도사본에새소스해시와이전해시를함께기록했다. 원본61장/labels.json/csv변경없음. 남은일은실사53번정렬오차,최초OFF확보절차,실제카메라보정과정답점멸영상,크기/회전오차범위평가다.

### 이전판 기록(v1): YOLO 없는 소등 렌즈 배열 검색

`src/arm_practice/tower_signal_search.py`는 고정 신호등의 예상 렌즈 폴리곤을 받아 주변 RGB에서 R/Y/G 배열을 찾는다. 카메라 실제 정답 자세·렌더 객체ID·마스크·점멸 정답 색은 검색 함수에 전달하지 않는다. 이번 시제품은 **확실한 OFF3장을 확보하고 카메라/노출/신호등이 고정된 조건**이다. 처음부터 켜져 있거나 카메라가 움직이는 상황의 일반 검출기는 아니다.

- `locate_body`: HSV(H: R≤12/≥165, Y15~38, G39~100; S≥45,V≥25) 증거와 예상 패치 형상을 비교. 주변60px, 크기0.85/0.925/1/1.075/1.15배를 검색한다. 각색 지지≥0.50, 평균≥0.65, 별도후보점수 차≥0.08을 요구한다. 같은 몸통의 이웃 픽셀은 하나의 후보로 묶는다. 후보없음/다중후보는 거부한다. 수치는 시뮬 개발용 후보값이며 실물 보정값이 아니다.
- 찾은 위치에서 OFF기준을 새로 만들고 영역을 고정한다. `read_registered`는 최소2색띠 지지≥0.30 및 기존 깊이 가림 검사를 통과한 경우 기존 `color_from_boxes`를 호출한다. 켜진 칸의 HSV가 반드시 해당 색이어야 한다는 규칙은 추가하지 않았다.
- 획득 실패나 몸통 소실은 UNKNOWN으로 내보낸다. 기존 예상 좌표로 조용히 되돌아가 OFF/색을 내지 않는다. 위치/노출이 바뀌면 기준을 재획득해야 한다. 현재 보정은 평행이동/크기이며 회전·몸통 형상 변화까지 추정하지 않는다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
# 각 실행의 출력 폴더는 새 이름 사용
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_isaac.py --field --camera random --random-views 2 --seed 34 --renderer PathTracing --emissive 1 --off-reference --locate-body --gui --sequence RRYYGGRGYRGY --pose-error-m 0.03 --out reports/body-search-recheck-3cm-01
# 신호등이 없는 장면: 30프레임 모두 UNKNOWN과 획득실패인지 확인
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_isaac.py --field --camera random --seed 34 --renderer PathTracing --off-reference --locate-body --hide-tower-test --gui --smoke --out reports/body-search-recheck-absent-01
```

시험 명령과 소스 해시는 `reports/20261110-body-search-gui/plan.json`, `source-manifest.json`. GUI시험을 완료했고 종합 결과는 같은 폴더의 `validation.json`이다. `summary.json`의 body_registration/found·shift_px/scale·sequence_match와 `projection_only`를 확인한다. `body_off_reference_*.png`와 `frame_*.png`는 검색 후 영역, `off_reference_*.png`는 보정 전 영역이다. `registration_read_ms`는 HSV존재확인 및 기존 색판독을 합친 노트북 CPU시간이고 순수 추가부하나 Jetson성능이 아니다. 렌더링/이전방식 비교 경로를 동시에 실행하므로 전체 시험 FPS를 배포 성능으로 쓰지 않는다.

#### 주변 검색 GUI 결과와 남은 제한

| 조건 | 투영만 순서 성공 | 주변 검색 후 순서 성공 |
|---|---:|---:|
| seed34 / 0cm / 3구도 | 3/3 | 3/3 |
| seed34 / 1cm / 2구도 | 1/2 | 2/2 |
| seed34 / 2cm / 2구도 | 1/2 | 2/2 |
| seed34 / 3cm / 2구도 | 1/2 | 2/2 |
| 새 seed35 / 3cm / 3구도 | 0/3 | 3/3 |

총12조건·6,840원본프레임: 검색 후 순서12/12·오판0, 같은 영상의 기존 투영 경로는6/12·오판1,280프레임. 순서는 RRYYGGRGYRGY(색별4회), 조건당570프레임이다. 프레임 정답은 이동하지 않았고 색판정 및 검색 모듈은 전 과정 고정했다. 탐색 개발에 사용한 이전 seed34 OFF렌더와 구분해 새 seed35 구도3개를 추가했다. 이것도 동일 시뮬 장면/조명/발광1 조건이며 독립 실물 검증은 아니다.

신호등 부재 장면에서는 획득 실패(body_not_found),30/30 UNKNOWN으로 거부했다. 숨김 전용 하네스는 램프 핸들이 무효화되지 않도록 `SetActive(False)` 대신 typed Xform의 상속 visibility로 숨겼다. 별도 `negative-source-manifest.json`에 이 하네스 차이를 기록했다. 실제 렌더에서도 신호등이 사라진 것을 확인했고 기존 양성 시험은 원래 하네스로 실행했다. 코드변경후 CPU67/67(추가 검색검사11개) 통과, 모든 Isaac프로세스 정상 종료.

관측한 최초검색 시간4.77~9.45ms, `read_registered` 프레임별시간의 조건별 중앙값25.01~25.92ms. 후자는 HSV존재확인·깊이검사·기존색판독을 포함한 함수 내부 경과시간이며, 다른 비교함수의 시간이 직접 더해진 값은 아니다. GUI/비교작업이 함께 실행되는 노트북 환경의 값이라 순수 추가부하나 Jetson성능으로 해석하지 않는다. 이 함수와 기존 `color_from_boxes` 모두 전체 프레임 배열을 처리하므로 ROI로 줄이고 별도 측정할 여지가 있다.

- **확인된OFF 필요:** 3색 지지≥0.5만으로 소등을 보장하지 않는다. 켜졌어도 색을 유지하는 렌즈는 이 조건을 통과할 수 있어 자동으로 confirmed_off=True로 바꾸지 않았다. 실제OFF획득 절차는 미정이다. 원본570프레임 중OFF는330이며,294는 전환근처를 제외한 채점OFF수다.
- **스케일은 정밀 크기측정이 아님:** 높은 시점에서는0cm도0.85배를 골랐다. 작은 내부패치가 높은 색일치 점수를 얻는 편향/불확실성으로 취급한다. 경계값이라는 이유만으로 거부하면 현재 통과조건도 거부하게 되므로, 실제 몸통경계/불확실성 기준을 먼저 평가해야 한다. 수평1cm가 항상3%의 크기변화를 뜻하지 않으며 시선방향·거리·원근에 좌우된다.
- **회전:** 알고 있는 카메라롤은 최초 투영폴리곤에 반영된다. 큰 절대롤 자체를 미지원하는 것이 아니라, 입력 자세의 미지 회전오차를 검색으로 보정하지 않는다. 이번에는 수평위치오차만 평가했다.
- **기준 비교:** 기존색규칙은 V≥40상승뿐 아니라 새 흰포화도 켜짐증거로 사용한다. 자세/노출의 문맥 식별은 호출자가 정확히 제공해야 하며 실제카메라 자동노출을 제어하거나 실제움직임을 독립 감지하지 않는다.
- **실물 미평가:** 이번 새 검색으로61장을 돌리지 않았다. 손으로표시한 축을 흔들어 검색회복을 재는 실사시험은 후속으로 가능하나, 수동형상/근처위치를 제공하는 조건임을 분리해야 한다. 시간상인접사진은 자세/노출일치OFF기준이 아니다.

### 코드 연결 API

`tier_rois` → `camera_context` → `make_off_reference(..., confirmed_off=True)`로 기준을 만든 뒤 `tower_signal_color(..., off_reference=ref, exposure_token=고정노출식별값)`를 호출한다. 기준은 영상 3장 이상과 동일 카메라/노출/ROI가 필요하다. 카메라가 움직이거나 노출이 바뀌면 재보정하거나 별도로 정렬해야 하며 오래된 기준을 강제로 재사용하지 않는다.

**실사에 남은 일:** 실제 렌즈 치수·K·장착 변환 확인, RGB 노출/게인 고정, 동일 자세의 확실한 꺼짐 영상 확보, 정답을 아는 12회 점멸 영상으로 독립 재검증. 현재 결과를 실물 인식 완료로 표시하지 않는다.

## 실행 환경

Python: `/home/pa33/pa-course/.venv-isaacsim/bin/python` (Isaac Sim 6.1).

실행 전 다른 Isaac 시험이 없는지와 GPU 여유를 확인한다. 한 번에 한 Isaac 시험만 실행한다.

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
nvidia-smi
```

현재 PC의 GUI 디스플레이는 `:1`, Xauthority는 `/run/user/1000/gdm/Xauthority`다. 일반 데스크톱 터미널은 기존 화면 환경을 사용한다. 비로그인 실행 환경에 DISPLAY가 없을 때만 명령 앞에 다음을 붙인다.

```bash
env DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority XDG_RUNTIME_DIR=/run/user/1000
```

출력 폴더가 이미 있으면 덮어쓰지 않고 중단한다. 재시험 시 아래 `run-...-01`의 번호를 바꾼다.

## 1. CPU 시험

```bash
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B -m unittest discover -s tests -p test_tower_signal.py -v
```

번짐·애매한 점등·전체 포화·반사·렌즈 영역·가림·신선도·랜덤 시점 재현성을 검사한다. 실제 LED 정확도 시험은 아니다.

## 2. 미로에서 짧은 확인

```bash
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera start --renderer PathTracing --smoke --gui --out reports/run-field-smoke-01
```

30프레임: 소등 20장 + 첫 색 10장. `summary.json`의 `frame_matches == frames`인지 확인하고, OFF/R 사진의 렌즈 칸이 실제 렌즈 옆면에 있는지 본다. 전체 12회 합격 판정은 하지 않는다.

GUI는 `PreviewCam`, 판독은 `/World/Cam`을 사용한다. 화면을 확대·회전해도 판독 카메라를 변경하지 않는다. Stage에서 `/World/Cam` 자체를 편집하면 시험이 무효가 될 수 있다.

## 3. 고정 출발 시점에서 전체 점멸

```bash
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera start --renderer PathTracing --emissive 0.5 1 2 --gui --out reports/run-field-full-01
```

발광 3조건 각각 570프레임. 순서 `RGYRGYGRGRGG`는 R4/Y2/G6이다. 시작 소등 1초 + 12회×2초 + 추가 소등 3.5초 = 28.5초의 명목 시간표를 사용한다. 실제 실행 속도 20Hz 달성을 입증하는 시험은 아니다.

## 4. 로봇 고정, 카메라 구도 랜덤 시험

```bash
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera random --random-views 5 --seed 33 --renderer PathTracing --emissive 1 --gui --out reports/run-field-random-01
```

각 시점에서 전체 12회 점멸을 시험한다. 동일 seed와 시점 번호는 동일 카메라 자세를 만든다. 발광 조건이 달라도 같은 시점을 재사용한다.

| 변수 | 초기 시험 범위 |
|---|---|
| 로봇 중심에서 전방 거리 | 0.10~0.35 m |
| 좌우 거리 | -0.10~+0.10 m |
| 카메라 지면 높이 | 0.25~0.70 m |
| 신호등 중심을 향한 기본 방향 대비 yaw/pitch | 각각 ±8도 |
| roll | ±10도 |

범위는 영상 시험용 후보값이다. 모든 시점이 실제 팔로 도달 가능하다는 뜻이 아니다. 실제 장착이 확정되면 `RANDOM_VIEW_RANGES`를 바꾼다. 화면 밖·너무 작음·가림 조건도 실패 원인과 함께 남기며 성공한 시점만 골라 집계하지 않는다.

카메라 자체의 위치·각도 변화와 자세 추정 오차는 다르다. 후자는 별도 `--pose-error-m 0.02`로 시험한다. 기본값 0은 이상적으로 보정된 카메라 자세를 가정한다.

## 결과 파일과 합격 기준

| 파일 | 내용 |
|---|---|
| `summary.json` | 조건별 순서 일치, 정확도, UNKNOWN 사유, 점수 선택, 카메라 자세, 기존 규칙 비교 |
| `frames_*.jsonl` | 모든 프레임의 명목 정답·신규/이전 판정·밝은 면적·흰 포화 면적·점수 |
| `frame_*_OFF/R/Y/G.png` | 렌즈 판독 칸을 표시한 장면 |
| `debug_*.png`, `debug_*.npz` | 최초 오판 유형의 RGB/깊이/사용 카메라 자세 |
| `error.txt` | 예외로 중단된 실행의 오류 |

전체 시간표의 개발 합격 기준:

1. `smoke_or_truncated`가 false.
2. 조건마다 `recorder_complete`와 `sequence_match`가 true이고 `recorder_error`가 null.
3. 기록 순서가 정답과 같고 색별 개수도 일치.
4. 전환 순간 ±1프레임을 제외한 `frame_errors == 0`. 전환 프레임도 기록기에는 그대로 전달한다.
5. RGB에서 실제 점등·소등이 시간표와 맞는지 확인. 명령만 바뀌고 이미지가 남으면 판정 규칙의 평가 근거로 쓰지 않는다.

스크립트 종료 코드 0은 합격을 뜻하지 않는다. `error.txt` 유무와 summary를 같이 확인한다. 조건별 실패도 정상 종료로 기록하는 도구다.

## 이상할 때 수정할 곳

| 증상 | 먼저 확인 | 수정 위치/원칙 |
|---|---|---|
| 소등 구간에도 원본 영상이 계속 켜짐 | 렌더 모드·재질 전환·프레임 갱신 | 시험 스크립트의 `set_lamps`, `render`. RaytracedLighting 잔상은 미해결. PathTracing 비교. 정답을 임의로 뒤로 밀어 합격시키지 않음 |
| 원본은 소등인데 R로 판독 | 빨강 칸에 흰 덮개·윗면이 들어가는지 | `tier_boxes`, TOWER 높이, K/카메라 자세. 밝기 임계값부터 바꾸지 않음 |
| 실제 한 칸 켜짐인데 UNKNOWN | 정확한 렌즈 영역, 각 칸 ratios/white_ratios/scores | `PARAMS`의 dominance 기준은 실사 정답 영상으로 조정. 단순 최대값 강제 선택 금지 |
| 실사에서 흰 포화가 한 칸만 뚜렷하지만 UNKNOWN | 색 정답·반사 여부·노출 | 개선판은 한 칸 ≥0.30·나머지 ≤0.10 조건을 추가했다. 덮개/반사 여부와 실사 정답 영상으로 검증 필요 |
| outside_image/too_small/view_too_vertical | 구도·가시 크기 | 카메라 범위·장착 정보 수정. 최소 크기 기준을 무조건 낮추지 않음 |
| occluded | 깊이 영상에 미로 벽 등이 앞을 가리는지 | 실제 가림이면 UNKNOWN 유지. RGB/depth 정렬·깊이 단위도 확인 |
| 기록기 Unknown signal color | 해당 프레임의 판정 사유 | 판독/렌더 원인부터 수정. 기록기에서 UNKNOWN을 임의의 색으로 대체하지 않음 |
| 렌더 버퍼가 비거나 크기가 다름 | error.txt와 annotator 결과 | render의 명시적 캡처·형식 검사 확인 |
| GUI 조작 후 카메라 오류 | 조작 대상이 PreviewCam인지 | 판독용 Cam을 직접 편집하지 않음 |
| 디스크가 빠르게 줄어듦 | 이전 scene.usda 파일 | 현재 기본값은 장면 저장 안 함. 필요할 때만 `--save-scene`으로 scene.usdc 한 번 저장 |

## 판정 규칙과 변경 위치

- `src/arm_practice/tower_signal.py`: `TOWER`, `PARAMS`, `tier_boxes`, `tower_signal_color`.
- `scripts/test_tower_signal_isaac.py`: 미로 로드, 임시 판·바코드 제외, 재질·렌더, PreviewCam, 시점 샘플링, 결과 기록.
- `tests/test_tower_signal.py`: CPU 회귀 시험.
- 연결용 `r12_sensor_ros.py`, `r12_sensor_sim.py`, `mission_scene_objects.py`는 별도 사본이며 통합 실행 검증이 끝난 배포본이 아니다.

## 초기 실사 사진 무작위 진단 (개선 전 기록)

단일 클래스 `tower_light` 라벨은 위치만 제공하며 켜진 색 정답은 없다. `on` 폴더에도 꺼져 보이는 프레임이 있어 폴더명을 프레임별 색/켜짐 정답으로 쓰지 않는다. 사진은 Isaac에 텍스처로 넣지 않고 실제 RGB 그대로 후보 판독 함수에 넣는다. YOLO는 실행하지 않는다.

### 표본과 정답

- 위치 라벨이 있는 1025/1026 촬영 묶음 12개에서 seed 33으로 4장씩 선택하고 순서를 섞었다: 총 48장.
- 예측을 보기 전에 제가 시각 확인한 임시 R/Y/G/OFF 정답을 `samples.json`에 따로 기록했다. 원본 데이터 라벨은 수정하지 않았다. 사용자가 확인한 정답이나 제어기 점등 로그가 아니다.
- 완전한 3단이 보이는 46장: R13/Y4/G2/OFF27. 하단이 화면 밖인 1장과 상단이 잘린 1장도 처리 결과에 남기되 46장 집계에서는 구분했다.
- 기존 위치 박스를 세로 3등분하고 각 칸의 가운데 60%를 사용한다. 덮개/기둥을 포함한 라벨이나 비스듬한 구도에서는 칸이 어긋날 수 있다. 이 시험은 위치 검출기/실제 카메라 자세 투영 검증이 아니다.

### 실행

이미 검토한 표본으로 재평가:

```bash
cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal"
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B scripts/test_tower_signal_photos.py --out reports/20261110-signal-improved-real-seed33
```

아래 표는 초기 기록이다. 현재 재평가는 상단 개선판 결과 형식으로 저장된다. 새 표본을 뽑으려면 새 폴더로 `--prepare --seed 34 --per-group 4`를 사용하고 해당 이미지용 렌즈 몸통 주석도 새로 지정한다. 생성된 `contact_*.png`와 원본을 먼저 보고, 예측 실행 전 `samples.json`의 truth/annotation_valid/three_tiers_visible를 검토해야 정확도 집계 대상이 된다. 미정 정답을 모델 예측이나 폴더명으로 자동 채우지 않는다.

### 결과와 판정

| 시각 임시 정답 | 일치 | UNKNOWN | 다른 상태 판정 |
|---|---:|---:|---:|
| R 13장 | 6 | 7 | 0 |
| Y 4장 | 4 | 0 | 0 |
| G 2장 | 1 | 1 | 0 |
| OFF 27장 | 24 | 0 | R 1 / Y 2 |
| 합계 46장 | 35 | 8 | 3 |

이는 **라벨 박스 3등분 + 현재 후보 판정 + 시각 임시 정답의 진단 결과**다. 일반 실물 정확도 76.1%로 인증하지 않는다. Y/G 표본이 작고 같은 촬영 묶음의 프레임들은 서로 비슷하다. 기존 개발 자료이며 독립 검증 세트도 아니다.

실패 예시:

- `photo_001`: G가 켜졌으나 빨강 판독 칸이 흰 덮개에 걸려 UNKNOWN. 칸 위치 문제를 먼저 해결해야 한다.
- `photo_010`: 꺼져 보이는 사진인데 흰 덮개가 R 칸에 들어가 R. 영역 문제와 밝기 판정의 영향이 섞였다.
- `photo_023` 등: R이 확실히 밝아도 옆 Y 칸 번짐 때문에 우세 조건을 못 넘고 UNKNOWN. 실사 기준 보완이 필요한 후보 사례다.
- `photo_003`, `photo_016`: 신호등 일부가 화면 밖이다. 라벨 박스 3등분만으로 잘림을 검출하지 못하므로 이 도구를 그대로 운영용 위치 처리로 쓰지 않는다.

현재 임계값은 바꾸지 않았다. 같은 점수 계산을 `color_from_boxes`로 분리해 Isaac 투영 경로와 실사 ROI 경로가 같은 함수를 쓴다. 기존 CPU 36개와 새 사진 진단 회귀 4개, 총 40개 통과했다. 실제 점멸 영상 검증이 완료됐다는 의미는 아니다.

출력: `reports/20261110-real-photos-seed33/summary.json`, `predictions.jsonl`, `samples.json`, `contact_*.png`, `photo_*_prediction.jpg`, `cpu-tests.log`. 색별/UNKNOWN/꺼짐 오판과 잘림 사진을 모두 남긴다. 실제 점멸 시간·12회 기록은 정답 영상으로 별도 시험해야 한다.

## 바코드 해독 시험 (실물 검증 전)

- 네 종류 모두 줄무늬 **가로 60 × 세로 15 mm**, 여백 포함 라벨 **가로 약 75 × 세로 약 25 mm**.
- `references/barcodes-photo.png`가 이번 실사 원본이다. `barcodes-reference.png`의 이전 치수는 사용하지 않는다.
- `prepare_barcode_assets.py`가 사진의 네 줄무늬 사각형을 원근 보정하고 흰 여백을 추가한다. 바코드 패턴을 재생성하지 않는다. 원본/변환/해시/해독 결과는 `references/barcodes-manifest.json`에 기록한다.
- 네 텍스처는 모두 각각 P2-01~04로 해독 확인. 저해상도 사진의 리샘플링은 실제 인쇄물의 세부 정보를 복원하지 않는다.
- 부착은 수평, 세로로 선 평면, 월드 +X 방향. 임시 회색 판을 복구하지 않고 작은 라벨만 추가한다. 부착 위치는 미확정 후보다.
- ZXing-cpp는 **전체 카메라 RGB 이미지**에서 Code 128을 해독한다. 정답 코드·장면 좌표·투영 영역은 해독에 전달하지 않는다. 별도 학습/YOLO 없음.
- 기본은 원본 해독. `--barcode-upscale 2`는 원본이 전혀 안 읽힌 프레임에만 전체 화면을 INTER_CUBIC 2배 확대하여 재시도한다. 정답 영역 크롭이나 바코드 패턴 복원은 하지 않는다. `raw_read_count`, `decode_scale`, 해독 시간을 함께 기록한다. 확대가 잃어버린 세부 정보를 복구하는 것은 아니다.
- 한 코드만 3프레임 연속 읽혀야 안정 판독. 누락/다중 코드/허용하지 않은 값은 연속 횟수를 초기화한다. 이전 판독을 새 프레임 결과로 재사용하지 않는다.

### 의존성과 CPU 확인

현재 `vendor/`에 zxing-cpp 3.1.1을 로컬 설치했다. Isaac 가상환경 자체는 변경하지 않는다. 다른 PC에서:

```bash
/home/pa33/.local/bin/uv pip install --python /home/pa33/pa-course/.venv-isaacsim/bin/python --target vendor --no-deps -r requirements-barcode.txt
PYTHONDONTWRITEBYTECODE=1 /home/pa33/pa-course/.venv-isaacsim/bin/python -B -m unittest discover -s tests -p 'test_*.py' -v
```

### GUI: 바코드 48조건 + 신호등 짧은 확인

```bash
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera barcode --barcode P2-01 --barcode-sweep --renderer PathTracing --gui --smoke --out reports/run-barcode-sweep-01
```

48조건 = 네 코드 × 세 부착 위치 × X축 거리 0.20/0.30/0.40/0.60 m. 부착 위치 중심은 `(0.15,1.03,0.14)`, `(0.15,0.82,0.25)`, `(0.15,1.15,0.30)` m. 카메라의 좌우/높이 차이는 각각 `(0,0.03)`, `(0.05,0.10)`, `(-0.08,0.20)` m. 카메라가 라벨을 향하도록 하므로 멀리/높이/옆에서 보는 원근 효과는 포함하지만 라벨 자체를 기울여 붙이지 않는다.

로봇 기준 위치는 고정이며 카메라 위치는 영상 시험용 후보다. 팔 도달성·최종 장착·실제 카메라 노출은 검증하지 않았다. 각 조건은 최소 4렌더 후 균일하지 않은 RGB가 들어올 때까지 최대 32렌더를 기다린다. 이후 8프레임을 해독하며, 채점 중 검정/단색 버퍼가 나오면 무효 오류로 중단한다. 렌더 실패를 바코드 미해독에 포함하지 않는다.

### GUI: 신호등 12회 + 바코드 동시 판독

```bash
/home/pa33/pa-course/.venv-isaacsim/bin/python scripts/test_tower_signal_isaac.py --field --camera barcode --barcode P2-01 --renderer PathTracing --gui --out reports/run-signal-barcode-full-01
```

`--barcode-center X Y Z`로 라벨 위치를 지정한다. 기본 `(0.15,1.03,0.14)` m. `--camera barcode`는 이 중심 + `(0.20,-0.03,0.07)` m의 카메라가 신호등 중심을 보는 가정 구도다. 640×480, fx=400은 기존 시험과 같으며 D435 실측 보정값은 아니다.

원본과 확대 전처리를 비교하려면 위 명령에 `--barcode-upscale 2`를 추가하고 새 출력 폴더를 사용한다. 신호등 12회와 동시 시험에서도 같은 옵션을 쓸 수 있다. 처리 시간은 이 노트북의 CPU 값이며 Jetson 지연 성능으로 해석하지 않는다.

### 판정과 수정할 곳

- `barcode-sweep-summary.json`: 조건별 8/8 정확 해독, `wrong_frames == 0`, `stable_correct_frames > 0`이면 해당 조건만 통과. 실패 조건도 전부 포함한다.
- `barcode-sweep-frames.jsonl`: 프레임별 해독 문자열/검출 위치/시간/안정 판독. `barcode_*.png`: 마지막 프레임과 해독 위치.
- 신호등 `summary.json`의 각 결과 `barcode`: 정확/오해독/안정 판독 프레임 수. 전체 `barcodes_*.jsonl`도 남긴다. 신호등 순서 채점은 별도로 유지한다.
- 줄이 뭉개져 미해독이면 `bars_projection.width_px`와 원본 PNG부터 확인. 실제 카메라에서는 관측 거리·초점·노출·해상도를 검토한다. 시뮬 라벨을 임의로 크게 해서 기존 실패를 통과로 바꾸지 않는다.
- 빈 라벨/좌우 반전이면 `barcode_support.add_label`의 UV/재질/텍스처 경로를 확인한다. 읽히는 코드는 맞지만 간헐 누락이면 샘플별 결과와 픽셀 크기를 확인한다.
- 오해독이 있으면 실패로 유지한다. 정답 문자열을 강제로 출력하거나 장면의 코드값을 판독 결과로 사용하지 않는다.
- 사진 기반 텍스처 손실이 의심되면 원본 인쇄 파일/고해상도 정면 사진으로 자산을 교체하고 원본 해시부터 새 기록을 만든다.

## 실사에 적용할 때 수정할 순서

1. **RGB/자세/렌즈 위치 보정**: `K`는 시뮬 fx=400을 실제 D435 값으로 교체한다. 왜곡 보정 영상과 그에 맞는 K를 쓴다. `TOWER`의 렌즈 높이와 카메라 월드 변환을 실제 값으로 바꾸고 원본 영상 위에 `tier_boxes`를 그려 칸 위치부터 확인한다. 자세는 제어 쪽 제공값의 좌표계/단위/시각을 맞춘다. 깊이 가림 검사를 쓰면 RGB와 정렬된 깊이도 필요하다.
2. **RGB 노출/게인 고정**: 실제 배경과 관측 자세에서 렌즈 번짐과 반사를 비교해 설정을 정한다. 값은 아직 미정이다. 노출값·게인·해상도·카메라 파라미터를 영상과 함께 보관한다. 깊이 센서 노출 설정을 RGB 노출로 착각하지 않는다.
3. **신호등 후보 규칙 수정**: 정답 12회 점멸 영상과 빨강 UNKNOWN 사례로 `PARAMS`의 v_on=200, white_v=235, white_s_max=80, white_weight=.25, dominance_ratio=1.8, dominance_margin=.10을 평가한다. 포화 우선 조건은 후보이며 현재 시뮬 결과로 확정하지 않는다. 실제 다중 점등/반사도 무조건 단일 색으로 만들지 않는다.
4. **바코드 실물 원본 입력**: 실제 인쇄물 네 개를 예상 자세에서 촬영해 `decode_rgb`에 전체 영상을 넣는다. 미해독이면 줄무늬 가시 크기·초점·밝기·노출·지원 해상도를 비교한다. 전체 화면 2배 전처리는 원본 결과와 별도로 평가한다. 사진 텍스처 기반 시뮬의 미해독을 실물 인쇄물의 실패 거리로 간주하지 않는다.
5. **다른 녹화로 최종 확인**: 보정에 사용하지 않은 영상에서 신호등 12회 순서/개수/UNKNOWN, 바코드 오해독/누락/연속 판독을 확인한다. 실제 촬영 시각 기반으로 점멸 길이와 종료를 확인한다. 시뮬의 명목 20 Hz 타임스탬프를 실물 캡처 시간 대신 쓰지 않는다.

필요 자료: 실제 고정 노출 정답 점멸 영상, 실제 네 인쇄물 원본 영상, 촬영 설정·카메라 보정·제어 자세. 가능한 경우 바코드 인쇄 원본 파일이나 고해상도 정면 사진으로 저해상도 텍스처 한계를 줄인다.

## 검토 시 해석 정정

- 실사 48장은 더 이상 전부 검토 대기가 아니다. 현재 결과는 위 실사 진단 절과 `real-photos-seed33/summary.json`에 있다. 임시 정답의 시각 검토와 확정된 실사 정답 영상은 구분한다.
- 줄무늬 크기 60×15 mm는 전달받은 값으로 반영했다. 여백 포함 라벨 75×25 mm는 대략값이고 부착 위치는 미정이다.
- 초기 P2-01 0.20 m의 972바이트 파일은 실제 검정 버퍼였다. 미해독으로 분류한 초기 집계는 정정 대상이다. 단순히 32회 기다려도 첫 구도가 검정인 실행은 error.txt로 무효 처리했고, 다음 실행은 초기 캡처 4회와 첫 시험 구도를 분리해 모든 채점 RGB에 내용 검사를 한다. 재시험 결과는 별도로 기록하며 초기 실패 증거는 보존한다.
- 이 자료로 실물의 판독 한계가 0.20 m라고 결론 내리지 않는다. 저해상도 사진 패턴·필터링·렌더·노출·카메라 구도가 섞여 있다. 모듈당 2 px 같은 목표값을 이 데이터의 실물 성공/실패 경계로 사용하지 않는다.
- RaytracedLighting에서 잔상이 재현되고 PathTracing에서 사라진 비교는 있다. 렌더러 내부 원인까지 확정하거나 동일 현상이 모든 tutor 장면에서 반드시 생긴다고 단정하지 않는다. 원래 런타임에 연결하기 전 별도 확인이 필요하다.
- 흰 포화 우선 규칙은 흰 덮개/반사도 선택할 수 있다. 먼저 실제 렌즈 ROI와 잘림을 보정한 뒤 실사 정답 영상으로 후보를 비교해야 한다. 초기 진단은 임계값 변경 없이 수행했고, 이후 개선판에서 렌즈 ROI·흰 포화 우선·OFF 기준 비교를 추가했다. 실사 정답 영상 검증 전 상태는 유지한다.

## 상태 표기

**후보 규칙, 실사 정답 영상 검증 전.** 실사에서 확실히 켜진 빨강 두 장이 UNKNOWN인 문제가 남아 있다. 시뮬 결과는 기하·칸 위치·기록 흐름과 렌더 상태 확인용이다. RaytracedLighting의 지연/밝기 문제와 실물의 번짐·노출 차이를 해결한 근거가 아니다. PathTracing에서 영상이 시간표를 따라가더라도 실물 임계값을 인증하지 않는다.

팀 상태 문서에 인계할 문구: `신호등: 후보 규칙, 실사 정답 영상 검증 전. 바코드: 실사 패턴 기반 렌더 해독 시험이며 실제 인쇄물·카메라 검증 전.` 팀 저장소 `docs/R12_STATUS.md`는 이 작업에서 수정하지 않는다.

## 바코드 추가 GUI 실행 결과 (2026-11-10)

요약: `reports/20261110-barcode-validation.json`. 초기 실행은 종료했다. **후속 정정: 아래 초기 두 실행의 P2-01/low_front/0.20m 마지막 PNG는 972바이트인 검정 영상이다. 이 조건은 렌더 무효이며, 초기 0/48·3/48을 유효 영상 48조건의 성능으로 해석하면 안 된다.** 초기 자료는 보존하고 RGB 유효성 검사를 추가해 재시험한다. 아래 결과는 **후보 규칙, 실사 정답 영상 검증 전** 상태의 영상 시험이다.

| 출력 폴더 (reports 아래) | 결과 |
|---|---|
| `20261110-barcode-field-gui-smoke` | PathTracing GUI, 미로 벽 유지, 48조건 시도 중 첫 조건은 검정 렌더 무효. 나머지 저장 영상에서도 해독 없음(오해독 0). 신호등 동시 짧은 확인 30/30 일치, P2-01 0/30 해독 |
| `20261110-barcode-field-gui-full-upscale2` | 동일 48조건 시도에 검정 렌더 무효 1조건 포함. 원본이 안 읽힐 때만 전체 화면 2배 전처리로 3조건 통과, 오해독 0. low_front/X거리 0.20 m의 P2-02·03·04만 8/8 해독. P2-01 및 나머지 조건은 미해독 |
| 같은 full 폴더의 신호등+P2-02 | 570프레임 전체 점멸에서 신호등 순서 RGYRGYGRGRGG 일치. 바코드 P2-02는 160/570프레임 정확 해독, 오해독 0, 3연속 이후 안정 판독 프레임 144. 원본 해독은 0프레임. 계속 안정적으로 읽히는 상태는 아님 |

### RGB 유효성 검사 후 재시험

`reports/20261110-barcode-field-gui-valid-rgb`는 32회 워밍업에도 첫 구도가 검정이라 무효 중단했다. `error.txt`를 보존한다.

`reports/20261110-barcode-field-gui-valid-rgb-02`는 렌더 제품 초기화 4캡처(미채점)를 먼저 수행하고 실제 시험 구도로 전환했다. 모든 바코드 조건은 최소 4회 워밍업 뒤 각 채점 RGB가 단색이 아닌지 검사했다. GUI PathTracing에서 **유효 영상 48조건 중 3조건 통과**, 오해독 0으로 재현했다. P2-02/03/04의 low_front 0.20 m만 각각 8/8 해독했다. P2-01 첫 조건은 이제 177,819바이트의 정상 장면이지만 0/8 미해독이다. 미해독의 근본 원인은 아직 분리하지 않았다.

같은 재실행에서 신호등 전체 570프레임·12회 순서 일치, 채점 오판 0. 동시 P2-02는 다시 160/570 해독·안정 프레임 144·오해독 0이었다. 초기화 구간은 신호 정답 시간표에 포함되지 않으며 채점된 정답을 사후 이동하지 않았다. 신호등 임계값은 이전과 같다. CPU 최신 전체 42/42 통과.

이 재시험도 실물 판독 거리/정확도의 근거가 아니다. 초기 무효 실행과 최신 유효 실행은 별도로 해석한다.

동시 시험의 P2-02 해독 160프레임은 Y 40장 + G 120장이며, OFF 330장/R 80장은 미해독이었다. 이 렌더 입력에서는 점등 상태에 따라 해독 결과가 달라졌다. 조명·색·픽셀 리샘플링의 원인은 분리하지 않았으므로 실물 특성으로 단정하지 않는다.

CPU는 기존 24개 + 바코드 12개, 총 36개 통과했다. 전체 시험의 코드/텍스처 manifest 해시는 해당 출력 폴더 `source-manifest.json`에 기록했다. 실패 출력도 보존한다.

이 3/48은 실제 경기장 성공률이 아니다. 작은 사진 텍스처를 다시 축소·렌더한 결과이며, 실제 인쇄물/카메라 영상과 분리해 해석한다. 2배 전처리의 효과도 해당 입력에서만 확인했다. 바코드 가까운 조건의 X거리 0.20 m는 광학 거리 약 0.202 m이고, 모든 실물 구도에서 20 cm면 읽힌다는 뜻이 아니다.

## 2026-11-10 실행 기록

세부 코드 해시와 결과는 `reports/20261110-validation.json`에 기록한다. 시험 중 발견된 실패 자료도 보존한다.

- CPU: 로컬 시험 24개, tutor 신호 신선도 시험 8개 통과.
- `field-synchronized`: PreviewCam 분리 전 실행. 카메라 자세 변경 예외로 중단.
- `field-transition-check`, `field-material-binding-check`: PreviewCam 분리 후 실행. RaytracedLighting에서 소등 구간의 잔상 지속. 판정 규칙 합격 근거로 사용하지 않음.
- `field-pathtracing-transition`: 같은 미로·카메라·임계값, 70/70프레임 일치. 기존 규칙은 50/70. 전체 순서 시험은 아님.
- `field-pathtracing-full`: 실제 미로 벽 유지, 회색 판·바코드 제외. 고정 출발 시점에서 발광 0.5/1/2 모두 순서 일치, 원본 1,710프레임 오판 0. 같은 영역의 이전 판정 규칙은 3조건 모두 순서 실패(채점 프레임 정확도 79.52%).
- `field-pathtracing-random-seed33`: 로봇 고정, 후보 범위에서 seed 33의 카메라 5시점. 5/5 순서 일치, 원본 2,850프레임 오판 0. 같은 입력의 이전 규칙도 이 5시점은 통과했다.
- 합계: PathTracing 8조건, 4,560프레임 일치. 해당 seed·렌더·장면·가정된 치수 범위의 결과이며 실제 카메라 정확도나 전체 팔 작업 범위의 성공률은 아니다.
- 최종 두 시험은 자동 장면 저장을 끈 상태다. 앞선 원인 분석용 scene.usda(파일당 약 297 MB)는 보존돼 있다.
- 실행은 모두 종료됐다. 결과를 재현하려면 위 명령에서 `--renderer PathTracing`을 명시한다.

남은 일: 실측 렌즈 높이 확정, 실제 카메라 내부/장착 보정, 고정 노출 정답 점멸 영상 평가, 실제 팔 도달 가능한 시점 범위 반영, 실시간 렌더 잔상 원인 분리.


## 2026-11-13 B26/B27 로컬 검토 및 최신 시뮬 변경 반영

- 질량·관성 보정(mass_scale=0.100011), 4Nm 구동 설정(B26/B27), 펌웨어 목표각 검사,
  보정 자세→기준 바코드 자세→접기(stow_retract) 순서를 반영했다.
- B26(신호 먼저)·B27(바코드 먼저+보정+접기+재관측)은 각각12회/810프레임 정답.
- 결과·입력·프로세스 메타데이터와 독립검토는 `results/assembly-sim-B26`, `results/assembly-sim-B27`.
  영상·원본 프레임/physics 로그·CAD는 로컬 `reports/`에만 보관한다.
- B26 결과 프레임 수1175는 종료 후2초를 제외한 값이며 실제 영상/로그1235프레임.
  원본을 보존하고 review_20261113.json에 불일치를 기록했다. 이후 run.py의 최종 집계를 수정했다.
  집계 수정 후 새 Isaac 실행은 하지 않았고 CPU 회귀 검사를 추가했다.
- 실물 초기 팔 자세는 미정이다. 그리퍼를 바닥 근처에 둔 시작 자세는 이번 경로 검증에 포함되지 않는다.
  상단 카메라 비가림 확인과 측정 관절값에서의 상승/접근 경로 검증이 필요하다.
- 브랜치의 상대 r12 탐색, NZ2_ASSEMBLY_DIR/NZ2_FLOOR_USD, 시험 화면 test_data와
  zxing-cpp 설치 안내는 유지했다. 이번 코드는 ROS/Nav2/실물 통합 배포판이 아니다.
