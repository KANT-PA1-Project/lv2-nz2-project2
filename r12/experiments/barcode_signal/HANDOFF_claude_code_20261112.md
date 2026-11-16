# 인수인계 — 신호등·바코드 인지 + 미션 판단 (Cowork → Claude Code, 2026-11-12)

> 이 문서를 처음 읽고 시작할 것. 대상 범위: **하드웨어(주행·팔) 제외, 신호등 색 검출 + 바코드 검출 → 정렬 방향(오름/내림) + L1~L3 퍽 개수 판단까지.**
> 작업 PC: pa33 (Legion Pro 5). 카메라: D435 (USB). 신호등: OpenCR(sign_tower_v3) 단독 구동.

## 0. 작업 규칙 (반드시 지킬 것)
- 답변·주석은 한국어.
- **사용자가 요청하기 전에는 코드 수정 금지.** 제안만 하고 승인받기.
- 다른 에이전트 파일은 수정 전 반드시 확인받기: `live_mission_optimized.py`, `mission_summary.py`, `mission_live_support.py`, `replay_mission_summary.py`, `tower_signal*.py`, `barcode_support.py`
- tutor 저장소(`~/git/lv2-nz2-project2`) `AGENTS.md` 준수: 합의 없이 공용 코드 덮어쓰기 금지, 키·venv·원본 로그 커밋 금지. 커밋·푸시는 사용자 지시가 있을 때만.
- 불확실한 값은 `[가정]`/`[추정]`으로 표시.

## 1. 미션 규칙 요약 (바탕화면 `mission.pdf` v5)
- 신호등 (80, 950, 0) mm, +X 방향. 로봇 관측 위치 (734, 950), −X 방향. 바코드 중심 (80, 650, 450) mm.
- 점멸: 1 s 켜짐 / 1 s 꺼짐 × 12회, 끝나고 3 s 꺼짐. 색별 1~6회. 모르는 색·누락 → 오류 정지.
- 바코드 P2-01/03 = 오름차순, P2-02/04 = 내림차순. 동률이면 R → Y → G. 그룹 순서대로 L1 → L2 → L3, 퍽 개수 = 점멸 횟수.
- 인지·판단 배점: 순서 12 + 색별 횟수 9 + 바코드 4 + 배정 9 = **34점**.
- tutor 로직: `r12/src/arm_practice/mission.py` `SignalRecorder`(UNKNOWN 또는 프레임 간격 > 0.25 s → 오류), `plan()`(42행 정렬).
  `r12/scripts/r12_sensor_ros.py` 443~448행: 바코드 → arming → recorder, 464~466행 `observed_mission_plan.json`, 467행 출발 조건.

## 2. 파일 위치
경로 기준: `~/git/lv2-nz2-project2/r12/experiments/barcode_signal/`

| 파일 | 만든 쪽 | 내용 |
|---|---|---|
| `scripts/live_mission_optimized.py` | 다른 에이전트 | **현재 실사용 런타임.** HSV 신호등 + 바코드(zxing-cpp). `v` 키로 mp4 녹화 |
| `scripts/mission_summary.py` | 다른 에이전트 | 바코드 확정: 5 s 창(`BARCODE_WINDOW_S`) 안에서 같은 코드 3회(`BARCODE_MIN_MATCHES`). 미등록 코드 무시, 등록 코드 2종이면 보류 |
| `scripts/mission_live_support.py`, `replay_mission_summary.py` | 다른 에이전트 | 로그(jsonl) 기록·판단 재생 (영상 재처리 아님) |
| `scripts/tower_yolo_signal.py` | Cowork | YOLO `tower_light`(models/current.pt, class 0) → 3층 칸 → 색 판정. 꺼짐 30프레임 자동 고정. 바코드 동시 해독. 마지막 줄 `RESULT {json}` |
| `scripts/check_mission_perception.py` | Cowork | 신호등 OpenCR을 USB로 제어(STOP/RAND/START)하며 34점 채점. **신호등 USB가 PC에 있어야 함** |
| `scripts/live_tower_check.py` | Cowork | tutor 포팅 로직(`tier_rois`/`locate_body`) 시각화 점검용 |
| `~/git/p2_perception/firmware/sign_tower_v3/` | Cowork | 신호등 단독 구동 펌웨어 (업로드 완료, CRC OK). README 참고 |
| `scripts/test_mission_camera_isaac.py`, `test_mission_1080p_isaac.py`, `test_tower_signal_isaac.py` | 다른 에이전트 (최신) | Isaac 시험 (T6) |
| `validation_set_20261110/` | Cowork | 신호등 검증 이미지 61장 (R16 Y15 G14 OFF16), 라벨 확인 완료 |
| `diffs/*.diff`, `src/arm_practice/tower_signal.py` | Cowork | tutor 쪽 신호등 판정 교체 **제안** (투영 기반). **tutor 에 미적용** |

YOLO 모델
- 팀: `~/git/p2_perception/models/current.pt` (9클래스, 0 = `tower_light`, 실사 AP50 0.994 val / 0.977 test, imgsz 640)
- tutor: `~/Downloads/drive-download-20261104T022039Z-1-001/yolo_best.pt` (6클래스: red/green cylinder/cuboid, box, apriltag. imgsz 800, 10/30 학습). **신호등·바코드 클래스 없음**

## 3. 이번 세션에서 확인된 사실
### 바코드
1. **원인은 해상도.** 640×480에서 관측 거리 라벨 폭 ≈ 55 px → 모듈당 ≈ 0.6 px (Code128 "P2-0x" ≈ 90 모듈) → 막대가 뭉개져 정보 자체가 없음.
2. **라벨 크롭이 핵심.** 어두운 벽 + 좁은 여백 때문에 전체 프레임 디코딩 실패 → 흰 라벨만 잘라 원근 보정하면 읽힘 (720p 영상: 0/663 → 388/663). 현재 `live_mission_optimized.py` 168~335행에 반영됨 (`barcode_regions` → `barcode_label_regions` → `decode_with_reader`).
3. **읽히는 조건: 라벨 폭 ≥ 120 px (≈97% 성공).** 100~119 px ≈ 22%, < 100 px = 0%.
   추정 거리 [가정: 라벨 폭 75 mm, D435 컬러 HFOV 69°]: 640 → 0.29 m, 720p → 0.58 m(실측 일치), 1080p → 0.87 m.
   관측 위치에서 바코드까지 ≈ 0.72 m, 24.6° 비스듬 → 720p ≈ 88 px(실패), 1080p ≈ 132 px(가능, 여유 적음).
4. **1080p 실행에서 잘 잡힘 (사용자 확인):**
   `"$HOME/yolo_env/bin/python" -B scripts/live_mission_optimized.py --auto-start --width 1920 --height 1080 --fps 30 --exposure 150 --wb 4600 --barcode-upscale 2 --out reports/live-optimized-1080p`
5. **확대(upscale)는 2배까지만 의미 있음.** 640 영상(657장) 시험: ×1 2장, ×2 35장, ×3 32장, ×4 33장, ×3 lanczos 38장. 먼 구간은 모두 0. 시간은 배율²에 비례.
6. **2026-11-11 17:45/17:46 영상**(`reports/live-optimized-720p/camera_1794386649848806430.mp4`, `…6734.mp4`)은 폴더명과 달리 **실제 640×480**. 읽힌 74장 모두 가까이 간 구간(라벨 크롭 단계, 67장은 확대 필요).
7. **tutor YOLO apriltag 클래스로 바코드 찾기 → 부적합.** 먼 바코드 0.87/0.41로 우연히 잡지만, 신호등 몸통 0.76·받침 0.63·볼트 0.52 오검출, 가까운 바코드는 놓침. 필요하면 `barcode` 클래스를 새로 학습 (라벨은 디코딩 성공 프레임 꼭짓점으로 자동 생성 가능). 우선순위 낮음.
8. 바코드 디코딩은 별도 스레드에서 5 Hz. **확정 후에도 디코딩을 멈추지 않음** (`process_native` 358~383행) → 신호 관측 중 실패 경로(전 단계)를 계속 돌림.

### 카메라 (ROS)
- `realsense2_camera` 4.57.7 기본 컬러 출력은 보통 640×480@30 [확인 필요]. 1080p를 쓰려면 `rgb_camera.color_profile:=1920x1080x30` 지정 필요.
- 1080p RGB8 30 fps ≈ 187 MB/s → Jetson에서 DDS 부하·프레임 간격 위험 (신호 기록은 0.25 s 초과 시 오류).

### 신호등
- 실기 실행에서 "Unknown signal color"로 실패하는 경우가 잦음 (UNKNOWN 1프레임이면 recorder 즉시 오류). **원인 로그 미기록 → 미해결.**

## 4. 제안된 변경 (모두 미적용, 승인 필요)
| # | 대상 | 내용 | 기대 효과 |
|---|---|---|---|
| C1 | `live_mission_optimized.py` `process_native` | 바코드 확정(`session.barcode` 존재) 후 디코딩 중지 | 신호 관측 중 CPU 부하 제거 |
| C2 | 같은 파일 `decode_with_reader` | 640일 때 "전체 프레임 2배" 단계 생략 (성공 사례 0) | 실패 시 비용 감소 |
| C3 | 같은 파일 입력부 | `.bag`(`rs.config().enable_device_from_file`) 또는 mp4 입력 옵션 | 녹화 영상으로 디코딩 옵션 반복 시험 |
| C4 | 신호 판정 | UNKNOWN 발생 시 원인(칸별 밝기·채도·박스 크기) 로그 기록 | Unknown 실패 원인 분석 |
| C5 | tutor `r12_sensor_ros.py` 등 | Cowork outputs `diffs/*.diff` 적용 여부 결정 | 투영 기반 신호등 판정 |
| C6 | Jetson 런치 | `rgb_camera.color_profile`·노출 150·WB 4600 고정 | 실기 조건 = 시험 조건 |

## 5. 해야 할 테스트 (순서대로)
**T1. 1080p 바코드 거리 한계 (개발 PC)**
- 위 1080p 명령으로 실행, 바코드에서 0.3 → 0.5 → 0.7 → 0.9 m 순으로 멈춰 각 10 s. P2-01~04 각각.
- 결과: `reports/live-optimized-1080p/mission_*.jsonl`의 `decode_stage`, `decode_ms`, 확정 시점.
- 합격 기준 [제안]: 0.75 m에서 5 s 안에 확정, 오확정 0.

**T2. 관측 위치 통합 (신호등 + 바코드 한 화면)**
- 카메라를 (734, 950) 상당 거리·높이로 두고 신호등 OpenCR 전원만 연결(15 s 후 자동 시작, 20 s 간격 반복).
- 확인: 바코드 확정 → 꺼짐 고정 → 12회 기록 → 판단(오름/내림, L1~L3 개수). 핸드폰으로 신호등 같이 촬영해 정답 대조.
- 5회 이상 반복, UNKNOWN 오류 횟수 기록.

**T3. 640 vs 720p vs 1080p 부하 (PC → Jetson)**
- 각 해상도에서 `metrics`의 `decode_ms`, 프레임 간격 최대값, CPU%.
- Jetson에서는 1080p 15 fps도 측정. **프레임 간격 0.25 s 초과 0회**가 조건.

**T4. Unknown signal 원인 (C4 승인 후)**
- 실패 프레임 저장 → 검증 세트(`validation_set_20261110`)와 비교.

**T5. 34점 채점 (선택, 신호등 USB를 PC에 연결할 때만)**
- `python scripts/check_mission_perception.py --barcode P2-01 --model ~/git/p2_perception/models/current.pt --exposure 150 --wb 4600 --rounds 3`
- 결과: `reports/mission_check/<시각>.json`

**T6. Isaac (선택)** — PC `scripts/` 에 이미 있음 (다른 에이전트가 11-12 갱신, 읽기만 확인함)
- `test_mission_camera_isaac.py` (`test_mission_1080p_isaac.py` 는 같은 것의 호환 진입점): 로봇 없이 카메라 시점만 둔 픽셀 시험.
  바코드 원본 해상도, 신호등 `--signal-width` 축소, 실사 런타임과 같은 헬퍼 사용. 노출·WB 는 기록만 하고 렌더에 적용 안 함.
  주요 옵션: `--width/--height`(기본 640×480), `--barcode P2-0x`, `--barcode-distance`(기본 0.15 m), `--tower-distance`(기본 0.30 m), `--barcode-upscale`, `--smoke`, `--auto-start`, `--out`(필수).
  → 거리 기본값이 실제 관측 거리(바코드 ≈ 0.72 m)보다 훨씬 가까움. **0.5 / 0.72 / 0.9 m 를 640·720p·1080p 로 돌려 3장 3번 거리 표와 비교**할 것.
- `test_tower_signal_isaac.py`: 신호등 색 판정 단독 렌더 시험 (`--smoke` 먼저).
- 실행: `/home/pa33/pa-course/.venv-isaacsim/bin/python` (headless), GPU 여유 1 GB 이상. 장시간은 백그라운드 + 로그 파일.
- 결과는 `--out` 폴더(예: `reports/isaac_<날짜>/`)에 남기면 Cowork 쪽에서도 분석 가능.

## 6. 미결정·확인 필요
- 바코드 라벨 실제 폭(mm) — 거리 계산 전부 영향
- 해상도 전략: A) 1080p 상시+15 fps / **B) 720p + 바코드는 0.5 m 이내로 접근(추천)** / C) 바코드 단계에서만 1080p
- 시작 자세가 바코드만 보이는 경우: 바코드 확정 → 관측 자세로 이동 → **정지 후** 꺼짐 고정(박스 흔들림 3 px 초과 시 재시작) → READY. 신호 시작 전 이동 시간이 규정상 허용되는지 확인
- 관측 위치에서 카메라 높이·피치로 바코드(450 mm)와 신호등이 수직 화각(약 ±21°) 안에 같이 들어오는지
- Jetson 런치 파일의 실제 컬러 해상도
- sign_tower_v3: 릴레이 핀 5/6/7 = R/Y/G, 종료 소등 3 s에 마지막 1 s 포함 여부, 퍽 구성 상수(기본 3/3/3/3)
- Unknown signal 원인
