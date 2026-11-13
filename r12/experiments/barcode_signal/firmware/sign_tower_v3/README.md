# sign_tower_v3 — 신호등 단독 구동 (PC 연결 불필요)

OpenCR 에 업로드해 두면 이후에는 **OpenCR 전원 + 신호등 릴레이 배선만으로** 혼자 동작한다.
규칙은 `mission.pdf` v5 02·04항: 12회 점멸, 1회 = 1 s 켜짐 + 1 s 소등, 마지막 점멸 후 3 s 연속 소등, 색별 1~6회.
`sign_tower_v2` 는 그대로 두고 새로 만든 버전이다 (v2 = PC 시리얼로 조작, v3 = 단독 자동).

## 동작

| 시점 | 동작 |
|---|---|
| 전원 ON | 전체 소등, `FIRST_DELAY_MS`(15 s) 대기 → 로봇이 관측 자세·꺼짐 고정할 시간 |
| 재생 | 무작위 12회 (1 s 켜짐 / 1 s 소등) → 마지막 후 3 s 소등 |
| 반복 | `REPEAT_GAP_MS`(20 s) 소등 후 **새 무작위**로 다시 재생 (`AUTO_REPEAT=false` 면 1회만) |
| SW1 버튼 | 대기 건너뛰고 바로 시작 (새 무작위) |
| SW2 버튼 | 즉시 전체 소등·정지 (SW1 로 재시작) |
| OpenCR LED | LED1 깜빡임 = 대기 중, LED2 = 재생 중, LED4 = 설정 오류 |

## 무작위 패턴 규칙
- R·Y·G 각 1~6회, 합계 12
- **실제 퍽 구성으로 풀 수 있는 조합만** 선택: Y 에 쓰는 빨강(빨강총수−R)·초록(초록총수−G)이 한 모양(원기둥 또는 직육면체) 개수 안에 들어가야 함
- 가능한 (R,Y,G) 조합 중 하나를 고르게 뽑고 순서를 섞음. 전원마다 다른 패턴 (아날로그 잡음 + 시각으로 씨앗)
- 퍽 구성은 `sign_tower_v3.ino` 위쪽 `PUCK_RED_CYL / PUCK_RED_BOX / PUCK_GREEN_CYL / PUCK_GREEN_BOX` (합계 12).
  기본값 3/3/3/3 은 [가정] (공개 예제 빨강6·초록6). **회차 등록표대로 바꿔서 업로드**
- 항상 같은 패턴이 필요하면 `FIXED_SEQ = "RGRRGRGGYGYG"` 처럼 넣고 업로드

## 업로드 (개발 PC, 1회만)
v2 README 의 Arduino IDE / arduino-cli 절차와 같다. 저장소 루트에서:
```bash
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR firmware/sign_tower_v3
arduino-cli upload  -p /dev/ttyACM0 --fqbn OpenCR:OpenCR:OpenCR firmware/sign_tower_v3
```
- **신호등용 OpenCR 만 PC 에 연결한 상태에서** 업로드 (스월브·팔 OpenCR 에 잘못 올리면 모터 펌웨어가 지워짐)
- 업로드 후 USB 를 빼고 OpenCR 전원(12 V 또는 USB 전원 어댑터)만 연결하면 15 s 뒤 시작

## 정답 기록 (채점 대조용)
PC 없이 돌리면 어떤 패턴이었는지 기록이 남지 않는다. 필요할 때 셋 중 하나:
1. 노트북을 USB 로 연결해 두고 기록: `python3 -m serial.tools.miniterm /dev/ttyACM0 115200 | tee tower_log.txt`
   (`SEQ,<12글자>,R=,Y=,G=` 와 `EVT,<ms>,<번호>,<색>,ON|OFF` 출력. USB 를 꽂으면 OpenCR 이 재시작될 수 있음 → 꽂은 뒤 시작)
2. `FIXED_SEQ` 로 정해 둔 패턴 사용
3. 신호등을 핸드폰으로 같이 촬영

시리얼 명령(연결했을 때만): `START` `STOP` `AUTO 1|0` `RAND` `Q1` `SEQ <12글자>` `PUCKS rc rb gc gb` `LAMP R|Y|G|OFF` `STATUS` `HELP`
(`PUCKS` 는 전원을 끄면 지워짐. 영구 변경은 상수 수정 후 업로드)

## 시험 순서
1. 램프 연결 전: USB 연결 → `STOP` → `LAMP R` / `LAMP Y` / `LAMP G` 로 릴레이 순서 확인
2. 타이밍: 전원 ON → 15 s 뒤 시작, 로그 EVT 간격 1000 ms, `DONE,26000` 확인
3. SW2 정지 → 즉시 전체 소등, SW1 → 바로 새 패턴 시작
4. USB 없이 전원만: 15 s 대기 후 자동 재생, 20 s 간격으로 반복되는지

호스트(PC) 시뮬레이션으로 확인한 것: 점등은 항상 1개 이하, EVT 1000 ms 간격, 시작~DONE 26 s, 반복 간격 20 s,
기본 퍽 3/3/3/3 에서 가능한 15개 조합을 고르게 선택(2만 회, 풀이 불가 0), 빨강8·초록4 구성에서는 그 구성으로 가능한 조합만 생성,
풀 수 없는 `SEQ` 거부. 실제 OpenCR 컴파일·업로드는 아직 안 함.

## 확인 필요
- 릴레이 핀 5·6·7 = R·Y·G 순서 (v2 와 같은 [가정])
- 3 s 종료 소등에 마지막 점멸의 소등 1 s 포함 여부 ([가정] 포함. 아니면 `END_OFF_MS = 4000`)
- 회차 퍽 구성 (PUCK_* 상수)
- `FIRST_DELAY_MS` 15 s 가 로봇 준비(관측 자세·바코드·꺼짐 고정)에 충분한지. 실제 경기에서는 운영자가 시작하므로 SW1 로 시작하고 `AUTO_RUN` 대기를 길게 두는 편이 안전
