// =============================================================
// 보드   : 스월브 왼쪽 OpenCR (FL·RL: 구동 2 + 조향 2 + IMU)
// 파일   : config.h
// 역할   : 모터 ID·단위·부팅 기본값·하드 상한. 코드 본문에 숫자를 직접 쓰지 않는다.
//
// [값의 출처]
//   - 기본값은 handoff "OpenCR1 스월브 구동+조향 펌웨어" §2·§4 와 같다.
//   - 단일 출처는 bringup/config/swerve_drive.yaml 이다(아직 모듈 4개 기준). 호스트(브리지)가
//     부팅 후 'P' 명령으로 YAML 값을 내려보내면 여기 기본값을 덮어쓴다.
//     YAML 을 바꾸면 이 파일을 고칠 필요가 없다. 여기 값은 YAML 이 오기 전
//     안전한 출발점일 뿐이다. 모듈별 값(4개)은 브리지가 이 보드 몫 2개만 골라 보낸다.
//   - HARD_* 상한은 P 명령으로도 넘을 수 없다(호스트 버그 방어).
//   - [확인] 표시는 handoff §8 미결 항목이다. 지면/조립 보정 후 YAML 로 확정한다.
// =============================================================
#pragma once

#include <stdint.h>

#define FW_NAME    "opencr_swerve_left"
#define BOARD_SIDE "left"   // cfg 의 "side". 브리지가 포트와 보드를 맞춰 보는 용도
// 핸드셰이크 식별자. 'I' 응답과 cfg 의 "board". USB 허브에서 ttyACM 번호가 바뀌어도
// 호스트가 이 값으로 보드를 찾는다. 보드마다 반드시 달라야 한다(중복이면 브리지가 경고).
#define BOARD_ROLE "swerve_left"
#define BOARD_KIND "swerve"  // 같은 펌웨어 계열. arm 보드는 "arm"
#define FW_VERSION "0.2.2"

// ---------------- 버스 / 호스트 ----------------
constexpr uint32_t DXL_BAUDRATE         = 1000000;  // [확인] 모터 설정과 일치해야 함
constexpr float    DXL_PROTOCOL_VERSION = 2.0f;     // XM430 = 프로토콜 2.0
constexpr uint32_t HOST_BAUDRATE        = 115200;   // OpenCR USB CDC (실제 속도와 무관)

// ---------------- 모듈 매핑 (왼쪽 보드) ----------------
// 순서: FL, RL (앞, 뒤). 'C' 명령과 피드백 "mod" 배열이 모두 이 순서를 따른다.
// 구동 FL1 FR2 RR3 RL4 (특성화), 조향 FL7 FR6 RR8 RL5 (2026-11-12 실물 확인. 예전 handoff 표와 다르다).
constexpr uint8_t MODULE_COUNT = 2;
constexpr char    MODULE_NAMES[MODULE_COUNT][3] = {"FL", "RL"};
constexpr uint8_t DRIVE_IDS[MODULE_COUNT] = {1, 4};
constexpr uint8_t STEER_IDS[MODULE_COUNT] = {7, 5};  // FL=7, RL=5 (실물 확인). home·sdir 벤치 전 미확정

// ---------------- 기능 스위치 ----------------
// 조향: true이면 조향 모터도 초기화·피드백·토크·위치 명령을 사용한다.
// 자가진단 중에는 모든 조향 목표를 0 rad로 유지하고 조향 토크를 켠다.
// 실제 모터 ID·배선·회전 방향·기계적 0도 위치를 확인한 뒤 사용한다.
constexpr bool STEER_ENABLE = true;
// IMU: 왼쪽·오른쪽 보드 모두 자체 IMU를 초기화하고 50 Hz로 전송한다.
constexpr bool IMU_ENABLE   = true;

// ---------------- 기구 (URDF r20d) ----------------
constexpr float GEAR_G         = 4.0f;    // 증속: 바퀴 = 모터 × 4
constexpr float WHEEL_RADIUS_M = 0.033f;  // 참고용(펌웨어는 각속도만 다룸)

// ---------------- XM430 컨트롤 테이블 단위 (ROBOTIS e-manual) ----------------
constexpr float   RPM_PER_LSB        = 0.229f;    // Present/Goal Velocity
constexpr float   MA_PER_LSB         = 2.69f;     // Present Current, Current Limit
constexpr float   VOLT_PER_LSB       = 0.1f;      // Present Input Voltage
constexpr int32_t TICKS_PER_REV      = 4096;      // 위치 해상도
constexpr int32_t STEER_CENTER_RAW   = 2048;      // 조향 0 rad 기준 틱 (home_offset 이 보정)
constexpr int32_t POSITION_RAW_MIN   = 0;         // Position 모드(3) 단일회전 범위
constexpr int32_t POSITION_RAW_MAX   = 4095;

// ---------------- 구동 기본값 (handoff §4 drive) ----------------
constexpr float   DEF_MAX_MOTOR_RPM   = 65.0f;    // 포화 ~70rpm@12V 바로 아래
constexpr float   DEF_MIN_MOTOR_RPM   = 1.0f;     // 미만 명령은 0 (미세 데드밴드)
constexpr float   DEF_CMD_GAIN        = 1.0f;     // 추종 ~1:1
constexpr float   DEF_CMD_OFFSET_RPM  = 0.0f;
constexpr float   DEF_MODULE_TRIM[MODULE_COUNT]  = {1.0f, 1.0f};
constexpr int8_t  DEF_DRIVE_DIR[MODULE_COUNT]    = {1, 1};   // [확인] 지면 전진부호
constexpr float   DEF_CURRENT_LIMIT_A = 1.5f;     // [지면 튜닝] 구동 4개(좌·우 보드 합)×1.5=6A < 배터리 9A
// 구동 Profile Acceleration raw (1 = 214.577 rev/min²).
// 10 → 모터 약 36 rpm/s → 바퀴 표면 약 0.5 m/s² (0→65rpm 약 1.8 s). "gentle" 출발값.
constexpr uint32_t DEF_DRIVE_PROFILE_ACC = 10;    // [지면 튜닝]

// ---------------- 조향 기본값 (handoff §4 steer) ----------------
constexpr float  DEF_STEER_RANGE_RAD = 1.5708f;   // ±90° (URDF revolute)
constexpr float  DEF_STEER_HOME_RAD[MODULE_COUNT] = {0.0f, 0.0f};  // [확인]
constexpr int8_t DEF_STEER_DIR[MODULE_COUNT]      = {1, 1};  // [확인]
// 조향 프로파일: 목표는 가능한 한 빠른 조향.
// 속도 330 = 75.6 rpm = XM430-W210 Velocity Limit 기본값(12V 무부하 77 rpm). 사실상 최고속.
// 가속 300 ≈ 1073 rpm/s → 최고속까지 약 0.07 s. 90° 이동 약 0.27 s.
// 0 으로 두면 Dynamixel 규칙대로 속도 0 = 프로파일 없음(Step), 가속 0 = 무한 가속.
// 더 빠르지만 조향 동시 전류 스파이크가 커진다. 피드백 ia 를 보며 조정. [벤치·지면 튜닝]
constexpr uint32_t DEF_STEER_PROFILE_VEL = 330;   // 1 = 0.229 rpm
constexpr uint32_t DEF_STEER_PROFILE_ACC = 300;   // 1 = 214.577 rev/min²

// ---------------- 안전 기본값 (handoff §4 safety) ----------------
constexpr uint32_t DEF_WATCHDOG_MS   = 500;
constexpr float    DEF_TEMP_WARN_C   = 50.0f;
constexpr float    DEF_TEMP_DERATE_C = 65.0f;
constexpr float    DEF_TEMP_ABORT_C  = 70.0f;
constexpr float    DEF_BATT_WARN_V   = 11.1f;
constexpr float    DEF_BATT_DERATE_V = 10.5f;
constexpr float    DEF_BATT_STOP_V   = 10.0f;
// derate 시 속도 상한·가속에 곱하는 비율. handoff 에 값이 없어 둔 출발값. [지면 튜닝]
constexpr float    DEF_DERATE_SCALE  = 0.5f;
// 소프트웨어 과전류 트립: |전류| > current_limit 이 이 시간 이상 이어지면 정지.
// XM430 Current Limit(38) 레지스터는 Velocity(1)·Position(3) 모드에서 적용되지
// 않으므로(e-manual) 펌웨어가 직접 감시한다. [지면 튜닝]
constexpr uint32_t DEF_OVERCURRENT_TRIP_MS = 300;

// ---------------- 휴면(Auto-Sleep) 기본값 ----------------
constexpr uint32_t DEF_IDLE_TIMEOUT_MS = 5000; // 5초 동안 C 명령이 없으면 자동 토크 OFF
constexpr uint32_t HARD_MAX_IDLE_TIMEOUT_MS = 60000;

// ---------------- 하드웨어 에러 자동복구 ----------------
// 모터가 하드웨어 에러(과부하·과열·입력전압·전기충격·엔코더)로 스스로 토크를 끄면
// REBOOT 명령으로만 풀린다(e-manual). 펌웨어가 조건을 확인한 뒤 그 모터만
// 재부팅 → 재설정 → 토크 ON 한다. 복구 중에는 구동 0, 조향 유지.
constexpr uint8_t  DEF_HW_RETRY_MAX      = 3;      // 연속 시도 상한. 넘으면 E 로 복구. 0 = 자동복구 끔
constexpr uint32_t DEF_HW_RETRY_DELAY_MS = 1000;   // 마지막 에러 관측 후 기다리는 시간
constexpr uint32_t HW_RETRY_RESET_MS     = 30000;  // 이 시간 에러가 없으면 시도 횟수를 0 으로
constexpr uint32_t HW_REBOOT_WAIT_MS     = 500;    // 재부팅 뒤 첫 응답 확인까지 대기 [벤치 확인]
constexpr uint32_t HW_REBOOT_TIMEOUT_MS  = 3000;   // 이 안에 응답·설정이 안 되면 그 시도는 실패

// ---------------- 하드 상한 (P 명령으로도 못 넘음) ----------------
constexpr float    HARD_MAX_MOTOR_RPM   = 70.0f;    // 무부하 포화 ~70rpm@12V
constexpr float    HARD_STEER_RANGE_RAD = 1.5708f;  // URDF ±90°
constexpr float    HARD_MAX_CURRENT_A   = 2.25f;    // 배터리 9A ÷ 구동 4(좌·우 보드 합). [확인] 전원 구성이 바뀌면 다시 정한다
constexpr float    HARD_TEMP_ABORT_MAX_C = 79.0f;   // 80°C = HW 과열 셧다운, 절대 도달 금지
constexpr float    HARD_BATT_STOP_MIN_V = 9.0f;     // 3S LiPo 셀당 3.0V
constexpr uint32_t HARD_MAX_WATCHDOG_MS = 500;
constexpr float    HARD_MIN_DERATE_SCALE = 0.1f;

// ---------------- 주기 ----------------
constexpr uint32_t CONTROL_PERIOD_MS  = 10;   // 100 Hz: 동기 읽기 → 안전 → 동기 쓰기
constexpr uint32_t FEEDBACK_PERIOD_MS = 20;   // 50 Hz 상태 JSON
constexpr uint32_t IMU_PERIOD_MS      = 20;   // 50 Hz IMU JSON [interfaces 와 맞춤]
constexpr uint32_t HWERR_PERIOD_MS    = 200;  // Hardware Error Status 동기 읽기
constexpr uint32_t BATT_PERIOD_MS     = 100;  // 보드 입력 전압 샘플
constexpr uint8_t  BATT_STOP_SAMPLES  = 3;    // 연속 3샘플(0.3 s) ≤ stop 이면 정지
constexpr float    BATT_EMA_ALPHA     = 0.2f; // warn/derate 판단용 평활
constexpr uint8_t  COMM_FAIL_MAX      = 5;    // 연속 동기읽기 누락 허용 횟수
constexpr uint32_t DXL_READ_TIMEOUT_MS = 3;   // 동기읽기 모터당 응답 대기

// ---------------- IMU (OpenCR 내장 ICM-20648, 코어 1.5.3 설정: ±2g, ±2000dps) ----------------
// 환산 계수는 OpenCR 코어의 turtlebot3_ros2 와 같은 값(데이터시트 감도 16384 LSB/g, 16.4 LSB/dps).
constexpr float IMU_ACC_MPS2_PER_LSB = 0.000598550415f;  // 9.80665 / 16384
constexpr float IMU_GYRO_RADS_PER_LSB = 0.0010642f;      // (1/16.4) × π/180

constexpr uint32_t MISSING_PROBE_MS   = 500;  // 미검출 모터 재확인(ping) 주기

// ---------------- 상태 표시: LED·버저 (handoff §5.6) ----------------
// 핀 이름은 OpenCR 코어(chip.h) 정의. 색 배치와 켜짐 레벨은 문서끼리 달라 [벤치 확인]:
//   e-manual: 사용자 LED 는 LD3 빨강·LD4 초록·LD5 파랑(어느 USER 번호인지 표기 없음), HIGH = 켜짐.
//   코어 variant.cpp setLedOn(): LOW 를 써서 켠다.
// 벤치에서 색이 다르면 아래 핀만, 반대로 켜지면 LED_ACTIVE_LOW 만 바꾼다.
constexpr int  LED_RED_PIN       = BDPIN_LED_USER_1;  // [벤치 확인]
constexpr int  LED_GREEN_PIN     = BDPIN_LED_USER_2;  // [벤치 확인]
constexpr int  LED_BLUE_PIN      = BDPIN_LED_USER_3;  // [벤치 확인]
constexpr int  LED_HEARTBEAT_PIN = BDPIN_LED_STATUS;  // STATUS LED (부트로더도 씀)
constexpr bool LED_ACTIVE_LOW    = true;              // [벤치 확인]
constexpr int  BUZZER_PIN        = BDPIN_BUZZER;      // tone()/noTone()

// 연결 폴트(짧은 비프 1/2/3회) 전용 패턴. 다른 폴트와 소리 성격을 다르게 한다.
constexpr uint16_t BEEP_SHORT_HZ    = 2000;
constexpr uint32_t BEEP_SHORT_ON_MS = 120;
constexpr uint32_t BEEP_SHORT_GAP_MS = 150;
constexpr uint32_t BEEP_GROUP_PAUSE_MS = 1000;
// 치명(과열·저전압): 끊김 없이 빠른 연속음
constexpr uint16_t BEEP_CRIT_HZ     = 3000;
constexpr uint32_t BEEP_CRIT_ON_MS  = 80;
constexpr uint32_t BEEP_CRIT_OFF_MS = 80;
// 기타(IMU 무응답·과전류·HW 에러·통신 두절·워치독·비상정지·자가진단 실패): 긴 삐 반복
constexpr uint16_t BEEP_LONG_HZ     = 1200;
constexpr uint32_t BEEP_LONG_ON_MS  = 600;
constexpr uint32_t BEEP_LONG_PERIOD_MS = 2000;
// 배터리 경고(≤ batt_warn): TurtleBot3 처럼 주기적 단비프, 낮은 음으로 연결 폴트와 구분
constexpr uint16_t BEEP_BATT_HZ     = 600;
constexpr uint32_t BEEP_BATT_ON_MS  = 60;
constexpr uint32_t BEEP_BATT_PERIOD_MS = 10000;
// ① READY: POST 통과 + 시리얼 준비. 짧고 경쾌한 상승 3음 도–미–솔(C6 E6 G6), 약 0.3 s.
constexpr uint8_t  READY_JINGLE_N = 3;
constexpr uint16_t READY_JINGLE_HZ[READY_JINGLE_N] = {1047, 1319, 1568};
constexpr uint16_t READY_JINGLE_MS[READY_JINGLE_N] = {100, 100, 120};
// ② BRIDGE LINKED: 첫 유효 프레임. 징글보다 짧고 높은 2음 틱 "띠릭" (0 Hz = 쉼).
constexpr uint8_t  LINK_TICK_N = 3;
constexpr uint16_t LINK_TICK_HZ[LINK_TICK_N] = {2600, 0, 3400};
constexpr uint16_t LINK_TICK_MS[LINK_TICK_N] = {40, 30, 50};
// USB 전원만 있을 때(배터리 없음)는 저전압 치명음을 내지 않는다(벤치에서 계속 울리지 않게).
constexpr float    NO_BATTERY_V     = 6.0f;

// ---------------- 버튼 자가진단: 주행 시험 ----------------
// SW1 을 3초 길게 누르면 최고속도로 앞으로 SELFTEST_DISTANCE_M, 멈췄다가 뒤로 같은 거리를 간다.
// 로봇이 실제로 움직인다. 앞뒤로 빈 공간을 확보하거나 바퀴를 띄운 벤치에서 한다. 시리얼로는 시작할 수 없다.
// 이 보드는 자기 쪽 구동 2개만 돌린다. 지면에서 한쪽 보드만 실행하면 차체가 돈다.
constexpr int      BTN_SELFTEST_PIN   = BDPIN_PUSH_SW_1;  // 3 s 길게 눌러 시작
constexpr int      BTN_STOP_PIN       = BDPIN_PUSH_SW_2;  // 누르면 즉시 중단
constexpr bool     BTN_ACTIVE_HIGH    = true;             // [벤치 확인] 눌렀을 때 HIGH
constexpr uint32_t BTN_DEBOUNCE_MS    = 30;
constexpr uint32_t SELFTEST_HOLD_MS   = 3000;
constexpr float    SELFTEST_DISTANCE_M = 10.0f;           // 앞으로 가는 거리 = 뒤로 가는 거리
// 목표 속도는 max_rpm(derate 반영)이지만 이 값을 넘지 않는다. 0.6 m/s = 프로젝트 주행 속도 상한(AGENTS.md).
// 10 m 에서는 기본 가속(pacc 10)으로도 이 속도에 닿아 대부분의 구간을 이 속도로 달린다(한 방향 약 18 s).
constexpr float    SELFTEST_MAX_SPEED_MPS = 0.6f;
constexpr float    SELFTEST_DIST_TOL_M = 0.05f;           // 간 거리·돌아온 위치 허용 오차
// 한 방향 시간 제한 = (거리 ÷ 목표 속도) × 배수 + 여유. 넘으면 불합격(걸림·미연결·거리 미달).
// 10 m, 0.6 m/s 면 약 38 s.
constexpr float    SELFTEST_TIMEOUT_FACTOR = 2.0f;
constexpr uint32_t SELFTEST_TIMEOUT_MARGIN_MS = 5000;
constexpr float    SELFTEST_MIN_SPEED_MPS = 0.05f;        // 목표 속도가 이보다 느리면 시작하지 않는다(끝나지 않음)
constexpr float    SELFTEST_STOP_RPM  = 1.0f;             // 이보다 느리면 멈춘 것으로 본다(모터 rpm)
constexpr uint32_t SELFTEST_SETTLE_MS = 300;              // 멈춘 뒤 방향을 바꾸기 전 기다리는 시간
constexpr float    SELFTEST_LATENCY_S = 0.04f;            // 제동 거리 계산에 더하는 제어 지연 여유(4주기)
constexpr float    SELFTEST_STEER_TOL_DEG = 2.0f;         // 출발 전 조향 정면 정렬 허용 오차 (조향이 켜져 있을 때)
constexpr uint32_t SELFTEST_STEER_TIMEOUT_MS = 1000;      // 이 시간 안에 정렬되지 않으면 중단
constexpr float    PROFILE_ACC_REV_MIN2_PER_LSB = 214.577f;  // Profile Acceleration 1단위 (rev/min²)
// 아래 전류 기준선과 허용 배수는 예전 자가진단(한 모터씩 10 rpm)이 쓰던 값이다.
// 지금 주행 시험은 쓰지 않지만 P st_nl / st_tol 과 YAML 이 그대로 보내므로 받아서 저장만 한다.
constexpr float    DEF_NOLOAD_CURRENT_MA[MODULE_COUNT] = {140.0f, 74.0f};
constexpr float    DEF_SELFTEST_CURRENT_TOL = 2.0f;

// ---------------- 무선 리모컨 비상정지 (TurtleBot3 RC-100 + BT-410) ----------------
// BT-410(수신 쪽)을 OpenCR UART 포트에 꽂으면 Serial2 57600 으로 버튼 패킷이 들어온다
// (OpenCR 코어 RC100 라이브러리). 마스크에 든 버튼을 누르면 비상정지(flt 32).
// 수신기가 없으면 아무것도 들어오지 않으므로 켜 두어도 영향 없다.
// 주의: 리모컨은 버튼을 누를 때만 패킷을 보내서, 연결이 끊긴 것을 펌웨어가 알 수 없다.
//       시험 전마다 버튼을 눌러 flt 32 가 뜨는지 확인하고 E 로 푼다.
constexpr bool     RC100_ESTOP_ENABLE = true;
constexpr int      RC100_PORT         = 1;       // 1 = Serial2 (TurtleBot3 기본 연결). 아래와 맞출 것
#define            RC100_SERIAL         Serial2
constexpr uint16_t RC100_ESTOP_MASK   = 0x03FF;  // 모든 버튼(U D L R 1~6). 급할 때 아무거나 눌러도 선다

// ---------------- 비상정지 입력 핀 (배선 확정 전 비활성) ----------------
constexpr int  ESTOP_PIN          = -1;     // -1 = 사용 안 함
constexpr bool ESTOP_ACTIVE_LOW   = true;   // [확인] 배선 확정 후
