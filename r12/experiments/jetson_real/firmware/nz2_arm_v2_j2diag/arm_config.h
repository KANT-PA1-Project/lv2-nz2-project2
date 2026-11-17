// =====================================================================
//  arm_config.h  -  NZ2 OpenCR#2(팔) 펌웨어 상수 모음. 값은 여기서만 바꾼다.
//  위치: firmware/nz2_arm_v2/arm_config.h  (nz2_arm_v2.ino 와 같은 폴더)
//
//  [설계 원칙] 관절 한계각은 URDF(ARM_LOWER/UPPER, control/defaults.hpp)와 이 파일이 같은 값이어야 한다.
//  한계각을 바꾸면 두 곳을 같이 고치고, 아래 FW_LIMITS_REV 숫자를 올려 Q/? 회신으로 버전 불일치를 눈으로 확인한다.
//
//  모터 순서(모든 배열·회신 공통): [0..3]=관절 ID 11,12,13,14 / [4]=손목 롤 ID 16 / [5]=그리퍼 ID 15
// =====================================================================
#pragma once
#include <stdint.h>

#define FW_VERSION      "2.0"
#define FW_LIMITS_REV   8          // 한계각 상수를 바꿀 때마다 +1 (URDF 쪽과 대조용)

// ---- 모터 ----------------------------------------------------------
#define N_MOTOR   6
#define GRIP_IDX  5
static const uint8_t MOTOR_ID[N_MOTOR] = {11, 12, 13, 14, 16, 15};

// ---- 시험용: 연결된 모터만 쓰기 ---------------------------------------
//  비트 i = 모터 i (순서는 위 MOTOR_ID). 0x3F = 6개 전부(경기용). 단일 모터 시험은 ID 11 만 -> 0x01, 관절 4개 -> 0x0F.
//  마스크에 없는 모터는 "없는 것"으로 취급해 부팅 FAULT·폴링에서 제외한다. ※ 경기 전에는 반드시 0x3F 로 되돌릴 것
#ifndef ARM_ACTIVE_MASK
#define ARM_ACTIVE_MASK 0x3F
#endif

// ---- 다이나믹셀 버스 (OpenCR 1.0 의 DXL TTL 포트) ---------------------
#define DXL_SERIAL    Serial3
static const int      DXL_DIR_PIN  = 84;
static const uint32_t DXL_BAUD     = 1000000;
static const uint32_t USB_BAUD     = 115200;      // Jetson 쪽 arm_bridge 와 동일해야 함 (USB CDC 라 실제로는 무시됨)

// ---- 관절 각도 한계 [deg] (팔 0deg = 모터 raw 180deg) -----------------
//  ★ 확장 예정: 늘릴 관절·목표 각도가 확정되면 여기만 수정한다. 아래 값은 원본(nz2_arm.ino) 그대로. [가정]
// 팔 0도에 해당하는 모터 raw 값 (모터별, 모든 모터의 Homing Offset 은 0 = 공장 기본값 이어야 한다. 위자드로 HO 를 쓰지 않는다).
//  팔 각도[deg] = (raw - 영점) * 360/4096.  순서는 MOTOR_ID 와 같다: {11, 12, 13, 14, 16, 15}
//  - ID11, ID16: 영점이 raw 0/4095 경계 근처(4087/4091)라서 "확장 위치 모드(4)"로 쓴다. 이 모드는 한 바퀴 제한이 없고,
//    부팅(reinit) 때 현재 raw 에서 가장 가까운 영점(4087 + 4096*k)을 골라 그 세션의 영점으로 쓴다.
//    부팅 자세가 영점 기준 [J_MIN-3, J_MAX+3] 도 밖이면 어느 쪽인지 알 수 없으므로 토크 ON 을 거부한다.
//  - ID12/13/14: 영점이 2048 에서 수 tick 어긋난 값(2045/2035/2040)을 펌웨어가 보정.
//  - ID15(그리퍼, 전류기반 위치 모드=멀티턴): 완전 열림 raw 1534 가 팔 0도.
static const int32_t ARM_ZERO_RAW[N_MOTOR] = {4087, 2045, 2035, 2040, 4091, 1534};
// 확장 위치 모드(4)로 쓰는 모터. 이 모터에는 EEPROM 위치 제한이 적용되지 않으므로 펌웨어 클램프가 유일한 한계 방어선이다.
static const bool ARM_EXT_POSITION[N_MOTOR] = {true, false, false, false, true, false};
static const float EXT_BOOT_MARGIN_DEG = 3.0f;   // 부팅 자세가 한계 밖 이만큼까지는 허용 (한계+마진 < 180 이어야 영점 가지가 모호하지 않다)
// 위자드에 들어 있어야 하는 Homing Offset [raw]. 부팅 때 읽어 다르면 그 모터는 사용 금지 (영점이 틀리면 위험).
static const int32_t EXPECT_HOMING_OFFSET[N_MOTOR] = {0, 0, 0, 0, 0, 0};
#define ARM_CHECK_HOMING_OFFSET 1
static const int32_t HOMING_OFFSET_TOL = 2;     // 허용 오차 [raw]
static const float J_MIN[N_MOTOR] = {-175,  -95, -111,  -99, -170, -134};   // [0]=ID11 -175 (케이블 -343~-187, 위치모드 한계 -180) [1]=ID12 -95 (차체닿음 -102 에서 7도 안쪽) [2]=ID13 -111 (관절 끝 -118 에서 7도 안쪽). [3]=ID14 -99 (관절 끝 -106 에서 7도 안쪽). [4]=ID16 -170 (선 팽팽 -362~+264, 위치모드 한계 내) [5]=ID15 그리퍼 -134 (0=완전 열림, 음수=닫힘, 닫힘 끝 -141.6 에서 7도 안쪽)
static const float J_MAX[N_MOTOR] = { 175,  113,   82,  117,  170,   -3};   // [0]=ID11 +175 (케이블 +359~+319) [1]=ID12 +113 (차체닿음 +120 에서 7도 안쪽) [2]=ID13 +82 (관절 끝 +90.5 에서 7도 안쪽) [3]=ID14 +117 (관절 끝 +124 에서 7도 안쪽) [5]=ID15 -3 (열림 끝 0 에서 3도 안쪽). 단 접힌 자세에서는 아래 FOLD_* 가 +0 으로 줄인다
//  Homing Offset: 쓰지 않는다(전부 0). ID11/16 은 확장 위치 모드 + 부팅 때 영점 자동 선택, 나머지는 위 ARM_ZERO_RAW 로 보정
//  그리퍼: 0=완전 열림, 음수=닫힘. 쥐는 각도(닿는 각도): 3cm 퍽(원통 지름 3, 직육면체 정면) -106 / 4.24cm(직육면체 대각선) -82 / 6cm(누운 퍽) -46. 닫는 목표는 닿는 각도보다 약 10도 더 닫는다(전류 제한 안에서 쥐는 힘)

// ---- 접힌 자세 연동 한계 (자세에 따라 달라지는 한계) ---------------------
//  측정: 어깨(ID12) 약 -95 deg + 팔꿈치(ID13) 약 +91 deg 로 링크가 서로 닿는 접힌 자세에서는 손목(ID14)이 +6.5 deg 까지만 돈다.
//  그래서 "팔꿈치 > FOLD_ELBOW_DEG 이고 어깨 < FOLD_SHOULDER_DEG" 인 접힘 영역에서는 손목 최대를 FOLD_WRIST_MAX_DEG 로 줄이고,
//  손목이 FOLD_WRIST_ENTRY_DEG 보다 크게 펴져 있으면 접힘 영역에 들어가지 못하게 팔꿈치 목표를 FOLD_ELBOW_DEG 에서 멈춘다.
//  [가정] 경계값 45 / -80 은 접힌 자세(+91 / -95)보다 바깥으로 잡은 보수적 값. 실측으로 조정할 것. 끄려면 ARM_FOLD_INTERLOCK 을 0 으로
#define ARM_FOLD_INTERLOCK 1
static const float FOLD_ELBOW_DEG      = 45.0f;    // 팔꿈치(ID13)가 이 값보다 크면
static const float FOLD_SHOULDER_DEG   = -80.0f;   // 어깨(ID12)가 이 값보다 작으면 접힘 영역
static const float FOLD_WRIST_MAX_DEG  = 0.0f;     // 접힘 영역에서 손목(ID14) 최대 (측정 한계 +6.5)
static const float FOLD_WRIST_ENTRY_DEG = 3.0f;    // 손목이 이 값 이하일 때만 접힘 영역에 진입 허용
static const uint32_t FOLD_POS_MAX_AGE_MS = 300;   // 현재 위치 캐시가 이보다 오래되면 "모름"으로 보고 안전 쪽(접힘으로 간주)
// 모터 EEPROM(Min/Max Position Limit)은 펌웨어 한계보다 이만큼 "바깥"에 둔다.
// 펌웨어 클램프가 먼저 걸리고, 펌웨어 버그/수동 조작 시에는 EEPROM 한계가 2차 방어선이 된다.
static const float EEPROM_LIMIT_MARGIN_DEG = 2.0f;
#define ARM_WRITE_EEPROM_LIMITS 1   // 컴파일 에러(ControlTableItem 이름 없음)가 나면 0 으로 두고 위자드로 직접 설정

// ---- 모터 설정 -------------------------------------------------------
static const float CURRENT_UNIT_MA = 2.69f;       // XM430 전류 단위 [mA/LSB]
// 전류 제한(토크 상한) [mA] [가정]: 충돌 시 힘을 제한한다. 무부하로 움직여 Q 의 전류를 본 뒤 +30% 여유로 조정.
static const float CURRENT_LIMIT_MA[N_MOTOR] = {600, 1200, 900, 600, 400, 300};
// 위치 모드에서 실제 속도 상한 = Profile Velocity (단위 0.229 rpm). 60 ≈ 13.7 rpm ≈ 82 deg/s
static const int32_t PROFILE_VEL[N_MOTOR] = {60, 60, 60, 60, 60, 60};
static const int32_t PROFILE_ACC[N_MOTOR] = {20, 20, 20, 20, 20, 20};   // 단위 214.577 rev/min^2
// 비상정지 때만 쓰는 큰 가속도: 20 이면 82 deg/s 에서 멈추는 데 약 0.19 s, 약 8 deg 더 간다 -> 200 이면 약 0.02 s
static const int32_t ESTOP_PROFILE_ACC = 200;

// ---- 그리퍼 (전류 기반 위치 모드: 퍽을 쥐어도 Overload 가 안 난다) -------
static const float GRIP_CURRENT_DEFAULT_MA = 150.0f;   // [가정] 시작 시 쥐는 힘. 퍽 실측 후 조정

// ---- 안전 타이밍 -----------------------------------------------------
static const uint32_t WATCHDOG_MS        = 150;   // 명령(A/G/K) 두절 판정 [ms]. control_master 와 동일(결정 사항)
static const uint32_t POLL_MS            = 10;    // 모터 1개씩 순환 폴링 간격 (6개 일주 60 ms)
static const uint8_t  COMM_FAIL_LIMIT    = 3;     // 연속 통신 실패 N회면 모터 오프라인(전원 차단/단선)으로 판정
static const uint32_t POS_CACHE_MAX_AGE_MS = 100; // 비상정지 때 읽기 실패 시 캐시 위치를 써도 되는 최대 나이
static const int      ESTOP_RETRY_MS     = 20;    // 비상정지 정지 쓰기가 실패한 모터 재시도 간격

// ---- 하드웨어 비상정지 입력 (OpenCR GPIO) -----------------------------
//  ESTOP_PIN = -1 이면 사용 안 함 (배선 여부 미확정). 배선하면 핀 번호를 넣는다.
//  권장 배선: NC(상시 닫힘) 접점 -> 핀과 GND. INPUT_PULLUP 이므로 눌림/선 끊김 = HIGH.
static const int      ESTOP_PIN          = -1;
static const int      ESTOP_ACTIVE_LEVEL = 1;     // HIGH 가 비상정지 (NC 배선 기준)
static const uint32_t ESTOP_DEBOUNCE_MS  = 5;

// ---- 선택: 다이나믹셀 Bus Watchdog (OpenCR 자체가 멎을 때의 마지막 방어선) ----
//  0 = 사용 안 함. 1 로 켜기 전에 모터 1개로 동작을 먼저 확인할 것 (README 시험 절차 참고).
#define ARM_USE_BUS_WATCHDOG 0
static const int32_t BUS_WATCHDOG_UNITS = 15;     // 20 ms 단위 -> 300 ms

// ---- 부팅 정책 -------------------------------------------------------
//  true : 부팅 직후 토크 OFF + 래치(LOCKED). 사람이 'R' 후 'T 1' 을 보내야 움직인다 (스월브 OpenCR#1 과 같은 정책)
//  false: 원본처럼 부팅 즉시 토크 ON
static const bool     BOOT_LATCH         = true;

// ---- 시리얼 줄 길이 ---------------------------------------------------
#define LINE_MAX_LEN 96
