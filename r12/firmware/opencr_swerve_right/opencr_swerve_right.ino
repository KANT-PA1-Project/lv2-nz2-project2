// 보드   : 스월브 오른쪽 OpenCR (FR·RR: 구동 2 + 조향 2 + IMU)
// 파일   : opencr_swerve_right.ino
// 근거   : handoffs/HANDOFF_opencr1_swerve_drive_firmware.md (tutor 기준)
//          opencr1_swerve(보드 1개·모듈 4개)를 좌/우 보드용으로 나눈 것이다.
//          좌·우 스케치는 따로 관리하는 별도 사본이다. 한쪽을 고치면 다른 쪽도 같이 고친다.
// 역할   : Jetson(ROS) 이 IK 로 만든 모듈별 명령을 그대로 서보하고 상태를 돌려준다.
//          구동(ID 2·3) = Velocity 모드, 조향(ID 6·8) = Position 모드. IK 는 하지 않는다.
//          모듈 순서는 항상 FR, RR (앞, 뒤). 반대쪽 모듈은 다른 OpenCR 이 맡는다.
//          조향 모터는 STEER_ENABLE = true 일 때 제어한다. 자가진단 중에는 0 rad를 유지하고 토크를 켠다.
//          자가진단이 끝나거나 중단돼도 토크는 끄지 않는다(구동 0 = 능동 제동, 조향 0 rad 유지).
//          실제 ID·배선·방향을 확인한 뒤 사용한다.
//
// [시리얼 프로토콜] USB CDC, 한 줄 = 명령 1개 / 레코드 1개. 최종 포맷은 interfaces 패키지 소유.
//  호스트 → OpenCR
//   C <en> <wFR> <wRR> <sFR> <sRR>
//        w = 바퀴 각속도 rad/s, s = 조향각 rad, en = 0/1. 50~100 Hz 로 보낸다.
//        유효한 C 만 워치독을 갱신한다(keep-alive 겸용).
//        STEER_ENABLE = false 여도 s 값은 받아서 형식만 검사하고 쓰지 않는다(프레임 형식은 같다).
//        조향이 ±range 밖이거나 숫자가 아니면 프레임 전체를 거부하고 구동 0, 조향 유지.
//   E    토크 ON: 하드웨어 에러 모터 재부팅 → 전 모터 재설정 → 해제 조건 확인 → 래치 고장 해제
//        → 조향 현재각 유지로 켬.
//   D    토크 OFF.
//   S    구동 0 (조향 유지). 다음 C 가 오면 다시 따른다.
//   ?    설정과 레지스터 확인값(cfg) 송신.
//   I    핸드셰이크. {"hello":1,"proto":1,"board":"swerve_right",...} 한 줄.
//        USB 허브에서 ttyACM 번호가 바뀌어도 호스트가 이 응답으로 보드를 찾는다.
//        설정 조회(?)와 달리 모터 레지스터를 읽지 않아 즉시 답한다. 상태 전송이 아니다.
//   P <key> <값...>  파라미터 설정. 토크 OFF 일 때만. 브리지가 부팅 후 YAML 값을 내려보낸다.
//        key: max_rpm min_rpm gain offset trim(2) ddir(2) cur_a pacc
//             range home(2) sdir(2) spvel spacc wd_ms temp(3) batt(3) derate oc_ms
//             hw_retry hw_retry_ms st_nl(2) st_tol
//        (2)=FR RR 순서 2값, temp=warn derate abort, batt=warn derate stop
//  OpenCR → 호스트 (JSON 한 줄)
//   상태 50 Hz : {"t":ms,"bv":V,"ia":A,"en":0/1,"tq":0/1,"wd":0/1,"flt":bits,"wrn":bits,"ds":배율,
//                 "rcv":자동복구 중 0/1,
//                 "mod":[{"c":"FR","ok":1,"wv":rad/s,"sp":rad,"cur":mA,"tc":°C,"e":hw_err,
//                         "scur":mA,"stc":°C,"se":hw_err}, RR]}
//                 cur/tc/e = 구동 모터, scur/stc/se = 조향 모터, ia = 이 보드 모터 전류 절대값 합.
//                 sp/scur/stc/se 는 STEER_ENABLE = true 일 때만 나온다.
//   IMU 50 Hz  : {"t":ms,"board":"right","imu":{"ok":1,"q":[w,x,y,z],"g":[rad/s×3],"a":[m/s²×3]}}
//                 센서 축 그대로 보낸다. 융합·좌표 변환은 ROS.
//                 IMU_ENABLE = true 인 보드는 board 식별자와 함께 이 레코드를 보낸다.
//   설정       : {"cfg":1,...}   이벤트: {"evt":...}   오류: {"err":"이유",...}
//
// [안전]
//   - 시작은 토크 OFF·구동 0. E 를 받아야 켜지고, 켤 때 조향 목표 = 현재각.
//   - 워치독: C 가 watchdog_ms 끊기면 구동 0, 조향 유지. C 가 다시 오면 복귀(래치 안 함).
//   - 래치 고장: 원인이 사라져도 저절로 풀리지 않고 E 를 받아야 풀리는 고장.
//     과열(abort), 저전압(stop), 과전류, 모터 통신 두절, 비상정지(입력 핀 또는 RC-100 리모컨 버튼).
//     고장 시 구동 0(토크 유지 = 능동 제동)·조향 유지,
//     원인 모터(과열·과전류)는 토크 OFF. 한 모터 고장이라도 이 보드의 구동 2개를 모두 세운다.
//   - 주의: 반대쪽 보드는 이 보드의 고장·워치독·비상정지를 모른다. 한쪽만 서면 차체가 돈다.
//     브리지가 두 보드의 상태(flt, wd, en)를 보고 한쪽이 서면 반대쪽도 세워야 한다.
//   - 하드웨어 에러는 자동복구: 쿨다운(hw_retry_ms) 후 전압·온도가 괜찮으면 그 모터만
//     재부팅 → 재설정 → 토크 ON. 성공하면 고장이 풀리고 다음 C 부터 다시 움직인다.
//     연속 hw_retry 번 실패하면 래치로 남는다(E 로 수동 복구). 30 s 무사하면 횟수 초기화.
//   - derate(온도 ≥ temp_derate 또는 전압 ≤ batt_derate): 속도 상한과 구동 가속에 derate 배율.
//   - 속도 상한을 넘으면 이 보드의 2바퀴를 같은 비율로 줄인다. 바퀴별로 자르면 진행 방향이 틀어진다.
//     좌·우 보드 사이의 비율은 맞춰지지 않는다. 브리지가 상한 안의 값만 보내야 한다.
//   - Current Limit(38) 레지스터는 기록하지만 Velocity(1)·Position(3) 모드에서는 모터가
//     적용하지 않는다(ROBOTIS e-manual). 실제 보호는 펌웨어의 과전류 트립(oc_ms)이 맡는다.
//   - EEPROM 항목(운전 모드, Current Limit)은 값이 다를 때만 쓴다. Temperature Limit 은 건드리지 않는다.
//
// [거버넌스] tutor 브랜치. 업로드·플래싱·커밋은 사람이 직접. 반드시 바퀴를 띄운 벤치에서 먼저 시험.
//
// [빌드] arduino-cli 1.5.1 / OpenCR core 1.5.3 / Dynamixel2Arduino 0.8.1 (README "환경과 버전" 표)
//   arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR repo/firmware/opencr_swerve_right
//   arduino-cli upload  --fqbn OpenCR:OpenCR:OpenCR -p /dev/ttyACM0 repo/firmware/opencr_swerve_right
// [부팅] IMU_ENABLE = true 인 보드는 IMU 자이로 보정(최대 5 s) 동안 로봇을 움직이지 않는다.
// =============================================================
#include <Dynamixel2Arduino.h>
#include <IMU.h>
#include <RC100.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

#if !defined(__OPENCR__)
  #error "이 스케치는 OpenCR 1.0 전용입니다."
#endif

#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;  // OpenCR 내장 DYNAMIXEL 방향핀

using namespace ControlTableItem;

// XM430 컨트롤 테이블 주소 (동기 읽기/쓰기는 주소를 직접 쓴다)
const uint16_t ADDR_HW_ERROR      = 70;
const uint16_t ADDR_GOAL_VELOCITY = 104;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t ADDR_FB_START      = 126;   // Present Current(126) ~ Present Temperature(146)
const uint8_t  MODE_RAW_VELOCITY  = 1;
const uint8_t  MODE_RAW_POSITION  = 3;
const int32_t  CURRENT_LIMIT_RAW_MAX = 1193;
const int32_t  PROFILE_RAW_MAX    = 32767;
const uint8_t  STATUS_ALERT_BIT   = 0x80;  // 상태 패킷 에러 필드: 하드웨어 에러 발생 중
const uint8_t  HWERR_OVERHEAT_BIT = 0x04;  // Hardware Error Status bit2: 과열
const uint32_t REG_TIMEOUT_MS     = 10;    // 개별 레지스터 읽기/쓰기 응답 대기
const uint16_t MAX_RX_BYTES_PER_LOOP = 256;

const uint8_t N_MOTORS = MODULE_COUNT * 2;  // 배열 크기. 인덱스 0..1 = 구동, 2..3 = 조향
// 실제로 다루는 모터 수. 조향이 꺼져 있으면 구동만 다루고 조향 모터에는 아무것도 보내지 않는다.
const uint8_t N_ACTIVE = STEER_ENABLE ? N_MOTORS : MODULE_COUNT;
const float TWO_PI_F = 6.28318530718f;

// 동기 읽기 한 번으로 받는 모터당 상태 (주소 126~146 연속 21바이트)
struct __attribute__((packed)) MotorFeedback {
  int16_t  current;        // 126, ×2.69 mA
  int32_t  velocity;       // 128, ×0.229 rpm
  int32_t  position;       // 132, 틱
  int32_t  velocity_traj;  // 136
  int32_t  position_traj;  // 140
  uint16_t input_voltage;  // 144, ×0.1 V
  uint8_t  temperature;    // 146, °C
};
static_assert(sizeof(MotorFeedback) == 21, "MotorFeedback must match addresses 126..146");

// 런타임 파라미터. 기본값은 config.h, 확정값은 YAML → 'P' 명령.
struct Params {
  float    max_motor_rpm;
  float    min_motor_rpm;
  float    cmd_gain;
  float    cmd_offset_rpm;
  float    trim[MODULE_COUNT];
  int8_t   drive_dir[MODULE_COUNT];
  float    current_limit_a;
  uint32_t drive_profile_acc;
  float    steer_range_rad;
  float    steer_home_rad[MODULE_COUNT];
  int8_t   steer_dir[MODULE_COUNT];
  uint32_t steer_profile_vel;
  uint32_t steer_profile_acc;
  uint32_t watchdog_ms;
  uint32_t idle_timeout_ms; // [추가] 자동 휴면 타임아웃
  float    temp_warn_c, temp_derate_c, temp_abort_c;
  float    batt_warn_v, batt_derate_v, batt_stop_v;
  float    derate_scale;
  uint32_t overcurrent_trip_ms;
  uint8_t  hw_retry_max;
  uint32_t hw_retry_delay_ms;
  float    noload_current_ma[MODULE_COUNT];  // 자가진단 전류 기준선
  float    selftest_current_tol;             // 기준선 대비 허용 배수
};

enum RecoverPhase : uint8_t {
  RCV_IDLE,     // 복구 대상 없음 또는 쿨다운·조건 대기
  RCV_BOOTING,  // 재부팅 보냄, 응답·재설정 대기
};

enum FaultBits : uint16_t {
  FLT_HWERR       = 1 << 0,
  FLT_OVERTEMP    = 1 << 1,
  FLT_UNDERVOLT   = 1 << 2,
  FLT_OVERCURRENT = 1 << 3,
  FLT_COMM        = 1 << 4,
  FLT_ESTOP       = 1 << 5,
  FLT_SELFTEST    = 1 << 6,   // 버튼 자가진단 불합격
};

enum WarnBits : uint16_t {
  WRN_TEMP   = 1 << 0,   // 최고 온도 ≥ temp_warn
  WRN_BATT   = 1 << 1,   // 평활 전압 ≤ batt_warn
  WRN_DERATE = 1 << 2,   // derate 동작 중
  WRN_INIT   = 1 << 3,   // 초기화 안 된 모터 있음
  WRN_IMU    = 1 << 4,   // IMU 무응답 (주행은 막지 않음, 융합은 ROS 판단)
};

enum SelfTestPhase : uint8_t {
  ST_IDLE,  // 대기
  ST_HOLD,  // 버튼을 누르고 있는 중 (3 s 전에 떼면 취소)
  ST_RUN,   // 실행 중 (모터가 움직임)
};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
cIMU imu;
RC100 remote;  // TurtleBot3 무선 리모컨 (비상정지 전용)
Params prm;

// 동기 읽기: 받은 버퍼(_rx)는 응답이 확인된 모터만 확정값으로 복사한다.
MotorFeedback fb[N_MOTORS];
MotorFeedback fb_rx[N_MOTORS];
uint8_t hw_err[N_MOTORS];
uint8_t hw_err_rx[N_MOTORS];
uint8_t status_err[N_MOTORS];
DYNAMIXEL::InfoSyncReadInst_t sr_fb;
DYNAMIXEL::InfoSyncReadInst_t sr_err;
DYNAMIXEL::XELInfoSyncRead_t sr_fb_xel[N_MOTORS];
DYNAMIXEL::XELInfoSyncRead_t sr_err_xel[N_MOTORS];
uint8_t sr_fb_pkt[64];
uint8_t sr_err_pkt[64];
// 동기 읽기 목록 j번째 → 모터 인덱스 k. 재부팅 중인 모터는 목록에서 뺀다.
// (응답 없는 모터가 끼면 그 뒤 모터까지 못 읽어 통신 두절로 오판한다.)
uint8_t sr_map[N_MOTORS];
uint8_t sr_count = 0;

// 동기 쓰기: 구동 Goal Velocity, 조향 Goal Position
int32_t goal_vel_raw[MODULE_COUNT];
int32_t goal_pos_raw[MODULE_COUNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_vel;
DYNAMIXEL::InfoSyncWriteInst_t sw_pos;
DYNAMIXEL::XELInfoSyncWrite_t sw_vel_xel[MODULE_COUNT];
DYNAMIXEL::XELInfoSyncWrite_t sw_pos_xel[MODULE_COUNT];
uint8_t sw_vel_pkt[64];
uint8_t sw_pos_pkt[64];

//휴면상태 전역변수
bool is_sleeping = false; // 현재 자동 휴면 상태인지 여부

bool     motor_ok[N_MOTORS];      // ping + 설정 성공
bool     fresh[N_MOTORS];         // 이번 동기 읽기에서 응답함
uint8_t  comm_fail[N_MOTORS];
bool     oc_active[N_MOTORS];
uint32_t oc_since_ms[N_MOTORS];
bool     fault_off[N_MOTORS];     // 과열·과전류로 토크를 끈 모터
int32_t  vel_limit_raw[N_MOTORS];  // Velocity Limit(44) 레지스터 (구동은 속도 상한에 사용)
bool     hwerr_fresh[N_MOTORS];   // 마지막 Hardware Error 동기 읽기에서 응답함
// 미검출 모터: 연속 무응답이면 동기 읽기 목록에서 빼고 주기적으로 ping 해서 돌아오면
// 재설정(configureMotor)까지 마친 뒤 다시 넣는다. 다시 꽂으면 E 없이 READY 로 복귀한다.
// (목록에 남겨 두면 그 뒤 모터까지 못 읽어서 어느 모터가 빠졌는지 알 수 없다.)
bool     missing[N_MOTORS];
uint8_t  probe_idx = 0;
uint32_t t_probe   = 0;

// 하드웨어 에러 자동복구
bool         hw_bad[N_MOTORS];    // 하드웨어 에러가 관측된 모터 (복구 대상)
bool         rcv_busy[N_MOTORS];  // 재부팅 중 (읽기 목록·고장 판정에서 제외)
bool         needs_torque[N_MOTORS];  // 복구됐지만 아직 토크를 다시 켜지 못한 모터
RecoverPhase rcv_phase     = RCV_IDLE;
uint32_t     rcv_t0        = 0;
uint8_t      rcv_attempts  = 0;
bool         rcv_gave_up   = false;
uint32_t     last_hwerr_ms = 0;

bool     torque_on   = false;
bool     motion_en   = false;     // 마지막 C 의 en
bool     wd_tripped  = true;      // 첫 C 전까지는 정지 상태로 본다
uint32_t last_cmd_ms = 0;
uint16_t faults      = 0;         // 래치
uint16_t warns       = 0;
float    cmd_wheel[MODULE_COUNT];      // 바퀴 각속도 rad/s
int32_t  cmd_steer_raw[MODULE_COUNT];  // 검증된 조향 목표 틱
bool     derate_on   = false;
float    speed_scale = 1.0f;
float    bv_ema      = 0.0f;
float    bv_last     = 0.0f;
uint8_t  bv_low_cnt  = 0;

uint32_t t_ctrl = 0, t_fb = 0, t_imu = 0, t_err = 0, t_batt = 0;

// 상태 표시
bool     imu_ok    = false;
bool     linked    = false;   // ② BRIDGE LINKED: 브릿지 프레임(C)이 들어오는 중
bool     wd_alarm  = false;   // 링크 끊김 경보 (프레임이 다시 오면 풀림, 래치 아님)
uint16_t buzzer_hz = 0;       // 지금 울리는 주파수, 0 = 꺼짐
// 짧은 멜로디(징글·틱). 재생 중에는 버저 패턴보다 우선한다. 논블로킹.
const uint16_t* mel_hz = nullptr;
const uint16_t* mel_ms = nullptr;
uint8_t  mel_n  = 0;
uint32_t mel_t0 = 0;

// 버튼 (0 = 자가진단 SW1, 1 = 중단 SW2)
const int BTN_PINS[2] = {BTN_SELFTEST_PIN, BTN_STOP_PIN};
bool     btn_raw[2]   = {false, false};
bool     btn_state[2] = {false, false};
uint32_t btn_t[2]     = {0, 0};

// 버튼 자가진단: 최고속도로 앞으로 SELFTEST_DISTANCE_M, 멈춤, 뒤로 같은 거리.
// 이 보드의 구동 2개를 함께 돌린다. 반대쪽 보드의 바퀴는 이 보드가 돌릴 수 없다.
enum SelfTestStep : uint8_t {
  STS_ALIGN,     // 조향을 정면(0 rad)으로 맞춤 (조향이 켜져 있을 때만)
  STS_FORWARD,   // 앞으로
  STS_BACKWARD,  // 뒤로 (출발 위치까지)
};
SelfTestPhase st_phase    = ST_IDLE;
uint32_t      st_hold_t0  = 0;
SelfTestStep  st_step     = STS_ALIGN;
uint32_t      st_step_t0  = 0;
bool          st_still    = false;  // 모든 바퀴가 감속을 끝내고 멈춘 것으로 보임
uint32_t      st_still_t0 = 0;      // 그렇게 보이기 시작한 시각
float         st_rpm      = 0.0f;   // 이번 시험의 목표 모터 rpm (시작할 때 값)
uint32_t      st_leg_timeout_ms = 0;  // 한 방향 시간 제한
int32_t       st_start_pos[MODULE_COUNT];  // 출발 위치(틱). Velocity 모드의 위치는 여러 바퀴를 이어서 센다
bool          st_braking[MODULE_COUNT];    // 이번 구간에서 감속을 시작함 (다시 가속하지 않는다)
float         st_fwd_m[MODULE_COUNT];      // 앞으로 간 거리(m)
float         st_end_m[MODULE_COUNT];      // 끝났을 때 출발점에서 벗어난 거리(m, + = 앞)
float         st_vmax[MODULE_COUNT];       // 최고 모터 rpm(절댓값)
float         st_imax[MODULE_COUNT];       // 최고 전류 mA(절댓값)

// ---------------------------------------------------------------
// 단위 변환 (경계에서만, ROS 쪽은 m·rad)
// ---------------------------------------------------------------
uint8_t motorId(uint8_t k) {
  return k < MODULE_COUNT ? DRIVE_IDS[k] : STEER_IDS[k - MODULE_COUNT];
}

// 바퀴 ÷ g = 모터
float wheelRadsToMotorRpm(float wheel_rad_s) {
  return wheel_rad_s * 60.0f / TWO_PI_F / GEAR_G;
}

// 모터 × g = 바퀴
float motorRpmToWheelRads(float motor_rpm) {
  return motor_rpm * GEAR_G * TWO_PI_F / 60.0f;
}

float ticksPerRad() {
  return (float)TICKS_PER_REV / TWO_PI_F;
}

int32_t wrapRaw(int32_t raw) {
  int32_t r = raw % TICKS_PER_REV;
  return r < 0 ? r + TICKS_PER_REV : r;
}

// 조향각(rad, 로봇 기준) → Goal Position 틱. 범위 밖이면 false (명령 거부).
bool steerRadToRaw(uint8_t m, float theta, int32_t* out) {
  if (!isfinite(theta) || fabsf(theta) > prm.steer_range_rad) return false;
  const float motor_rad = prm.steer_home_rad[m] + prm.steer_dir[m] * theta;
  const int32_t raw = STEER_CENTER_RAW + (int32_t)lroundf(motor_rad * ticksPerRad());
  if (raw < POSITION_RAW_MIN || raw > POSITION_RAW_MAX) return false;
  *out = raw;
  return true;
}

// Present Position 틱 → 조향각(rad, 로봇 기준). steerRadToRaw 의 역변환.
float steerRawToRad(uint8_t m, int32_t raw) {
  const float motor_rad = (float)(wrapRaw(raw) - STEER_CENTER_RAW) / ticksPerRad();
  return prm.steer_dir[m] * (motor_rad - prm.steer_home_rad[m]);
}

int32_t currentLimitRaw() {
  int32_t raw = (int32_t)lroundf(prm.current_limit_a * 1000.0f / MA_PER_LSB);
  return raw > CURRENT_LIMIT_RAW_MAX ? CURRENT_LIMIT_RAW_MAX : raw;
}

// derate 중에는 가속도 줄인다(raw 가 작을수록 완만).
uint32_t driveProfileAcc() {
  uint32_t acc = (uint32_t)lroundf(prm.drive_profile_acc * speed_scale);
  return acc < 1 ? 1 : acc;
}

bool estopActive() {
  if (ESTOP_PIN < 0) return false;
  const int level = digitalRead(ESTOP_PIN);
  return ESTOP_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

// 비상정지(핀·리모컨 공통): 래치 고장으로 올리고 구동 명령을 0 으로.
// 토크는 끄지 않는다. 토크를 끄면 바퀴가 굴러가지만, 켠 채 속도 0 이면 능동 제동된다.
// source: 리모컨 버튼값, 핀이면 0
void triggerEstop(uint16_t source) {
  for (uint8_t m = 0; m < MODULE_COUNT; m++) cmd_wheel[m] = 0.0f;
  raiseFault(FLT_ESTOP, -1, source);
}

// RC-100 패킷 수신. 라이브러리는 한 번에 1바이트를 읽어 패킷이 완성되면 true.
void readRemote() {
  if (!RC100_ESTOP_ENABLE) return;
  uint8_t budget = 64;
  while (budget-- > 0 && RC100_SERIAL.available() > 0) {
    if (!remote.available()) continue;
    const uint16_t buttons = remote.readData();  // 버튼을 놓으면 0 이 온다
    if (buttons & RC100_ESTOP_MASK) triggerEstop(buttons);
  }
}

// ---------------------------------------------------------------
// JSON 출력 도우미
// ---------------------------------------------------------------
void printFloatArray(const float* v, uint8_t n, uint8_t digits) {
  Serial.print('[');
  for (uint8_t i = 0; i < n; i++) {
    if (i) Serial.print(',');
    Serial.print(v[i], digits);
  }
  Serial.print(']');
}

void printIntArray(const int8_t* v, uint8_t n) {
  Serial.print('[');
  for (uint8_t i = 0; i < n; i++) {
    if (i) Serial.print(',');
    Serial.print(v[i]);
  }
  Serial.print(']');
}

void sendErr(const char* why, int16_t id) {
  Serial.print("{\"err\":\"");
  Serial.print(why);
  Serial.print('"');
  if (id >= 0) {
    Serial.print(",\"id\":");
    Serial.print(id);
  }
  Serial.println('}');
}

const char* faultName(uint16_t bit) {
  switch (bit) {
    case FLT_HWERR:       return "hw_error";
    case FLT_OVERTEMP:    return "overtemp";
    case FLT_UNDERVOLT:   return "undervolt";
    case FLT_OVERCURRENT: return "overcurrent";
    case FLT_COMM:        return "motor_lost";
    case FLT_ESTOP:       return "estop";
    case FLT_SELFTEST:    return "selftest";
    default:              return "unknown";
  }
}

// 래치 고장 설정. 새로 생긴 비트만 이벤트로 남긴다.
void raiseFault(uint16_t bit, int16_t id, float value) {
  if (faults & bit) return;
  faults |= bit;
  Serial.print("{\"evt\":\"fault\",\"f\":\"");
  Serial.print(faultName(bit));
  Serial.print("\",\"id\":");
  Serial.print(id);
  Serial.print(",\"v\":");
  Serial.print(value, 2);
  Serial.print(",\"t\":");
  Serial.print(millis());
  Serial.println('}');
}

// ---------------------------------------------------------------
// 파라미터
// ---------------------------------------------------------------
void loadDefaults() {
  prm.max_motor_rpm  = DEF_MAX_MOTOR_RPM;
  prm.min_motor_rpm  = DEF_MIN_MOTOR_RPM;
  prm.cmd_gain       = DEF_CMD_GAIN;
  prm.cmd_offset_rpm = DEF_CMD_OFFSET_RPM;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    prm.trim[m]           = DEF_MODULE_TRIM[m];
    prm.drive_dir[m]      = DEF_DRIVE_DIR[m];
    prm.steer_home_rad[m] = DEF_STEER_HOME_RAD[m];
    prm.steer_dir[m]      = DEF_STEER_DIR[m];
  }
  prm.current_limit_a     = DEF_CURRENT_LIMIT_A;
  prm.drive_profile_acc   = DEF_DRIVE_PROFILE_ACC;
  prm.steer_range_rad     = DEF_STEER_RANGE_RAD;
  prm.steer_profile_vel   = DEF_STEER_PROFILE_VEL;
  prm.steer_profile_acc   = DEF_STEER_PROFILE_ACC;
  prm.watchdog_ms         = DEF_WATCHDOG_MS;
  prm.idle_timeout_ms     = DEF_IDLE_TIMEOUT_MS; // [추가] 기본 5초
  prm.temp_warn_c         = DEF_TEMP_WARN_C;
  prm.temp_derate_c       = DEF_TEMP_DERATE_C;
  prm.temp_abort_c        = DEF_TEMP_ABORT_C;
  prm.batt_warn_v         = DEF_BATT_WARN_V;
  prm.batt_derate_v       = DEF_BATT_DERATE_V;
  prm.batt_stop_v         = DEF_BATT_STOP_V;
  prm.derate_scale        = DEF_DERATE_SCALE;
  prm.overcurrent_trip_ms = DEF_OVERCURRENT_TRIP_MS;
  prm.hw_retry_max        = DEF_HW_RETRY_MAX;
  prm.hw_retry_delay_ms   = DEF_HW_RETRY_DELAY_MS;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) prm.noload_current_ma[m] = DEF_NOLOAD_CURRENT_MA[m];
  prm.selftest_current_tol = DEF_SELFTEST_CURRENT_TOL;
}

// 홈 오프셋 + 조향 범위가 단일회전 틱 범위(0~4095) 안에 들어가는지
bool steerRangeFits(float home_rad, float range_rad) {
  const float span = (fabsf(home_rad) + range_rad) * ticksPerRad();
  return STEER_CENTER_RAW - span >= POSITION_RAW_MIN && STEER_CENTER_RAW + span <= POSITION_RAW_MAX;
}

bool isSign(float v) {
  return v == 1.0f || v == -1.0f;
}

bool isIntInRange(float v, float lo, float hi) {
  return v == floorf(v) && v >= lo && v <= hi;
}

// ---------------------------------------------------------------
// 다이나믹셀 설정 / 토크
// ---------------------------------------------------------------
// 동기 읽기 목록을 재부팅 중이 아닌 모터로 다시 만든다.
void rebuildSyncReadLists() {
  uint8_t j = 0;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (rcv_busy[k] || missing[k]) continue;
    sr_fb_xel[j].id          = motorId(k);
    sr_fb_xel[j].p_recv_buf  = (uint8_t*)&fb_rx[k];
    sr_fb_xel[j].error       = 0;
    sr_err_xel[j].id         = motorId(k);
    sr_err_xel[j].p_recv_buf = &hw_err_rx[k];
    sr_err_xel[j].error      = 0;
    sr_map[j] = k;
    j++;
  }
  sr_count = j;
  sr_fb.xel_count  = j;
  sr_err.xel_count = j;
  sr_fb.is_info_changed  = true;
  sr_err.is_info_changed = true;
}

void setupSyncInfos() {
  sr_fb.packet.p_buf        = sr_fb_pkt;
  sr_fb.packet.buf_capacity = sizeof(sr_fb_pkt);
  sr_fb.packet.is_completed = false;
  sr_fb.addr        = ADDR_FB_START;
  sr_fb.addr_length = sizeof(MotorFeedback);
  sr_fb.p_xels      = sr_fb_xel;

  sr_err.packet.p_buf        = sr_err_pkt;
  sr_err.packet.buf_capacity = sizeof(sr_err_pkt);
  sr_err.packet.is_completed = false;
  sr_err.addr        = ADDR_HW_ERROR;
  sr_err.addr_length = 1;
  sr_err.p_xels      = sr_err_xel;

  rebuildSyncReadLists();

  sw_vel.packet.p_buf        = sw_vel_pkt;
  sw_vel.packet.buf_capacity = sizeof(sw_vel_pkt);
  sw_vel.packet.is_completed = false;
  sw_vel.addr        = ADDR_GOAL_VELOCITY;
  sw_vel.addr_length = sizeof(int32_t);
  sw_vel.p_xels      = sw_vel_xel;
  sw_vel.xel_count   = MODULE_COUNT;

  sw_pos.packet.p_buf        = sw_pos_pkt;
  sw_pos.packet.buf_capacity = sizeof(sw_pos_pkt);
  sw_pos.packet.is_completed = false;
  sw_pos.addr        = ADDR_GOAL_POSITION;
  sw_pos.addr_length = sizeof(int32_t);
  sw_pos.p_xels      = sw_pos_xel;
  sw_pos.xel_count   = MODULE_COUNT;

  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    sw_vel_xel[m].id     = DRIVE_IDS[m];
    sw_vel_xel[m].p_data = (uint8_t*)&goal_vel_raw[m];
    sw_pos_xel[m].id     = STEER_IDS[m];
    sw_pos_xel[m].p_data = (uint8_t*)&goal_pos_raw[m];
  }
  sw_vel.is_info_changed = true;
  sw_pos.is_info_changed = true;
}

// ping → 토크 OFF → 운전 모드 → Current Limit → 프로파일. 토크는 켜지 않는다.
// 전원이 다시 들어온 모터는 RAM(프로파일)이 0 으로 돌아가므로 E 때마다 다시 부른다.
bool configureMotor(uint8_t k) {
  const uint8_t id = motorId(k);
  const bool drive = k < MODULE_COUNT;

  if (!dxl.ping(id)) return false;
  if (!dxl.torqueOff(id)) return false;

  const int32_t mode = dxl.readControlTableItem(OPERATING_MODE, id, REG_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) return false;
  const int32_t want_mode = drive ? MODE_RAW_VELOCITY : MODE_RAW_POSITION;
  if (mode != want_mode && !dxl.setOperatingMode(id, drive ? OP_VELOCITY : OP_POSITION)) return false;

  const int32_t cl = dxl.readControlTableItem(CURRENT_LIMIT, id, REG_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) return false;
  const int32_t want_cl = currentLimitRaw();
  if (cl != want_cl && !dxl.writeControlTableItem(CURRENT_LIMIT, id, want_cl, REG_TIMEOUT_MS)) return false;

  const int32_t vl = dxl.readControlTableItem(VELOCITY_LIMIT, id, REG_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) return false;
  vel_limit_raw[k] = vl;

  if (drive) {
    if (!dxl.writeControlTableItem(PROFILE_ACCELERATION, id, driveProfileAcc(), REG_TIMEOUT_MS)) return false;
    if (!dxl.writeControlTableItem(GOAL_VELOCITY, id, 0, REG_TIMEOUT_MS)) return false;
  } else {
    if (!dxl.writeControlTableItem(PROFILE_VELOCITY, id, prm.steer_profile_vel, REG_TIMEOUT_MS)) return false;
    if (!dxl.writeControlTableItem(PROFILE_ACCELERATION, id, prm.steer_profile_acc, REG_TIMEOUT_MS)) return false;
  }
  return true;
}

void writeGoals() {
  if (!torque_on) return;
  sw_vel.is_info_changed = true;
  dxl.syncWrite(&sw_vel);
  if (!STEER_ENABLE) return;
  sw_pos.is_info_changed = true;
  dxl.syncWrite(&sw_pos);
}

void disableTorque() {
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    cmd_wheel[m]    = 0.0f;
    goal_vel_raw[m] = 0;
  }
  if (torque_on) {
    sw_vel.is_info_changed = true;
    dxl.syncWrite(&sw_vel);
  }
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (motor_ok[k]) dxl.writeControlTableItem(TORQUE_ENABLE, motorId(k), 0, REG_TIMEOUT_MS);
  }
  torque_on = false;
  motion_en = false;
}

// 하드웨어 에러 관측. 쿨다운은 "새로" 에러가 난 시각부터 잰다
// (에러 상태는 재부팅 전까지 계속 읽히므로 매번 시각을 갱신하면 복구가 영영 시작되지 않는다).
void noteHwError(uint8_t k, float value) {
  if (!hw_bad[k]) {
    hw_bad[k]     = true;
    last_hwerr_ms = millis();
  }
  raiseFault(FLT_HWERR, motorId(k), value);
}

// 동기 읽기: 목록 앞에서부터 연속으로 응답한 모터 수만 돌아온다. 그 뒤는 이번 주기 미확인.
void readFeedback() {
  for (uint8_t k = 0; k < N_ACTIVE; k++) fresh[k] = false;
  if (sr_count == 0) return;
  const uint8_t n = dxl.syncRead(&sr_fb, DXL_READ_TIMEOUT_MS);
  for (uint8_t j = 0; j < sr_count && j < n; j++) {
    const uint8_t k = sr_map[j];
    fresh[k] = true;
    memcpy(&fb[k], &fb_rx[k], sizeof(MotorFeedback));
    status_err[k] = sr_fb_xel[j].error;
    comm_fail[k]  = 0;
  }
  // 응답이 끊긴 첫 모터만 무응답으로 센다. 그 뒤 모터는 이번 주기에 판단할 수 없다.
  if (n < sr_count) {
    const uint8_t k = sr_map[n];
    if (comm_fail[k] < 255) comm_fail[k]++;
    if (comm_fail[k] >= COMM_FAIL_MAX) {
      missing[k] = true;
      rebuildSyncReadLists();
      Serial.print("{\"evt\":\"motor_missing\",\"id\":");
      Serial.print(motorId(k));
      Serial.print(",\"t\":");
      Serial.print(millis());
      Serial.println('}');
    }
  }
}

// 미검출 모터를 하나씩 돌아가며 ping. 응답하면 재설정하고 목록에 다시 넣는다.
// 전원이 다시 들어온 모터는 설정(RAM: 운전 모드·프로파일)이 초기화되고 토크도 꺼져 있다.
// 여기서 configureMotor() 까지 해 두어야 다시 꽂았을 때 E 없이 READY 로 돌아온다.
// (토크는 configureMotor 가 끈 상태로 둔다. 주행 중 재연결은 FLT_COMM 래치가 남아 E 로 푼다.)
void probeMissing() {
  for (uint8_t i = 0; i < N_ACTIVE; i++) {
    const uint8_t k = (probe_idx + i) % N_ACTIVE;
    if (!missing[k] || rcv_busy[k]) continue;
    probe_idx = (k + 1) % N_ACTIVE;
    if (!dxl.ping(motorId(k))) return;

    // 응답만 하고 설정에 실패하면 미검출로 남겨 다음 주기에 다시 시도한다.
    if (!configureMotor(k)) return;

    motor_ok[k]  = true;
    missing[k]   = false;
    comm_fail[k] = 0;
    rebuildSyncReadLists();
    Serial.print("{\"evt\":\"motor_found\",\"id\":");
    Serial.print(motorId(k));
    Serial.print(",\"t\":");
    Serial.print(millis());
    Serial.println('}');
    return;  // 한 번에 하나만 (ping 실패 시 약 10 ms 대기)
  }
}

// 반환: 응답한 모터 수. 응답한 모터는 hwerr_fresh 로 표시한다.
uint8_t readHwErrors() {
  for (uint8_t k = 0; k < N_ACTIVE; k++) hwerr_fresh[k] = false;
  if (sr_count == 0) return 0;
  const uint8_t n = dxl.syncRead(&sr_err, DXL_READ_TIMEOUT_MS);
  for (uint8_t j = 0; j < n && j < sr_count; j++) {
    const uint8_t k = sr_map[j];
    hwerr_fresh[k] = true;
    hw_err[k] = hw_err_rx[k];
    if (hw_err[k] != 0) noteHwError(k, hw_err[k]);
  }
  return n;
}

void sampleBattery() {
  bv_last = getPowerInVoltage();
  bv_ema = (bv_ema <= 0.0f) ? bv_last : bv_ema + BATT_EMA_ALPHA * (bv_last - bv_ema);
  if (bv_last <= prm.batt_stop_v) {
    if (bv_low_cnt < 255) bv_low_cnt++;
    if (bv_low_cnt >= BATT_STOP_SAMPLES) raiseFault(FLT_UNDERVOLT, -1, bv_last);
  } else {
    bv_low_cnt = 0;
  }
}

// 진행 중인 자동복구를 멈추고 상태를 비운다(E 가 직접 복구하므로).
void cancelRecovery() {
  bool changed = false;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (rcv_busy[k]) { rcv_busy[k] = false; changed = true; }
  }
  if (changed) rebuildSyncReadLists();
  rcv_phase    = RCV_IDLE;
  rcv_attempts = 0;
  rcv_gave_up  = false;
}

// E: 하드웨어 에러 모터 재부팅 → 전 모터 재설정 → 해제 조건 확인 → 래치 고장 해제
//    → 조향 현재각 유지 → 토크 ON. 사람이 보낸 명령이라 여기서는 기다려도(블로킹) 된다.
void enableTorque() {
  // configureMotor 가 토크를 끄므로 실패 경로에서도 상태가 맞게 먼저 내린다.
  torque_on = false;
  motion_en = false;
  cancelRecovery();

  // 하드웨어 에러는 재부팅으로만 풀린다(e-manual). 전원을 다시 넣지 않아도 된다.
  readHwErrors();
  bool rebooted = false;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!hw_bad[k]) continue;
    dxl.reboot(motorId(k));
    rebooted = true;
  }
  if (rebooted) delay(HW_REBOOT_WAIT_MS);

  for (uint8_t k = 0; k < N_ACTIVE; k++) motor_ok[k] = false;
  const uint32_t t0 = millis();
  while (true) {
    bool all_ok = true;
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (!motor_ok[k]) motor_ok[k] = configureMotor(k);
      all_ok = all_ok && motor_ok[k];
    }
    if (all_ok || !rebooted || millis() - t0 > HW_REBOOT_TIMEOUT_MS) break;
    delay(50);  // 재부팅 중인 모터가 깨어나기를 기다린다
  }
  for (uint8_t k = 0; k < N_ACTIVE; k++) missing[k] = !motor_ok[k];
  rebuildSyncReadLists();
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!motor_ok[k]) { sendErr("motor_init", motorId(k)); return; }
  }

  readFeedback();
  readHwErrors();
  sampleBattery();
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!fresh[k] || !hwerr_fresh[k]) { sendErr("no_response", motorId(k)); return; }
    if (hw_err[k] != 0) { sendErr("hw_error_persists", motorId(k)); return; }
    if (fb[k].temperature >= prm.temp_derate_c) { sendErr("too_hot", motorId(k)); return; }
  }
  if (bv_last <= prm.batt_stop_v) { sendErr("battery_low", -1); return; }
  if (estopActive()) { sendErr("estop_active", -1); return; }
  for (uint8_t m = 0; STEER_ENABLE && m < MODULE_COUNT; m++) {
    // 단일회전 범위 밖 값을 감아서 목표로 쓰면 한 바퀴를 거꾸로 돌 수 있으므로 켜지 않는다.
    const int32_t pos = fb[MODULE_COUNT + m].position;
    if (pos < POSITION_RAW_MIN || pos > POSITION_RAW_MAX) { sendErr("steer_pos_out_of_range", STEER_IDS[m]); return; }
  }

  faults = 0;
  bv_low_cnt = 0;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    comm_fail[k]    = 0;
    oc_active[k]    = false;
    fault_off[k]    = false;
    hw_bad[k]       = false;
    needs_torque[k] = false;
    status_err[k]   = 0;
  }

  // 켜는 순간 움직이지 않게: 구동 0, 조향 목표 = 현재각. 새 C 가 와야 움직인다.
  motion_en  = false;
  wd_tripped = true;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    cmd_wheel[m]     = 0.0f;
    goal_vel_raw[m]  = 0;
    goal_pos_raw[m]  = STEER_ENABLE ? fb[MODULE_COUNT + m].position : STEER_CENTER_RAW;
    cmd_steer_raw[m] = goal_pos_raw[m];
  }
  sw_vel.is_info_changed = true;
  sw_pos.is_info_changed = true;
  if (!dxl.syncWrite(&sw_vel) || (STEER_ENABLE && !dxl.syncWrite(&sw_pos))) {
    sendErr("goal_write", -1);
    return;
  }

  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!dxl.torqueOn(motorId(k))) {
      torque_on = true;  // disableTorque 가 이미 켠 모터도 끄게 한다
      disableTorque();
      sendErr("torque_on", motorId(k));
      return;
    }
  }
  torque_on = true;
}

// ---------------------------------------------------------------
// 제어 주기
// ---------------------------------------------------------------
bool motionAllowed() {
  if (!torque_on || !motion_en || wd_tripped || faults != 0) return false;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!motor_ok[k]) return false;
  }
  return true;
}

void setDerate(bool on) {
  if (on == derate_on) return;
  derate_on   = on;
  speed_scale = on ? prm.derate_scale : 1.0f;
  const uint32_t acc = driveProfileAcc();
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    if (motor_ok[m]) dxl.writeControlTableItem(PROFILE_ACCELERATION, DRIVE_IDS[m], acc, REG_TIMEOUT_MS);
  }
  Serial.print("{\"evt\":\"derate\",\"on\":");
  Serial.print(on ? 1 : 0);
  Serial.print(",\"t\":");
  Serial.print(millis());
  Serial.println('}');
}

// 고장 원인 모터만 토크 OFF (나머지 구동은 computeGoals 가 0 으로 세운다)
void torqueOffFaulted(uint8_t k) {
  if (!torque_on || fault_off[k]) return;
  dxl.writeControlTableItem(TORQUE_ENABLE, motorId(k), 0, REG_TIMEOUT_MS);
  fault_off[k] = true;
}

void printRecoverEvent(const char* what) {
  Serial.print("{\"evt\":\"");
  Serial.print(what);
  Serial.print("\",\"try\":");
  Serial.print(rcv_attempts);
  Serial.print(",\"ids\":[");
  bool first = true;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!rcv_busy[k] && !hw_bad[k]) continue;
    if (!first) Serial.print(',');
    Serial.print(motorId(k));
    first = false;
  }
  Serial.print("],\"t\":");
  Serial.print(millis());
  Serial.println('}');
}

// 복구된 모터의 토크를 다시 켠다. 구동은 목표 0, 조향은 붙잡고 있던 목표각.
// 실패하면 하드웨어 에러로 다시 올려 다음 복구 시도에 맡긴다.
void restoreTorque() {
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!needs_torque[k]) continue;
    const uint8_t id = motorId(k);
    bool ok;
    if (k < MODULE_COUNT) {
      ok = dxl.writeControlTableItem(GOAL_VELOCITY, id, 0, REG_TIMEOUT_MS);
    } else {
      ok = dxl.writeControlTableItem(GOAL_POSITION, id, goal_pos_raw[k - MODULE_COUNT], REG_TIMEOUT_MS);
    }
    ok = ok && dxl.writeControlTableItem(TORQUE_ENABLE, id, 1, REG_TIMEOUT_MS);
    if (ok) {
      needs_torque[k] = false;
    } else {
      noteHwError(k, 0.0f);
    }
  }
}

// 하드웨어 에러 자동복구 (논블로킹). 제어 주기마다 부른다.
// IDLE: 쿨다운·전압·온도 조건이 맞으면 에러 모터만 재부팅 → BOOTING: 깨어나면 재설정·에러 해제 확인
// → 모두 성공하면 고장 해제 후 토크 ON. 시간 안에 안 되면 그 시도는 실패, 쿨다운부터 다시.
void recoveryTick(uint32_t now) {
  if (rcv_phase == RCV_IDLE) {
    if (!(faults & FLT_HWERR)) {
      if (rcv_attempts > 0 && now - last_hwerr_ms > HW_RETRY_RESET_MS) rcv_attempts = 0;
      return;
    }
    if (prm.hw_retry_max == 0) return;
    if (rcv_attempts >= prm.hw_retry_max) {
      if (!rcv_gave_up) {
        rcv_gave_up = true;
        printRecoverEvent("hw_recover_giveup");  // 이제 E 로만 복구
      }
      return;
    }
    if (now - last_hwerr_ms < prm.hw_retry_delay_ms) return;  // 쿨다운
    if (bv_last <= prm.batt_stop_v) return;  // 전압이 돌아오기 전에 재부팅하면 다시 걸린다

    uint8_t n_bad = 0;
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (!hw_bad[k]) continue;
      n_bad++;
      // 과열 에러이거나 아직 뜨거우면 식을 때까지 기다린다.
      if ((hw_err[k] & HWERR_OVERHEAT_BIT) && !fresh[k]) return;
      if (fresh[k] && fb[k].temperature >= prm.temp_derate_c) return;
    }
    if (n_bad == 0) return;

    rcv_attempts++;
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (hw_bad[k]) rcv_busy[k] = true;
    }
    rebuildSyncReadLists();
    printRecoverEvent("hw_recover");
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (rcv_busy[k]) dxl.reboot(motorId(k));
    }
    rcv_phase = RCV_BOOTING;
    rcv_t0    = now;
    return;
  }

  // RCV_BOOTING
  if (now - rcv_t0 < HW_REBOOT_WAIT_MS) return;

  bool all_ready = true;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!rcv_busy[k] || !hw_bad[k]) continue;  // hw_bad 가 풀린 모터는 준비 완료
    if (!configureMotor(k)) { all_ready = false; continue; }
    const int32_t e = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, motorId(k), REG_TIMEOUT_MS);
    if (dxl.getLastLibErrCode() != DXL_LIB_OK || e != 0) { all_ready = false; continue; }
    hw_bad[k]       = false;
    hw_err[k]       = 0;
    status_err[k]   = 0;
    needs_torque[k] = true;  // 재부팅으로 토크가 꺼졌다
  }

  if (!all_ready) {
    if (now - rcv_t0 <= HW_REBOOT_TIMEOUT_MS) return;
    printRecoverEvent("hw_recover_fail");
    for (uint8_t k = 0; k < N_ACTIVE; k++) rcv_busy[k] = false;
    rebuildSyncReadLists();
    last_hwerr_ms = now;  // 쿨다운부터 다시
    rcv_phase     = RCV_IDLE;
    return;
  }

  printRecoverEvent("hw_recovered");
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!rcv_busy[k]) continue;
    rcv_busy[k]  = false;
    comm_fail[k] = 0;
    fault_off[k] = false;
  }
  rebuildSyncReadLists();
  faults &= ~FLT_HWERR;
  last_hwerr_ms = now;  // 시도 횟수 초기화(30 s) 기준
  rcv_phase     = RCV_IDLE;
  // 다른 래치 고장이 남아 있거나 D 상태면 토크는 E 때 켠다.
  if (torque_on && faults == 0) restoreTorque();
}

void updateSafety(uint32_t now) {
  float max_tc = 0.0f;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    const uint8_t id = motorId(k);
    if (rcv_busy[k]) continue;  // 재부팅 중: 응답 없음이 정상
    if (missing[k]) {
      if (torque_on) raiseFault(FLT_COMM, id, comm_fail[k]);
      continue;
    }
    if (!fresh[k]) continue;
    if (status_err[k] & STATUS_ALERT_BIT) noteHwError(k, status_err[k]);

    const float tc = fb[k].temperature;
    if (tc > max_tc) max_tc = tc;
    if (tc >= prm.temp_abort_c) {
      raiseFault(FLT_OVERTEMP, id, tc);
      torqueOffFaulted(k);
    }

    const float cur_ma = fabsf(fb[k].current * MA_PER_LSB);
    if (cur_ma > prm.current_limit_a * 1000.0f) {
      if (!oc_active[k]) {
        oc_active[k]   = true;
        oc_since_ms[k] = now;
      } else if (now - oc_since_ms[k] >= prm.overcurrent_trip_ms) {
        raiseFault(FLT_OVERCURRENT, id, cur_ma);
        torqueOffFaulted(k);  // 가드에 밀고 있는 조향 등은 계속 힘을 주면 과열된다
      }
    } else {
      oc_active[k] = false;
    }
  }

  if (estopActive()) triggerEstop(0);

  setDerate(max_tc >= prm.temp_derate_c || bv_ema <= prm.batt_derate_v);

  warns = 0;
  if (max_tc >= prm.temp_warn_c)      warns |= WRN_TEMP;
  if (bv_ema <= prm.batt_warn_v)      warns |= WRN_BATT;
  if (derate_on)                      warns |= WRN_DERATE;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!motor_ok[k]) { warns |= WRN_INIT; break; }
  }
  if (IMU_ENABLE && !imu_ok) warns |= WRN_IMU;
}

// 구동 목표: 바퀴 각속도 → 모터 rpm → gain·trim·데드밴드·offset → 비율 유지 상한 → 부호.
// 조향 목표: 허용될 때만 명령각, 아니면 현재 목표 유지.
void computeGoals() {
  const bool allow = motionAllowed();
  float rpm[MODULE_COUNT];
  float over = 1.0f;  // 상한 대비 최대 초과 비율

  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    rpm[m] = 0.0f;
    if (!allow) continue;
    float r = wheelRadsToMotorRpm(cmd_wheel[m]) * prm.cmd_gain * prm.trim[m];
    if (fabsf(r) < prm.min_motor_rpm) {
      r = 0.0f;
    } else {
      r += copysignf(prm.cmd_offset_rpm, r);
    }
    rpm[m] = r;

    float cap = prm.max_motor_rpm * speed_scale;
    const float reg_cap = vel_limit_raw[m] * RPM_PER_LSB;  // 모터 Velocity Limit 을 넘으면 쓰기가 거부됨
    if (reg_cap > 0.0f && reg_cap < cap) cap = reg_cap;
    if (cap <= 0.0f) {
      over = INFINITY;
    } else if (fabsf(r) / cap > over) {
      over = fabsf(r) / cap;
    }
  }

  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    const float r = isfinite(over) ? rpm[m] / over : 0.0f;
    int32_t raw = (int32_t)lroundf(prm.drive_dir[m] * r / RPM_PER_LSB);
    if (vel_limit_raw[m] > 0) {
      if (raw >  vel_limit_raw[m]) raw =  vel_limit_raw[m];
      if (raw < -vel_limit_raw[m]) raw = -vel_limit_raw[m];
    }
    goal_vel_raw[m] = raw;
    if (allow) goal_pos_raw[m] = cmd_steer_raw[m];
  }
}

// ---------------------------------------------------------------
// 송신
// ---------------------------------------------------------------
void sendStatus() {
  float total_a = 0.0f;
  for (uint8_t k = 0; k < N_ACTIVE; k++) total_a += fabsf(fb[k].current * MA_PER_LSB) / 1000.0f;

  Serial.print("{\"t\":");    Serial.print(millis());
  Serial.print(",\"bv\":");   Serial.print(bv_ema, 2);
  Serial.print(",\"ia\":");   Serial.print(total_a, 2);
  Serial.print(",\"en\":");   Serial.print(motionAllowed() ? 1 : 0);
  Serial.print(",\"tq\":");   Serial.print(torque_on ? 1 : 0);
  Serial.print(",\"wd\":");   Serial.print(wd_tripped ? 1 : 0);
  Serial.print(",\"flt\":");  Serial.print(faults);
  Serial.print(",\"wrn\":");  Serial.print(warns);
  Serial.print(",\"ds\":");   Serial.print(speed_scale, 2);
  Serial.print(",\"rcv\":");  Serial.print(rcv_phase == RCV_BOOTING ? 1 : 0);
  Serial.print(",\"st\":");   Serial.print((int)st_phase);
  Serial.print(",\"lnk\":");  Serial.print(linked ? 1 : 0);
  uint16_t miss = 0;
  for (uint8_t k = 0; k < N_ACTIVE; k++) if (missing[k]) miss |= (1u << k);
  Serial.print(",\"miss\":"); Serial.print(miss);
  Serial.print(",\"mod\":[");
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    const uint8_t d = m;
    const uint8_t s = MODULE_COUNT + m;
    const float wv = prm.drive_dir[m] * motorRpmToWheelRads(fb[d].velocity * RPM_PER_LSB);
    if (m) Serial.print(',');
    Serial.print("{\"c\":\"");  Serial.print(MODULE_NAMES[m]);
    Serial.print("\",\"ok\":"); Serial.print(fresh[d] && (!STEER_ENABLE || fresh[s]) ? 1 : 0);
    Serial.print(",\"wv\":");   Serial.print(wv, 3);
    if (STEER_ENABLE) {
      Serial.print(",\"sp\":"); Serial.print(steerRawToRad(m, fb[s].position), 4);
    }
    Serial.print(",\"cur\":");  Serial.print(lroundf(fb[d].current * MA_PER_LSB));
    Serial.print(",\"tc\":");   Serial.print(fb[d].temperature);
    Serial.print(",\"e\":");    Serial.print(hw_err[d]);
    if (STEER_ENABLE) {
      Serial.print(",\"scur\":"); Serial.print(lroundf(fb[s].current * MA_PER_LSB));
      Serial.print(",\"stc\":");  Serial.print(fb[s].temperature);
      Serial.print(",\"se\":");   Serial.print(hw_err[s]);
    }
    Serial.print('}');
  }
  Serial.println("]}");
}

void sendImu() {
  float q[4], g[3], a[3];
  for (uint8_t i = 0; i < 4; i++) q[i] = imu.quat[i];
  for (uint8_t i = 0; i < 3; i++) {
    g[i] = imu.gyroData[i] * IMU_GYRO_RADS_PER_LSB;
    a[i] = imu.accData[i] * IMU_ACC_MPS2_PER_LSB;
  }
  // 보드 식별자를 함께 전송한다. 브릿지는 이 값으로 좌우 IMU를 구분한다.
  Serial.print("{\"t\":");          Serial.print(millis());
  Serial.print(",\"board\":\""); Serial.print(BOARD_SIDE); Serial.print("\"");
  Serial.print(",\"imu\":{\"ok\":"); Serial.print(imu.bConnected ? 1 : 0);
  Serial.print(",\"q\":");          printFloatArray(q, 4, 5);
  Serial.print(",\"g\":");          printFloatArray(g, 3, 4);
  Serial.print(",\"a\":");          printFloatArray(a, 3, 3);
  Serial.println("}}");
}

// 핸드셰이크 응답: 이 보드가 누구인지 한 줄로. 모터 레지스터를 읽지 않아 즉답한다.
void sendIdentity() {
  Serial.print("{\"hello\":1,\"proto\":1,\"board\":\"" BOARD_ROLE "\"");
  Serial.print(",\"side\":\"" BOARD_SIDE "\"");
  Serial.print(",\"kind\":\"" BOARD_KIND "\"");
  Serial.print(",\"fw\":\"" FW_NAME " " FW_VERSION "\"");
  Serial.print(",\"order\":[");
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    if (m) Serial.print(',');
    Serial.print('"'); Serial.print(MODULE_NAMES[m]); Serial.print('"');
  }
  Serial.println("]}");
}

// 설정 송신. with_regs 이면 모터 레지스터를 직접 읽어 실제 적용값을 보인다(수용기준 6).
void sendConfig(bool with_regs) {
  Serial.print("{\"cfg\":1,\"fw\":\"" FW_NAME " " FW_VERSION "\"");
  Serial.print(",\"board\":\"" BOARD_ROLE "\"");
  Serial.print(",\"tq\":");  Serial.print(torque_on ? 1 : 0);
  Serial.print(",\"flt\":"); Serial.print(faults);
  // 컴파일 고정값: 브리지가 YAML 과 같은지 비교하는 용도
  Serial.print(",\"gear\":");     Serial.print(GEAR_G, 2);
  Serial.print(",\"dxl_baud\":"); Serial.print(DXL_BAUDRATE);
  Serial.print(",\"side\":\"" BOARD_SIDE "\"");
  Serial.print(",\"steer_en\":"); Serial.print(STEER_ENABLE ? 1 : 0);
  Serial.print(",\"imu_en\":");   Serial.print(IMU_ENABLE ? 1 : 0);
  Serial.print(",\"order\":[");
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    if (m) Serial.print(',');
    Serial.print('"'); Serial.print(MODULE_NAMES[m]); Serial.print('"');
  }
  Serial.print("],\"drive_ids\":[");
  for (uint8_t m = 0; m < MODULE_COUNT; m++) { if (m) Serial.print(','); Serial.print(DRIVE_IDS[m]); }
  Serial.print("],\"steer_ids\":[");
  for (uint8_t m = 0; m < MODULE_COUNT; m++) { if (m) Serial.print(','); Serial.print(STEER_IDS[m]); }
  Serial.print("],\"ok\":[");
  for (uint8_t k = 0; k < N_ACTIVE; k++) { if (k) Serial.print(','); Serial.print(motor_ok[k] ? 1 : 0); }
  Serial.print("],\"prm\":{");
  Serial.print("\"max_rpm\":");  Serial.print(prm.max_motor_rpm, 2);
  Serial.print(",\"min_rpm\":"); Serial.print(prm.min_motor_rpm, 2);
  Serial.print(",\"gain\":");    Serial.print(prm.cmd_gain, 4);
  Serial.print(",\"offset\":");  Serial.print(prm.cmd_offset_rpm, 2);
  Serial.print(",\"trim\":");    printFloatArray(prm.trim, MODULE_COUNT, 4);
  Serial.print(",\"ddir\":");    printIntArray(prm.drive_dir, MODULE_COUNT);
  Serial.print(",\"cur_a\":");   Serial.print(prm.current_limit_a, 3);
  Serial.print(",\"pacc\":");    Serial.print(prm.drive_profile_acc);
  Serial.print(",\"range\":");   Serial.print(prm.steer_range_rad, 4);
  Serial.print(",\"home\":");    printFloatArray(prm.steer_home_rad, MODULE_COUNT, 4);
  Serial.print(",\"sdir\":");    printIntArray(prm.steer_dir, MODULE_COUNT);
  Serial.print(",\"spvel\":");   Serial.print(prm.steer_profile_vel);
  Serial.print(",\"spacc\":");   Serial.print(prm.steer_profile_acc);
  Serial.print(",\"wd_ms\":");   Serial.print(prm.watchdog_ms);
  Serial.print(",\"temp\":[");   Serial.print(prm.temp_warn_c, 1); Serial.print(',');
                                 Serial.print(prm.temp_derate_c, 1); Serial.print(',');
                                 Serial.print(prm.temp_abort_c, 1); Serial.print(']');
  Serial.print(",\"batt\":[");   Serial.print(prm.batt_warn_v, 2); Serial.print(',');
                                 Serial.print(prm.batt_derate_v, 2); Serial.print(',');
                                 Serial.print(prm.batt_stop_v, 2); Serial.print(']');
  Serial.print(",\"derate\":");  Serial.print(prm.derate_scale, 2);
  Serial.print(",\"oc_ms\":");   Serial.print(prm.overcurrent_trip_ms);
  Serial.print(",\"hw_retry\":");    Serial.print(prm.hw_retry_max);
  Serial.print(",\"hw_retry_ms\":"); Serial.print(prm.hw_retry_delay_ms);
  Serial.print(",\"st_nl\":");  printFloatArray(prm.noload_current_ma, MODULE_COUNT, 1);
  Serial.print(",\"st_tol\":"); Serial.print(prm.selftest_current_tol, 2);
  Serial.print('}');

  if (with_regs) {
    // 응답 없는 모터는 null. 레지스터 값 그대로(raw)와 환산값을 같이 보인다.
    Serial.print(",\"reg\":{\"mode\":[");
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (k) Serial.print(',');
      const int32_t v = motor_ok[k] ? dxl.readControlTableItem(OPERATING_MODE, motorId(k), REG_TIMEOUT_MS) : 0;
      if (motor_ok[k] && dxl.getLastLibErrCode() == DXL_LIB_OK) Serial.print(v); else Serial.print("null");
    }
    Serial.print("],\"cl_ma\":[");
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (k) Serial.print(',');
      const int32_t v = motor_ok[k] ? dxl.readControlTableItem(CURRENT_LIMIT, motorId(k), REG_TIMEOUT_MS) : 0;
      if (motor_ok[k] && dxl.getLastLibErrCode() == DXL_LIB_OK) Serial.print(v * MA_PER_LSB, 0); else Serial.print("null");
    }
    Serial.print("],\"pacc\":[");
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (k) Serial.print(',');
      const int32_t v = motor_ok[k] ? dxl.readControlTableItem(PROFILE_ACCELERATION, motorId(k), REG_TIMEOUT_MS) : 0;
      if (motor_ok[k] && dxl.getLastLibErrCode() == DXL_LIB_OK) Serial.print(v); else Serial.print("null");
    }
    Serial.print("],\"spvel\":[");
    for (uint8_t m = 0; STEER_ENABLE && m < MODULE_COUNT; m++) {
      const uint8_t k = MODULE_COUNT + m;
      if (m) Serial.print(',');
      const int32_t v = motor_ok[k] ? dxl.readControlTableItem(PROFILE_VELOCITY, motorId(k), REG_TIMEOUT_MS) : 0;
      if (motor_ok[k] && dxl.getLastLibErrCode() == DXL_LIB_OK) Serial.print(v); else Serial.print("null");
    }
    Serial.print("],\"vlim_rpm\":[");
    for (uint8_t k = 0; k < N_ACTIVE; k++) {
      if (k) Serial.print(',');
      if (motor_ok[k]) Serial.print(vel_limit_raw[k] * RPM_PER_LSB, 1); else Serial.print("null");
    }
    Serial.print("]}");
  }
  Serial.println('}');
}

// ---------------------------------------------------------------
// 수신
// ---------------------------------------------------------------
// 공백으로 구분된 숫자를 최대 max_n 개 읽는다. 숫자가 아니거나 개수 초과면 -1.
int8_t parseFloats(char* p, float* out, uint8_t max_n) {
  uint8_t n = 0;
  while (true) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return n;
    if (n >= max_n) return -1;
    char* end = nullptr;
    const float v = strtof(p, &end);
    if (end == p || !isfinite(v)) return -1;
    if (*end != ' ' && *end != '\t' && *end != '\0') return -1;
    out[n++] = v;
    p = end;
  }
}

// 프레임 거부: 구동 0, 조향 유지. 워치독은 갱신하지 않는다.
void rejectDriveCommand(const char* why, int16_t module) {
  for (uint8_t m = 0; m < MODULE_COUNT; m++) cmd_wheel[m] = 0.0f;
  motion_en = false;
  Serial.print("{\"err\":\"");
  Serial.print(why);
  Serial.print('"');
  if (module >= 0) {
    Serial.print(",\"c\":\"");
    Serial.print(MODULE_NAMES[module]);
    Serial.print('"');
  }
  Serial.println('}');
}

void handleDriveCommand(char* args) {
  const uint8_t N = 1 + 2 * MODULE_COUNT;
  float v[N];
  if (parseFloats(args, v, N) != N) { rejectDriveCommand("bad_frame", -1); return; }
  if (v[0] != 0.0f && v[0] != 1.0f) { rejectDriveCommand("bad_en", -1); return; }

  // [추가] 만약 자동 휴면(토크 OFF) 상태였다면 명령이 들어올 때 자동으로 토크 켜기(E 동작)
  if (!torque_on) {
    enableTorque();
    is_sleeping = false;
  }

  int32_t raw[MODULE_COUNT];
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    if (!steerRadToRaw(m, v[1 + MODULE_COUNT + m], &raw[m])) { rejectDriveCommand("steer_range", m); return; }
  }

  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    cmd_wheel[m]     = v[1 + m];
    cmd_steer_raw[m] = raw[m];
  }
  motion_en   = (v[0] == 1.0f);
  last_cmd_ms = millis();
  wd_tripped  = false;
  wd_alarm    = false;
  
  if (!linked) {
    linked = true;  
    startMelody(LINK_TICK_HZ, LINK_TICK_MS, LINK_TICK_N);
    Serial.print("{\"evt\":\"bridge_linked\",\"t\":");
    Serial.print(last_cmd_ms);
    Serial.println('}');
  }
}

void applyDriveProfileNow() {
  const uint32_t acc = driveProfileAcc();
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    if (motor_ok[m]) dxl.writeControlTableItem(PROFILE_ACCELERATION, DRIVE_IDS[m], acc, REG_TIMEOUT_MS);
  }
}

void applySteerProfileNow() {
  if (!STEER_ENABLE) return;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    const uint8_t k = MODULE_COUNT + m;
    if (!motor_ok[k]) continue;
    dxl.writeControlTableItem(PROFILE_VELOCITY, STEER_IDS[m], prm.steer_profile_vel, REG_TIMEOUT_MS);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, STEER_IDS[m], prm.steer_profile_acc, REG_TIMEOUT_MS);
  }
}

// P <key> <값...> : 토크 OFF 일 때만. 하드 상한·순서 조건을 어기면 거부하고 기존 값을 유지한다.
void handleParam(char* args) {
  if (torque_on) { sendErr("param_needs_torque_off", -1); return; }

  while (*args == ' ' || *args == '\t') args++;
  char* key = args;
  while (*args && *args != ' ' && *args != '\t') args++;
  if (*args) *args++ = '\0';

  // 값은 모듈 수(2)만큼이거나 최대 3개(temp, batt)다.
  const uint8_t MAX_VALUES = MODULE_COUNT > 3 ? MODULE_COUNT : 3;
  float v[MAX_VALUES];
  const int8_t n = parseFloats(args, v, MAX_VALUES);
  if (n < 0) { sendErr("param_value", -1); return; }

  bool ok = false;
  if (!strcmp(key, "max_rpm") && n == 1) {
    ok = v[0] > 0.0f && v[0] <= HARD_MAX_MOTOR_RPM && v[0] > prm.min_motor_rpm;
    if (ok) prm.max_motor_rpm = v[0];
  } else if (!strcmp(key, "min_rpm") && n == 1) {
    ok = v[0] >= 0.0f && v[0] < prm.max_motor_rpm;
    if (ok) prm.min_motor_rpm = v[0];
  } else if (!strcmp(key, "gain") && n == 1) {
    ok = v[0] > 0.0f;
    if (ok) prm.cmd_gain = v[0];
  } else if (!strcmp(key, "offset") && n == 1) {
    ok = v[0] >= 0.0f && v[0] < prm.max_motor_rpm;
    if (ok) prm.cmd_offset_rpm = v[0];
  } else if (!strcmp(key, "trim") && n == MODULE_COUNT) {
    ok = true;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) ok = ok && v[m] > 0.0f;
    if (ok) for (uint8_t m = 0; m < MODULE_COUNT; m++) prm.trim[m] = v[m];
  } else if (!strcmp(key, "ddir") && n == MODULE_COUNT) {
    ok = true;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) ok = ok && isSign(v[m]);
    if (ok) for (uint8_t m = 0; m < MODULE_COUNT; m++) prm.drive_dir[m] = (int8_t)v[m];
  } else if (!strcmp(key, "sdir") && n == MODULE_COUNT) {
    ok = true;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) ok = ok && isSign(v[m]);
    if (ok) for (uint8_t m = 0; m < MODULE_COUNT; m++) prm.steer_dir[m] = (int8_t)v[m];
  } else if (!strcmp(key, "home") && n == MODULE_COUNT) {
    ok = true;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) ok = ok && steerRangeFits(v[m], prm.steer_range_rad);
    if (ok) for (uint8_t m = 0; m < MODULE_COUNT; m++) prm.steer_home_rad[m] = v[m];
  } else if (!strcmp(key, "range") && n == 1) {
    ok = v[0] > 0.0f && v[0] <= HARD_STEER_RANGE_RAD;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) ok = ok && steerRangeFits(prm.steer_home_rad[m], v[0]);
    if (ok) prm.steer_range_rad = v[0];
  } else if (!strcmp(key, "cur_a") && n == 1) {
    ok = v[0] > 0.0f && v[0] <= HARD_MAX_CURRENT_A;
    if (ok) {
      prm.current_limit_a = v[0];
      // EEPROM 항목: 토크 OFF 상태에서만 쓸 수 있다. 다를 때만 쓴다.
      const int32_t want = currentLimitRaw();
      for (uint8_t k = 0; k < N_ACTIVE; k++) {
        if (!motor_ok[k]) continue;
        const int32_t cur = dxl.readControlTableItem(CURRENT_LIMIT, motorId(k), REG_TIMEOUT_MS);
        if (dxl.getLastLibErrCode() != DXL_LIB_OK || cur != want) {
          dxl.writeControlTableItem(CURRENT_LIMIT, motorId(k), want, REG_TIMEOUT_MS);
        }
      }
    }
  } else if (!strcmp(key, "pacc") && n == 1) {
    ok = isIntInRange(v[0], 1, PROFILE_RAW_MAX);
    if (ok) { prm.drive_profile_acc = (uint32_t)v[0]; applyDriveProfileNow(); }
  } else if (!strcmp(key, "spvel") && n == 1) {
    // 0 = 프로파일 없음(Step), 조향 최고속 시험용
    ok = isIntInRange(v[0], 0, PROFILE_RAW_MAX);
    if (ok) { prm.steer_profile_vel = (uint32_t)v[0]; applySteerProfileNow(); }
  } else if (!strcmp(key, "spacc") && n == 1) {
    // 0 = 무한 가속
    ok = isIntInRange(v[0], 0, PROFILE_RAW_MAX);
    if (ok) { prm.steer_profile_acc = (uint32_t)v[0]; applySteerProfileNow(); }
  } else if (!strcmp(key, "wd_ms") && n == 1) {
    ok = isIntInRange(v[0], 1, HARD_MAX_WATCHDOG_MS);
    if (ok) prm.watchdog_ms = (uint32_t)v[0];
    
  } 
  // ===== [여기 아래에 idle_ms 처리 블록을 추가하세요] =====
  else if (!strcmp(key, "idle_ms") && n == 1) {
    ok = isIntInRange(v[0], 1000, HARD_MAX_IDLE_TIMEOUT_MS);
    if (ok) prm.idle_timeout_ms = (uint32_t)v[0];
  }
  // ========================================================
  
  else if (!strcmp(key, "temp") && n == 3) {
    ok = v[0] > 0.0f && v[0] < v[1] && v[1] < v[2] && v[2] <= HARD_TEMP_ABORT_MAX_C;
    if (ok) { prm.temp_warn_c = v[0]; prm.temp_derate_c = v[1]; prm.temp_abort_c = v[2]; }
  } else if (!strcmp(key, "batt") && n == 3) {
    ok = v[0] > v[1] && v[1] > v[2] && v[2] >= HARD_BATT_STOP_MIN_V;
    if (ok) { prm.batt_warn_v = v[0]; prm.batt_derate_v = v[1]; prm.batt_stop_v = v[2]; }
  } else if (!strcmp(key, "derate") && n == 1) {
    ok = v[0] >= HARD_MIN_DERATE_SCALE && v[0] <= 1.0f;
    if (ok) {
      prm.derate_scale = v[0];
      if (derate_on) { speed_scale = prm.derate_scale; applyDriveProfileNow(); }
    }
  } else if (!strcmp(key, "oc_ms") && n == 1) {
    ok = isIntInRange(v[0], 1, 1000);
    if (ok) prm.overcurrent_trip_ms = (uint32_t)v[0];
  } else if (!strcmp(key, "hw_retry") && n == 1) {
    ok = isIntInRange(v[0], 0, 10);  // 0 = 자동복구 끔
    if (ok) prm.hw_retry_max = (uint8_t)v[0];
  } else if (!strcmp(key, "hw_retry_ms") && n == 1) {
    ok = isIntInRange(v[0], 100, 60000);
    if (ok) prm.hw_retry_delay_ms = (uint32_t)v[0];
  } else if (!strcmp(key, "st_nl") && n == MODULE_COUNT) {
    ok = true;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) ok = ok && v[m] > 0.0f;
    if (ok) for (uint8_t m = 0; m < MODULE_COUNT; m++) prm.noload_current_ma[m] = v[m];
  } else if (!strcmp(key, "st_tol") && n == 1) {
    ok = v[0] > 1.0f && v[0] <= 10.0f;
    if (ok) prm.selftest_current_tol = v[0];
  } else {
    sendErr("param_key_or_count", -1);
    return;
  }

  if (!ok) { sendErr("param_rejected", -1); return; }
  sendConfig(false);
}

void handleLine(char* line) {
  // 자가진단 중 호스트가 C/E/D/S 를 보내면 자가진단을 멈추고 호스트 명령을 따른다.
  if (st_phase == ST_RUN && strchr("CEDS", line[0]) != nullptr) abortSelfTest("host_command");
  switch (line[0]) {
    case 'C': handleDriveCommand(line + 1); break;
    case 'E': enableTorque();  sendConfig(false); break;
    case 'D': disableTorque(); sendConfig(false); break;
    case 'S': for (uint8_t m = 0; m < MODULE_COUNT; m++) cmd_wheel[m] = 0.0f; break;
    case '?': sendConfig(true); break;
    case 'I': sendIdentity(); break;
    case 'P': handleParam(line + 1); break;
    default:  sendErr("unknown_cmd", -1); break;
  }
}

void readHostSerial() {
  static char buf[192];
  static uint8_t len = 0;
  static bool overflow = false;
  uint16_t budget = MAX_RX_BYTES_PER_LOOP;  // 호스트가 몰아 보내도 제어 주기를 굶기지 않게

  while (budget-- > 0 && Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (overflow) {
        sendErr("line_too_long", -1);
      } else if (len > 0) {
        buf[len] = '\0';
        handleLine(buf);
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    } else {
      overflow = true;
    }
  }
}

// ---------------------------------------------------------------
// 상태 표시: LED·버저 (handoff §5.6)
// ---------------------------------------------------------------
void ledWrite(int pin, bool on) {
  digitalWrite(pin, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

// 같은 주파수면 다시 부르지 않는다(tone 재시작 잡음 방지).
void buzzerSet(uint16_t hz) {
  if (hz == buzzer_hz) return;
  if (hz) tone(BUZZER_PIN, hz); else noTone(BUZZER_PIN);
  buzzer_hz = hz;
}

// 멜로디 시작: ① READY 징글(도–미–솔), ② LINKED 틱(띠릭). 재생은 updateIndicators 가 한다.
void startMelody(const uint16_t* hz, const uint16_t* ms, uint8_t n) {
  mel_hz = hz;
  mel_ms = ms;
  mel_n  = n;
  mel_t0 = millis();
}

// 재생 중이면 지금 울릴 주파수를 out 에 넣고 true. 끝났으면 false.
bool melodyNow(uint32_t now, uint16_t* out) {
  if (mel_n == 0) return false;
  uint32_t t = now - mel_t0;
  for (uint8_t i = 0; i < mel_n; i++) {
    if (t < mel_ms[i]) { *out = mel_hz[i]; return true; }
    t -= mel_ms[i];
  }
  mel_n = 0;
  return false;
}

// 연결 폴트 코드: 1 = 구동 미검출, 2 = 조향 미검출, 3 = 둘 다, 0 = 없음
uint8_t connectionCode() {
  bool drive = false, steer = false;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!missing[k]) continue;
    if (k < MODULE_COUNT) drive = true; else steer = true;
  }
  if (drive && steer) return 3;
  if (drive) return 1;
  if (steer) return 2;
  return 0;
}

bool selfTestArmed() {
  if (st_phase != ST_IDLE || faults != 0 || rcv_phase != RCV_IDLE || motionAllowed()) return false;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!motor_ok[k] || missing[k]) return false;
  }
  return true;
}

// 우선순위: 자가진단 > 치명 > 연결 폴트 > 기타 폴트(링크 끊김 포함) > 경고(앰버) > READY(초록).
// 파랑: ② LINKED(브릿지 프레임 수신 중) 점멸, ① READY 만이면 꺼짐.
// 징글·틱 멜로디가 재생 중이면 버저는 멜로디가 우선.
void updateIndicators(uint32_t now) {
  ledWrite(LED_HEARTBEAT_PIN, (now / 500) % 2 == 0);  // 1 Hz 하트비트. 멈추면 MCU 정지

  bool red = false, green = false, blue = false;
  uint16_t hz = 0;

  if (st_phase == ST_HOLD) {
    // 누르는 동안: 1초마다 높아지는 비프, 파랑 켜짐. 3초 전에 떼면 취소.
    const uint32_t held = now - st_hold_t0;
    green = true;
    blue  = true;
    if (held % 1000 < 100) hz = 1000 + 500 * (held / 1000);
  } else if (st_phase == ST_RUN) {
    green = true;
    blue  = (now / 100) % 2;
  } else {
    const bool critical = (faults & FLT_OVERTEMP) ||
                          ((faults & FLT_UNDERVOLT) && bv_last >= NO_BATTERY_V);
    const uint8_t conn = connectionCode();
    const bool other = (faults & ~(uint16_t)(FLT_OVERTEMP | FLT_UNDERVOLT)) || (warns & WRN_IMU) || wd_alarm;

    if (critical) {
      const bool on = now % (BEEP_CRIT_ON_MS + BEEP_CRIT_OFF_MS) < BEEP_CRIT_ON_MS;
      red = on;
      hz  = on ? BEEP_CRIT_HZ : 0;
    } else if (conn) {
      const uint32_t slot = BEEP_SHORT_ON_MS + BEEP_SHORT_GAP_MS;
      const uint32_t t = now % (conn * slot + BEEP_GROUP_PAUSE_MS);
      const bool on = (t / slot) < conn && (t % slot) < BEEP_SHORT_ON_MS;
      red = on;
      hz  = on ? BEEP_SHORT_HZ : 0;
    } else if (other) {
      red = (now / 250) % 2;
      hz  = (now % BEEP_LONG_PERIOD_MS) < BEEP_LONG_ON_MS ? BEEP_LONG_HZ : 0;
    } else if (faults) {
      red = true;  // 배터리 없이 USB 전원만: 저전압이지만 울리지 않음
    } else if (warns & (WRN_TEMP | WRN_BATT | WRN_DERATE | WRN_INIT)) {
      red   = true;  // 빨강 + 초록 = 앰버
      green = true;
      if ((warns & WRN_BATT) && (now % BEEP_BATT_PERIOD_MS) < BEEP_BATT_ON_MS) hz = BEEP_BATT_HZ;
    } else {
      green = true;  // READY
    }

    // 파랑: ② LINKED(브릿지 프레임 수신 중)이면 점멸, ① READY 만이면 꺼짐.
    if (linked) blue = (now / 100) % 2;
  }

  uint16_t mel;
  if (melodyNow(now, &mel)) hz = mel;

  ledWrite(LED_RED_PIN, red);
  ledWrite(LED_GREEN_PIN, green);
  ledWrite(LED_BLUE_PIN, blue);
  buzzerSet(hz);
}

// 부팅 자가진단(POST). 모터를 움직이지 않는다.
// ① LED 스윕 ② 배터리 ③ 구동/조향 그룹 ping(설정 단계에서 이미 함) ④ IMU ⑤ HW 에러·온도
void runPost() {
  sampleBattery();
  if (bv_last <= prm.batt_stop_v) raiseFault(FLT_UNDERVOLT, -1, bv_last);
  imu_ok = IMU_ENABLE && imu.bConnected;
  readFeedback();
  readHwErrors();

  bool ok = !(faults & FLT_UNDERVOLT) && (imu_ok || !IMU_ENABLE);
  Serial.print("{\"post\":{\"bv\":"); Serial.print(bv_last, 2);
  Serial.print(",\"drive_missing\":[");
  bool first = true;
  for (uint8_t k = 0; k < MODULE_COUNT; k++) {
    if (!missing[k]) continue;
    if (!first) Serial.print(',');
    Serial.print(motorId(k)); first = false; ok = false;
  }
  Serial.print("],\"steer_missing\":[");
  first = true;
  for (uint8_t k = MODULE_COUNT; k < N_ACTIVE; k++) {
    if (!missing[k]) continue;
    if (!first) Serial.print(',');
    Serial.print(motorId(k)); first = false; ok = false;
  }
  Serial.print("],\"imu\":"); Serial.print(IMU_ENABLE ? (imu_ok ? 1 : 0) : -1);  // -1 = 이 보드는 IMU 를 쓰지 않음
  Serial.print(",\"hw_err\":[");
  first = true;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!hwerr_fresh[k] || hw_err[k] == 0) continue;
    if (!first) Serial.print(',');
    Serial.print(motorId(k)); first = false; ok = false;
  }
  Serial.print("],\"hot\":[");
  first = true;
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    if (!fresh[k] || fb[k].temperature < prm.temp_derate_c) continue;
    if (!first) Serial.print(',');
    Serial.print(motorId(k)); first = false; ok = false;
  }
  Serial.print("],\"ok\":"); Serial.print(ok ? 1 : 0);
  Serial.println("}}");
  if (ok) startMelody(READY_JINGLE_HZ, READY_JINGLE_MS, READY_JINGLE_N);  // ① READY
}

// ---------------------------------------------------------------
// 버튼 자가진단: 최고속도로 앞으로 SELFTEST_DISTANCE_M(10 m), 뒤로 같은 거리. 로봇이 실제로 움직인다.
// 이 보드의 구동 2개만 돌린다. 지면에서 한쪽 보드만 실행하면 차체가 돈다.
// ---------------------------------------------------------------
void printSelfTestEvent(const char* what) {
  Serial.print("{\"evt\":\"");
  Serial.print(what);
  Serial.print("\",\"t\":");
  Serial.print(millis());
  Serial.println('}');
}

// 자가진단을 끝내거나 중단할 때 토크는 끄지 않는다. 토크를 끄면 조향축이 풀려
// 차체가 방향을 잃고 바퀴가 굴러간다. 구동만 0(능동 제동)으로 세우고 조향은 0 rad 로 잡는다.
void holdSelfTestStop() {
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    cmd_wheel[m]    = 0.0f;
    goal_vel_raw[m] = 0;
  }
  if (STEER_ENABLE) {
    for (uint8_t m = 0; m < MODULE_COUNT; m++) {
      int32_t raw;
      if (steerRadToRaw(m, 0.0f, &raw)) {
        goal_pos_raw[m]  = raw;
        cmd_steer_raw[m] = raw;
      }
    }
  }
  motion_en = false;
  if (!torque_on) return;
  sw_vel.is_info_changed = true;
  dxl.syncWrite(&sw_vel);
  if (STEER_ENABLE) {
    sw_pos.is_info_changed = true;
    dxl.syncWrite(&sw_pos);
  }
}

void abortSelfTest(const char* why) {
  if (st_phase == ST_IDLE) return;
  const bool was_running = (st_phase == ST_RUN);
  st_phase = ST_IDLE;
  if (was_running) holdSelfTestStop();
  Serial.print("{\"evt\":\"selftest_abort\",\"why\":\"");
  Serial.print(why);
  Serial.print("\",\"t\":");
  Serial.print(millis());
  Serial.println('}');
}

// 모터 틱 → 바퀴가 굴러간 거리(m). 모터 1회전 = 바퀴 GEAR_G 회전.
float ticksToMeters(int32_t ticks) {
  return (float)ticks / TICKS_PER_REV * GEAR_G * TWO_PI_F * WHEEL_RADIUS_M;
}

// 출발점에서 이 바퀴가 굴러간 거리(m). 로봇 전진 방향이 +.
float selfTestTravel(uint8_t m) {
  return prm.drive_dir[m] * ticksToMeters(fb[m].position - st_start_pos[m]);
}

// 지금 속도에서 구동 프로파일 가속도로 멈출 때까지 더 굴러가는 거리(m) + 제어 지연 여유.
// Velocity 모드의 Profile Acceleration 은 가속과 감속에 똑같이 적용된다.
float selfTestBrakeDistance(uint8_t m) {
  const float v_rev_s  = fabsf(fb[m].velocity * RPM_PER_LSB) / 60.0f;
  const float a_rev_s2 = driveProfileAcc() * PROFILE_ACC_REV_MIN2_PER_LSB / 3600.0f;
  const float rev = v_rev_s * v_rev_s / (2.0f * a_rev_s2) + v_rev_s * SELFTEST_LATENCY_S;
  return rev * GEAR_G * TWO_PI_F * WHEEL_RADIUS_M;
}

// 목표 속도 = 낼 수 있는 최고속도: 속도 상한(derate 반영), 프로젝트 속도 상한, 모터 Velocity Limit 중 가장 낮은 값.
// 거리가 짧으면 가속 프로파일 때문에 이 속도에 닿기 전에 감속을 시작한다.
float selfTestTargetRpm() {
  float rpm = prm.max_motor_rpm * speed_scale;
  const float cap = wheelRadsToMotorRpm(SELFTEST_MAX_SPEED_MPS / WHEEL_RADIUS_M);
  if (rpm > cap) rpm = cap;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    const float reg_cap = vel_limit_raw[m] * RPM_PER_LSB;
    if (reg_cap > 0.0f && reg_cap < rpm) rpm = reg_cap;
  }
  return rpm;
}

void beginSelfTestStep(SelfTestStep step, uint32_t now) {
  st_step    = step;
  st_step_t0 = now;
  st_still   = false;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    st_braking[m] = false;
    if (step == STS_FORWARD) st_start_pos[m] = fb[m].position;  // 여기가 출발점
  }
}

void startSelfTest(uint32_t now) {
  enableTorque();  // 재설정·조건 확인·조향 현재각 유지로 토크 ON
  if (!torque_on) {
    st_phase = ST_IDLE;
    sendErr("selftest_enable_failed", -1);
    return;
  }
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    st_fwd_m[m] = st_end_m[m] = 0.0f;
    st_vmax[m] = st_imax[m] = 0.0f;
  }
  st_rpm = selfTestTargetRpm();
  const float v_mps = motorRpmToWheelRads(st_rpm) * WHEEL_RADIUS_M;
  if (v_mps < SELFTEST_MIN_SPEED_MPS) {
    st_phase = ST_IDLE;
    holdSelfTestStop();
    sendErr("selftest_speed_too_low", -1);
    return;
  }
  st_leg_timeout_ms = (uint32_t)(SELFTEST_DISTANCE_M / v_mps * SELFTEST_TIMEOUT_FACTOR * 1000.0f) +
                      SELFTEST_TIMEOUT_MARGIN_MS;
  st_phase = ST_RUN;
  beginSelfTestStep(STEER_ENABLE ? STS_ALIGN : STS_FORWARD, now);
  printSelfTestEvent("selftest_start");
}

void finishSelfTest() {
  bool pass = true;
  Serial.print("{\"selftest\":{\"dist_m\":"); Serial.print(SELFTEST_DISTANCE_M, 2);
  Serial.print(",\"rpm\":");                  Serial.print(st_rpm, 1);
  // 1 = 조향을 정면으로 맞추고 달림, 0 = 조향 꺼짐(조향축을 잡지 않은 채 달림)
  Serial.print(",\"steer\":");                Serial.print(STEER_ENABLE ? 1 : 0);
  Serial.print(",\"drive\":[");
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    const bool ok = fabsf(st_fwd_m[m] - SELFTEST_DISTANCE_M) <= SELFTEST_DIST_TOL_M &&
                    fabsf(st_end_m[m]) <= SELFTEST_DIST_TOL_M;
    pass = pass && ok;
    if (m) Serial.print(',');
    Serial.print("{\"c\":\"");    Serial.print(MODULE_NAMES[m]);
    Serial.print("\",\"id\":");   Serial.print(DRIVE_IDS[m]);
    Serial.print(",\"fwd_m\":");  Serial.print(st_fwd_m[m], 3);
    Serial.print(",\"end_m\":");  Serial.print(st_end_m[m], 3);
    Serial.print(",\"vmax\":");   Serial.print(st_vmax[m], 1);
    Serial.print(",\"imax\":");   Serial.print(st_imax[m], 0);
    Serial.print(",\"ok\":");     Serial.print(ok ? 1 : 0);
    Serial.print('}');
  }
  Serial.print("],\"result\":\"");
  Serial.print(pass ? "pass" : "fail");
  Serial.println("\"}}");

  st_phase = ST_IDLE;
  holdSelfTestStop();  // 토크 유지: 시험이 끝나도 조향을 잡고 구동은 0(능동 제동)
  if (pass) startMelody(READY_JINGLE_HZ, READY_JINGLE_MS, READY_JINGLE_N);
  else raiseFault(FLT_SELFTEST, -1, 0.0f);  // 래치: 결과를 보고 E 로 해제
}

// 제어 주기마다 computeGoals 다음에 불러 목표를 덮어쓴다.
// 조향 정렬(켜져 있을 때) → 앞으로 SELFTEST_DISTANCE_M → 멈춤 → 뒤로 출발점까지 → 결과.
// 바퀴마다 남은 거리가 제동 거리 이하가 되면 목표를 0 으로 내려 프로파일 감속으로 멈춘다.
void selfTestTick(uint32_t now) {
  if (st_phase != ST_RUN) return;
  if (faults != 0 || !torque_on) { abortSelfTest("fault"); return; }

  const uint32_t elapsed = now - st_step_t0;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) goal_vel_raw[m] = 0;

  // 자가진단이 진행되는 모든 단계에서 조향 목표를 0 rad로 유지한다.
  // STEER_ENABLE=true이면 조향 모터 토크도 enableTorque()에서 켜져 계속 유지된다.
  if (STEER_ENABLE) {
    for (uint8_t m = 0; m < MODULE_COUNT; m++) {
      int32_t steeringZeroPositionRaw;
      if (!steerRadToRaw(m, 0.0f, &steeringZeroPositionRaw)) {
        abortSelfTest("steer_range");
        return;
      }
      goal_pos_raw[m] = steeringZeroPositionRaw;
    }
  }

  if (st_step == STS_ALIGN) {
    bool aligned = true;
    for (uint8_t m = 0; m < MODULE_COUNT; m++) {
      int32_t raw;
      if (!steerRadToRaw(m, 0.0f, &raw)) { abortSelfTest("steer_range"); return; }
      goal_pos_raw[m] = raw;
      const uint8_t s = MODULE_COUNT + m;
      const float err_deg = fabsf((float)(wrapRaw(fb[s].position) - raw)) / ticksPerRad() * 360.0f / TWO_PI_F;
      if (!fresh[s] || err_deg > SELFTEST_STEER_TOL_DEG) aligned = false;
    }
    if (aligned) {
      beginSelfTestStep(STS_FORWARD, now);
    } else if (elapsed >= SELFTEST_STEER_TIMEOUT_MS) {
      abortSelfTest("steer_align");
      raiseFault(FLT_SELFTEST, -1, 0.0f);
    }
    return;
  }

  if (elapsed >= st_leg_timeout_ms) {
    abortSelfTest("timeout");  // 걸림·미연결·거리 미달
    raiseFault(FLT_SELFTEST, -1, 0.0f);
    return;
  }

  const bool  forward = (st_step == STS_FORWARD);
  const float dir     = forward ? 1.0f : -1.0f;
  // 달리는 중에 derate 가 걸리면 낮아진 상한을 따른다.
  const float rpm_cmd = fminf(st_rpm, selfTestTargetRpm());
  bool all_still = true;
  for (uint8_t m = 0; m < MODULE_COUNT; m++) {
    const float travel    = selfTestTravel(m);
    const float remaining = forward ? SELFTEST_DISTANCE_M - travel : travel;
    const float rpm_now   = fabsf(fb[m].velocity * RPM_PER_LSB);
    const float cur_now   = fabsf(fb[m].current * MA_PER_LSB);
    if (rpm_now > st_vmax[m]) st_vmax[m] = rpm_now;
    if (cur_now > st_imax[m]) st_imax[m] = cur_now;

    if (!st_braking[m] && remaining <= selfTestBrakeDistance(m)) st_braking[m] = true;
    if (!st_braking[m]) goal_vel_raw[m] = (int32_t)lroundf(prm.drive_dir[m] * dir * rpm_cmd / RPM_PER_LSB);
    if (!st_braking[m] || rpm_now >= SELFTEST_STOP_RPM) all_still = false;
  }

  // 모든 바퀴가 멈춘 뒤 SELFTEST_SETTLE_MS 동안 그대로면 다음 단계로
  if (!all_still) { st_still = false; return; }
  if (!st_still) { st_still = true; st_still_t0 = now; return; }
  if (now - st_still_t0 < SELFTEST_SETTLE_MS) return;

  if (forward) {
    for (uint8_t m = 0; m < MODULE_COUNT; m++) st_fwd_m[m] = selfTestTravel(m);
    beginSelfTestStep(STS_BACKWARD, now);
  } else {
    for (uint8_t m = 0; m < MODULE_COUNT; m++) st_end_m[m] = selfTestTravel(m);
    finishSelfTest();
  }
}

bool readBtnRaw(int pin) {
  const int v = digitalRead(pin);
  return BTN_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

// 버튼: SW1 3초 길게 = 자가진단 시작(조건 맞을 때만), SW2 = 즉시 중단. 시리얼로는 시작 불가.
void updateButtons(uint32_t now) {
  bool pressed_edge[2] = {false, false};
  bool released_edge[2] = {false, false};
  for (uint8_t i = 0; i < 2; i++) {
    const bool raw = readBtnRaw(BTN_PINS[i]);
    if (raw != btn_raw[i]) {
      btn_raw[i] = raw;
      btn_t[i]   = now;
    } else if (now - btn_t[i] >= BTN_DEBOUNCE_MS && btn_state[i] != raw) {
      btn_state[i] = raw;
      if (raw) pressed_edge[i] = true; else released_edge[i] = true;
    }
  }

  if (pressed_edge[1] && st_phase != ST_IDLE) abortSelfTest("stop_button");

  if (pressed_edge[0] && st_phase == ST_IDLE) {
    if (selfTestArmed()) {
      st_phase   = ST_HOLD;
      st_hold_t0 = now;
      printSelfTestEvent("selftest_hold");
    } else {
      sendErr("selftest_not_ready", -1);
    }
  }
  if (st_phase == ST_HOLD) {
    if (released_edge[0] || !btn_state[0]) {
      st_phase = ST_IDLE;
      printSelfTestEvent("selftest_cancel");
    } else if (now - st_hold_t0 >= SELFTEST_HOLD_MS) {
      startSelfTest(now);
    }
  }
}

// ---------------------------------------------------------------
// Arduino
// ---------------------------------------------------------------
void setup() {
  Serial.begin(HOST_BAUDRATE);
  loadDefaults();

  // POST ①: LED 스윕 (보드가 살아 있다는 표시)
  pinMode(LED_HEARTBEAT_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  const int sweep[4] = {LED_HEARTBEAT_PIN, LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN};
  for (uint8_t i = 0; i < 4; i++) ledWrite(sweep[i], false);
  for (uint8_t i = 0; i < 4; i++) { ledWrite(sweep[i], true); delay(80); ledWrite(sweep[i], false); }
  pinMode(BTN_SELFTEST_PIN, INPUT);
  pinMode(BTN_STOP_PIN, INPUT);
  if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, ESTOP_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  if (RC100_ESTOP_ENABLE) remote.begin(RC100_PORT);

  // IMU 사용 설정이 켜져 있으면 센서를 시작하고 자이로를 보정한다.
  // 보정이 끝날 때까지 로봇을 움직이지 않는다.
  if (IMU_ENABLE) imu.begin();

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  setupSyncInfos();

  // POST ③: 이 보드의 구동(켜져 있으면 조향도) 모터 ping·설정. 응답 없는 모터는 미검출로 표시.
  for (uint8_t k = 0; k < N_ACTIVE; k++) {
    motor_ok[k] = configureMotor(k);  // 토크 OFF 로 시작
    missing[k]  = !motor_ok[k];
  }
  rebuildSyncReadLists();

  runPost();  // ② 배터리 ④ IMU ⑤ HW 에러·온도 → {"post":...}, 통과하면 칩

  const uint32_t now = millis();
  last_cmd_ms = now;
  t_ctrl = t_fb = t_imu = t_err = t_batt = t_probe = now;
  sendConfig(false);
  sendIdentity();  // 부팅 직후 한 번. 호스트가 늦게 열면 'I' 로 다시 받는다.
}

void loop() {
  readRemote();  
  readHostSerial();
  if (IMU_ENABLE) imu.update();

  const uint32_t now = millis();

  // 워치독(데드맨): 유효한 C 가 끊기면 구동 0, 조향 유지
  if (!wd_tripped && now - last_cmd_ms > prm.watchdog_ms) {
    wd_tripped = true;
    Serial.print("{\"evt\":\"watchdog\",\"t\":");
    Serial.print(now);
    Serial.println('}');
  }

  // [추가] 자동 휴면 (Auto-Sleep): 토크가 켜져 있고, prm.idle_timeout_ms 동안 C 명령이 없으면 토크 자동 차단
  if (torque_on && !is_sleeping && (now - last_cmd_ms > prm.idle_timeout_ms)) {
    disableTorque();
    is_sleeping = true;
    Serial.print("{\"evt\":\"auto_sleep\",\"t\":");
    Serial.print(now);
    Serial.println('}');
  }

  // 링크 끊김 로직...
  if (linked && now - last_cmd_ms > prm.watchdog_ms) {
    linked   = false;
    wd_alarm = true;
    Serial.print("{\"evt\":\"link_lost\",\"t\":");
    Serial.print(now);
    Serial.println('}');
  }

  // ... (이하 기존 제어 주기 루프 동일)
  if (now - t_batt >= BATT_PERIOD_MS) {
    t_batt = now;
    sampleBattery();
  }
  // ...
}