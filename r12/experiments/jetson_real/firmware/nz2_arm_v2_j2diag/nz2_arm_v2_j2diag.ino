// =====================================================================
//  nz2_arm_v2.ino  -  NZ2 OpenCR#2: OpenMANIPULATOR-X + 손목 롤 (모터 6개: 관절 11~14, 롤 16, 그리퍼 15)
//  위치: firmware/nz2_arm_v2/nz2_arm_v2.ino  (같은 폴더의 arm_config.h 필요. 원본 firmware/nz2_arm/ 은 수정하지 않음)
//  라이브러리: Dynamixel2Arduino (0.8.2 에서 쓰던 API 만 사용). 빌드: OpenCR 보드, Arduino IDE.
//  ※ 호스트(g++) 시뮬레이션으로 상태기계는 시험했으나, OpenCR 실보드 업로드·실모터 검증은 아직 안 함.
//
//  ---- 원본(nz2_arm.ino) 대비 바뀐 점 (비상정지 관련 결함 수정) ----
//   1) 'S' 가 FAULT 를 덮어써서, S -> R 하면 모터 하드웨어 오류가 조용히 풀렸다.   -> 상태를 독립 래치 플래그로 분리
//   2) 정지 = "목표 = 현재 위치" 인데 현재 위치 읽기가 실패하면 이전 목표를 그대로 둬서 계속 움직였다.
//      -> 재시도 + (나이 100 ms 이하) 캐시 위치 사용 + 실패 모터는 20 ms 마다 재시도하고 S 회신에 fail 마스크로 알림
//   3) Profile Acceleration 20 으로는 82 deg/s 에서 멈추는 데 약 0.19 s(약 8 deg) 걸린다.
//      -> 비상정지 순간에만 가속도를 200 으로 올려 약 0.02 s 로 줄이고, R 에서 원래 값으로 복구
//   4) 모터 전원이 끊겼다 돌아와도(릴레이 차단) 펌웨어는 "정상"으로 믿었다 (읽기 실패를 무시).
//      -> 연속 통신 실패 3회면 FAULT(모터 오프라인). R 에서 모터를 다시 ping·재설정
//   5) sscanf("%f") 는 newlib-nano 에서 기본으로 float 를 지원하지 않아, 실보드에서 A 명령이 전부 무시될 수 있다.
//      -> strtof 기반 파서로 교체, NaN/Inf/남는 글자가 있는 줄은 버림
//   6) 'S' 문자는 줄 중간에 와도 즉시 비상정지 (개행 누락·줄 붙음에도 안전). 다른 명령에는 'S' 글자가 없다.
//   7) Q 폴링이 워치독을 살려 두던 문제: 이제 A/G/K 만 "명령 살아 있음"으로 친다.
//   8) 부팅 시 토크 OFF + 래치(BOOT_LATCH): 사람이 'R' 다음 'T 1' 을 보내야 움직인다 (OpenCR#1 과 같은 정책)
//   9) 모터 EEPROM 에 전류 제한·위치 제한(펌웨어 한계 ±2 deg)을 써서 2차 방어선 추가, 하드웨어 E-stop GPIO 입력(선택)
//
//  ---- 시리얼 프로토콜 (USB, 줄 단위 텍스트, Jetson arm_bridge -> OpenCR). 각도는 deg, 0 = 모터 raw 180 deg ----
//   ?                       장치 식별      -> "NZ2_ARM fw=2.0 limits_rev=1 ok=<정상 모터 수>/<사용 모터 수> fail=<ID 목록 또는 ->"
//   A j1 j2 j3 j4 roll grip 목표 각도      (회신 없음, 100 Hz)
//   G pos cur               그리퍼 목표 각도 + 전류(mA) (회신 없음)
//   K                       생존 신호 (움직임 없이 워치독만 갱신)
//   T 0|1                   토크 OFF/ON. ON 은 래치 해제 후에만, 목표를 현재 위치로 맞춘 뒤 켠다 -> "OK T 1" / "ERR T ..."
//   S                       비상 정지: 전 모터 목표 = 현재 위치 (토크 유지, 그리퍼는 쥔 채 유지) -> "OK S fail=0x.. ms=.."
//   R                       래치 해제(비상정지·FAULT·부팅 잠금). FAULT 면 모터를 다시 ping·설정한다. 토크는 안 켠다
//   Q                       상태 회신      -> "P <state> <각도 deg x6> <전류 mA x6>"  (캐시 값, 최대 약 60 ms 전)
//   E                       하드웨어 오류  -> "E <Hardware Error Status x6>"
//   D                       진단 회신      -> "D <state> <fault_code> <fault_motor> <hold_fail> <offline> <torque> <cmd_age_ms> <bad_lines> <limits_rev> <estop_ms>"
//  모터 순서(모든 배열·회신): 관절 11, 12, 13, 14, 손목 롤 16, 그리퍼 15
//  state: 0 정상 / 1 워치독 정지(명령 두절, A/G 가 오면 자동 복귀) / 2 비상정지 래치 / 3 FAULT 래치 / 4 부팅 잠금
//  fault_code: 0 없음 / 1 모터 하드웨어 오류 / 2 모터 통신 두절(오프라인) / 3 부팅 시 모터 설정 실패
// =====================================================================
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "arm_config.h"

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#define BIT(i) ((uint8_t)(1u << (i)))

enum ArmState : uint8_t { ST_OK = 0, ST_WD_HOLD = 1, ST_ESTOP = 2, ST_FAULT = 3, ST_LOCKED = 4 };
enum FaultCode : uint8_t { F_NONE = 0, F_MOTOR_HW = 1, F_MOTOR_COMM = 2, F_BOOT_MOTOR = 3 };

// ---- 전역 상태 -------------------------------------------------------
bool     motor_ok[N_MOTOR];            // ping + 설정 성공
uint8_t  hw_err[N_MOTOR];              // Hardware Error Status 마지막 값
int32_t  pos_cache[N_MOTOR];           // 마지막으로 읽은 현재 위치 [raw]
bool     pos_valid[N_MOTOR];
uint32_t pos_cache_ms[N_MOTOR];
float    cur_ma[N_MOTOR];              // 마지막으로 읽은 현재 전류 [mA]
uint8_t  comm_fail_cnt[N_MOTOR];       // 연속 통신 실패 횟수
uint8_t  comm_fail_mask = 0;           // 오프라인으로 판정된 모터 (폴링·쓰기에서 제외해 버스 블로킹 방지)

bool     estop_latched = false;        // 비상정지 래치 (S 또는 하드웨어 핀)
bool     fault_latched = false;        // FAULT 래치 (모터 하드웨어 오류·오프라인)
bool     locked        = false;        // 부팅 잠금 (R 필요)
bool     wd_hold       = false;        // 워치독 정지 (래치 아님)
bool     torque_on     = false;
bool     mt_reported   = false;        // 직전 T 1 이 멀티턴 가드로 거부돼 사유를 이미 출력했는지
FaultCode fault_code   = F_NONE;
uint8_t  fault_motor   = 0xFF;
uint8_t  hold_fail_mask = 0;           // 정지 쓰기가 아직 성공하지 못한 모터
uint32_t estop_ms      = 0;            // 마지막 비상정지 처리 시간 [ms]

bool     have_rx = false;              // 토크 ON 뒤 첫 명령을 받은 뒤에만 워치독 동작
uint32_t last_cmd_ms = 0;              // 마지막 A/G/K 수신 시각
uint32_t bad_lines = 0;
uint32_t last_poll_ms = 0, last_retry_ms = 0, pin_seen_ms = 0;
int      poll_idx = 0;

// ---- 작은 도구들 -----------------------------------------------------
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static bool  finiteF(float v) { return v == v && v > -1e9f && v < 1e9f; }
int32_t  zero_eff[N_MOTOR];            // 이번 세션의 영점 raw (확장 위치 모드 모터는 reinit 때 4096 의 정수배만큼 옮겨 현재 위치에 가장 가깝게 고른다)
static float rawToDeg(int i, int32_t raw) { return (raw - zero_eff[i]) * (360.0f / 4096.0f); }
static int32_t degToRawClamped(int i, float deg) {
  int32_t r = zero_eff[i] + (int32_t)(deg * (4096.0f / 360.0f));
  if (ARM_EXT_POSITION[i]) return r;                                         // 확장 위치: 0~4095 제한 없음 (한계는 J_MIN/J_MAX 클램프가 보장)
  return r < 0 ? 0 : (r > 4095 ? 4095 : r);
}
// 현재 raw 에 가장 가까운 영점(ARM_ZERO_RAW + 4096*k)을 고른다
static int32_t nearestZero(int32_t base, int32_t raw) {
  int32_t d = raw - base;
  int32_t k = (d >= 0) ? (d + 2048) / 4096 : -((-d + 2047) / 4096);
  return base + 4096 * k;
}

// "n1 n2 ..." 를 n 개의 float 로. 개수가 다르거나 NaN/Inf/찌꺼기 문자가 있으면 false
static bool parseFloats(const char* s, float* out, int n) {
  for (int k = 0; k < n; k++) {
    while (*s == ' ' || *s == '\t') s++;
    char* end = nullptr;
    float v = strtof(s, &end);
    if (end == s || !finiteF(v)) return false;
    out[k] = v;
    s = end;
  }
  while (*s == ' ' || *s == '\t') s++;
  return *s == 0;
}

bool isActive(int i) { return (ARM_ACTIVE_MASK & BIT(i)) != 0; }
bool allMotorsOk() {
  for (int i = 0; i < N_MOTOR; i++) if (isActive(i) && !motor_ok[i]) return false;
  return true;
}
bool motorUsable(int i) { return isActive(i) && motor_ok[i] && !(comm_fail_mask & BIT(i)); }

bool commandsBlocked() { return fault_latched || estop_latched || locked; }

ArmState currentState() {
  if (fault_latched) return ST_FAULT;
  if (estop_latched) return ST_ESTOP;
  if (locked)        return ST_LOCKED;
  if (wd_hold)       return ST_WD_HOLD;
  return ST_OK;
}

// ---- 모터 읽기/설정 --------------------------------------------------
bool readPresentRaw(int i, int32_t* raw) {
  float p = dxl.getPresentPosition(MOTOR_ID[i]);
  if (dxl.getLastLibErrCode() != 0) return false;
  *raw = (int32_t)p;
  pos_cache[i] = *raw; pos_valid[i] = true; pos_cache_ms[i] = millis();
  return true;
}

bool setupMotor(int i) {
  const uint8_t id = MOTOR_ID[i];
  bool ok = true;
  ok &= dxl.torqueOff(id);                                                   // EEPROM 은 토크 OFF 에서만 쓸 수 있다
#if ARM_CHECK_HOMING_OFFSET
  {                                                                          // Homing Offset 이 기대값과 다르면 영점이 틀린 것 -> 이 모터는 사용 금지
    int32_t ho = dxl.readControlTableItem(ControlTableItem::HOMING_OFFSET, id);
    if (dxl.getLastLibErrCode() != 0) { ok = false; }
    else if (ho < EXPECT_HOMING_OFFSET[i] - HOMING_OFFSET_TOL || ho > EXPECT_HOMING_OFFSET[i] + HOMING_OFFSET_TOL) {
      Serial.print("ERR HO id="); Serial.print((int)id); Serial.print(" read="); Serial.print((int)ho);
      Serial.print(" expect="); Serial.println((int)EXPECT_HOMING_OFFSET[i]);
      ok = false;
    }
  }
#endif
  ok &= dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, id, 0);
  ok &= dxl.setOperatingMode(id, i == GRIP_IDX ? OP_CURRENT_BASED_POSITION : (ARM_EXT_POSITION[i] ? OP_EXTENDED_POSITION : OP_POSITION));
  ok &= dxl.writeControlTableItem(ControlTableItem::CURRENT_LIMIT, id, (int32_t)(CURRENT_LIMIT_MA[i] / CURRENT_UNIT_MA));
#if ARM_WRITE_EEPROM_LIMITS
  if (i != GRIP_IDX && !ARM_EXT_POSITION[i]) {                               // 그리퍼(모드 5)·확장 위치(모드 4)는 멀티턴이라 EEPROM 위치 제한이 적용되지 않는다
    ok &= dxl.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id, degToRawClamped(i, J_MIN[i] - EEPROM_LIMIT_MARGIN_DEG));
    ok &= dxl.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id, degToRawClamped(i, J_MAX[i] + EEPROM_LIMIT_MARGIN_DEG));
  }
#endif
#if ARM_USE_BUS_WATCHDOG
  ok &= dxl.writeControlTableItem(ControlTableItem::BUS_WATCHDOG, id, BUS_WATCHDOG_UNITS);
#endif
  ok &= dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, PROFILE_VEL[i]);
  ok &= dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, PROFILE_ACC[i]);
  if (i == GRIP_IDX) ok &= dxl.setGoalCurrent(id, GRIP_CURRENT_DEFAULT_MA, UNIT_MILLI_AMPERE);
  return ok;
}

void restoreProfiles() {                                                    // 비상정지용 큰 가속도 -> 원래 값
  for (int i = 0; i < N_MOTOR; i++)
    if (motorUsable(i)) dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, MOTOR_ID[i], PROFILE_ACC[i]);
}

// ---- 정지 (목표 = 현재 위치) ------------------------------------------
// fast_stop: Profile Acceleration 을 ESTOP 값으로 올려 빨리 선다. 반환: 정지 쓰기에 실패한 모터 마스크
uint8_t holdAll(bool fast_stop, uint8_t only_mask) {
  uint8_t fail = 0;
  for (int i = 0; i < N_MOTOR; i++) {                                       // 순서: 큰 관절부터
    if (!(only_mask & BIT(i)) || !motorUsable(i)) continue;
    int32_t raw = 0;
    bool got = readPresentRaw(i, &raw);                                      // 1) 새로 읽기
    if (!got) {
      if (pos_valid[i] && (uint32_t)(millis() - pos_cache_ms[i]) <= POS_CACHE_MAX_AGE_MS) raw = pos_cache[i];  // 2) 캐시 (타임아웃 재시도로 시간 쓰지 않음)
      else if (!readPresentRaw(i, &raw)) { fail |= BIT(i); continue; }       // 3) 한 번 더 읽고, 안 되면 재시도 목록으로
    }
    if (fast_stop) dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, MOTOR_ID[i], ESTOP_PROFILE_ACC);
    bool w = false;
    for (int t = 0; t < 2 && !w; t++) w = dxl.setGoalPosition(MOTOR_ID[i], (float)raw);   // raw 단위
    if (!w) fail |= BIT(i);
  }
  return fail;
}

// 아직 살아 있다고 믿는 모터를 ping 한 번씩 훑어 오프라인을 즉시 확정 (전원 차단 시 모터마다 3번씩 타임아웃을 기다리지 않도록)
void sweepPing() {
  for (int i = 0; i < N_MOTOR; i++)
    if (motorUsable(i) && !dxl.ping(MOTOR_ID[i])) comm_fail_mask |= BIT(i);
}

void raiseFault(FaultCode code, uint8_t motor) {
  if (fault_latched) return;                                                 // 최초 원인 유지 (S 가 덮어쓰지 못함)
  fault_latched = true; fault_code = code; fault_motor = motor; wd_hold = false;
  if (code == F_MOTOR_COMM) sweepPing();                                     // 살아 있는 모터만 골라서
  hold_fail_mask |= holdAll(true, 0x3F);                                     // 그 자리에 세운다
}

// 읽기/쓰기 결과를 보고 연속 실패면 모터를 오프라인으로 판정
void noteComm(int i, bool comm_ok) {
  if (comm_ok) { comm_fail_cnt[i] = 0; return; }
  if (comm_fail_cnt[i] < 255) comm_fail_cnt[i]++;
  if (comm_fail_cnt[i] >= COMM_FAIL_LIMIT && !(comm_fail_mask & BIT(i))) {
    comm_fail_mask |= BIT(i);
    raiseFault(F_MOTOR_COMM, (uint8_t)i);
  }
}

void triggerEstop() {
  if (estop_latched) { Serial.println("OK S already"); return; }
  estop_latched = true; wd_hold = false;                                     // 래치를 먼저: 이후 어떤 명령도 통과 못 함
  uint32_t t0 = millis();
  hold_fail_mask |= holdAll(true, 0x3F);
  estop_ms = millis() - t0;
  Serial.print("OK S fail=0x"); Serial.print((unsigned)hold_fail_mask, HEX);
  Serial.print(" ms="); Serial.println((unsigned long)estop_ms);
}

void retryHold(uint32_t now) {                                               // 정지 쓰기에 실패한 모터를 계속 재시도
  if (!(estop_latched || fault_latched) || !hold_fail_mask) return;
  if ((uint32_t)(now - last_retry_ms) < (uint32_t)ESTOP_RETRY_MS) return;
  last_retry_ms = now;
  hold_fail_mask = holdAll(true, hold_fail_mask);
}

// ---- 토크 -----------------------------------------------------------
bool setTorque(bool on) {
  if (!on) {
    for (int i = 0; i < N_MOTOR; i++) if (motorUsable(i)) dxl.torqueOff(MOTOR_ID[i]);
    torque_on = false;
    return true;
  }
  if (!allMotorsOk() || comm_fail_mask) return false;
  restoreProfiles();
  int32_t raw[N_MOTOR] = {0};
  for (int i = 0; i < N_MOTOR; i++) if (isActive(i) && !readPresentRaw(i, &raw[i])) return false;   // 하나라도 못 읽으면 켜지 않음
  // 멀티턴 가드: 위치 모드 목표는 0~4095(한 바퀴)만 받는다. 현재 위치가 그 밖이면(손으로 한 바퀴 이상 돌린 뒤 모터 전원을 안 껐을 때)
  // 목표=현재 쓰기가 거부돼 옛 목표로 몇 바퀴를 되감아 케이블이 끊길 수 있다 -> 토크를 켜지 않는다. 해결: 모터 12 V 재투입(재부팅)
  uint8_t mt = 0, pz = 0;
  for (int i = 0; i < N_MOTOR; i++) {
    if (!isActive(i)) continue;
    if (ARM_EXT_POSITION[i]) {                                               // 확장 위치: 영점 기준 한계(+마진) 밖이면 영점 가지가 모호 -> 거부
      const float d = rawToDeg(i, raw[i]);
      if (d < J_MIN[i] - EXT_BOOT_MARGIN_DEG || d > J_MAX[i] + EXT_BOOT_MARGIN_DEG) pz |= BIT(i);
    } else if (i != GRIP_IDX && (raw[i] < 0 || raw[i] > 4095)) mt |= BIT(i);   // 그리퍼(모드 5)는 멀티턴이 정상
  }
  if (mt) { mt_reported = true; Serial.print("ERR T 1 multiturn mask=0x"); Serial.print((unsigned)mt, HEX); Serial.println(" (모터 12V 재투입 필요)"); return false; }
  if (pz) { mt_reported = true; Serial.print("ERR T 1 pose mask=0x"); Serial.print((unsigned)pz, HEX); Serial.println(" (팔을 한계 안 자세로 돌려놓고 R)"); return false; }
  for (int i = 0; i < N_MOTOR; i++) if (isActive(i)) dxl.setGoalPosition(MOTOR_ID[i], (float)raw[i]); // 켜기 전에 목표 = 현재 (튐 방지)
  bool all_on = true;
  for (int i = 0; i < N_MOTOR; i++) if (isActive(i)) all_on &= dxl.torqueOn(MOTOR_ID[i]);
  if (!all_on) { setTorque(false); return false; }
  torque_on = true; have_rx = false; wd_hold = false;
  return true;
}

// 모든 모터를 ping·재설정하고 하드웨어 오류를 확인. 반환: 문제 있는 모터 마스크 (0 이면 모두 정상). 토크는 OFF 로 끝난다
uint8_t reinitMotors() {
  uint8_t bad = 0;
  torque_on = false;
  for (int i = 0; i < N_MOTOR; i++) {
    comm_fail_cnt[i] = 0; pos_valid[i] = false; cur_ma[i] = 0;
    if (!isActive(i)) { motor_ok[i] = false; continue; }                     // 시험 마스크 밖: 없는 모터
    motor_ok[i] = dxl.ping(MOTOR_ID[i]) && setupMotor(i);
    if (!motor_ok[i]) { bad |= BIT(i); comm_fail_mask |= BIT(i); continue; }
    comm_fail_mask &= (uint8_t)~BIT(i);
    zero_eff[i] = ARM_ZERO_RAW[i];
    if (ARM_EXT_POSITION[i]) {                                               // 확장 위치 모터: 현재 raw 에 가장 가까운 영점 선택
      int32_t r0;
      if (!readPresentRaw(i, &r0)) { motor_ok[i] = false; bad |= BIT(i); comm_fail_mask |= BIT(i); continue; }
      zero_eff[i] = nearestZero(ARM_ZERO_RAW[i], r0);
    }
    int32_t e = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, MOTOR_ID[i]);
    if (dxl.getLastLibErrCode() != 0) { motor_ok[i] = false; bad |= BIT(i); comm_fail_mask |= BIT(i); continue; }
    hw_err[i] = (uint8_t)e;
    if (e != 0) bad |= BIT(i);                                               // 하드웨어 오류는 모터 전원 재투입(재부팅)으로만 풀린다
  }
  return bad;
}

// ---- 명령 -----------------------------------------------------------
// 현재 각도 [deg]. 캐시가 없거나 오래되면 valid=false
static float presentDeg(int i, bool* valid) {
  *valid = pos_valid[i] && (uint32_t)(millis() - pos_cache_ms[i]) < FOLD_POS_MAX_AGE_MS;
  return rawToDeg(i, pos_cache[i]);
}

// 접힌 자세 연동 한계: 어깨(1)·팔꿈치(2)가 접힘 영역이면 손목(3)을 FOLD_WRIST_MAX_DEG 이하로 제한하고,
// 손목이 아직 펴져 있으면 접힘 영역 진입(팔꿈치 목표)을 막는다. g[] 는 이미 개별 한계로 자른 목표 각도.
// 현재 위치를 모르면(읽기 실패·오래됨) 접힘 영역 안으로 간주한다(안전 쪽).
void applyFoldInterlock(float* g) {
#if ARM_FOLD_INTERLOCK
  if (!(motorUsable(1) && motorUsable(2) && motorUsable(3))) return;
  bool vs, ve, vw;
  const float ps = presentDeg(1, &vs), pe = presentDeg(2, &ve), pw = presentDeg(3, &vw);
  const bool in_now  = (!vs || !ve) ? true : (pe > FOLD_ELBOW_DEG && ps < FOLD_SHOULDER_DEG);
  const bool in_goal = (g[2] > FOLD_ELBOW_DEG && g[1] < FOLD_SHOULDER_DEG);
  if (!in_now && !in_goal) return;
  if (g[3] > FOLD_WRIST_MAX_DEG) g[3] = FOLD_WRIST_MAX_DEG;                   // 접힘 영역: 손목 최대 제한
  if (!in_now && in_goal && (!vw || pw > FOLD_WRIST_ENTRY_DEG))                // 손목이 아직 펴져 있으면 진입 보류
    g[2] = FOLD_ELBOW_DEG;
#else
  (void)g;
#endif
}

void moveArm(const float* a) {
  float g[N_MOTOR];
  for (int i = 0; i < N_MOTOR; i++) g[i] = clampf(a[i], J_MIN[i], J_MAX[i]);
  applyFoldInterlock(g);
  for (int i = 0; i < N_MOTOR; i++) {
    if (!motorUsable(i)) continue;
    dxl.setGoalPosition(MOTOR_ID[i], (float)degToRawClamped(i, g[i]));
    const bool comm_ok = (dxl.getLastLibErrCode() == 0);
    noteComm(i, comm_ok);
    if (!comm_ok || fault_latched) return;                                   // 통신 오류면 이번 프레임 중단. FAULT 가 걸렸으면 정지 목표를 덮어쓰지 않도록 더 쓰지 않음
  }
}

void moveGripper(float deg, float cur) {
  if (!motorUsable(GRIP_IDX)) return;
  float c = clampf(cur, 0.0f, CURRENT_LIMIT_MA[GRIP_IDX]);
  dxl.setGoalCurrent(MOTOR_ID[GRIP_IDX], c, UNIT_MILLI_AMPERE);
  float v = clampf(deg, J_MIN[GRIP_IDX], J_MAX[GRIP_IDX]);
  dxl.setGoalPosition(MOTOR_ID[GRIP_IDX], (float)degToRawClamped(GRIP_IDX, v));
  noteComm(GRIP_IDX, dxl.getLastLibErrCode() == 0);
}

bool estopPinAsserted() {
  if (ESTOP_PIN < 0) return false;
  return digitalRead(ESTOP_PIN) == ESTOP_ACTIVE_LEVEL;
}

void pollEstopPin(uint32_t now) {
  if (ESTOP_PIN < 0) return;
  if (estopPinAsserted()) {
    if (pin_seen_ms == 0) pin_seen_ms = now ? now : 1;
    if ((uint32_t)(now - pin_seen_ms) >= ESTOP_DEBOUNCE_MS && !estop_latched) triggerEstop();
  } else {
    pin_seen_ms = 0;
  }
}

void handleRelease() {
  if (estopPinAsserted()) { Serial.println("ERR R estop_pin_pressed"); return; }
  if (fault_latched) {
    uint8_t bad = reinitMotors();
    if (bad) {
      Serial.print("ERR R reinit fail=0x"); Serial.print((unsigned)bad, HEX);
      Serial.println(" (하드웨어 오류는 모터 전원 재투입 필요)");
      return;
    }
    fault_latched = false; fault_code = F_NONE; fault_motor = 0xFF;
  } else {
    restoreProfiles();
  }
  estop_latched = false; locked = false; wd_hold = false; hold_fail_mask = 0;
  Serial.print("OK R state="); Serial.println((int)currentState());
}

// ---- 회신 -----------------------------------------------------------
void replyIdentity() {
  int n = 0;
  int n_active = 0;
  for (int i = 0; i < N_MOTOR; i++) { if (motor_ok[i]) n++; if (isActive(i)) n_active++; }
  Serial.print("NZ2_ARM fw="); Serial.print(FW_VERSION);
  Serial.print(" limits_rev="); Serial.print((int)FW_LIMITS_REV);
  Serial.print(" ok="); Serial.print(n); Serial.print("/"); Serial.print(n_active); Serial.print(" fail=");
  if (n == n_active) Serial.print("-");
  for (int i = 0; i < N_MOTOR; i++) if (isActive(i) && !motor_ok[i]) { Serial.print((int)MOTOR_ID[i]); Serial.print(","); }
  Serial.println();
}

void replyStatus() {
  Serial.print("P "); Serial.print((int)currentState());
  for (int i = 0; i < N_MOTOR; i++) {
    Serial.print(" ");
    if (pos_valid[i]) Serial.print(rawToDeg(i, pos_cache[i]), 2); else Serial.print("nan");
  }
  for (int i = 0; i < N_MOTOR; i++) { Serial.print(" "); Serial.print(cur_ma[i], 0); }
  Serial.println();
}

void replyErrors() {
  Serial.print("E");
  for (int i = 0; i < N_MOTOR; i++) { Serial.print(" "); Serial.print((int)hw_err[i]); }
  Serial.println();
}

void replyDiag() {
  long age = have_rx ? (long)(millis() - last_cmd_ms) : -1;
  Serial.print("D "); Serial.print((int)currentState());
  Serial.print(" "); Serial.print((int)fault_code);
  Serial.print(" "); Serial.print((int)fault_motor);
  Serial.print(" "); Serial.print((int)hold_fail_mask);
  Serial.print(" "); Serial.print((int)comm_fail_mask);
  Serial.print(" "); Serial.print(torque_on ? 1 : 0);
  Serial.print(" "); Serial.print(age);
  Serial.print(" "); Serial.print((unsigned long)bad_lines);
  Serial.print(" "); Serial.print((int)FW_LIMITS_REV);
  Serial.print(" "); Serial.println((unsigned long)estop_ms);
}

// I는 역할 식별 응답만 출력한다. 모터/제어 상태/설정 접근 없음.
void replyHandshake() {
  Serial.print("{\"hello\":1,\"proto\":1,\"board\":\"arm\",\"kind\":\"arm\",\"fw\":\"nz2_arm v2 handshake1\",\"fw_version\":\"");
  Serial.print(FW_VERSION);
  Serial.print("\",\"limits_rev\":"); Serial.print((int)FW_LIMITS_REV);
  Serial.print(",\"diag_rev\":2");
  Serial.println(",\"order\":[\"j1\",\"j2\",\"j3\",\"j4\",\"roll\",\"grip\"]}");
}

// Read-only shoulder diagnostics. Each failed register read remains explicit.
void replyJ2Item(const char* name, uint8_t item, uint16_t timeout_ms) {
  const int32_t value = dxl.readControlTableItem(item, MOTOR_ID[1], timeout_ms);
  const int32_t lib_error = dxl.getLastLibErrCode();
  const uint8_t packet_error = dxl.getLastStatusPacketError();
  const bool ok = lib_error == 0 && packet_error == 0;
  Serial.print("{\"j2_diag\":1,\"id\":12,\"item\":\""); Serial.print(name);
  Serial.print("\",\"ok\":"); Serial.print(ok ? "true" : "false");
  Serial.print(",\"raw\":");
  if (ok) Serial.print(value); else Serial.print("null");
  Serial.print(",\"lib_err\":"); Serial.print(lib_error);
  Serial.print(",\"packet_err\":"); Serial.print(packet_error);
  Serial.println("}");
}

void replyJ2Load() {
  // Four reads, each bounded to 10 ms. The host must maintain its normal watchdog heartbeat.
  Serial.println("{\"j2_diag\":1,\"event\":\"begin\",\"kind\":\"load\"}");
  replyJ2Item("input_voltage", ControlTableItem::PRESENT_INPUT_VOLTAGE, 10);
  replyJ2Item("present_current", ControlTableItem::PRESENT_CURRENT, 10);
  replyJ2Item("present_pwm", ControlTableItem::PRESENT_PWM, 10);
  replyJ2Item("temperature", ControlTableItem::PRESENT_TEMPERATURE, 10);
  Serial.println("{\"j2_diag\":1,\"event\":\"end\"}");
}

void replyJ2Settings() {
  // A full configuration read can take longer than the watchdog period on a faulty bus.
  if (torque_on) { Serial.println("ERR V torque_on; use W for load telemetry"); return; }
  Serial.println("{\"j2_diag\":1,\"event\":\"begin\",\"kind\":\"settings\"}");
  replyJ2Item("model_number", ControlTableItem::MODEL_NUMBER, 50);
  replyJ2Item("operating_mode", ControlTableItem::OPERATING_MODE, 50);
  replyJ2Item("current_limit", ControlTableItem::CURRENT_LIMIT, 50);
  replyJ2Item("pwm_limit", ControlTableItem::PWM_LIMIT, 50);
  replyJ2Item("goal_pwm", ControlTableItem::GOAL_PWM, 50);
  replyJ2Item("position_p_gain", ControlTableItem::POSITION_P_GAIN, 50);
  replyJ2Item("position_i_gain", ControlTableItem::POSITION_I_GAIN, 50);
  replyJ2Item("position_d_gain", ControlTableItem::POSITION_D_GAIN, 50);
  replyJ2Item("profile_velocity", ControlTableItem::PROFILE_VELOCITY, 50);
  replyJ2Item("profile_acceleration", ControlTableItem::PROFILE_ACCELERATION, 50);
  replyJ2Item("torque_enable", ControlTableItem::TORQUE_ENABLE, 50);
  replyJ2Item("input_voltage", ControlTableItem::PRESENT_INPUT_VOLTAGE, 50);
  replyJ2Item("present_current", ControlTableItem::PRESENT_CURRENT, 50);
  replyJ2Item("present_pwm", ControlTableItem::PRESENT_PWM, 50);
  replyJ2Item("temperature", ControlTableItem::PRESENT_TEMPERATURE, 50);
  replyJ2Item("hardware_error", ControlTableItem::HARDWARE_ERROR_STATUS, 50);
  Serial.println("{\"j2_diag\":1,\"event\":\"end\"}");
}

// Temporary RAM-only j2 P tuning. All writes require both host and motor torque OFF.
void tuneJ2P(const char* args) {
  float value;
  if (!parseFloats(args, &value, 1) || !isfinite(value)
      || value < 800 || value > 1200 || value != (int32_t)value) {
    Serial.println("ERR J range=800..1200 integer"); return;
  }
  if (torque_on || !motorUsable(1)) {
    Serial.println("ERR J torque_or_motor"); return;
  }
  const uint8_t id = MOTOR_ID[1];
  int32_t enabled = dxl.readControlTableItem((uint8_t)ControlTableItem::TORQUE_ENABLE, id, (uint32_t)50);
  if (dxl.getLastLibErrCode() || dxl.getLastStatusPacketError() || enabled != 0) {
    Serial.println("ERR J actual_torque"); return;
  }
  int32_t previous = dxl.readControlTableItem((uint8_t)ControlTableItem::POSITION_P_GAIN, id, (uint32_t)50);
  if (dxl.getLastLibErrCode() || dxl.getLastStatusPacketError()) {
    Serial.println("ERR J read_previous"); return;
  }
  bool written = dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, (int32_t)value);
  int32_t actual = dxl.readControlTableItem((uint8_t)ControlTableItem::POSITION_P_GAIN, id, (uint32_t)50);
  if (!written || dxl.getLastLibErrCode() || dxl.getLastStatusPacketError()
      || actual != (int32_t)value) {
    dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, previous);
    int32_t restored = dxl.readControlTableItem((uint8_t)ControlTableItem::POSITION_P_GAIN, id, (uint32_t)50);
    bool rollback_ok = !dxl.getLastLibErrCode() && !dxl.getLastStatusPacketError()
        && restored == previous;
    Serial.print("ERR J rollback_ok="); Serial.println(rollback_ok ? 1 : 0); return;
  }
  Serial.print("OK J previous="); Serial.print(previous);
  Serial.print(" actual="); Serial.println(actual);
}

void handleLine(const char* line) {
  const char c = line[0];
  if (c == 0) return;
  const char* args = line + 1;
  float a[6];

  switch (c) {
    case 'I': replyHandshake(); return;
    case 'V': replyJ2Settings(); return;
    case 'W': replyJ2Load(); return;
    case 'J': tuneJ2P(args); return;
    case '?': replyIdentity(); return;
    case 'Q': replyStatus(); return;
    case 'E': replyErrors(); return;
    case 'D': replyDiag(); return;
    case 'S': triggerEstop(); return;                                        // 평소엔 readSerial 이 먼저 처리한다 (안전망)
    case 'R': handleRelease(); return;
    case 'K':
      last_cmd_ms = millis(); have_rx = true;
      return;
    case 'T': {
      float v;
      if (!parseFloats(args, &v, 1)) { bad_lines++; return; }
      if (v >= 0.5f) {
        if (commandsBlocked()) { Serial.println("ERR T 1 blocked(R 필요)"); return; }
        mt_reported = false;
        if (setTorque(true)) Serial.println("OK T 1"); else if (!mt_reported) Serial.println("ERR T 1 motor");
      } else {
        setTorque(false);                                                    // 토크 OFF 는 언제나 허용 (정비용)
        Serial.println("OK T 0");
      }
      return;
    }
    case 'A':
      if (!parseFloats(args, a, 6)) { bad_lines++; return; }
      if (commandsBlocked() || !torque_on) return;                           // 조용히 버림 (100 Hz 라 회신하면 회선이 막힌다)
      last_cmd_ms = millis(); have_rx = true; wd_hold = false;
      moveArm(a);
      return;
    case 'G':
      if (!parseFloats(args, a, 2)) { bad_lines++; return; }
      if (commandsBlocked() || !torque_on) return;
      last_cmd_ms = millis(); have_rx = true; wd_hold = false;
      moveGripper(a[0], a[1]);
      return;
    default:
      bad_lines++;
      return;
  }
}

// ---- 감시 -----------------------------------------------------------
void checkWatchdog(uint32_t now) {
  if (!have_rx || !torque_on || wd_hold || commandsBlocked()) return;
  if ((uint32_t)(now - last_cmd_ms) >= WATCHDOG_MS) {
    hold_fail_mask |= holdAll(false, 0x3F);                                  // 통상 가속도로 그 자리 정지 (토크 유지, 래치 아님)
    wd_hold = true;
  }
}

// 모터 1개씩 순환: 하드웨어 오류 + 현재 위치 + 현재 전류. 오프라인 모터는 건너뛴다 (타임아웃으로 루프가 멎는 것 방지)
void pollOne(uint32_t now) {
  if ((uint32_t)(now - last_poll_ms) < POLL_MS) return;
  last_poll_ms = now;
  const int i = poll_idx;
  poll_idx = (poll_idx + 1) % N_MOTOR;
  if (!motorUsable(i)) return;

  int32_t e = dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, MOTOR_ID[i]);
  const bool ok = (dxl.getLastLibErrCode() == 0);
  noteComm(i, ok);
  if (!ok) return;
  hw_err[i] = (uint8_t)e;
  int32_t raw;
  readPresentRaw(i, &raw);
  float c = dxl.getPresentCurrent(MOTOR_ID[i], UNIT_MILLI_AMPERE);
  if (dxl.getLastLibErrCode() == 0) cur_ma[i] = c;
  if (e != 0) raiseFault(F_MOTOR_HW, (uint8_t)i);
}

// ---- 시리얼 수신 -----------------------------------------------------
void readSerial() {
  static char buf[LINE_MAX_LEN];
  static int n = 0;
  static bool discard = false;                                               // 줄이 너무 길거나 'S' 를 만난 줄은 개행까지 버림
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'S') { triggerEstop(); n = 0; discard = true; continue; }      // 개행을 기다리지 않는다
    if (ch == '\n') {
      if (!discard && n > 0) { buf[n] = 0; handleLine(buf); }
      n = 0; discard = false;
    } else if (ch != '\r') {
      if (n < LINE_MAX_LEN - 1) buf[n++] = ch; else discard = true;
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  for (int i = 0; i < N_MOTOR; i++) zero_eff[i] = ARM_ZERO_RAW[i];

  // DXL 전원 (OpenCR 1.0). 라이브러리가 켜 주지만 버전과 무관하게 확실히 켠다
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(500);                                                                // 모터 전원 안정 대기

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
  if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);

  for (int i = 0; i < N_MOTOR; i++) { motor_ok[i] = false; hw_err[i] = 0; pos_valid[i] = false; cur_ma[i] = 0; comm_fail_cnt[i] = 0; }

  const uint8_t bad = reinitMotors();
  if (bad) {
    fault_latched = true; fault_code = F_BOOT_MOTOR; fault_motor = 0xFF;     // 모터가 빠지면 토크를 켜지 않는다
    for (int i = 0; i < N_MOTOR; i++) if (bad & BIT(i)) { Serial.print("ERR boot id="); Serial.println((int)MOTOR_ID[i]); }
  } else if (BOOT_LATCH) {
    locked = true;                                                           // 'R' -> 'T 1' 을 기다린다
  } else {
    setTorque(true);
  }
}

void loop() {
  readSerial();
  const uint32_t now = millis();
  pollEstopPin(now);
  checkWatchdog(now);
  retryHold(now);
  pollOne(now);
}
