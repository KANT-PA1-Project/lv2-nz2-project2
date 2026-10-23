// =============================================================
// 보드   : OpenCR #1 (스월브)
// 파일   : opencr1_swerve.ino
// 역할   : 스월브 8모터(조향 4 + 구동 4) + IMU 제어
//
// [입력]  Jetson(opencr_bridge_node)의 명령: 4모듈 조향각·구동속도 (또는 차체 속도)
// [출력]  Jetson으로 상태 송신: 모터 위치·속도, IMU, 에러 플래그
// [해야 할 일 / TODO]
//   - setup(): Dynamixel2Arduino 초기화, 모터 핑, 운전 모드 설정, 토크 ON
//   - loop(): 명령 수신 → 제한 적용 → 모터 반영 → 상태·IMU 송신
//   - [미정] 스월브 역기구학을 여기서 할지 Jetson에서 할지 (Jetson이면 모듈 명령을 그대로 반영)
//   - [확인 필요] Jetson 통신 방식: micro-ROS / 커스텀 시리얼 프로토콜 — 결정 후 구현
// [안전]  - 전류·속도·위치(조향 ±90°) 제한
//         - 비상정지(E-stop) 입력 시 전 모터 정지
//         - 상위 명령이 COMMAND_TIMEOUT_MS 동안 없으면 전 모터 정지
//         - 하드웨어 에러(과전류·과열·전압) 확인, 에러 시 토크 OFF
// =============================================================
#include <Dynamixel2Arduino.h>

#include "config.h"

// OpenCR의 다이나믹셀 포트
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

uint32_t last_command_ms = 0;

void stopAllMotors() {
  // TODO(제어): 구동 모터 속도 0, 조향은 현재 위치 유지
}

void setup() {
  Serial.begin(HOST_BAUDRATE);   // Jetson 연결 (USB)
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // TODO(제어): STEER_IDS·DRIVE_IDS 모터 핑 확인
  // TODO(제어): 조향 = 위치 모드, 구동 = 속도 모드 설정
  // TODO(제어): 전류·속도 제한 레지스터 설정 후 토크 ON
  // TODO(제어): IMU 초기화 (OpenCR 내장 IMU)
  // TODO(제어): ESTOP_PIN 입력 설정
  last_command_ms = millis();
}

void loop() {
  // TODO(제어): Jetson 명령 수신 → 수신하면 last_command_ms 갱신
  // TODO(제어): 제한 적용 후 모터에 반영

  // [안전] 통신 타임아웃 → 전 모터 정지
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stopAllMotors();
  }

  // TODO(제어): 비상정지 입력 확인 → 정지
  // TODO(제어): 하드웨어 에러 상태 읽기 → 에러면 토크 OFF
  // TODO(제어): 모터 상태·IMU 송신
}
