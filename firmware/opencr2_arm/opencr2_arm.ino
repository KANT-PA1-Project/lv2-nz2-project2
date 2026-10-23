// =============================================================
// 보드   : OpenCR #2 (로봇팔)
// 파일   : opencr2_arm.ino
// 역할   : 로봇팔 관절(XM430-W350 + 손목 롤) + 그리퍼 제어
//
// [입력]  Jetson(opencr_bridge_node)의 관절 목표 위치·그리퍼 명령
// [출력]  Jetson으로 상태 송신: 관절 위치·전류, 에러 플래그
// [해야 할 일 / TODO]
//   - setup(): Dynamixel2Arduino 초기화, 모터 핑, 위치(또는 전류 기반 위치) 모드, 토크 ON
//   - loop(): 명령 수신 → 범위·전류 제한 → 모터 반영 → 상태 송신
//   - 그리퍼는 전류 제한으로 퍽을 너무 세게 쥐지 않게
//   - [확인 필요] Jetson 통신 방식: micro-ROS / 커스텀 시리얼 프로토콜 — 결정 후 구현
// [안전]  - 관절 범위·전류 제한
//         - 비상정지(E-stop) 입력 시 정지
//         - 상위 명령이 COMMAND_TIMEOUT_MS 동안 없으면 현재 자세 유지(정지)
//         - 하드웨어 에러 확인, 에러 시 토크 OFF (팔이 떨어지지 않게 자세 주의)
// =============================================================
#include <Dynamixel2Arduino.h>

#include "config.h"

#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

uint32_t last_command_ms = 0;

void holdPosition() {
  // TODO(제어): 모든 관절 목표를 현재 위치로
}

void setup() {
  Serial.begin(HOST_BAUDRATE);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // TODO(제어): JOINT_IDS 모터 핑 확인
  // TODO(제어): 운전 모드 설정, 범위·전류 제한 레지스터 설정 후 토크 ON
  // TODO(제어): ESTOP_PIN 입력 설정
  last_command_ms = millis();
}

void loop() {
  // TODO(제어): Jetson 명령 수신 → 수신하면 last_command_ms 갱신
  // TODO(제어): 범위·전류 제한 적용 후 모터에 반영

  // [안전] 통신 타임아웃 → 현재 자세 유지
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    holdPosition();
  }

  // TODO(제어): 비상정지 입력 확인 → 정지
  // TODO(제어): 하드웨어 에러 상태 읽기 → 에러면 토크 OFF
  // TODO(제어): 관절 상태 송신
}
