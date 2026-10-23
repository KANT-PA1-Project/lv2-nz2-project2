// =============================================================
// 보드   : OpenCR #1 (스월브)
// 파일   : config.h
// 역할   : 모터 ID·통신 속도·제한값·타임아웃 상수. 코드에 숫자를 직접 쓰지 않는다
//
// [해야 할 일 / TODO]
//   - 값은 모두 플레이스홀더. TODO: 구글시트 모터 ID/치수로 채우기
// [안전]  제한값은 실측 후 보수적으로. 0으로 두면 모터가 움직이지 않도록 구현한다
// =============================================================
#pragma once

#include <stdint.h>

// 다이나믹셀 버스
constexpr uint32_t DXL_BAUDRATE = 1000000;       // TODO: 모터 설정과 맞추기
constexpr float DXL_PROTOCOL_VERSION = 2.0f;

// Jetson ↔ OpenCR (USB)
constexpr uint32_t HOST_BAUDRATE = 115200;        // TODO: 통신 방식 확정 후

// 모터 ID: 모듈 순서 FL, FR, RL, RR (TODO: 구글시트)
constexpr uint8_t MODULE_COUNT = 4;
constexpr uint8_t STEER_IDS[MODULE_COUNT] = {0, 0, 0, 0};   // 조향 XM430
constexpr uint8_t DRIVE_IDS[MODULE_COUNT] = {0, 0, 0, 0};   // 구동 XM430

// 제한값 (TODO)
constexpr uint16_t MAX_CURRENT_MA = 0;            // 전류 제한
constexpr int32_t MAX_DRIVE_VELOCITY = 0;         // 구동 속도 제한 (다이나믹셀 단위)
constexpr float STEER_LIMIT_DEG = 90.0f;          // 조향 ±90°

// 상위 명령이 이 시간 동안 없으면 전 모터 정지 [ms]
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;      // TODO

// 비상정지 입력 핀 (TODO: 배선 확정 후)
constexpr int ESTOP_PIN = -1;
