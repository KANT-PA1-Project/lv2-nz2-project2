// =============================================================
// 보드   : OpenCR #2 (로봇팔)
// 파일   : config.h
// 역할   : 팔 모터 ID·관절 범위·전류 제한·통신 속도 상수
//
// [해야 할 일 / TODO]
//   - 값은 모두 플레이스홀더. TODO: 구글시트 기준으로 채우기
//   - 관절 범위는 nz2_control/config/arm_params.yaml과 같은 값으로
// =============================================================
#pragma once

#include <stdint.h>

constexpr uint32_t DXL_BAUDRATE = 1000000;        // TODO
constexpr float DXL_PROTOCOL_VERSION = 2.0f;
constexpr uint32_t HOST_BAUDRATE = 115200;         // TODO: 통신 방식 확정 후

// 관절 순서: joint1, joint2, joint3, joint4, 손목 롤, 그리퍼 (모두 XM430-W350)
constexpr uint8_t JOINT_COUNT = 6;
constexpr uint8_t JOINT_IDS[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};          // TODO

// 관절 범위 (다이나믹셀 위치 단위, TODO)
constexpr int32_t JOINT_MIN[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};
constexpr int32_t JOINT_MAX[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};

// 전류 제한 [mA] (TODO)
constexpr uint16_t MAX_CURRENT_MA[JOINT_COUNT] = {0, 0, 0, 0, 0, 0};

constexpr uint32_t COMMAND_TIMEOUT_MS = 500;       // TODO
constexpr int ESTOP_PIN = -1;                      // TODO: 배선 확정 후
