# r12/firmware

스월브 OpenCR 펌웨어 두 사본을 둔다.

| 스케치 | 보드 | 모듈 |
| --- | --- | --- |
| `opencr_swerve_left` | 스월브 왼쪽 | FL·RL (구동 + 조향 + IMU) |
| `opencr_swerve_right` | 스월브 오른쪽 | FR·RR (구동 + 조향 + IMU) |

두 스케치는 **별도 사본**이다. 한쪽을 고치면 반드시 다른 쪽도 같이 고친다.
보드 이름·모듈 코드·모터 ID 말고는 본문이 같아야 한다.

## 빌드

```bash
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR r12/firmware/opencr_swerve_left
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR r12/firmware/opencr_swerve_right
```

업로드·플래싱은 사람이 확인한 뒤 바퀴를 띄운 벤치에서 한다.

## 문서

- [`../docs/OPENCR_FAULT_RECOVERY.md`](../docs/OPENCR_FAULT_RECOVERY.md) — 과열·저전압·과전류·
  통신 두절·HW 에러·미검출 모터·IMU 무응답·비상정지의 감지, 즉시 동작, 자동복구 조건
- [`../docs/OPENCR_HANDSHAKE.md`](../docs/OPENCR_HANDSHAKE.md) — 보드 식별 핸드셰이크,
  시리얼 프로토콜, 브리지 안전 규칙
