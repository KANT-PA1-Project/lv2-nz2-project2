# opencr1_swerve

OpenCR #1 스월브 (조향 4 + 구동 4 + IMU) 펌웨어 스켈레톤이다. 상수는 `config.h`, 동작은 `opencr1_swerve.ino`의 TODO 주석을 따른다.

## 빌드·업로드

보드는 OpenCR 1.0, fqbn `ROBOTIS:OpenCR:OpenCR`. 필요한 도구·버전은 저장소 루트 README의 "환경과 버전" 표를 따른다.

```bash
arduino-cli compile --fqbn ROBOTIS:OpenCR:OpenCR firmware/opencr1_swerve
arduino-cli upload  --fqbn ROBOTIS:OpenCR:OpenCR -p /dev/ttyACM0 firmware/opencr1_swerve
```

포트(`/dev/ttyACM0`)는 연결 순서에 따라 바뀐다. 업로드 전에 `arduino-cli board list`로 확인한다.

## 배선 요약

- Jetson ↔ OpenCR: USB (통신 방식 `[확인 필요]`: micro-ROS / 커스텀 시리얼)
- OpenCR ↔ 다이나믹셀: TTL 버스, 12 V
- 비상정지 입력 핀: `config.h`의 `ESTOP_PIN` (배선 확정 후)
