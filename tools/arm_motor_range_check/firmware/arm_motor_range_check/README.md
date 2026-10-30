# firmware/arm_motor_range_check — 전용 펌웨어 없음 (ROBOTIS `usb_to_dxl` 사용)

이슈 #17은 OpenCR용 `.ino`(`arm_motor_range_check.ino`, `config.h`)를 요구했지만,
이 도구는 **자체 펌웨어를 두지 않고** ROBOTIS가 제공하는 `usb_to_dxl` 예제를 사용한다.

- 이유: 호스트 프로그램(`../../host/monitor.py`)이 `dynamixel_sdk`로 모터를 직접 읽으므로,
  OpenCR는 USB↔다이나믹셀 패스스루 역할만 하면 된다. 자체 `.ino`는 중복이다.
- 모터 ID·통신 속도·범위 상수는 `host/monitor.py` 상단 상수 블록에서 바꾼다.

## OpenCR#2에 usb_to_dxl 올리기

1. Arduino IDE → 보드 `OpenCR` 선택 (보드 매니저에서 OpenCR 설치 필요)
2. 파일 → 예제 → OpenCR → **Etc → usb_to_dxl** 열기
3. 포트(`/dev/ttyACM0` 등) 선택 후 업로드

CLI 사용 시 fqbn은 `ROBOTIS:OpenCR:OpenCR` (프로젝트 펌웨어와 동일).

## 주의 (실격·고장 방지)

- `usb_to_dxl`을 올리면 **`firmware/opencr2_arm`(제어 펌웨어)이 지워진다.** 측정 후 반드시 제어 펌웨어를 다시 업로드할 것.
- 제어 펌웨어(워치독·E-stop 포함)가 올라간 상태에서는 이 도구를 같은 포트에 쓸 수 없다. 또는 U2D2를 별도로 연결해 사용한다.
- 다이나믹셀 위자드와 동시 실행 불가 (포트 점유). 위자드는 완전히 종료.
