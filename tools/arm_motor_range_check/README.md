# arm_motor_range_check — 로봇팔 가동범위 측정 도구

OpenMANIPULATOR-X 개조 팔(XM430-W350 ×6)의 관절별 가동범위를 **손으로 돌려서** 측정하고,
URDF/펌웨어 한계값으로 쓸 최소·최대 각도를 CSV로 남기는 개발용 도구다. 대회 로봇 코드와는 별개다.
(이슈 #17)

## 구성

```
tools/arm_motor_range_check/
├── README.md
├── requirements.txt
├── firmware/arm_motor_range_check/README.md   # 전용 .ino 없음 — usb_to_dxl 사용 안내
└── host/
    └── monitor.py                             # 웹 대시보드 + 측정·CSV 기록
```

## 이슈 #17 대비 변경점

| 이슈 요구 | 실제 | 이유 |
|---|---|---|
| `firmware/…/*.ino`, `config.h` | 없음, ROBOTIS `usb_to_dxl` 사용 | PC가 dynamixel_sdk로 직접 접근하므로 자체 펌웨어 불필요 |
| `host/monitor.py` | 동일 이름, 웹 대시보드(`http://127.0.0.1:8765`) 형태 | 실시간 그래프·마커 기록 |

## 사용 순서

실행 위치 표기: **[개발 PC]**

1. **[개발 PC]** OpenCR#2에 `usb_to_dxl` 업로드 → `firmware/arm_motor_range_check/README.md`
2. **[개발 PC]** 한 번만 환경 준비
   ```bash
   python3 -m venv ~/dxlenv
   ~/dxlenv/bin/pip install -r tools/arm_motor_range_check/requirements.txt
   ```
3. 팔 12 V 전원 ON, OpenCR#2를 USB로 연결 (다이나믹셀 위자드는 종료)
4. **[개발 PC]** 실행
   ```bash
   ~/dxlenv/bin/python tools/arm_motor_range_check/host/monitor.py --port /dev/ttyACM0
   ~/dxlenv/bin/python tools/arm_motor_range_check/host/monitor.py --mock       # 모터 없이 화면 시험
   ~/dxlenv/bin/python tools/arm_motor_range_check/host/monitor.py --analyze host/range_logs/xxx.csv
   ```
5. 브라우저에서 `http://127.0.0.1:8765` → 토크 OFF → 관절을 손으로 끝까지 돌림 → 기록 종료
6. `host/range_logs/arm_range_*_summary.csv`에서 모터별 최소·최대·제안 한계(안쪽 여유 `--margin`, 기본 5°) 확인

## 안전

- 이 도구가 모터에 쓰는 것은 **토크 OFF** 하나뿐이다. 위치·EEPROM은 쓰지 않는다.
- **토크를 끄면 팔이 중력으로 떨어진다.** 한 손으로 팔을 받친 상태에서 누를 것.
- 측정 후 제어 펌웨어(`firmware/opencr2_arm`)를 반드시 다시 올릴 것.

## 각도 기준

위치 모드 raw 2048 = 0° (`raw % 4096`을 −180~180°로 접음, 경계 넘나듦은 연속 각도로 처리).
제어 펌웨어의 관절별 영점(`ARM_ZERO_RAW`)과는 다를 수 있으므로, 측정값은 **범위 폭** 판단용으로 쓰고
펌웨어 한계로 옮길 때는 영점 차이를 보정한다. [확인 필요]

## 출력

`host/range_logs/` (git 제외): `arm_range_YYYYmmdd_HHMMSS.csv`(원본 샘플+마커), `*_summary.csv`(요약)
