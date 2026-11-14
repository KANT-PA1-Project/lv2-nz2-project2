# OpenCR 스월브 고장·자동복구

스월브 좌·우 보드(`r12/firmware/opencr_swerve_left`, `opencr_swerve_right`)가
고장을 어떻게 감지하고, 무엇을 즉시 멈추며, 언제 스스로 푸는지를 정리한다.
두 스케치는 같은 동작을 하며 보드 이름·모터 ID·모듈 순서만 다르다.

원칙:

- **큰 문제만 사람이 확인한다.** 비상정지와 자가진단 불합격은 래치로 남겨 `E` 를 받아야 풀린다.
- **나머지는 원인이 사라지면 스스로 푼다.** 과열·저전압·과전류·통신 두절은 해제 조건이
  `AUTO_CLEAR_HOLD_MS`(2 s) 동안 이어지면 자동으로 풀고, 하드웨어 에러는 모터를 재부팅해 복구한다.
- **고장 나도 조향은 붙잡는다.** 고장 시 구동은 0(능동 제동)이고 토크는 유지해 조향축이
  마지막 각도를 잡고 있게 한다. 과열·과전류가 난 모터만 예외로 토크를 끈다.
- **한 보드가 서면 차체가 돈다.** 반대쪽 보드는 이 보드의 `flt`/`wd` 를 모른다. 브리지가
  `stop_on_fault` 로 두 보드를 함께 세운다(`OPENCR_HANDSHAKE.md`).

수치의 기본값은 모두 `config.h` 의 컴파일 상수다. `P` 명령으로 바꿀 수 있는 것은 아래
[임계값과 P 키](#임계값과-p-키)에 표시했다.

## 한눈에 보기

| 현상 | 비트/필드 | 즉시 동작 | 자동복구 | 남는 제한 |
| --- | --- | --- | --- | --- |
| 과열 | `flt` 2 `overtemp` | 구동 0, 해당 모터 토크 OFF, 빨강+연속음 | 60 °C 이하 2 s → 해제, 토크 복귀 | 다시 70 °C 를 넘으면 재발 |
| 저전압 | `flt` 4 `undervolt` | 구동 0 | 10.5 V 위로 2 s → 해제 | 배터리를 충전해야 |
| 과전류 | `flt` 8 `overcurrent` | 해당 모터 토크 OFF | 전류가 한계 아래로 2 s → 해제, 토크 복귀 | 기구 걸림이 남으면 재발 |
| 모터 통신 두절 | `flt` 16 `motor_lost` | 구동 0 | 500 ms 마다 ping → 재설정 성공 후 2 s → 해제 | 배선·ID·전원이 고장이면 계속 |
| 하드웨어 에러 | `flt` 1 `hw_error` | 해당 모터만 재부팅 | 성공하면 즉시 해제 | 3회 연속 실패 후 60 s 간격 재시도 |
| 미검출 모터 | `flt` 16, `miss` | 그 모터 제외, 구동 0 | 다시 보이면 재설정 후 토크 복귀 | 배선·ID 확인 |
| IMU 무응답 | `wrn` 16 `WRN_IMU` | **주행을 막지 않음** | 정지 중 2 s 마다 재초기화 | 센서 자체 고장이면 계속 |
| 비상정지 | `flt` 32 `estop` | 구동 0, 토크 유지 | **없음(래치)** | 스위치를 풀고 `R`(E) |
| 자가진단 불합격 | `flt` 64 `selftest` | 구동 0, 토크 유지 | **없음(래치)** | `E`(`R`) 필요 |
| 워치독 | `wd` | 구동 0, 조향 유지 | `C` 가 오면 즉시 복귀 | 래치 아님 |
| 링크 끊김 | `lnk`, `evt:link_lost` | 브리지 프레임 끊김 표시 | `C` 가 오면 복귀 | 래치 아님 |
| 자동 휴면 | `evt:auto_sleep` | 토크 OFF(전력 절약) | `C` 또는 `E` 로 복귀 | 5 s 무명령이면 다시 |

`flt` 는 비트마스크다. `en`(`motionAllowed()`)은 토크 ON + `motion_en` + `wd_tripped=0` +
**`flt == 0` + 모든 모터가 `ok`** 일 때만 1 이다. 고장이 하나라도 있으면 `en=0` 이지만,
토크는 비상정지·자가진단 포함해 계속 켜져 있다.

## 고장 비트

| 값 | `faultName` | 발생 조건 | 즉시 동작 | 자동 해제 |
| --- | --- | --- | --- | --- |
| 1 | `hw_error` | 상태 패킷 에러 필드 `0x80`, 또는 Hardware Error Status ≠ 0 | 그 모터만 재부팅·재설정 | 자동복구 성공 시 |
| 2 | `overtemp` | 모터 온도 ≥ `temp_abort_c`(70 °C) | 구동 0, 그 모터 토크 OFF | 60 °C 이하 2 s |
| 4 | `undervolt` | 입력 전압 ≤ `batt_stop_v`(10.0 V) 가 3샘플(0.3 s) 연속 | 구동 0 | 평활 전압 > 10.5 V 가 2 s |
| 8 | `overcurrent` | 전류 > `current_limit_a`(1.5 A) 가 `oc_ms`(300 ms) 연속 | 그 모터 토크 OFF | 전류가 한계 이하로 2 s |
| 16 | `motor_lost` | 동기 읽기가 `COMM_FAIL_MAX`(5)회 연속 누락 | 구동 0 | 미검출 모터가 없고 2 s |
| 32 | `estop` | RC-100 버튼 또는 물리 비상정지 핀 | 구동 0, 토크 유지 | **`E` 만** |
| 64 | `selftest` | 자가진단 거리 미달·타임아웃·조향 정렬 실패 | 구동 0, 토크 유지 | **`E` 만** |

`WRN_*` 는 고장이 아닌 경고다. `wrn` 비트 1=온도, 2=배터리, 4=derate, 8=초기화 안 된 모터,
16=IMU 무응답. 경고만으로는 `en` 이 막히지 않는다.

## 래치 고장 자동 해제

제어 주기(10 ms)마다 `readFeedback()` → `updateSafety()` → `recoveryTick()` →
`autoClearFaults()` 순서로 돈다. `autoClearFaults()` 는 아래 조건이 **2 s 연속** 만족될 때만
비트를 지운다. 조건이 한 번이라도 깨지면 타이머는 처음부터 다시 센다.

| 비트 | 해제 조건 | 히스테리시스 |
| --- | --- | --- |
| `overtemp` | 응답이 확인된 모든 모터 온도 ≤ `temp_derate_c - 5 °C` (60 °C) | 10 °C (abort 70 → clear 60) |
| `undervolt` | `bv_ema > batt_stop_v + 0.5 V` (10.5 V) | 0.5 V |
| `overcurrent` | 어느 모터도 `oc_active` 가 아니고 미검출 모터가 없음 | 전류 한계 |
| `motor_lost` | 미검출 모터가 없음 | — |

미검출·재부팅 중·피드백이 오래된 모터가 하나라도 있으면 온도 판단은 **보류**한다
(모르는 상태에서 과열 래치를 풀지 않는다). 해제되면 `{"evt":"fault_clear","f":[...]}` 를 보내고,
토크가 켜져 있으면 과열·과전류로 꺼 두었던 모터의 토크를 다시 켠다. 구동 목표는 0,
조향은 마지막 목표각을 유지한다.

`FLT_ESTOP` 과 `FLT_SELFTEST` 는 이 함수가 건드리지 않는다. 사람이 원인을 확인하고
`E`(텔레옵 `R` 키) 를 보내야 한다.
`E` 는 `faults = 0` 으로 만들고 모든 모터를 다시 설정한다. 켤 때 조건이 나쁘면
(`no_response`, `hw_error_persists`, `too_hot`, `battery_low`, `estop_active`,
`steer_pos_out_of_range`) 켜지지 않고 `{"err":...}` 를 보낸다.

## 복구 키: `E` 와 `R`

래치 고장(`estop`, `selftest`)은 자동으로 풀리지 않는다. 사람이 확인한 뒤 보드에 `E` 를
보내야 한다. `E` 는 `faults = 0` 으로 만들고 모든 모터를 다시 설정하며 토크를 켠다.

- **직접 시리얼**: 보드에 `E\n` 한 줄.
- **텔레옵 `R` 키**: `/r12/swerve_clear_faults`(`std_msgs/Empty`)를 발행하면
  브리지가 **양 보드에 `E`** 를 보낸다. `E` 를 사람이 확인한 뒤에만 보내도록
  브리지는 자동으로 `E` 를 보내지 않는다.
- `R` 은 먼저 텔레옵의 이동 목표를 0 으로 비우고 1 s 에 한 번만 발행한다.
  브리지도 받은 즉시 저장된 휠 목표를 0 으로 비우고 `cmd_time = None` 으로 두어,
  새 `C` 가 올 때까지 `en=0` 을 유지한다.

`E` 가 거절되는 경우는 아래 [수동 복구가 필요한 경우](#수동-복구가-필요한-경우)와
`{"err":...}` 응답을 참고한다. 특히 비상정지가 아직 눌려 있으면 `estop_active` 로 거절된다.

## 하드웨어 에러 자동복구

다이나믹셀은 과부하·과열·입력전압·전기충격·엔코더 에러가 나면 스스로 토크를 끄고,
`REBOOT` 명령으로만 푼다. 펌웨어가 그 모터만 재부팅해 다시 살린다.

상태는 `RCV_IDLE` / `RCV_BOOTING` 두 단계다.

1. `FLT_HWERR` 가 서면 `hw_retry_ms`(1 s) 쿨다운을 기다린다.
2. 배터리가 `batt_stop_v` 이하이거나, 모터 온도가 `temp_derate_c` 이상이면 재부팅하지 않는다.
   과열 에러(`0x04`)인데 아직 응답이 없으면 식을 때까지 기다린다.
3. 문제 모터만 `reboot()` 하고 `HW_REBOOT_WAIT_MS`(0.5 s) 기다린다.
4. 재설정(`configureMotor`)과 Hardware Error Status 재확인을 `HW_REBOOT_TIMEOUT_MS`(3 s) 동안 반복한다.
   성공하면 그 모터는 `needs_torque` 에 올라가고, `faults` 에서 `FLT_HWERR` 가 지워진다.
   다른 래치 고장이 없으면 토크를 다시 켠다.
5. 실패하면 `hw_recover_fail` 을 보내고 쿨다운부터 다시 센다.
   `hw_retry`(3)회를 연속 실패하면 `hw_recover_giveup` 을 한 번 보내고,
   **영구 포기가 아니라 60 s 간격으로 계속 재시도**한다. `E` 를 보내면 즉시 다시 시도한다.
6. 마지막 에러 후 30 s 동안 에러가 없으면 시도 횟수를 0 으로 되돌린다.

`hw_retry = 0` 이면 자동복구를 끈다. 이때는 `E` 로만 복구된다.

## 미검출 모터 복구 (E 불필요)

- 동기 읽기가 5회 연속 누락되면 그 모터를 `missing` 으로 표시하고 `motor_missing` 을 보낸다.
  토크가 켜져 있으면 `FLT_COMM` 도 선다.
- `probeMissing()` 이 500 ms 마다 미검출 모터를 **한 번에 하나씩** `ping` 한다.
  응답이 오면 `configureMotor()` 로 다시 설정하고 `motor_found` 를 보낸다.
- 다시 설정하면 모터 토크가 꺼지므로, 토크가 켜져 있던 상태면 `needs_torque` 에 올려
  다음 복구 판단에서 토크를 다시 켠다.
- 미검출 모터가 모두 사라지고 2 s 지나면 `FLT_COMM` 이 자동 해제된다.
  `E` 를 보내지 않아도 READY 로 돌아온다.

모터를 뽑았다 꽂을 때는 ID 와 배선이 맞아야 한다. 잘못된 ID 로 꽂으면 계속 미검출로 남고,
그동안 이 보드의 구동은 0 이다.

## IMU 무응답

`IMU_ENABLE = true` 인 보드만 해당한다. `imu.bConnected` 가 false 면 `WRN_IMU` 를 세우지만
**주행을 막지 않는다**(융합 여부는 ROS 가 판단).

재초기화는 `imu.begin()` 이 자이로 보정으로 최대 5 s 멈추기 때문에 **정지 중에만** 한다.
조건은 `!motionAllowed() && wheelsStill()` 이고, 간격은 `IMU_RETRY_MS`(2 s)다.
시도할 때마다 `{"evt":"imu_retry","ok":0/1}` 를 보낸다.

## 워치독·링크·자동 휴면

| 기능 | 조건 | 동작 | 복귀 |
| --- | --- | --- | --- |
| 워치독 | 유효한 `C` 가 `watchdog_ms`(500 ms) 끊김 | `wd_tripped=1`, 구동 0, 조향 유지 | `C` 수신 |
| 링크 끊김 | 브리지 프레임이 같은 시간 끊김 | `linked=0`, `wd_alarm`, 파랑 LED off, 긴 삐 | `C` 수신(`bridge_linked`) |
| 자동 휴면 | 토크 ON + `st_phase == ST_IDLE` + `idle_ms`(5 s) 무명령 | `disableTorque()`, `auto_sleep` | `C`(자동 `E`) 또는 `E` |

자동 휴면은 전력 절약용이라 조향 토크도 푼다. 자가진단 중에는 호스트가 `C` 를 보내지 않으므로
`ST_IDLE` 에서만 동작하게 막아 두었다(시험 중 조향이 풀리면 직진 시험이 실패한다).

워치독과 링크 끊김은 래치가 아니다. `C` 한 프레임이 오면 바로 복귀한다.

## 자가진단

- `SW1`(`BDPIN_PUSH_SW_1`)을 3 s 길게 누르면 시작한다. 누르는 중 떼면 취소.
- `enableTorque()` 로 토크를 켠 뒤, 조향을 0 rad 로 맞추고(허용 오차 2°, 1 s 안에 못 맞추면 중단),
  최고 0.6 m/s 로 10 m 전진 → 정지 → 10 m 후진한다.
- **끝나거나 중단돼도 토크를 끄지 않는다.** `holdSelfTestStop()` 이 구동 목표만 0 으로 만들고
  조향은 0 rad 를 계속 잡는다(능동 제동). 예전에는 `disableTorque()` 로 조향이 풀려 직진이 깨졌다.
- 중단 조건: 아무 고장 비트, `SW2` 누름, 호스트의 `C`/`E`/`D`/`S`, 조향 정렬 실패, 타임아웃.
  이때는 토크를 유지한 채 `selftest_abort` 를 보낸다.
- 결과가 불합격이면 `FLT_SELFTEST` 를 래치한다. 결과 JSON(`{"selftest":...}`)을 보고 `E` 로 푼다.
  합격이면 고장을 세우지 않는다.
- 자동 휴면은 자가진단 중 동작하지 않는다.

## 비상정지

두 가지 입력이 있고, 둘 다 `FLT_ESTOP` 래치를 세운다. 해제는 `E`(텔레옵 `R` 키) 뿐이다.

### RC-100 무선 리모컨 (현재 켜져 있음)

```c
constexpr bool     RC100_ESTOP_ENABLE = true;
constexpr int      RC100_PORT         = 1;       // Serial2
constexpr uint16_t RC100_ESTOP_MASK   = 0x03FF;  // 모든 버튼
```

BT-410 수신기를 OpenCR `Serial2` 에 꽂고, 버튼이 하나라도 눌리면 정지한다.
리모컨은 버튼을 누를 때만 패킷을 보내므로 "떼면 자동 해제"가 불가능하다(래치 유지).
수신기 배터리가 죽으면 아무 입력도 받지 못한다.

### 물리 비상정지 핀 (현재 꺼져 있음, NC + 래치로 확정)

```c
constexpr int  ESTOP_PIN        = -1;     // -1 = 사용 안 함. 켤 때 BDPIN_GPIO_1(50) 등
constexpr bool ESTOP_ACTIVE_LOW = false;  // NC(평소 닫힘): HIGH = 눌림/단선
```

`-1` 이라 지금은 감시하지 않는다. 쓰려면 OpenCR GPIO 헤더의 남는 핀
(`BDPIN_GPIO_1`~`BDPIN_GPIO_18` = 50~67)을 지정한다. 아래 핀은 피한다:
22~25 사용자 LED, 26~27 DIP 스위치, 28·37~39 IMU SPI, 29 배터리 ADC, 31 버저,
32 다이나믹셀 전원, 34~35 자가진단 버튼, 36 STATUS LED.

| 배선 | 평소 | 눌림/단선 | 설정 |
| --- | --- | --- | --- |
| 일반 푸시버튼: 핀 — 스위치 — GND | HIGH(내부 풀업) | LOW | `ESTOP_ACTIVE_LOW = true` |
| **NC 비상정지: 핀 — NC 스위치 — GND** | LOW | HIGH | `ESTOP_ACTIVE_LOW = false` |

NC(fail-safe) 방식은 스위치를 누르거나 **선이 끊어져도** HIGH 가 되어 정지한다.
산업용 비상정지는 이 방식을 쓴다. 보드가 두 대이므로 스위치 하나에 두 보드의 핀을
병렬로 연결해야 한다(한 보드에만 달면 브리지가 죽었을 때 반대쪽은 계속 돈다).

```
좌 보드 GPIO 50 ──┐
                  ├── NC 비상정지 스위치 ──── GND (공통)
우 보드 GPIO 50 ──┘
```

핀에 외부 전원을 넣지 말고 스위치로 GND 에만 붙인다(내부 풀업 사용).
물리 핀이 비상정지 상태인데 `E` 를 보내면 `{"err":"estop_active"}` 로 거절된다.
스위치를 푼 뒤 `E` 를 보내야 한다. RC-100 은 버튼을 놓은 뒤 `E` 를 보내면 되고,
수신기에 새 패킷이 오면 다시 걸릴 수 있다.

### 복구 절차 (NC + 래치)

1. 위험 → 비상정지 버튼을 누른다. 구동 0, 조향 토크 유지, 빨강 LED + 긴 삐.
2. 로봇 주변을 확인하고 버튼을 돌려 푼다. **아직 정지 상태다(래치).**
3. 텔레옵에서 `R` 을 누른다(또는 시리얼 `E`). 초록 READY 로 돌아온다.
4. 새 주행 명령(`W`/`A`/`S`/`D`)을 준다.

버튼이 눌린 채로 `R` 을 누르면 `estop_active` 로 거절되고 상태는 그대로다.
RC-100 도 같은 절차다. `E` 는 절대 자동으로 보내지 않는다.

> 펌웨어 비상정지는 **모터 전원을 끊는 장치가 아니다.** 구동 목표를 0 으로 만들고
> 토크를 유지하는 능동 제동이다. 진짜 전원 차단이 필요하면 12 V 모터 전원이나
> `DXL_PWR_EN` 에 별도 접점을 두는 하드웨어 비상정지를 함께 쓴다.

## 임계값과 P 키

`P <key> <값...>` 은 **토크 OFF 일 때만** 먹는다(`param_needs_torque_off`).

| 항목 | 기본값 | 상수 | P 키 |
| --- | --- | --- | --- |
| 온도 warn / derate / abort | 50 / 65 / 70 °C | `DEF_TEMP_*_C` | `temp <warn> <derate> <abort>` |
| 배터리 warn / derate / stop | 11.1 / 10.5 / 10.0 V | `DEF_BATT_*_V` | `batt <warn> <derate> <stop>` |
| derate 배율 | 0.5 | `DEF_DERATE_SCALE` | `derate <0.1~1.0>` |
| 과전류 한계 | 1.5 A | `DEF_CURRENT_LIMIT_A` | `cur_a <A>` |
| 과전류 트립 시간 | 300 ms | `DEF_OVERCURRENT_TRIP_MS` | `oc_ms <ms>` |
| 워치독 | 500 ms | `DEF_WATCHDOG_MS` | `wd_ms <ms>` |
| 자동 휴면 | 5 s | `DEF_IDLE_TIMEOUT_MS` | `idle_ms <ms>` |
| HW 재시도 횟수 / 간격 | 3회 / 1 s | `DEF_HW_RETRY_*` | `hw_retry`, `hw_retry_ms` |
| 최고 속도 | 65 rpm | `DEF_MAX_MOTOR_RPM` | `max_rpm` |

컴파일 상수라 `P` 로 못 바꾸는 것: 자동 해제 유지 시간(`AUTO_CLEAR_HOLD_MS` 2 s),
온도·전압 히스테리시스(5 °C / 0.5 V), IMU 재시도 간격(2 s),
HW 포기 후 재시도(60 s), 통신 누락 허용(5회), 미검출 재확인(500 ms).
바꾸려면 `config.h` 를 고치고 좌·우를 함께 다시 빌드한다.

하드 상한(`P` 로도 못 넘음): 모터 70 rpm, 조향 ±90°, 전류 2.25 A, 온도 79 °C,
배터리 정지 9.0 V, 워치독 500 ms, derate 0.1.

## LED·부저

STATUS LED 는 1 Hz 하트비트다. 멈추면 MCU 가 멈춘 것이다.

| 상태 | LED | 소리 |
| --- | --- | --- |
| READY | 초록 | 없음 |
| LINKED (브리지 프레임 수신) | 초록 + 파랑 점멸 | 링크 순간 짧은 3음 |
| 경고(온도·배터리·derate·초기화) | 빨강+초록(앰버) | 배터리 경고만 10 s 마다 낮은 단비프 |
| 치명(과열, 배터리 있는 저전압) | 빨강 점멸 | 3 kHz 빠른 연속음 |
| 모터 연결 코드 1/2/3 | 빨강 | 짧은 비프 1/2/3회 묶음 |
| 기타 고장·IMU·워치독 | 빨강 2 Hz | 2 s 마다 긴 삐 |
| 배터리 없이 USB 전원만 | 빨강 고정 | 없음 |
| 자가진단 대기(버튼 누르는 중) | 초록+파랑 | 1초마다 높아지는 비프 |
| 자가진단 실행 중 | 초록, 파랑 점멸 | 없음 |

## 시리얼 이벤트

한 줄 = JSON 1개. `evt` 는 상태 변화의 순간에만 나온다.

| `evt` | 의미 | 예 |
| --- | --- | --- |
| `fault` | 고장 비트가 새로 섬 | `{"evt":"fault","f":"overtemp","id":1,"v":71.20,"t":12345}` |
| `fault_clear` | 자동 해제된 비트 | `{"evt":"fault_clear","f":["overtemp","overcurrent"],"t":...}` |
| `motor_missing` | 동기 읽기 5회 연속 누락 | `{"evt":"motor_missing","id":4,"t":...}` |
| `motor_found` | 미검출 모터 재설정 성공 | `{"evt":"motor_found","id":4,"t":...}` |
| `hw_recover` | HW 에러 모터 재부팅 시작 | `{"evt":"hw_recover","try":1,"ids":[1],"t":...}` |
| `hw_recover_fail` | 그 시도 실패 | `{"evt":"hw_recover_fail","try":1,"ids":[1],"t":...}` |
| `hw_recovered` | 재부팅·재설정 성공 | `{"evt":"hw_recovered","try":1,"ids":[1],"t":...}` |
| `hw_recover_giveup` | 3회 실패, 60 s 간격 재시도로 전환 | `{"evt":"hw_recover_giveup","try":3,"ids":[1],"t":...}` |
| `derate` | derate 진입/해제 | `{"evt":"derate","on":1,"t":...}` |
| `watchdog` | `C` 끊김 | `{"evt":"watchdog","t":...}` |
| `link_lost` | 브리지 링크 끊김 | `{"evt":"link_lost","t":...}` |
| `bridge_linked` | 브리지 프레임 재개 | `{"evt":"bridge_linked","t":...}` |
| `auto_sleep` | 자동 휴면으로 토크 OFF | `{"evt":"auto_sleep","t":...}` |
| `imu_retry` | IMU 재초기화 시도 결과 | `{"evt":"imu_retry","ok":1,"t":...}` |
| `selftest_hold` | SW1 누르기 시작 | `{"evt":"selftest_hold","t":...}` |
| `selftest_cancel` | 3 s 전에 뗌 | `{"evt":"selftest_cancel","t":...}` |
| `selftest_start` | 시험 시작 | `{"evt":"selftest_start","t":...}` |
| `selftest_abort` | 중단(이유 포함) | `{"evt":"selftest_abort","why":"fault","t":...}` |

`selftest_abort` 의 `why`: `fault`, `stop_button`, `host_command`, `timeout`,
`steer_range`, `steer_align`.

그 밖에 `{"selftest":{...}}` 결과 레코드, `{"cfg":1,...}` 설정, `{"err":"..."}` 오류,
`{"hello":1,...}` 핸드셰이크가 있다.

상태 레코드(50 Hz)에서 복구 판단에 쓰는 필드:

| 필드 | 의미 |
| --- | --- |
| `flt` | 고장 비트마스크(위 표) |
| `wrn` | 경고 비트마스크 |
| `en` | `motionAllowed()`: 1 이어야 실제로 움직인다 |
| `tq` | 토크 ON/OFF |
| `wd` | 워치독 걸림 |
| `rcv` | HW 자동복구 재부팅 중 |
| `st` | 자가진단 단계(0 대기, 1 버튼 대기, 2 실행) |
| `lnk` | 브리지 링크 |
| `miss` | 미검출 모터 비트(bit0·1 구동, bit2·3 조향) |

## 수동 복구가 필요한 경우

| 상황 | 방법 |
| --- | --- |
| 비상정지 | 스위치를 풀고 텔레옵 `R`(또는 `E`) |
| 자가진단 불합격 | 결과 확인 후 `R`(또는 `E`) |
| HW 에러 3회 실패 | `E` 로 즉시 재시도. 안 보내도 60 s 마다 자동 재시도 |
| 모터 초기화 실패(`motor_init`) | 배선·ID·전원 확인 후 `E` |
| 배터리 부족(`battery_low`) | 충전 후 `E` |
| 과열로 `E` 거절(`too_hot`) | 식힌 뒤 `E` |
| `steer_pos_out_of_range` | 조향축을 범위 안으로 돌린 뒤 `E` |

`D` 는 토크 OFF, `S` 는 구동 0(조향 유지)이다. `S` 는 고장을 풀지 않는다.

## 변경 이력

- 2026-11-14: 텔레옵 `R` → `/r12/swerve_clear_faults` → 브리지가 양 보드에 `E` 를 보내는
  복구 경로 추가. 물리 비상정지는 NC(평소 닫힘) + 래치로 배선을 확정하고
  `ESTOP_ACTIVE_LOW = false` 로 바꿨다(핀은 아직 `-1` 로 비활성).
- 2026-11-14 (`1e1d534`): 과열·저전압·과전류·통신 두절 자동 해제, HW 에러 60 s 재시도,
  미검출 모터 토크 복귀, IMU 정지 중 재초기화, 자가진단 중 자동 휴면 차단,
  커밋 `45c3294` 에서 잘렸던 `loop()` 제어 주기 복원.
- 2026-11-13 (`e8931f8`): 자가진단 종료·중단 시 토크를 끄지 않고 조향 0 rad 유지.

## 관련 파일

- `r12/firmware/opencr_swerve_left/opencr_swerve_left.ino` — `updateSafety`, `autoClearFaults`,
  `recoveryTick`, `probeMissing`, `holdSelfTestStop`, `loop`
- `r12/firmware/opencr_swerve_right/opencr_swerve_right.ino` — 왼쪽과 같은 동작(별도 사본)
- `r12/firmware/opencr_swerve_left/config.h`, `.../opencr_swerve_right/config.h` — 임계값·상수
- `r12/ros2/src/r12_swerve_control/r12_swerve_control/opencr_bridge_node.py` —
  `/r12/swerve_clear_faults` → 양 보드 `E`
- `r12/ros2/src/r12_swerve_control/r12_swerve_control/real_teleop_node.py` — 복구 키 `R`
- [`OPENCR_HANDSHAKE.md`](OPENCR_HANDSHAKE.md) — 보드 식별·프로토콜·브리지 안전 규칙
