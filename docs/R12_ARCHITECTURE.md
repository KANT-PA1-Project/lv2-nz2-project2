# R12 아키텍처

## 목표 시스템

```mermaid
flowchart TD
  signal[시작 바코드·12점멸] --> mission[합법 미션 해독·요구 대상 선택]
  sensors[상단 카메라·라이다·dual IMU·엔코더] --> nav[cuVSLAM / nvBlox / Nav2·홀로노믹 경로 추종]
  mission --> nav
  nav --> arrival[러프한 차체 도착·정지]
  arrival --> settle[팔 관측 자세·연속 정착]
  settle --> wrist[손목 RGBD·반복 AprilTag·색/형상·정밀 위치]
  wrist --> approach[보정된 FK·오차 경계·충돌 검증 접근]
  approach --> grasp[그리퍼 파지·리프트·유지 확인]
  grasp --> carry[접은 팔·안전 운반]
  carry --> place[목적지 태그·주사위 눈 위치·놓기]
  place --> verify[해제·10mm/2s 배치 확인]
  verify --> fold[팔 복귀·다음 목표]
  fold --> mission
  safety[센서 신선도·충돌·토크·기울기 veto] --> nav
  safety --> approach
  safety --> carry
```

이는 목표 흐름이다. 모든 화살표가 통합 실행으로 검증된 것은 아니다. 현재 핵심 공백은 전체 센서 runtime 연결, 실제 카메라·그리퍼 보정, 미로 장애물 전체 coverage, 배치 자세 변환, 파지/운반 연결 및 정상 종료다.

## 관측과 현재 피드백의 분리

```mermaid
sequenceDiagram
  participant P as Isaac 소유 스레드
  participant C as NativeMissionCapture
  participant D as Decoder
  participant R as ROS 게시/구독 검증
  P->>C: 매 physics step 실제 encoder/native속도/동일step FD
  C->>C: 0.5초 연속 정착 history
  P->>C: 최대4render·200ms 내 metadata/pixels 확인
  C-->>P: 불변 RGBD·capture certificate
  P->>D: nonce·hash·인증된 capture
  D->>P: 초기 verify / 필요 시 current refresh
  P-->>D: 실제 현재 actuator read
  P->>R: 필수 topic 원본·stamp·hash
  R-->>P: 전체 수신 검증·ACK
  D-->>P: 성공 또는 정상 인식 거부
  P->>P: 양쪽 terminal 확인 후 buffer 해제
```

- 과거 촬영 FK와 최신 actuator 피드백은 별개다. ACK 시각을 촬영 시각으로 바꾸지 않는다.
- ReferenceTime은 렌더 기준 rational time이다. 실제 노출 시각이나 벽시계로 간주하지 않는다.
- 같은 호스트 IPC peer UID/PID·nonce·digest·재전송 거부, bounded framing, 실제 current read를 유지한다.
- 정상 인식 거부도 terminal 결과다. 성공으로 바꾸지 않고 다음 재관측으로 넘어간다. 통신 오류는 buffer/증거를 보존하고 새 관측을 막는다.
- GC는 정지·zero command·pending 없음·원본 상태 불변 조건에서 다음 history timestamp **이전**에만 수행한다.

## 코드 지도

| 경로 (`r12/` 기준) | 책임 |
|---|---|
| `src/arm_practice/native_rgbd/` | 캡처, encoder history, settling, FK, 인증 관측, 수명 관리 |
| `src/arm_practice/` | 미션·주행·인식·집기놓기 관련 코어 |
| `scripts/` | headless 시뮬레이션·분석·평가 실험 진입점 |
| `configs/` | 로봇·카메라·액추에이터·미션 계약 |
| `models/` | 주행/팔 정책, 인식 가중치, URDF/USD·scene |
| `assets/vendor/` | 참조 CAD/미로 자산 |
| `ros2/` | 팔 MoveIt 설정 등 기존 ROS 연동 |
| `experiments/pa24-v44/` | 검증된 5카메라 ready bench 소스 |
| `experiments/pa24-v45-unverified/` | 8RGB4depth 연결 준비본, 미실행 |
| `experiments/pa33-v135/` | 검증된 IPC/ROS observer 소스; 외부 모델·fixture 의존 |
| `experiments/pa28-v152/` | 종료 실패를 보존한 물리 smoke 소스, 합격 모델 아님 |

## 실행 분리

현재 로컬만 실행한다. 원격 재개 승인 시의 예약 allocation은 local87/표시88, pa24 24/25, pa33 33/34, pa28 28/29이며 DDS는 호스트 사이를 연결하지 않는다. 파일·포트·모델·실험 잠금은 호스트별 분리한다. 이 allocation 표는 현재 원격 기동 권한이 아니다.
