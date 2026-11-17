# NZ2 시스템 아키텍처 (제출용)

> 스켈레톤(2026-11-17). 상세 설계는 아래 기존 문서에 있으므로 여기서는 **제출 요건에 맞춘 요약 + 링크**만 둔다. 확정 안 된 항목은 `확인 필요`로 남긴다.
> 기존 근거: [`docs/planning/mission_fsm_design.md`](../../docs/planning/mission_fsm_design.md), [`docs/R12_ARCHITECTURE.md`](../../docs/R12_ARCHITECTURE.md), [`docs/nz2_dataflow_v3.html`](../../docs/nz2_dataflow_v3.html), [`docs/nav2/jetson_handoff_2026-11-15.md`](../../docs/nav2/jetson_handoff_2026-11-15.md)

## 1. 하드웨어·연산 배치
| 장치 | 역할 | 연결 |
|---|---|---|
| Jetson Orin Nano | 인식·상위 제어·Nav2 | TODO |
| OpenCR (스월브) | 구동·조향 모터, IMU | TODO (보드 수 확인 필요: README 2장 / PR #31 3장 분리) |
| OpenCR (팔) | OpenMANIPULATOR-X 개조 팔 | TODO |
| RealSense D435 | 신호·바코드·태그·퍽 인식 | 대수·장착 위치 확인 필요 |
| 2D 라이다 | 자기위치 | **확인 필요**: README 하드웨어 목록에는 없음, Jetson 핸드오프(11/15)는 LDS-03×4 사용 |

## 2. ROS 2 노드·토픽·메시지
| 노드 | 패키지 | 입력 | 출력 | 형식 |
|---|---|---|---|---|
| `signal_decoder` | `nz2_signal` | `/perception/traffic_light` | 신호 확정 이벤트 | TODO |
| `barcode_reader` | `nz2_perception` | 카메라 영상 | `/perception/barcodes` (`BarcodeArray`) | topic |
| `tag_reader`, `yolo_detector` | `nz2_perception` | 카메라 영상 | `/perception/puck_tracks` (`PuckArray`) | topic |
| `mission_fsm` | `nz2_mission` | 위 인식 결과, `/state/estop`, `/state/hw_error` | `arm_skill`, `navigate_to_pose` 요청 | action |
| 팔 스킬 서버 | TODO | `ArmSkill` goal | 결과·피드백 | action |
| 스월브 제어 | `nz2_swerve_nav` | `cmd_vel` | `SwerveCommand` | topic |
| OpenCR 브리지 | `nz2_control` | TODO | TODO | TODO |

TODO: 노드 그래프 그림(`rqt_graph` 캡처 또는 Mermaid). QoS 표.

## 3. 좌표계
- 경기장: mm, 원점 왼쪽 아래 기둥 중심, +X 오른쪽, +Y 위쪽, 바닥 Z=0 (`problem/arena.json`).
- ROS 내부: REP-103 m·rad. 변환 `arena_to_map`: TODO (오프셋·회전 값과 출처).
- TF 트리: `map → odom → base_link → (스월브 모듈 ×4)`, `base_link → arm_base → … → gripper → camera_gripper`, 고정 카메라 프레임: TODO.

## 4. 수행 순서와 단계별 완료 조건
| 단계 | FSM 상태 | 완료 조건 | 실패 시 |
|---|---|---|---|
| 관측(바코드) | OBSERVE | 등록 바코드 1개 확정 | 미등록 → SAFE_STOP |
| 관측(신호) | OBSERVE | 12회 + 3초 소등 확인, 각 색 1~6 | 미만·초과·불명 → SAFE_STOP |
| 계획 | SURVEY / VALIDATE / PLAN | 12개 전체 할당 유효 | TODO |
| 주행 | EXECUTE | TODO (위치 오차 기준) | TODO |
| 조작 | EXECUTE | 파지 확인(접촉·그리퍼 닫힘만으로 불인정), 배치 오차 ≤10 mm | TODO |
| 재검증 | VERIFY | 새 촬영 기반 장소별 ID·위치·중복·누락 판정 | `MISMATCH` → TODO |
| 복귀 | RETURN | 차체 전체가 출발 구역 안 3초 정지 | TODO |

상세: `mission_fsm_design.md` §3~§7.

## 5. 안전 정지·오류 처리 상태도
TODO: 상태도(Mermaid `stateDiagram-v2`). 최소 포함: 정상 흐름, `SAFE_STOP` 진입 조건(규칙 오류, `HW_ERROR`, E-stop, 무진행 감시), RESUME 경로.

## 6. 미정·확인 필요
- 카메라 대수·장착, 라이다 사용 여부, Jetson↔OpenCR 통신 방식, 스월브 IK 위치, 적재통 형식
