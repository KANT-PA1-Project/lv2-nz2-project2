# 프로젝트 2 — 주사위 기억 로봇 (팀 NZ2) 제출 안내

> 스켈레톤(2026-11-17). `TODO`는 채워야 할 곳이다. 저장소 전체 설명·하드웨어·**환경과 버전 표(단일 출처)**는 루트 [`README.md`](../README.md)에 있으며 여기서 복제하지 않는다.

## 1. 프로젝트 목표
출발 구역에서 바코드(Code 128)와 신호등 12회 점멸을 관측해 R·Y·G 횟수를 기억하고, 바코드 정렬 규칙으로 L1~L3에 그룹을 배정한 뒤 퍽 12개로 주사위 눈을 만들고, 새 관측으로 재검증하고 출발 구역으로 복귀한다. 규칙 원문: `mission.pdf`, 문제 조건: [`problem/problem.md`](problem/problem.md).

## 2. 사용 장비·소프트웨어 버전
- 하드웨어·버전: 루트 README "하드웨어 구성", "환경과 버전" 표 참조.
- 공식 경기에 실제 사용한 구성(카메라 대수·라이다 유무·펌웨어 커밋): TODO

## 3. 설치
루트 README "다른 기기에서 실행하기" 절차를 따른다. 추가 단계: TODO (Jetson 컨테이너·모델 엔진 빌드 등)

## 4. 실행 순서
| 단계 | 장치 | 명령 | 완료 확인 |
|---|---|---|---|
| 1. 하위 제어 기동 | Jetson | TODO | TODO |
| 2. 인식 기동 | Jetson | TODO (`nz2_perception`, `nz2_signal`) | TODO |
| 3. 주행 기동 | Jetson | TODO (`nz2_swerve_nav`) | TODO |
| 4. 미션 FSM | Jetson | TODO (`nz2_mission` `mission.launch.py`) | TODO |
| 5. 시작 명령 | 관제 PC | TODO | TODO |

## 5. 경기 초기화
경기장·퍽 초기화 절차는 [`problem/problem.md` "초기화와 경기 운영"](problem/problem.md)을 따른다. 로봇 측 초기화(로그 디렉터리, 체크포인트 삭제, 그리퍼·트레이 비우기, 초기 자세): TODO

## 6. 정지 방법
- 비상정지(RC-100 / 하드웨어 스위치): TODO
- 소프트웨어 정지(`/state/estop`, SAFE_STOP 진입 확인): TODO
- 전원 차단 순서: TODO

## 7. 주요 파일 경로
| 용도 | 경로 |
|---|---|
| 문제 패키지 | `project2/problem/` |
| 설계 | `project2/docs/architecture.md` |
| 시험 결과 | `project2/results/test-results.csv` |
| 관측 신호 | `project2/results/signal.json` |
| 할당·검증 | `project2/results/allocation.json` |
| 공식 경기 회차별 판정 | `project2/results/official/round-1/`, `round-2/` |
| 재현 시험 | `project2/results/reproduction.md` |
| 개인 기여 | `project2/docs/contributions.md` |
| 미션 파라미터 | `ros2_ws/src/planning/nz2_mission/config/` |

## 8. 재현 시 주의사항
- 실물 시험은 바퀴 띄운 벤치 → 지면 순. 비상정지 담당을 둔다.
- TB3 배터리(3S, 연속 약 9A)·OpenCR 10A 퓨즈 공유 → 팔·구동 동시 고부하 금지.
- XM430 온도 70°C 소프트 중단.
- 기록 시 펌웨어 버전·배터리 전압을 함께 남긴다.
- TODO: 현장 조명, 신호등 노출 설정 등 팀이 겪은 함정.

## 9. 공식 경기 결과 요약
| 회차 | 원점수 | 완주 | 판정표 | 영상 |
|---|---|---|---|---|
| 1 | TODO | TODO | [round-1](results/official/round-1/scorecard.md) | TODO |
| 2 | TODO | TODO | [round-2](results/official/round-2/scorecard.md) | TODO |

최종 반영 점수(높은 회차 전체 점수): TODO
