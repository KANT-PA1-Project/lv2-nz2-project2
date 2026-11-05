# 검증 기록

기록일: 2026-10-30. 작성 환경은 Windows이며 Ubuntu/GPU 대상 실행과 구분합니다.

| 항목 | 결과 | 의미 |
|---|---|---|
| CPU 단위 테스트 | 7개 통과 | 행동 제한·NaN·보상·URDF·설정 검사 |
| demo URDF 구조 검사 | 통과 | 5개 링크·3개 제어 관절·tool0 |
| Python 문법 | 검사 대상 전체 통과 | import/시뮬레이션 실행을 뜻하지 않음 |
| Bash 문법 | 스크립트·문서 명령 검사 | Ubuntu 설치 완료를 뜻하지 않음 |
| EA 소스 API 대조 | v3.0.0-EA 기준 | CLI·태스크 등록·ReachCfg·PPO·actuator 필드 |
| Ubuntu 24.04 설치 | 미실행 | 대상 장비에서 확인 필요 |
| USD 변환 / PhysX 실행 | 미실행 | GPU·Sim 환경에서 확인 필요 |
| PPO 학습 / 정책 재생 | 미실행 | 성공률·학습 수렴 미확인 |
| Lyrical 관절 통신 | 미실행 | 카메라 검증을 팔 검증으로 대체하지 않음 |
| 실물 구동기 제어 | 미실행 | 모터 드라이버·보정 필요 |

CPU 함수는 학습 수식을 설명하기 위한 독립 예제입니다. 실제 Lab의 Manager 설정과 보상 계수는 CODE_GUIDE에 구분했습니다. 예제 태스크의 위치 도달 성공은 접촉 안전성·그리퍼 집기 성공을 포함하지 않습니다.

## 대상 장비에서 남길 기록

Ubuntu 버전, `nvidia-smi`, Lab git SHA, `uv.lock`, pip freeze, ROS_DISTRO, apt 패키지 버전, URDF/USD/JSON 파일, 실제 명령, 로그 디렉터리, checkpoint, 성공률·충돌·지연을 기록합니다. 첫 실행은 16환경·10iteration으로 확인합니다.


## Ubuntu 로컬 구현 검증 — 2026-10-30

- Ubuntu 24.04.5, Python 3.12 CPU 가상환경에서 editable 설치 완료.
- CPU 테스트 13개 통과: 기존 7개 및 URDF/설정 교차 검사 6개.
- 교차 검사: 관절 누락/오타, TCP 오타, 위치·힘·속도 한계 초과, 보수적 한계, 관절 순서 변경, USD 필수 모드, CLI 실패 보고서 확인.
- demo URDF 검사·행동/보상 예제·사전 검사 CLI 실행 완료.
- NVIDIA GeForce RTX 5060 Laptop GPU, VRAM 8151 MiB, 드라이버 595.99.02 확인. `/opt/ros/lyrical` 존재 확인; ROS 통신은 미검증.
- USD 변환, PhysX, PPO 및 실물 제어는 이번 실행에서도 미검증.


## R12 제공 자산 통합

2026-10-30 Ubuntu에서 제공 R12 자산의 PhysX, PPO, 정책 평가·재생, ROS 브리지와 가상 궤적 액션을 실행했습니다. 최신 결과와 남은 범위는 [R12 실행 결과](R12_VALIDATION.md)를 참고하세요. 기존 표의 GPU 미실행 상태는 초기 demo 검증 기록입니다.
