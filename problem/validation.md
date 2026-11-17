# 문제 패키지 검증 기록

검증일 2026-11-17. 문제 ID는 NZ2-Q1-review다.

## 실행한 검사

| 검사 | 결과 |
| --- | --- |
| 신호 12개, R/Y/G 각 1~6, 합계 12 | 통과, R4·Y2·G6 |
| P2-01 오름차순과 동률 규칙 | 통과, L1=Y·L2=R·L3=G |
| 1초 ON·1초 OFF·마지막 3초 OFF | 통과, 첫 ON부터 종료까지 26초 |
| 퍽 12개 서로 다른 ID·등록 속성 | 통과 |
| 제공된 Q1 등록표·초기 좌표 일치 | 통과 |
| 시작 박스 6곳·각 2개·중심 X±40 | 통과 |
| Y 한 모양, R/G 색상, 전체 중복·누락 | 통과, 그룹별 유효 할당 6개 |
| 목적지별 눈 방향·X/Y 좌표·Z 높이 | 통과 |
| 제공 패키지 validate_problem.solve | 유효 할당 확인 |
| 팀 arm_practice.mission.plan | 정답 예시 ID·12개 좌표 모두 일치 |
| 원본 스냅샷·팀 참조 소스 SHA256 | 일치 |

위 결과는 입력된 문제의 오프라인 규칙·좌표 검증이다.
컴퓨터에서 Python 표준 라이브러리와 CPU 계획 함수로 수행했다.

## 잘못된 입력 거부 확인

복사한 임시 입력에 아래 변경을 각각 적용했다. 검사 후 임시 폴더는 제거했다.

| 입력 | 실제 결과 |
| --- | --- |
| 13번째 신호 추가 | 거부 |
| 마지막 소등을 1초로 변경 | 거부 |
| 목표 퍽 중심 Z를 100에서 70으로 변경 | 거부 |
| 2의 눈을 반대 대각선으로 변경 | 거부 |
| 다른 유효 할당으로 Y의 ID1과 R의 ID2 교환 | 허용 |

## 기존 설정과 다른 점

팀 r12/configs/mission/practice.json은 합성 연습용 속성을 사용한다.
공식 제공 Q1 등록표와 속성이 다른 ID는 1, 4, 5, 8, 9, 10이다.
이번 패키지는 sources/placement.csv와 sources/mission.json에 있는 제공 Q1을 따른다.
Downloads의 nz2_sim/config/placement.csv에도 가정한 ID·속성 대응이 남아 있다.
문제지 v5의 장소 태그는 접근 면 수직 부착이며, 팀 mission_markers.py에는 상단 부착 설정이 있다.
실물 설치와 실제 사용할 태그 변환은 현장 확인이 필요하다.

## 남은 완료 조건

- 실제 경기 퍽 12개의 속성·ID 및 초기 좌표 대조
- 신호등·바코드·태그·목적지·시작 박스 실물 설치 확인
- 출발·복귀 구역 경계 좌표와 차체 전체 진입 판정 확인
- 다른 팀의 검토 날짜·검토자·의견·수정 후 확인 기록
- 검토가 끝난 문제 버전의 project2-problem-submit 태그

review.json의 해당 항목은 pending으로 남겨뒀다.
validation.json의 logical_pass는 true, submission_ready는 false다.
실물 센서 관측·주행·파지·리프트·운반·배치·복귀 및 다른 팀 실행 기록은 미확인이다.

## 재실행

팀 레포 루트에서 실행한다.

```bash
python3 problem/validate.py --output problem/validation.json
```

원본 스냅샷·입력 데이터 SHA256, 모든 유효 그룹 할당은 validation.json과 sources.json에 있다.
