# R25 스워브 + 손목 롤 로봇팔 URDF · USD

`assembly_puck_slots` v25(최종) 배포본(Fusion 조립본)을 r25 이름으로 정리한 ROS description 패키지입니다. 원본 모델 파일은 `r12/models/r25/`에도 있으며, 이 패키지는 colcon 빌드/ROS 소비용 레이아웃입니다.

## 열 파일
- URDF: `urdf/r25.urdf`(`../meshes/`, 도구용), `urdf/r25_ros.urdf`(`package://r25_description/meshes/`, ROS용).
- USD: `usd/r25.usda`(래퍼) → `usd/r25.usdc`(메시 내장 crate).
- `PREVIEW.jpg`: 배포본 미리보기 렌더.
- `checks/verification.json`: 배포본 제공 MuJoCo 검증 결과(URDF). (v25 배포본에는 verification_usd.json 없음.)
- `checks/VALIDATION.json`: r25 구조 요약.

## 반영한 형상 (v25)
Fusion v25 조립본에서 1,219개 유효 바디를 16개 운동 링크로 묶었습니다. 링크 질량 합계 43.0208 kg. 시각 메시는 Fusion 표시 메시 원본. r21 대비 상단 카메라와 마스트가 35 mm 상승, 상단 카메라 4스트림 위치·라이다 마운트 4개·카메라 마운트·스토리지 메시·차체 관성을 갱신했습니다. 조인트 구조는 r21과 동일.

## 좌표와 관절
- 단위 m, Z 위, +X 전방 / +Y 왼쪽. 차륜 중심 X,Y ±124 mm, Z −148 mm. 타이어 접촉 반경 33 mm.
- 조향 4 + 바퀴 4 + 팔 5(arm_yaw·shoulder·elbow·wrist_pitch·wrist_roll) + 그리퍼 2(jaw_left/right, mimic) = 가동 관절 15, 제어 입력 14.
- 그리퍼 위치 0=CAD 원자세, −0.027686 m=최대 개방. 조향·팔 범위·게인·토크 한계는 시뮬 가정값.

## 미반영(R12 벤더 대비)
원본 CAD/생성 스크립트가 없어 scripts/ 및 R12식 분석 check 파일은 재현하지 않았습니다. 배포본 verification.json을 그대로 넣었습니다.
