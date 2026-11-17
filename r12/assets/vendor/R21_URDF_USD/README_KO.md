# R21 스워브 + 손목 롤 로봇팔 URDF · USD

`assembly_puck_slots` 배포본(Fusion 활성 문서 조립본)을 r21 이름으로 정리한 ROS description 패키지입니다. 원본 모델 파일은 `r12/models/r21/`에도 있으며, 이 패키지는 colcon 빌드/ROS 소비용 레이아웃입니다.

## 열 파일
- URDF: `urdf/r21.urdf` — `../meshes/` 상대경로. 직접 열거나 MuJoCo/도구용.
- URDF(ROS): `urdf/r21_ros.urdf` — `package://r21_description/meshes/` 경로. robot_state_publisher/ROS용.
- USD: `usd/r21.usda` — 같은 폴더의 `r21.usdc`를 참조하는 래퍼.
- 바이너리 USD: `usd/r21.usdc` — 메시 내장 crate 파일.
- `PREVIEW.jpg`: 배포본 미리보기 렌더.
- `checks/verification.json`, `checks/verification_usd.json`: 배포본이 제공한 MuJoCo 검증 결과(URDF / USD 각각).
- `checks/VALIDATION.json`: r21 구조 요약과 로컬 MuJoCo 로드 결과.

## 반영한 형상
Fusion 활성 문서 조립본에서 1,219개 유효 바디를 16개 운동 링크로 묶었습니다. 총질량 42.9553 kg, 시각 메시 4,296,288면(원본 그대로). 접촉은 박스와 타이어 원통으로 단순화. 색은 분류색이며 실물 재질값이 아닙니다.

## 좌표와 관절
- 단위 m, Z 위, +X 전방 / +Y 왼쪽. 차륜 중심 X,Y ±124 mm, Z −148 mm. 타이어 접촉 반경 33 mm.
- 조향 4축 + 바퀴 4축 + 팔 5축(arm_yaw·shoulder·elbow·wrist_pitch·wrist_roll) + 그리퍼 2축(jaw_left/right, mimic) = 가동 관절 15개, 제어 입력 14개.
- 그리퍼 CAD 초기 간격 19.63 mm, 손가락당 바깥쪽 27.686 mm 허용(위치 0=CAD 원자세, −0.027686 m=최대 개방).
- 바퀴 속도 단위: MuJoCo rad/s, USD 각속도 드라이브 deg/s.
- 조향·팔 범위, 구동 게인, 토크 한계는 시뮬레이션 가정이며 실물 간섭/안전 인증값이 아닙니다.

## 미반영(R12 벤더 대비)
원본 CAD/생성 스크립트가 없어 R12 벤더의 `scripts/`, `checks/body_geometry_map.json`·`current_sensor_placements.json`·`live_sensor_transforms.json`·`usd_link_paths.json`·`mesh_statistics.json`·`MUJOCO_IMPORT.json`, `SHA256.json` 외 분석 산출물은 재현하지 않았습니다. 대신 배포본이 제공한 `verification*.json`을 그대로 넣었습니다.
