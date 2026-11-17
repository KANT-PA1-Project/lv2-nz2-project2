# R30 스워브 + 손목 롤 로봇팔 URDF · USD (nz2fix v99)

robot_hardware_design/r30 배포본의 `real_final_plz_no_more_edit_v99_nz2fix.urdf`를 r30으로 정리한 ROS/Isaac description 패키지입니다. 원본은 그대로 사용했고(로봇명만 r30, 메시 경로만 레이아웃용 조정), base_footprint·관절·축은 손대지 않았습니다.

## 열 파일
- URDF: `urdf/r30.urdf`(`../meshes/`, 도구용), `urdf/r30_ros.urdf`(`package://r30_description/meshes/`, ROS용). base_footprint 루트 포함(odom→base_footprint TF).
- USD: `usd/r30.usda`(래퍼) → `usd/r30.usdc`. Isaac 학습용.
- `PREVIEW.jpg`, `checks/verification.json`(배포본 MuJoCo 검증), `checks/VALIDATION.json`(구조 요약).

## 핵심 (README_URDF.txt 기준)
- 원본 v99에서 빠졌던 base_footprint 링크·관절을 옛 assembly_nz2 값(z 0.181 m, yaw π)으로 복원.
- 옛 assembly_nz2 대비: 상단 카메라(mast_d435) 하향 60°→35°(pitch 0.6109 rad), 높이 +35 mm. 실물 마운트 v26=35°와 일치.
- 링크 질량 합계 43.1543 kg. 조향 4+바퀴 4+팔 5+그리퍼 2 = 가동 15, 제어 14. 조인트 구조는 r25/r28과 동일.
- `.bak` 원본(base_footprint 없음)은 TF 끊김으로 사용 금지(배포본에서 제외함).
- gz 제어 플러그인은 포함 안 됨(센서 gazebo 태그만). Gazebo 구동용이 아니라 ROS/Isaac 기술서.

## 앞뒤 수정 적용
54476f0 변환(base_link+직속 자식 R_z(π), 바퀴 축 0 1 0→0 -1 0, FL↔RR·FR↔RL + lidar front↔rear·left↔right 이름 스왑)을 적용해 검증된 r25_rev1과 44개 관절 전부 일치시켰습니다(mast 35°는 유지). MuJoCo scene.xml·USD는 이번에 손대지 않았으니 그쪽을 쓰면 별도 확인 필요. gz 런타임 재확인 권장.