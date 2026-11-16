# 팔 정책 1단계 점검 결과

2026-11-15. 로컬 모델·학습 설정·최종 URDF를 읽고 CPU 추론을 확인했다. 실물 구동은 수행하지 않았다.

## 기본 R12 정책

경로는 /home/pa33/git/lv2-nz2-project2/r12/models/r12/policy 이다.

- policy.onnx 입력 obs float32 [1,25], 출력 actions float32 [1,5].
- ONNX 검사와 CPU 추론 통과. TorchScript와 ONNX에 동일한 무작위 입력 32개를 넣었을 때 최대 절대 차이는 1.9073486328125e-6.
- obs_normalizer.forward는 입력을 그대로 반환한다. 정규화 통계가 필요하지 않다.
- 입력 순서는 관절 위치 5개, 관절 속도 5개, 목표 위치 3개와 quaternion xyzw 4개, 이전 raw action 5개, 목표와 arm_link5 원점의 위치 차이 ×10인 3개다. 위치·자세는 학습 로봇 root 기준이다.
- 관절 순서는 arm_joint1~5. 위치 rad, 속도 rad/s. 기본 관절 위치와 속도는 모두 0.
- 출력 처리식은 target = clamp(action × [0.5,0.25,0.25,0.25,0.25] + 0, lower, upper).
- lower는 [-0.5,-0.25,-0.25,-0.25,-0.25], upper는 [0.5,0.25,0.25,0.25,0.25] rad. 처리 결과는 절대 관절 위치 목표다.
- 학습 제어 주기는 30Hz. 학습 속도 한도는 0.5rad/s이며 배포 설정의 0.4rad/s와 구별해야 한다.
- 고정 베이스의 위치 도달 정책이며 성공 판정은 arm_link5 원점의 목표 거리 15mm다. 그리퍼 제어와 실물 TCP 보정, 집기·운반·놓기 동작은 포함되지 않는다.
- 학습 목표는 reach_targets.json의 FK 표본을 사용한다.

## 후보 2개

경로는 /home/pa33/git/lv2-nz2-project2/r12/models/candidates 아래다.

- pa28-arm-v145와 local-arm-gamma999 모두 ONNX 입력 obs [1,30], 출력 actions [1,5].
- 두 ONNX가 요구하는 policy.onnx.data 파일이 누락되어 직접 로드가 실패한다.
- 두 TorchScript policy.pt는 로드와 0 입력 추론이 성공했고 유한한 출력이 나왔다.
- 후보 폴더와 확인한 로컬 학습 소스에서 30개 입력의 정확한 의미·정규화·출력 처리 계약과 해당 학습 로봇 설정을 확정하지 못했다.
- pa28 평가의 1회 성공은 manifest에 fixed offline goal-only IK가 정책 출력을 대체했고 loaded policy unused라고 명시되어 있다. 학습 정책의 성공률로 사용할 수 없다.
- gamma999는 1회 평가에서 80초 시간 초과, 최종 거리 0.0898258686m로 기록됐다.
- 두 후보 모두 performance_accepted=false이며 평가 표본은 각각 1회다.

## 최종 URDF 비교

사용한 파일은 /home/pa33/Desktop/nz2_robot_v99_share./nz2_robot_v99_share/real_final_plz_no_more_edit_v99_nz2fix.urdf 이며 같은 폴더 README_URDF.txt를 읽었다.

- 학습 arm_joint1은 base_link에서 [0,0,0.275]m, 축 [0,0,1].
- 최종 arm_yaw_joint는 base_link에서 [-0.15384,-0.000125,0.05335]m, 축 [0,0,-1].
- 학습 arm_joint2 축은 [0,-1,0], 최종 shoulder_joint는 [0,1,0].
- 학습 팔의 +X 방향 링크 배치에 대해 최종 elbow/wrist_pitch 링크는 -X 방향으로 배치되어 있다. 단순 이름 치환으로 연결할 수 없다.
- 최종 base_footprint 복원과 yaw π를 포함한 전체 TF를 반영해 root 좌표를 변환하고 관절 영점·축 방향을 검증해야 한다. 베이스 원점 차이만으로 링크 길이 불일치를 단정하지 않는다.
- 실물 브리지의 j1,j2,j3,j4,roll 및 모터 좌표를 최종 URDF 관절 좌표로 연결하는 변환은 추가 구현·검증 대상이다.

## 브랜치·젯슨 확인 범위

- 팀 저장소의 현재 로컬 head와 remote-tracking ref 32개를 조회했다. 확인한 모델 경로에서 ONNX 3종의 고유 blob 집합 1개가 15개 ref에 있었다.
- 원격 fetch를 수행하지 않았으므로 서버의 최신 모든 브랜치 상태는 미검증이다.
- 젯슨 10.2.11.63 SSH 접속은 두 번 시간 초과됐다. 이번 점검에서 젯슨 모델과 로컬 모델의 해시 일치를 확인하지 못했다.
- 팀 저장소·모델·펌웨어·실물 상태는 변경하지 않았다.

## 다음 구현에 필요한 결과물

1. 최종 URDF FK와 모터→URDF 관절 변환.
2. 25개 관측 생성 및 출력 scale/clamp를 적용하는 ROS 어댑터.
3. 그리퍼 열기·접근·닫기·들기·운반·놓기 상태 전이와 실패 처리.
4. 후보 30차원 정책을 사용하려면 해당 학습 env/agent 설정과 누락 외부 가중치 또는 완전한 재수출 모델 확보.
