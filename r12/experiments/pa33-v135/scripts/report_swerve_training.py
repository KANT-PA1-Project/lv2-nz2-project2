#!/usr/bin/env python3
from pathlib import Path
import json,hashlib,zipfile,shutil,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12';data=json.loads((root/'reports/local/swerve/physics.json').read_text());training=json.loads((root/'reports/local/swerve/training.json').read_text())
fig,axes=plt.subplots(2,3,figsize=(14,7))
for ax,r in zip(axes.flat,data['results']):
 rows=r['trace'];t=[s['time'] for s in rows];v=np.array([s['measured_twist'] for s in rows])
 for i,label in enumerate(['vx (m/s)','vy (m/s)','wz (rad/s)']):
  ax.plot(t,v[:,i],label=label);ax.plot(t,[r['command'][i] if 1<=s<7 else 0 for s in t],color=f'C{i}',linestyle='--',alpha=.5)
 ax.set(title=r['case']+(' PASS' if r['passed'] else ' FAIL'),xlabel='Time (s)');ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.suptitle('Learned swerve actuation: wheel-driven PhysX\nSolid: measured body velocity; dashed: requested. Steady evaluation: 4-7 seconds.')
fig.tight_layout();fig.savefig(out/'R12_swerve_validation.png',dpi=140)
lines=['R12 스워브 구동 학습 결과 (2026-10-30)','', '최종 사용자 지시: 학습 대상은 스워브 구동. 위치 추정/지도/경로는 장착 센서와 cuVSLAM, nvBlox, Nav2가 담당.', '기존 지도 기반 waypoint_policy 실험은 별도 보존하며 실사용 제어기에서 제외. 해당 실험의 100/100 운동학 성공은 센서 SLAM 또는 물리 미션 완주 증거가 아님.','', '학습: 차체 vx,vy,wz -> 4바퀴 평면 속도 벡터를 역기구학 교사로 모방학습. 50,000개/50epoch; 별도 시드 10,000개 평가.', f"바퀴 평면 속도 RMSE {training['wheel_velocity_rmse_m_s']:.7f} m/s; 최대 성분 오차 {training['wheel_velocity_max_abs_error_m_s']:.7f} m/s.", '후처리: 조향 +/-90도, 조향 목표 변화율 1rad/s, 바퀴 속도 10rad/s 제한. 모든 바퀴가 목표 조향 0.25rad 이내일 때 구동.', '바닥 마찰/슬립/하중 대응 강화학습이 아닌 역기구학 초기 학습 단계. 속도 피드백 최적화 미적용.','', '물리시험: 원본 R12 이동 관절과 바퀴 원통 충돌체 사용. 지면, 중력, 모터 구동; 시험 도중 base pose/velocity 강제 추종 없음.', '각 시험 9초: 0-1초 정지, 1-7초 명령, 7-9초 정지. 평가는 4-7초 정상상태 RMSE + 마지막 정지 속도.', '통과 기준: 각 평면 속도 RMSE <0.03m/s, 회전 RMSE <0.08rad/s, 최종 평면 속도 <0.02m/s 및 회전 <0.04rad/s.', '초기 4/6 통과: 직진/후진 조향 전환 중 구동 문제. 목표 조향 누적 변화율과 전체 정렬 대기 수정; 초기 실패 로그 보존.']
for r in data['results']:lines.append(f"{r['case']}: passed={r['passed']}; requested={r['command']}; mean={r['steady_mean']}; RMSE={r['steady_rmse']}")
lines+=['','ROS 입력 /r12/cmd_vel TwistStamped(base_link), /r12/joint_states. 출력 /r12/swerve_joint_commands (시뮬레이션용 분리 토픽).','명령/상태 0.25초 미갱신 시 바퀴 정지. ROS loopback 확인; 실제 articulation 브리지/실물 송신 연결은 아직 미검증.','', '필수 피드백: OpenCR 2개 IMU (/opencr_1/imu, /opencr_2/imu), 조향 4개와 구동 4개 엔코더. 8개 엔코더 least-squares odometry 수학 구현/시험 완료. 두 IMU live fusion은 이 어댑터에 미연결; 기존 프로젝트도 geometry_verified=false.', '센서 통합 계약: configs/r12_navigation_contract.json. cuVSLAM 카메라 위치추정 -> nvBlox 깊이+pose ESDF -> holonomic Nav2 -> learned swerve.','2D 라이다는 Nav2 obstacle layer/충돌 감시에 사용. nvBlox 직접 LiDAR 통합은 검증된 3D LiDAR 모델일 때만. Ground-truth pose는 평가용으로만 사용.', '카메라/라이다 렌더링 토픽, 실측 calibration/TF, 센서 시간 동기, SLAM 추정 pose만 사용하는 전체 미로 완주는 미완료.','', '미로: 제공 USD 형상 유지. 좌하단 기둥 geo_3 중심을 (0,0)으로 평행 이동; 문서와 USD 치수 차이 있어 임시 정합.','박스: YOLO generate_dataset.py의 205x205x70mm 갈색 골판지 proxy와 일치. 실물 밤부랩 외형/치수 인증 아님. 중앙 큰 박스는 미션 410x410x70mm 4개 구성 유지.','시작 영역 600mm 정사각형은 도면에서 추정한 값으로 실측 미확인. 접근점은 탐색용 staging pose이며 집기 도달 보증 아님.','', '재현:', '.venv-r12/bin/python scripts/train_r12_swerve.py','OMNI_KIT_ACCEPT_EULA=YES bash scripts/r12_runtime.sh python scripts/validate_r12_swerve.py','source /opt/ros/lyrical/setup.bash && .venv-r12/bin/python scripts/r12_learned_swerve_node.py','', '참고:', 'https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html','https://nvidia-isaac-ros.github.io/v/release-4.0/repositories_and_packages/isaac_ros_nvblox/index.html']
(out/'R12_SWERVE_TRAINING.txt').write_text('\n'.join(lines)+'\n')
for name in ['training','physics','ros_adapter']:shutil.copy(root/f'reports/local/swerve/{name}.json',out/f'R12_swerve_{name}.json')
files=[str(p.relative_to(root)) for d in ['models/swerve','models/navigation','reports/local/swerve','assets/vendor/maze'] for p in (root/d).rglob('*') if p.is_file() and p.suffix!='.log']
files+=['configs/r12_navigation_contract.json','src/arm_practice/swerve.py','src/arm_practice/navigation.py','scripts/prepare_maze_navigation.py','scripts/train_maze_navigation.py','scripts/train_r12_swerve.py','scripts/validate_r12_swerve.py','scripts/r12_learned_swerve_node.py','scripts/report_swerve_training.py','tests/test_swerve.py']
with zipfile.ZipFile(out/'R12_swerve_maze_update.zip','w',zipfile.ZIP_DEFLATED) as z:
 for f in files:z.write(root/f,f)
 z.write(out/'R12_SWERVE_TRAINING.txt','README_SWERVE.txt')
with zipfile.ZipFile(out/'R12_swerve_maze_update.zip') as z:assert z.testzip() is None
(out/'SHA256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.name!='SHA256.json'},indent=2))
print('REPORT_COMPLETE',sum(r['passed'] for r in data['results']),len(data['results']))
