#!/usr/bin/env python3
from pathlib import Path
import json,shutil,zipfile,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12'
data=json.loads((root/'reports/local/r12_tcp_tracking.json').read_text())
fig,axes=plt.subplots(2,3,figsize=(13,7))
for j,r in enumerate(data['results']):
 target=np.array([s['target_m'] for s in r['samples']]);actual=np.array([s['actual_m'] for s in r['samples']]);errors=np.array([s['error_m'] for s in r['samples']])*1000
 horizontal=0 if j==2 else 1
 axes[0,j].plot(target[:,horizontal]*1000,target[:,2]*1000,label='target');axes[0,j].plot(actual[:,horizontal]*1000,actual[:,2]*1000,'--',label='measured');axes[0,j].set(xlabel=('X (mm)' if j==2 else 'Y (mm)'),ylabel='Z (mm)',title=r['name']);axes[0,j].legend()
 axes[0,j].set_aspect('equal',adjustable='datalim')
 axes[1,j].plot(np.arange(len(errors))/120,errors);axes[1,j].axhline(5,color='r',linestyle='--');axes[1,j].set(xlabel='Time (s)',ylabel='TCP error (mm)',title=f"RMS {r['tracking_rms_m']*1000:.3f} mm; max {max(errors):.3f} mm")
fig.suptitle('R12 CAD TCP: actuated physics tracking, 2 Nm limit\nPosition tracking only; no physical TCP calibration or grasp verification')
fig.tight_layout();fig.savefig(out/'R12_tcp_tracking.png',dpi=150)
for name in ['r12_tcp_tracking.json','r12_tcp_collisions.json']:
 shutil.copy(root/'reports/local'/name,out/name.replace('r12_','R12_'))
lines=['R12 CAD TCP 보정 및 경로 추종 결과','', '실물 보정 아님: 제공 CAD 손가락 끝 중앙을 기준으로 arm_link5에서 +X 0.12475 m에 grasp_tcp 프레임 추가.', 'URDF: models/r12/r12_tcp.urdf; USD: models/r12/r12_tcp.usda.', '기존 정책은 손목 기준이므로 이 TCP에 재사용하지 않음. 새 추종기는 Cartesian IK + 중력 보상 관절 PD.', '초기 실패 원인: USD angular drive의 degree 단위를 런타임 radian gain과 혼동. 초기 실제 런타임 강성 2291.8313, 감쇠 229.18312에서 40으로 가정한 중력 보상으로 약 24 mm 과보상. API로 40/4 명시 설정 후 재시험.', '최대토크 2 N·m, stiffness 40, damping 4. 위치 추종 기준 최대 5 mm, 속도 0.4 rad/s, 원래 관절범위.', '각 경로 8초, 120 Hz, 960개 측정 샘플. 시작 위치 설정 후 2초 안정화. 경로 추종 중 위치 강제설정 없음.', '자세 방향은 고정하지 않음. 물체/그리퍼 개폐/접촉은 시험하지 않음.','']
for r in data['results']:lines.append(f"{r['name']}: passed={r['passed']}, RMS={r['tracking_rms_m']*1000:.4f} mm, max={r['tracking_max_m']*1000:.4f} mm, final={r['final_error_m']*1000:.4f} mm, maxspeed={r['max_joint_speed_rad_s']:.5f} rad/s")
col=json.loads((root/'reports/local/r12_tcp_collisions.json').read_text());lines+=['','CAD 비인접 링크 표면 충돌 샘플 검사 (인접 링크 기본 중첩 제외; 연속 충돌 증명 아님):',json.dumps(col['results'],ensure_ascii=False),'','실행:','python3 scripts/prepare_r12_tcp.py','OMNI_KIT_ACCEPT_EULA=YES bash scripts/r12_runtime.sh python scripts/track_r12_tcp.py','PYTHONPATH=src .venv-r12/bin/python scripts/check_r12_tcp_collisions.py','', '실물 보정에는 여러 방향에서 동일 고정점에 공구 끝을 맞춘 플랜지 포즈 측정 및 최소제곱 pivot 보정이 필요. 이번 결과는 그 실측을 대체하지 않음.']
(out/'R12_TCP_TRACKING.txt').write_text('\n'.join(lines)+'\n')
files=['scripts/prepare_r12_tcp.py','scripts/track_r12_tcp.py','scripts/check_r12_tcp_collisions.py','scripts/report_r12_tcp_tracking.py','scripts/inspect_r12_geometry.py','src/arm_practice/kinematics.py','models/r12/r12_tcp.urdf','models/r12/r12_tcp.usda','configs/r12_tcp_calibration.json','reports/local/r12_tcp_tracking.json','reports/local/r12_tcp_tracking_initial.json','reports/local/r12_tcp_collisions.json']
with zipfile.ZipFile(out/'R12_tcp_tracking_update.zip','w',zipfile.ZIP_DEFLATED) as z:
 for p in files:z.write(root/p,p)
 z.write(out/'R12_TCP_TRACKING.txt','README.txt')
# Update full implementation archive without duplicate member names.
archive=out/'R12_implementation.zip';temporary=archive.with_suffix('.tmp')
with zipfile.ZipFile(archive) as src,zipfile.ZipFile(temporary,'w',zipfile.ZIP_DEFLATED) as dst:
 for item in src.infolist():
  if item.filename not in files:dst.writestr(item,src.read(item.filename))
 for p in files:dst.write(root/p,p)
temporary.replace(archive)
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
(out/'SHA256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file() and p.name!='SHA256.json'},indent=2))
print('\n'.join(lines))
