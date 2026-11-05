"""Prepare image-only replay manifest; excludes world placement and fixture labels."""
import json,hashlib
from pathlib import Path
import numpy as np,cv2
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';src=out/'static_stereo_v299';m=json.loads((src/'manifest.json').read_text());rows=m['rows'];assert len(rows)==6;by={r['camera']:r for r in rows};records=[]
for i in [1,2,3]:
 left=by[f'r07_camera{i}_left_optical'];right=by[f'r07_camera{i}_right_optical'];Tl=np.array(left['T_base_optical']);Tr=np.array(right['T_base_optical']);relative=np.linalg.inv(Tl)@Tr
 assert np.allclose(relative[:3,:3],np.eye(3),atol=1e-6)
 assert np.allclose(relative[:3,3],[.05,0,0],atol=1e-6),relative
 for side,r in [('left',left),('right',right)]:
  path=src/f'frame_{r["index"]:02d}.png';im=cv2.imread(str(path));assert im.shape==(480,640,3)
  records.append(dict(camera_frame=r['camera'],image=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),topic_base='/r12/'+('stereo' if i==1 else f'r07_camera{i}')+'/'+side,resolution=[640,480],K=[400.,0.,320.,0.,400.,240.,0.,0.,1.],P=[400.,0.,320.,-400*float(relative[0,3]) if side=='right' else 0.,0.,400.,240.,0.,0.,0.,1.,0.],T_base_optical=r['T_base_optical']))
calibration_files=['configs/sensors/camera_rig.json','models/r12/r12_arm.urdf'];hashes={p:hashlib.sha256((repo/p).read_bytes()).hexdigest() for p in calibration_files}
report=dict(schema='r12_static_stereo_replay_v1',images=records,calibration_source_sha256=hashes,scope='Static rendered snapshot per camera;sequential rendering of unchanged scene,not synchronized live capture. Simulation intrinsics/CAD transforms,not hardware calibration. World pose/fixture labels excluded;replay publisher must not emit odometry.',source_run_id_required=True,source_clock_epoch_required=True,pose_join_verified=False)
(out/'v299_replay_manifest.json').write_text(json.dumps(report,indent=2)+'\n');print('6 images;3 rectified pairs;baseline0.05m verified;replay manifest saved')
