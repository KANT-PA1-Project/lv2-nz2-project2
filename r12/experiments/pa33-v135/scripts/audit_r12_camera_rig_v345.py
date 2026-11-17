import json,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
r=Path(__file__).resolve().parents[1];sys.path[:0]=[str(r/'src'),str(r/'scripts/sensor_runtime')]
from camera_rig import transforms,PreparedCameraRig
rig=PreparedCameraRig(r);rng=np.random.default_rng(345);names=[j.get('name') for j in ET.parse(r/'models/r12/r12_arm.urdf').getroot().findall('joint') if j.get('type')!='fixed'];times={'original':[],'prepared':[]};error=0
for _ in range(100):
 joints=dict(zip(names,rng.uniform(-1,1,len(names))))
 t=time.perf_counter_ns();old=transforms(r,joints);times['original'].append((time.perf_counter_ns()-t)/1e6)
 t=time.perf_counter_ns();new=rig.transforms(joints);times['prepared'].append((time.perf_counter_ns()-t)/1e6)
 assert old.keys()==new.keys()
 for key in old:
  error=max(error,float(np.max(np.abs(old[key]-new[key]))));np.testing.assert_allclose(old[key],new[key],rtol=0,atol=1e-12)
report={'poses':100,'camera_transforms':800,'max_abs_error':error,'median_ms':{k:float(np.median(v)) for k,v in times.items()},'scope':'Offline numerical equivalence and CPU timing, not physical pose validation or GUI latency'}
(r.parents[1]/'outputs/r12/v345_fk_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
