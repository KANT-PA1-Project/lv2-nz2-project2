import json
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];run=root/'reports/local/observed_history_v159';m=json.loads(sorted((run/'observation_history').glob('*.json'))[-1].read_text());start=np.array(m['arm_measured_position']);model=CollisionModel();count=int(np.ceil(np.max(abs(start))/.005))+1;hits=[]
for i,t in enumerate(np.linspace(0,1,count)):
 q=start*(1-t);h=model.hits(q)
 if h:hits.append(dict(sample=i,q=q.tolist(),pairs=h))
r=dict(complete=True,start_frame=m['frame_index'],start_q=start.tolist(),goal_q=[0]*5,samples=count,max_joint_spacing_rad=.005,self_collision_samples=hits,execution_authorized=False,scope='Nominal straight measured-joint to zero-home, USD collision model and open gripper. Not continuous sweep, tracking/torque response or environment collision certificate.')
(root.parent.parent/'outputs/r12/v163_final_home_self.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
