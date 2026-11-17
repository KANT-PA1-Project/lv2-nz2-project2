#!/usr/bin/env python3
"""Compare measured PhysX wrist poses with independent URDF FK; no hardware I/O."""
import json, sys
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.rotations import quat_to_rot_matrix
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform
world=World(stage_units_in_meters=1.,physics_dt=1/120)
add_reference_to_stage(str(root/'models/r12/r12_arm.usda'),'/World/R12')
robot=world.scene.add(SingleArticulation('/World/R12',name='robot'))
world.reset()
from omni.usd import get_context
stage=get_context().get_stage()
paths=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='arm_link5']
print('TCP_PATHS',paths,flush=True)
wrist=SingleXFormPrim(paths[0])
names=robot.dof_names
rng=np.random.default_rng(20261030)
rows=[]
for i in range(51):
    q=np.zeros(5) if i==0 else rng.uniform([-1.4,-.3,-.3,-.3,-.3],[1.4,.3,.3,.3,.3])
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(5))
    world.step(render=True)
    actual=robot.get_joint_positions()
    pos,quat=wrist.get_world_pose()
    fk=forward_transform(root/'models/r12/r12_arm.urdf','arm_link5',dict(zip(names,actual)))
    rot=quat_to_rot_matrix(quat)
    tip=np.asarray(pos)+rot@np.array([.12475,0,0])
    expected_tip=fk[:3,3]+fk[:3,:3]@np.array([.12475,0,0])
    rows.append({'q':actual.tolist(),'wrist_error_m':float(np.linalg.norm(pos-fk[:3,3])), 'rotation_matrix_error':float(np.linalg.norm(rot-fk[:3,:3])), 'candidate_tip_error_m':float(np.linalg.norm(tip-expected_tip))})
result={'samples':rows,'max_wrist_error_m':max(r['wrist_error_m'] for r in rows),'max_candidate_tip_error_m':max(r['candidate_tip_error_m'] for r in rows),'max_rotation_matrix_error':max(r['rotation_matrix_error'] for r in rows),'candidate_tip_offset_m':[.12475,0,0],'scope':'Kinematic consistency only. CAD fingertip midpoint is uncalibrated. No Cartesian tracking/grasp test.'}
(root/'reports/local/r12_tcp_validation.json').write_text(json.dumps(result,indent=2))
print('TCP_RESULT',json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
world.stop();app.close()
