"""Read-only simulator truth for an isolated offline evaluator, never observations."""
import json,time
import numpy as np
from scipy.spatial.transform import Rotation

def matrix(position,quaternion_wxyz):
    p=np.asarray(position,float);q=np.asarray(quaternion_wxyz,float)
    if p.shape!=(3,)or q.shape!=(4,)or not np.isfinite(p).all()or not np.isfinite(q).all()or abs(np.linalg.norm(q)-1)>1e-5:raise ValueError('invalid_evaluator_pose')
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();T[:3,3]=p;return T
class EvaluatorCapture:
    def __init__(self,world,stage,robot,out):
        from isaacsim.core.prims import SingleRigidPrim
        self.world=world;self.stage=stage;self.robot=robot;self.out=out;self.rows=[]
        self.puck=world.scene.add(SingleRigidPrim('/World/Mission/Pucks/Puck0',name='evaluator_only_puck'))
    def record(self,frame):
        from pxr import UsdGeom
        start=time.monotonic_ns();step=int(self.world.current_time_step_index)
        def usd(path):return np.array(UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)).T
        objectT=matrix(*self.puck.get_world_pose());baseT=matrix(*self.robot.get_world_pose())
        cameraT=usd('/World/Spawn/R12/arm_link5/arm_camera_left_optical')@np.diag([1.,-1.,-1.,1.])
        if int(self.world.current_time_step_index)!=step:raise ValueError('evaluator_physics_advanced')
        self.rows.append(dict(frame=frame,physics_step=step,start_ns=start,end_ns=time.monotonic_ns(),T_world_object_physx=objectT.tolist(),T_world_object_USD=usd('/World/Mission/Pucks/Puck0').tolist(),T_world_base_physx=baseT.tolist(),T_world_base_USD=usd('/World/Spawn/R12/base_link').tolist(),T_world_camera_optical_USD=cameraT.tolist(),T_world_arm_link5_USD=usd('/World/Spawn/R12/arm_link5').tolist(),scope='EVALUATOR ONLY; never sensor observation or command',object_dimensions_m=[.03,.03,.06],motion_authorized=False))
        (self.out/'evaluator_only_ground_truth.json').write_text(json.dumps(self.rows,indent=2))
