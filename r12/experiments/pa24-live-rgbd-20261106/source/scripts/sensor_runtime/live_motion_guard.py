"""Current-execution geometry/feedback guard for one tiny diagnostic move.

No mission motion authority and no calibration authority. Collision-map
geometry never becomes the observed target. No serialized approval input.
"""
import hashlib
import itertools
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import UsdGeom,UsdPhysics
from runtime_safety_geometry import audit
from arm_practice.native_rgbd.live_view import LiveViewCandidate


class LiveMotionGuard:
    def __init__(self,root,stage,robot,arm,fingers,run_id,issuer_id,read_epoch,clock=time.monotonic_ns):
        self.root=Path(root);self.stage=stage;self.robot=robot;self.arm=arm;self.fingers=fingers
        self.run_id=run_id;self.issuer_id=issuer_id;self.epoch=read_epoch;self.clock=clock;self.consumed=False
        self.robot_prefix='/World/Spawn/R12'
        tree=ET.parse(self.root/'models/r12/r12_arm.urdf').getroot()
        limits={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in tree.findall('joint') if j.find('limit') is not None}
        self.joint_lower=np.array([limits[f'arm_joint{i}'][0]+.02 for i in range(1,6)])
        self.joint_upper=np.array([limits[f'arm_joint{i}'][1]-.02 for i in range(1,6)])
        self.anchor=np.asarray(robot.get_joint_positions())[arm].astype(float)
        self.finger_anchor=np.asarray(robot.get_joint_positions())[fingers].astype(float)
        self.half_width=.002;self.tracking=.001;self.max_step=.0004
        self.lower=np.maximum(self.anchor-self.half_width-self.tracking,self.joint_lower)
        self.upper=np.minimum(self.anchor+self.half_width+self.tracking,self.joint_upper)
        self.env_prims=[];self.robot_colliders=[]
        for p in stage.Traverse():
            path=str(p.GetPath())
            if not path.startswith(self.robot_prefix) and p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get():
                raise ValueError('unmodeled_dynamic_environment')
            if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
            if path.startswith(self.robot_prefix):self.robot_colliders.append(path)
            else:self.env_prims.append(p)
        if not self.env_prims:raise ValueError('missing_environment_geometry')
        native=np.asarray(robot._articulation_view._physics_view.get_link_transforms())[0]
        self.native_names=list(robot._articulation_view.body_names)
        self.base=self._native_frames(native)['base_link']
        spheres,fingerprint=self._environment()
        self.fingerprint=fingerprint
        self.report=audit(self.root,self.anchor,self.anchor+self.half_width,
                          tracking=self.tracking,obstacle_spheres=spheres)
        expected={self.robot_prefix+p.removeprefix('/Robot') for p in self.report['collider_paths']}
        if set(self.robot_colliders)!=expected:raise ValueError('live_collision_inventory_mismatch')
        if not self.report['all_self_bounds_proven'] or not self.report['environment_bounds_proven']:
            raise ValueError('joint_region_not_geometrically_clear')
        native_frames=self._native_frames(native);inverse=np.linalg.inv(self.base)
        max_translation=0.;max_rotation=0.
        for name,reference in self.report['model_body_frames_base'].items():
            if name not in native_frames:raise ValueError('native_body_missing:'+name)
            error=np.linalg.inv(np.asarray(reference))@inverse@native_frames[name]
            max_translation=max(max_translation,float(np.linalg.norm(error[:3,3])))
            max_rotation=max(max_rotation,float(Rotation.from_matrix(error[:3,:3]).magnitude()))
        if max_translation>5e-5 or max_rotation>1e-4:raise ValueError('native_CAD_frame_alignment_failed')
        self.report.update(scope='same-execution static bench joint-region geometry; diagnostic move only',
                           native_frame_translation_max_m=max_translation,native_frame_rotation_max_rad=max_rotation,
                           native_CAD_alignment_verified=True,live_environment_fingerprint=self.fingerprint,
                           live_collision_inventory_verified=True,robot_collider_count=len(self.robot_colliders),
                           external_collider_count=len(self.env_prims),motion_authorized=False,
                           calibration_accepted=False,full_mission_environment_certified=False)

    def _native_frames(self,native):
        frames={}
        for name,row in zip(self.native_names,native):
            T=np.eye(4);T[:3,3]=row[:3]
            T[:3,:3]=Rotation.from_quat(row[3:7]).as_matrix()
            frames[name]=T
        return frames

    def _environment(self):
        cache=UsdGeom.XformCache();inverse=np.linalg.inv(self.base);spheres=[];signature=[]
        for p in self.env_prims:
            if not p or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():raise ValueError('environment_geometry_changed')
            M=np.asarray(cache.GetLocalToWorldTransform(p)).T
            if p.IsA(UsdGeom.Cube):half=np.full(3,float(UsdGeom.Cube(p).GetSizeAttr().Get())/2)
            elif p.IsA(UsdGeom.Cylinder):
                shape=UsdGeom.Cylinder(p);half=np.full(3,float(shape.GetRadiusAttr().Get()));half['XYZ'.index(str(shape.GetAxisAttr().Get()))]=float(shape.GetHeightAttr().Get())/2
            else:raise ValueError('unsupported_live_environment_collider')
            vertices=np.array(list(itertools.product(*[[-v,v] for v in half])))
            world=(M@np.c_[vertices,np.ones(8)].T).T[:,:3]
            center=world.mean(axis=0);radius=float(np.linalg.norm(world-center,axis=1).max())
            spheres.append(((inverse@np.r_[center,1])[:3],radius))
            signature.append(dict(path=str(p.GetPath()),transform=M.tolist(),half=half.tolist()))
        return spheres,hashlib.sha256(json.dumps(signature,sort_keys=True).encode()).hexdigest()

    def check_state(self,q,qd,fd,contacts):
        q=np.asarray(q,float);qd=np.asarray(qd,float);fd=np.asarray(fd,float)
        if not np.isfinite(np.r_[q,qd,fd]).all():raise ValueError('nonfinite_live_motion_feedback')
        if np.any(q[self.arm]<self.lower) or np.any(q[self.arm]>self.upper):raise ValueError('native_joint_region_exit')
        if np.max(abs(q[self.fingers]-self.finger_anchor))>.001:raise ValueError('gripper_tracking_region_exit')
        if max(np.max(abs(qd[self.arm])),np.max(abs(fd[self.arm])))>=.02:raise ValueError('native_or_FD_velocity_abort')
        if contacts:raise ValueError('unexpected_contact_during_diagnostic_move')
        if self._environment()[1]!=self.fingerprint:raise ValueError('environment_geometry_changed')

    def admit(self,candidate):
        if not isinstance(candidate,LiveViewCandidate):raise ValueError('serialized_or_untrusted_candidate')
        if candidate['issuer_id']!=self.issuer_id:raise ValueError('candidate_issuer_mismatch')
        if self.consumed:raise ValueError('diagnostic_move_already_consumed')
        if candidate['run_id']!=self.run_id:raise ValueError('candidate_run_mismatch')
        if self.epoch()[0]!=candidate['capture_step']:raise ValueError('candidate_epoch_not_current')
        if not 0<=self.clock()-candidate['capture_lower_ns']<=500_000_000:raise ValueError('candidate_stale_at_dispatch')
        current=np.asarray(self.robot.get_joint_positions())[self.arm].astype(float)
        if not np.array_equal(current,candidate['q_start']):raise ValueError('candidate_native_start_changed')
        goal=np.asarray(candidate['q'],float)
        if goal.shape!=(5,) or not np.isfinite(goal).all():raise ValueError('invalid_live_candidate')
        if np.any(abs(goal-self.anchor)>self.half_width):raise ValueError('candidate_outside_certified_region')
        if np.any(goal<self.joint_lower) or np.any(goal>self.joint_upper):raise ValueError('candidate_joint_margin')
        if self._environment()[1]!=self.fingerprint:raise ValueError('environment_geometry_changed')
        delta=goal-current
        maximum=float(np.max(abs(delta)))
        if maximum<.00005:raise ValueError('candidate_does_not_require_measurable_move')
        bounded=current+delta*min(1.,self.max_step/maximum)
        self.consumed=True
        return bounded.copy()
